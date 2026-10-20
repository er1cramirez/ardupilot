/*
 * VisionStateEstimator.cpp
 *
 * Implementation of the custom state estimator for ArUco marker tracking.
 * Linear Kalman Filter with jerk model, 3 decoupled per-axis filters.
 *
 * State per axis: [position, velocity, acceleration, jerk]
 * Process model: constant jerk + white noise (snap as driving noise)
 * Measurement: position only (from ArUco via MAVLink, already in NED)
 *
 * References:
 *   - "Modeling and Analysis of Velocity, Acceleration and Jerk Models
 *      for GPS/INS Target Tracking" (Ref [5] in design doc)
 *   - User document: Filtro de Kalman lineal multi-rate con modelo de jerk
 */
#include "Copter.h"
#include "VisionStateEstimator.h"

// ---------------------------------------------------------------
// VisionStateEstimator public methods
// ---------------------------------------------------------------

void VisionStateEstimator::init()
{
    _initialized = false;
    _pending.valid = false;
    _last_predict_sec = 0.0f;
    _last_valid_meas_ms = 0;
    _log_counter = 0;
    _frozen = false;

    // Zero all filter states
    for (uint8_t i = 0; i < 3; i++) {
        memset(_kf[i].x, 0, sizeof(_kf[i].x));
        memset(_kf[i].P, 0, sizeof(_kf[i].P));
    }

    _position.zero();
    _velocity.zero();
    _acceleration.zero();
    _jerk.zero();
}

void VisionStateEstimator::predict(float now_sec)
{
    // First call: just record time, no prediction possible yet
    if (_last_predict_sec <= 0.0f) {
        _last_predict_sec = now_sec;
        return;
    }

    // Compute real dt from timestamps
    float dt = now_sec - _last_predict_sec;

    // Guard against bad dt values
    if (dt < DT_MIN) {
        return;  // Too soon, skip this cycle
    }
    if (dt > DT_MAX) {
        // Too large gap — reset time but don't predict (state would diverge)
        _last_predict_sec = now_sec;
        return;
    }

    // Only run KF if we have been initialized with at least one measurement
    if (_initialized) {
        // --- Sin mediciones recientes: congelar en vez de extrapolar ---
        if (!_pending.valid && (AP_HAL::millis() - _last_valid_meas_ms) > HOLD_MS) {
            if (!_frozen) {
                for (uint8_t i = 0; i < 3; i++) {
                    _kf[i].x[2] = 0.0f;   // aceleración
                    _kf[i].x[3] = 0.0f;   // jerk
                }
                _frozen = true;
                _update_outputs();
            }
            _last_predict_sec = now_sec;
            return;
        }

        // --- KF Predict step for each axis ---
        for (uint8_t i = 0; i < 3; i++) {
            _kf[i].predict(dt, SIGMA_JERK_SQ);
        }

        // --- Consume pending measurement if available ---
        if (_pending.valid) {
            // Scale R based on reprojection error:
            //   reproj <= REPROJ_NOMINAL: R = R_BASE
            //   reproj >  REPROJ_NOMINAL: R = R_BASE * (reproj / NOMINAL)^2
            float R_scaled = R_BASE;
            if (_pending.reprojection_error > REPROJ_NOMINAL) {
                float scale = _pending.reprojection_error / REPROJ_NOMINAL;
                R_scaled = R_BASE * scale * scale;
            }

            // Run KF update for each axis
            for (uint8_t i = 0; i < 3; i++) {
                _kf[i].update(_pending.position_ned[i], R_scaled);
            }
            _pending.valid = false;
        }

        // Update output vectors from KF states
        _update_outputs();

        // Log at ~10Hz (every 10th call at 100Hz)
        _log_counter++;
        if (_log_counter >= 10) {
            _log_counter = 0;
            _log_state();
        }
    }

    _last_predict_sec = now_sec;
}

bool VisionStateEstimator::update_measurement(const Vector3f &position_ned,
                                               uint8_t flags, float reprojection_error)
{
    // --- Flag-based gating ---
    // Must have valid_detection flag set
    if (!(flags & FLAG_VALID_DETECTION)) {
        return false;
    }

    // Reject stale frames
    if (flags & FLAG_STALE_FRAME) {
        return false;
    }

    // Reject if reprojection error is too high
    if (reprojection_error > REPROJ_REJECT) {
        return false;
    }

    // Initialize with first valid measurement, or re-initialize after a long gap
    if (!_initialized || (AP_HAL::millis() - _last_valid_meas_ms) > RESET_MS) {
        for (uint8_t i = 0; i < 3; i++) {
            _kf[i].init(position_ned[i], R_BASE, P_INIT_VEL,
                        P_INIT_ACC, P_INIT_JERK);
        }
        _initialized = true;
        _update_outputs();
    }

    // Apply low_quality multiplier to reprojection error for R scaling
    float effective_reproj = reprojection_error;
    if (flags & FLAG_LOW_QUALITY) {
        effective_reproj *= R_LOW_QUALITY_MULT;
    }

    // Store as pending measurement for next predict cycle
    _pending.position_ned = position_ned;
    _pending.reprojection_error = effective_reproj;
    _pending.valid = true;

    _last_valid_meas_ms = AP_HAL::millis();
    _frozen = false;
    return true;
}

bool VisionStateEstimator::is_healthy() const
{
    if (!_initialized) {
        return false;
    }
    return (AP_HAL::millis() - _last_valid_meas_ms) < TIMEOUT_MS;
}

// ---------------------------------------------------------------
// AxisKF methods
// ---------------------------------------------------------------

void VisionStateEstimator::AxisKF::init(float p0, float r_base,
                                         float p_init_vel, float p_init_acc,
                                         float p_init_jerk)
{
    // State: position from measurement, rest at zero (assume stationary start)
    x[0] = p0;
    x[1] = 0.0f;  // velocity
    x[2] = 0.0f;  // acceleration
    x[3] = 0.0f;  // jerk

    // Covariance: high uncertainty for derivatives, measurement noise for position
    memset(P, 0, sizeof(P));
    P[0][0] = r_base;
    P[1][1] = p_init_vel;
    P[2][2] = p_init_acc;
    P[3][3] = p_init_jerk;
}

void VisionStateEstimator::AxisKF::predict(float dt, float sigma_jerk_sq)
{
    // Pre-compute powers of dt
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    const float dt5 = dt3 * dt2;
    const float dt6 = dt3 * dt3;
    const float dt7 = dt4 * dt3;

    // ----- Build F(dt) 4x4: State transition matrix -----
    // F = [ 1    dt    dt²/2    dt³/6 ]
    //     [ 0     1      dt     dt²/2 ]
    //     [ 0     0       1       dt  ]
    //     [ 0     0       0        1  ]
    const float F[4][4] = {
        {1.0f, dt,   0.5f * dt2, dt3 / 6.0f},
        {0.0f, 1.0f, dt,         0.5f * dt2},
        {0.0f, 0.0f, 1.0f,       dt},
        {0.0f, 0.0f, 0.0f,       1.0f}
    };

    // ----- Build Q(dt) 4x4: Process noise covariance -----
    // Complete analytical form from jerk white noise model:
    // Q = σ_j² × [ dt⁷/252   dt⁶/72   dt⁵/30   dt⁴/24 ]
    //            [ dt⁶/72    dt⁵/20   dt⁴/8    dt³/6  ]
    //            [ dt⁵/30    dt⁴/8    dt³/3    dt²/2  ]
    //            [ dt⁴/24    dt³/6    dt²/2    dt     ]
    const float Q[4][4] = {
        {sigma_jerk_sq * dt7 / 252.0f, sigma_jerk_sq * dt6 / 72.0f,  sigma_jerk_sq * dt5 / 30.0f, sigma_jerk_sq * dt4 / 24.0f},
        {sigma_jerk_sq * dt6 / 72.0f,  sigma_jerk_sq * dt5 / 20.0f,  sigma_jerk_sq * dt4 / 8.0f,  sigma_jerk_sq * dt3 / 6.0f},
        {sigma_jerk_sq * dt5 / 30.0f,  sigma_jerk_sq * dt4 / 8.0f,   sigma_jerk_sq * dt3 / 3.0f,  sigma_jerk_sq * dt2 / 2.0f},
        {sigma_jerk_sq * dt4 / 24.0f,  sigma_jerk_sq * dt3 / 6.0f,   sigma_jerk_sq * dt2 / 2.0f,  sigma_jerk_sq * dt}
    };

    // ----- State prediction: x_new = F * x -----
    float x_new[4];
    for (uint8_t i = 0; i < 4; i++) {
        x_new[i] = 0.0f;
        for (uint8_t k = 0; k < 4; k++) {
            x_new[i] += F[i][k] * x[k];
        }
    }
    memcpy(x, x_new, sizeof(x));

    // ----- Covariance prediction: P_new = F * P * Fᵀ + Q -----
    // Step 1: FP = F * P
    float FP[4][4];
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            FP[i][j] = 0.0f;
            for (uint8_t k = 0; k < 4; k++) {
                FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }

    // Step 2: P = FP * Fᵀ + Q
    // Note: Fᵀ[k][j] = F[j][k]
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            P[i][j] = Q[i][j];
            for (uint8_t k = 0; k < 4; k++) {
                P[i][j] += FP[i][k] * F[j][k];
            }
        }
    }
}

float VisionStateEstimator::AxisKF::update(float z_meas, float R)
{
    // KF Measurement Update with H = [1, 0, 0, 0]
    // Since H only selects the position state, the math simplifies:
    //
    //   innovation:  y = z - x[0]
    //   innov. cov:  S = P[0][0] + R
    //   Kalman gain: K[i] = P[i][0] / S   (first column of P / S)
    //   state:       x[i] += K[i] * y
    //   covariance:  P[i][j] -= K[i] * P[0][j]

    // Innovation (residual)
    const float y = z_meas - x[0];

    // Innovation covariance
    const float S = P[0][0] + R;

    // Avoid division by zero or near-zero
    if (S < 1.0e-12f) {
        return y;
    }

    const float S_inv = 1.0f / S;

    // Kalman gain: K = P * H^T / S = P[:,0] / S
    float K[4];
    for (uint8_t i = 0; i < 4; i++) {
        K[i] = P[i][0] * S_inv;
    }

    // State update: x = x + K * y
    for (uint8_t i = 0; i < 4; i++) {
        x[i] += K[i] * y;
    }

    // Covariance update: P = (I - K*H) * P
    // Since H = [1,0,0,0], this simplifies to:
    //   P[i][j] = P[i][j] - K[i] * P[0][j]
    // We need to save P[0][j] first since row 0 gets modified
    float P_row0[4];
    memcpy(P_row0, P[0], sizeof(P_row0));

    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            P[i][j] -= K[i] * P_row0[j];
        }
    }

    return y;  // Return innovation for diagnostics
}

// ---------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------

void VisionStateEstimator::_update_outputs()
{
    for (uint8_t i = 0; i < 3; i++) {
        _position[i] = _kf[i].x[0];
        _velocity[i] = _kf[i].x[1];
        _acceleration[i] = _kf[i].x[2];
        _jerk[i] = _kf[i].x[3];
    }
}

void VisionStateEstimator::_log_state()
{
    AP::logger().Write("VEST", "TimeUS,px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz,h",
        "Qfffffffffffff",
        AP_HAL::micros64(),
        (double)_position.x, (double)_position.y, (double)_position.z,
        (double)_velocity.x, (double)_velocity.y, (double)_velocity.z,
        (double)_acceleration.x, (double)_acceleration.y, (double)_acceleration.z,
        (double)_jerk.x, (double)_jerk.y, (double)_jerk.z,
        (double)(is_healthy() ? 1.0f : 0.0f));
}
