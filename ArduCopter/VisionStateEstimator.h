/*
 * VisionStateEstimator.h
 *
 * Custom state estimator for ArUco marker tracking using a
 * linear Kalman Filter with jerk model.
 *
 * 3 decoupled per-axis filters, each with 4 states: [p, v, a, j]
 * - Prediction runs at fixed rate (100Hz via scheduler)
 * - Measurement updates arrive asynchronously via MAVLink
 * - All states are in NED frame (body->NED transform done before feeding)
 */
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

class VisionStateEstimator {
public:
    // Initialize the estimator (called once at startup)
    void init();

    // Prediction step — called at fixed rate (100Hz) with current time in seconds
    // Internally computes dt = now_sec - last_predict_sec
    void predict(float now_sec);

    // Feed a new measurement (already transformed to NED frame)
    // flags: bitmask from CUSTOM_VISION_DATA (bit0=valid, bit2=low_quality, bit3=stale)
    // reprojection_error: mean reprojection error in pixels
    // Returns true if the measurement was accepted for KF update
    bool update_measurement(const Vector3f &position_ned,
                            uint8_t flags, float reprojection_error);

    // --- Getters (all in NED frame) ---
    const Vector3f& get_position() const { return _position; }
    const Vector3f& get_velocity() const { return _velocity; }
    const Vector3f& get_acceleration() const { return _acceleration; }
    const Vector3f& get_jerk() const { return _jerk; }

    // Returns true if estimator has been initialized and has recent valid measurements
    bool is_healthy() const;

    // Time of last valid measurement in milliseconds
    uint32_t last_valid_measurement_ms() const { return _last_valid_meas_ms; }

    // Returns true if the estimator has been initialized with at least one measurement
    bool is_initialized() const { return _initialized; }

    // Milisegundos desde la última medición aceptada (UINT32_MAX si nunca)
    uint32_t ms_since_valid() const {
        return _initialized ? (AP_HAL::millis() - _last_valid_meas_ms) : UINT32_MAX;
    }

private:
    // ---------------------------------------------------------------
    // Per-axis Kalman Filter: 4 states [position, velocity, accel, jerk]
    // ---------------------------------------------------------------
    struct AxisKF {
        float x[4];        // State vector: [p, v, a, j]
        float P[4][4];     // Covariance matrix 4x4

        // Initialize state with first position measurement
        void init(float p0, float r_base, float p_init_vel,
                  float p_init_acc, float p_init_jerk);

        // Prediction step: apply F(dt) and Q(dt, sigma_jerk_sq)
        void predict(float dt, float sigma_jerk_sq);

        // Measurement update: scalar measurement z with variance R
        // H = [1, 0, 0, 0]
        // Returns innovation (residual)
        float update(float z_meas, float R);
    };

    AxisKF _kf[3];  // 0=x(N), 1=y(E), 2=z(D)

    // Consolidated outputs (updated after each predict/update cycle)
    Vector3f _position;
    Vector3f _velocity;
    Vector3f _acceleration;
    Vector3f _jerk;

    // --- Pending measurement (single-buffered, flag pattern) ---
    struct PendingMeasurement {
        Vector3f position_ned;
        float reprojection_error;
        bool valid;
    } _pending;

    // --- State ---
    bool _initialized;          // True after first valid measurement
    float _last_predict_sec;    // Timestamp of last predict() call
    uint32_t _last_valid_meas_ms;  // millis() of last accepted measurement
    uint8_t _log_counter;          // Counter to throttle logging to ~10Hz
    bool _frozen;                  // True mientras no hay mediciones (estado congelado)

    // --- Tunable parameters ---
    // Vision flags bitmask (matches CUSTOM_VISION_DATA flags field)
    static constexpr uint8_t FLAG_VALID_DETECTION = (1 << 0);
    static constexpr uint8_t FLAG_NEW_FRAME       = (1 << 1);
    static constexpr uint8_t FLAG_LOW_QUALITY      = (1 << 2);
    static constexpr uint8_t FLAG_STALE_FRAME      = (1 << 3);

    // Timeout for health check
    static constexpr uint32_t TIMEOUT_MS = 500;        // ms without valid measurement -> unhealthy
    // Ronda A: sin medición > HOLD_MS se congela el estado (no se extrapola con el modelo de jerk);
    // al recuperar tras > RESET_MS se reinicializa con la nueva medición.
    static constexpr uint32_t HOLD_MS = 100;
    static constexpr uint32_t RESET_MS = 500;

    // Measurement noise (position variance in m²)
    static constexpr float R_BASE = 1.0e-4f;             // Base measurement noise at nominal reprojection error

    // Reprojection error thresholds (pixels)
    static constexpr float REPROJ_NOMINAL = 1.0f;      // Below this: use R_BASE directly
    static constexpr float REPROJ_REJECT = 10.0f;       // Above this: reject measurement entirely
    // Between NOMINAL and REJECT: R scales as R_BASE * (reproj/NOMINAL)^2

    // R multiplier for low_quality flag
    static constexpr float R_LOW_QUALITY_MULT = 4.0f;  // 4x more noise when low_quality flag is set

    // Process noise: jerk spectral density (m/s³)²
    static constexpr float SIGMA_JERK_SQ = 500.0f;      // Tunable: depends on expected maneuvering

    // Initial covariance for unknown states
    static constexpr float P_INIT_VEL = 10.0f;         // (m/s)² initial velocity uncertainty
    static constexpr float P_INIT_ACC = 100.0f;        // (m/s²)² initial acceleration uncertainty
    static constexpr float P_INIT_JERK = 1000.0f;      // (m/s³)² initial jerk uncertainty

    // Minimum dt to avoid numerical issues
    static constexpr float DT_MIN = 1.0e-4f;           // 0.1ms minimum
    static constexpr float DT_MAX = 0.5f;              // 500ms maximum (skip if too large)

    // --- Internal helpers ---
    void _update_outputs();
    void _log_state();
};
