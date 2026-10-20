#include "Copter.h"
#define CTRL_TRANSFORMATION 0

// ---------------------------------------------------------------------------
// ModeLLC — aterrizaje por visión con campo de velocidades relativo.
// Ronda A (2026-10-20):
//   - Tubo del campo acotado por el cono de visibilidad de la cámara (fov_aware).
//   - Supervisor de pérdida de marcador (HOLD/TRACK/BLIND/ABORT/TOUCHDOWN) que
//     sustituye al fallback de poshold por GPS (no existe en el escenario sin GPS).
//   - El estimador se congela al perder mediciones y se reinicia al recuperarlas.
// ---------------------------------------------------------------------------

bool ModeLLC::init(bool ignore_checks)
{
    // Initialize position controller for Z axis if not already active
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // Set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    _return_home = true;
    _have_new_force_target = false;

    // Supervisor: arranca en HOLD y pasa a TRACK con mediciones continuas
    _sup = SupState::HOLD;
    _reacq_start_ms = 0;
    _td_low_start_ms = 0;
    gcs().send_text(MAV_SEVERITY_INFO, "LLC: init, sup=HOLD");

    return true;
}

void ModeLLC::setSupState(SupState s, const Vector3f& r_est)
{
    if (s == _sup) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    switch (s) {
    case SupState::BLIND:
        _blind_start_ms = now_ms;
        _td_low_start_ms = 0;
        break;
    case SupState::ABORT:
        _abort_posD_valid = ahrs.get_relative_position_D_origin(_abort_posD);
        _reacq_start_ms = 0;
        break;
    case SupState::TOUCHDOWN:
        _td_ramp_start_ms = now_ms;
        break;
    default:
        break;
    }
    AP::logger().Write("SUPV", "TimeUS,from,to,h,rho", "QBBff",
        AP_HAL::micros64(), (uint8_t)_sup, (uint8_t)s,
        (double)r_est.z, (double)norm(r_est.x, r_est.y));
    gcs().send_text(MAV_SEVERITY_INFO, "LLC: sup %u -> %u (h=%.2f rho=%.2f)",
        (unsigned)_sup, (unsigned)s, (double)r_est.z, (double)norm(r_est.x, r_est.y));
    _sup = s;
}

// Comando sin visión: nivelado (u_xy = 0) y lazo de velocidad vertical con la
// velocidad vertical del EKF (baro + IMU, disponible sin GPS). Plataforma estática.
Vector3f ModeLLC::verticalOnlyCommand(float rdz_des) const
{
    float vD = 0.0f;
    if (!ahrs.get_vert_pos_rate_D(vD)) {
        vD = 0.0f;
    }
    const float rdot_z = -vD;   // r = p_marcador - p_dron, marcador estático
    return Vector3f(0.0f, 0.0f, KP_Z * (rdot_z - rdz_des) - gOffset);
}

void ModeLLC::run()
{
    float psi_d = 0.0f;
    float psi_d_dot = 0.0f;

    Vector3f u_d(0.0f, 0.0f, 0.0f);
    Vector3f u_d_dot(0.0f, 0.0f, 0.0f);

    // --- Read vision estimator state ---
    const VisionStateEstimator& vest = copter.vision_estimator;
    Vector3f pos = vest.get_position();
    Vector3f vel = vest.get_velocity();
    Vector3f acc = vest.get_acceleration();
    const bool vision_ok = vest.is_healthy();
    const uint32_t now_ms = AP_HAL::millis();
    const bool fresh = vest.ms_since_valid() < T_LOSS_MS;

    // ---------------- Supervisor: transiciones ----------------
    switch (_sup) {
    case SupState::HOLD:
    case SupState::ABORT:
        // Volver a TRACK solo con mediciones continuas durante T_REACQ_MS
        if (!fresh) {
            _reacq_start_ms = 0;
        } else if (_reacq_start_ms == 0) {
            _reacq_start_ms = now_ms;
        } else if (vision_ok && (now_ms - _reacq_start_ms) > T_REACQ_MS) {
            setSupState(SupState::TRACK, pos);
        }
        break;
    case SupState::TRACK:
        if (!fresh) {
            const float rho_l = norm(pos.x, pos.y);
            if (pos.z < H_COMMIT && rho_l < RHO_COMMIT) {
                setSupState(SupState::BLIND, pos);
            } else {
                setSupState(SupState::ABORT, pos);
            }
        }
        break;
    case SupState::BLIND:
    case SupState::TOUCHDOWN:
        // Compromiso: no se regresa a TRACK durante el tramo final
        break;
    }

    // ---------------- Ley de control según estado ----------------
    switch (_sup) {
    case SupState::TRACK: {
        // Compute velocity field
        RelativeFieldOutput field = computeRelativeVelocityField(pos, _field_params);

        // Velocity tracking error
        Vector3f ev = vel - field.rdot_des;

        // Control law: u = kp * ev - gOffset * e_z
        u_d.x = KP_XY * ev.x;
        u_d.y = KP_XY * ev.y;
        u_d.z = KP_Z  * ev.z - gOffset;
        u_d_dot.zero();

        AP::logger().Write("VFLD", "TimeUS,rdx,rdy,rdz,rho,rref,erho,mur,muz", "Qffffffff",
            AP_HAL::micros64(),
            (double)field.rdot_des.x, (double)field.rdot_des.y, (double)field.rdot_des.z,
            (double)field.rho, (double)field.rho_ref, (double)field.e_rho,
            (double)field.mu_r, (double)field.mu_z);

        AP::logger().Write("VDBG", "TimeUS,vx,vy,vz,dx,dy,dz,ex,ey,ez", "Qfffffffff",
            AP_HAL::micros64(),
            (double)vel.x, (double)vel.y, (double)vel.z,
            (double)field.rdot_des.x, (double)field.rdot_des.y, (double)field.rdot_des.z,
            (double)ev.x, (double)ev.y, (double)ev.z);
        break;
    }
    case SupState::HOLD:
        u_d = verticalOnlyCommand(0.0f);
        break;
    case SupState::ABORT: {
        float rdz_des = V_ABORT;   // r.z crece = el dron sube
        float posD;
        if (_abort_posD_valid && ahrs.get_relative_position_D_origin(posD) &&
            (_abort_posD - posD) > ABORT_CLIMB_MAX) {
            rdz_des = 0.0f;        // tope de ascenso alcanzado: mantener altura
        }
        u_d = verticalOnlyCommand(rdz_des);
        break;
    }
    case SupState::BLIND: {
        u_d = verticalOnlyCommand(-V_BLIND);
        // Detección de contacto: |vD| bajo de forma sostenida mientras se pide descenso
        float vD = 0.0f;
        ahrs.get_vert_pos_rate_D(vD);
        if ((now_ms - _blind_start_ms) > 300 && fabsf(vD) < TD_VEL) {
            if (_td_low_start_ms == 0) {
                _td_low_start_ms = now_ms;
            } else if ((now_ms - _td_low_start_ms) > TD_TIME_MS) {
                _td_thr0 = u_d.length();
                setSupState(SupState::TOUCHDOWN, pos);
            }
        } else {
            _td_low_start_ms = 0;
        }
        break;
    }
    case SupState::TOUCHDOWN: {
        const float frac = 1.0f - ((now_ms - _td_ramp_start_ms) * 1.0e-3f) / TD_RAMP_S;
        const float thr = _td_thr0 * MAX(frac, 0.0f);
        u_d = Vector3f(0.0f, 0.0f, -thr);
        if (frac <= 0.0f && motors->armed()) {
            copter.arming.disarm(AP_Arming::Method::LANDED);
        }
        break;
    }
    }

    // Log the actual force command (vis = estado del supervisor)
    AP::logger().Write("UCMD", "TimeUS,ux,uy,uz,vis", "Qfffi",
        AP_HAL::micros64(),
        (double)u_d.x, (double)u_d.y, (double)u_d.z,
        (int32_t)_sup);

    // Handle motor spool states
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_z_controller(0.0f);
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        calculateVirtualMap(u_d, u_d_dot, psi_d, psi_d_dot, refQuaternion, refAngularVelocity);
        attitude_control->input_quaternion(refQuaternion, refAngularVelocity);
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        break;
    }

    // Throttle output
    attitude_control->set_throttle_out(refThrottle, true, g.throttle_filt);

    // Log estimator state
    AP::logger().Write("MLLC", "TimeUS,px,py,pz,vx,vy,vz,ax,ay,az,h", "Qffffffffff",
        AP_HAL::micros64(),
        (double)pos.x, (double)pos.y, (double)pos.z,
        (double)vel.x, (double)vel.y, (double)vel.z,
        (double)acc.x, (double)acc.y, (double)acc.z,
        (double)(vision_ok ? 1.0f : 0.0f));
}

// ---------------------------------------------------------------
// Relative velocity field computation
// ---------------------------------------------------------------
ModeLLC::RelativeFieldOutput ModeLLC::computeRelativeVelocityField(
    const Vector3f& r, const RelativeFieldParams& p) const
{
    RelativeFieldOutput out{};

    // 1) Height: r.z > 0 when drone is above marker (NED convention)
    //    Clamp to zero to avoid sign flip near contact
    const float h = MAX(r.z, 0.0f);

    // 2) Horizontal distance to vertical axis of the marker
    const float x = r.x;
    const float y = r.y;
    const float rho = sqrtf(x * x + y * y);
    out.rho = rho;

    // 3) Allowed radius dependent on height (adaptive tube)
    //    High altitude: rho_ref ≈ rho_max (relaxed centering)
    //    Near ground:   rho_ref → 0 (strict centering)
    out.rho_ref = p.rho_max * tanhf(p.k_slope * h);

    //    Ronda A: el tubo nunca excede una fracción beta del radio visible,
    //    rho_vis = h*tan(FOV/2) - s/2 (marcador completo dentro de la imagen, dron nivelado)
    if (p.fov_aware) {
        const float rho_vis = MAX(h * p.tan_fov_half - p.marker_half, 0.0f);
        out.rho_ref = MIN(out.rho_ref, p.beta * rho_vis);
    }

    // 4) Radial error: only active outside the tube
    float e_rho = rho - out.rho_ref;
    e_rho = MAX(e_rho, 0.0f);
    out.e_rho = e_rho;

    // 5) Radial correction intensity (smooth 0→1)
    out.mu_r = tanhf(p.k_r * e_rho);

    // 6) Desired horizontal relative velocity
    //    Points opposite to r_xy to reduce horizontal distance
    out.rdot_des.x = 0.0f;
    out.rdot_des.y = 0.0f;

    if (rho > p.rho_eps) {
        const float inv_rho = 1.0f / rho;
        const float speed_radial = p.v_xy_max * out.mu_r;

        out.rdot_des.x = -speed_radial * x * inv_rho;
        out.rdot_des.y = -speed_radial * y * inv_rho;
    }

    // 7) Descent enable factor: reduced when off-center
    out.mu_z = 1.0f / (1.0f + p.k_center * e_rho * e_rho);

    // 8) Desired vertical relative velocity
    //    Negative = descend (reduce h)
    //    Smoothly goes to zero near the platform
    if (h > p.h_eps) {
        out.rdot_des.z = -p.v_z_max * tanhf(p.k_z * h) * out.mu_z;
    } else {
        out.rdot_des.z = 0.0f;
    }

    return out;
}

// ---------------------------------------------------------------
// MAVLink message handling
// ---------------------------------------------------------------
bool ModeLLC::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_FORCE_VECTOR_TARGET: {
            // Verificar que el mensaje es para este sistema
            mavlink_force_vector_target_t packet;
            mavlink_msg_force_vector_target_decode(&msg, &packet);
            
            // if (packet.target_system != g.sysid_this_mav) {
            //     break;
            // }
            // custom log to check the mavlink com performance
            // I've added the sender time and msg sequence number to the message
            // Add the local time to the log to check the latency
            AP::logger().Write("ZCOM", "TimeUS,seq,fx,fy,fz,fxd,fyd,fzd", "QQffffff", 
                AP_HAL::micros64(),// Local timestamp
                packet.msg_seq,
                (float)packet.force_x, 
                (float)packet.force_y, 
                (float)packet.force_z, 
                (float)packet.force_derivative_x, 
                (float)packet.force_derivative_y, 
                (float)packet.force_derivative_z);
            // Actualizar el vector de fuerza
            _force_target_recvd.x = packet.force_x;
            _force_target_recvd.y = packet.force_y;
            _force_target_recvd.z = packet.force_z;
            _force_target_derivative_recvd.x = packet.force_derivative_x;
            _force_target_derivative_recvd.y = packet.force_derivative_y;
            _force_target_derivative_recvd.z = packet.force_derivative_z;

            if (CTRL_TRANSFORMATION) {
                // Convert to inertial frame
                Quaternion bodyQuaternion;
                ahrs.get_quat_body_to_ned(bodyQuaternion);
                bodyQuaternion.normalize();
                // For force vector
                _force_target = bodyQuaternion * _force_target_recvd; // Change to inertial frame
                _force_target.z += -gOffset; // Adjust for gravity offset, this is a constant value for the force vector in inertial frame
                // For force derivative vector
                _force_target_derivative = bodyQuaternion * _force_target_derivative_recvd;
            } else {
                // Do not convert to inertial frame, expect the force vector to be in inertial frame
                _force_target = _force_target_recvd;
                _force_target.z += -gOffset; // Adjust for gravity offset, this is a constant value for the force vector in inertial frame
                _force_target_derivative = _force_target_derivative_recvd;
            }
            
            _have_new_force_target = true;
            _last_force_target_ms = AP_HAL::millis();
            return true;
        }
    }
    return false;
}

/**
 * Calculates orientation control parameters for a quadrotor controller
 * 
 * @param u_d Desired thrust vector
 * @param u_dot_d Derivative of the desired thrust vector
 * @param psi_d Desired yaw angle in radians
 * @param psi_dot_d Derivative of the desired yaw angle in radians
 * @param refQuat Output reference quaternion
 * @param refOmega Output reference angular velocity
 * @return Thrust magnitude (norm of u_d)
 */
void ModeLLC::calculateVirtualMap(const Vector3f& u_d, const Vector3f& u_dot_d, 
    float psi_d, float psi_dot_d,
    Quaternion& refQuat, Vector3f& refOmega) {
    // Constants to avoid numerical issues
    constexpr float MIN_NORM = 4.6416e-04f;
    constexpr float MIN_DENOMINATOR = 1e-6f;
    
    // Calculate thrust magnitude (norm of u_d)
    float thrust = u_d.length();
    
    // Check for zero thrust
    if (thrust < MIN_NORM) {
        // For very small thrusts, maintain current orientation but zero angular velocity
        thrust = MIN_NORM;
        refOmega.zero();
        refThrottle = thrust;
        return;
    }
    
    // Calculate unit vector of thrust direction
    Vector3f uu = u_d / thrust;
    
    // Calculate the dot product for derivative calculation
    float u_dot = u_d.dot(u_dot_d);
    float thrust_cubed = thrust * thrust * thrust;
    
    // Calculate the derivative of the unit vector
    // uup = u_dot_d/norm_u - u_d*(u_d·u_dot_d)/norm_u^3
    Vector3f uup = u_dot_d / thrust - u_d * (u_dot / thrust_cubed);
    
    // Precalculate values for quaternion and angular velocity with safety checks
    float one_minus_uuz = 1.0f - uu.z;
    one_minus_uuz = one_minus_uuz < MIN_DENOMINATOR ? MIN_DENOMINATOR : one_minus_uuz;
    
    float u_3 = sqrtf(2.0f * one_minus_uuz);
    float inv_u_3 = 1.0f / u_3;
    float inv_one_minus_uuz = 1.0f / one_minus_uuz;
    
    // Precalculate trigonometric values
    float half_psi = psi_d * 0.5f;
    float cos_half_psi = cosf(half_psi);
    float sin_half_psi = sinf(half_psi);
    float cos_psi = cosf(psi_d);
    float sin_psi = sinf(psi_d);
    
    // Calculate the desired quaternion
    // Note: Adapt these component names to match your Quaternion class
    // Here I'm using the convention: w,x,y,z components
    refQuat.q1 = u_3 * cos_half_psi * 0.5f;  // or q1 depending on your implementation
    refQuat.q2 = (-uu.x * sin_half_psi + uu.y * cos_half_psi) * inv_u_3;  // or q2
    refQuat.q3 = (-uu.x * cos_half_psi - uu.y * sin_half_psi) * inv_u_3;  // or q3
    refQuat.q4 = sin_half_psi * u_3 * 0.5f;  // or q4
    
    // Ensure quaternion is normalized
    refQuat.normalize();
    
    // Calculate the desired angular velocity with minimal terms
    float term1 = uu.x * sin_psi - uu.y * cos_psi;
    float term2 = uu.x * cos_psi + uu.y * sin_psi;
    float term3 = uu.x * uup.y - uu.y * uup.x;
    
    refOmega.x = -uup.x * sin_psi + uup.y * cos_psi + uup.z * term1 * inv_one_minus_uuz;
    refOmega.y = -uup.x * cos_psi - uup.y * sin_psi + uup.z * term2 * inv_one_minus_uuz;
    refOmega.z = psi_dot_d - term3 * inv_one_minus_uuz;

    refThrottle = thrust;
}

