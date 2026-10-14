#include "Copter.h"
#define CTRL_TRANSFORMATION 0

// Set to 1 to enable vision-based velocity field control
// Set to 0 to use AHRS poshold (safe fallback)
#define USE_VISION_CONTROL 1

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
    gcs().send_text(MAV_SEVERITY_INFO, "Sys ID: %d", (int)g.sysid_this_mav);

    return true;
}

void ModeLLC::run()
{
    float psi_d = 0.0f;
    float psi_d_dot = 0.0f;

    Vector3f u_d(0.0f, 0.0f, 0.0f);
    Vector3f u_d_dot(0.0f, 0.0f, 0.0f);

    // --- Read vision estimator state (used by both control paths) ---
    const VisionStateEstimator& vest = copter.vision_estimator;
    Vector3f pos = vest.get_position();
    Vector3f vel = vest.get_velocity();
    Vector3f acc = vest.get_acceleration();
    const bool vision_ok = vest.is_healthy();

    // Flag to track which controller is active this cycle
    bool using_vision = false;

#if USE_VISION_CONTROL
    // --- Vision-based P controller on velocity field error ---
    if (vision_ok) {
        using_vision = true;

        // Compute velocity field
        RelativeFieldOutput field = computeRelativeVelocityField(pos, _field_params);

        // Velocity tracking error
        Vector3f ev = vel - field.rdot_des;

        // P controller gains (per-axis)
        constexpr float kp_xy = 0.03f;   // Horizontal gain
        constexpr float kp_z  = 0.095f;   // Vertical gain

        // Control law: u = kp * ev - gOffset * e_z
        u_d.x = kp_xy * ev.x;
        u_d.y = kp_xy * ev.y;
        u_d.z = kp_z  * ev.z - gOffset;

        // u_dot = 0 for initial P test
        u_d_dot.zero();

        // Log velocity field output
        AP::logger().Write("VFLD", "TimeUS,rdx,rdy,rdz,rho,rref,erho,mur,muz", "Qffffffff",
            AP_HAL::micros64(),
            (double)field.rdot_des.x, (double)field.rdot_des.y, (double)field.rdot_des.z,
            (double)field.rho,
            (double)field.rho_ref,
            (double)field.e_rho,
            (double)field.mu_r,
            (double)field.mu_z);

        // Debug log: r_dot, r_dot_des, e_v
        AP::logger().Write("VDBG", "TimeUS,vx,vy,vz,dx,dy,dz,ex,ey,ez", "Qfffffffff",
            AP_HAL::micros64(),
            (double)vel.x, (double)vel.y, (double)vel.z,
            (double)field.rdot_des.x, (double)field.rdot_des.y, (double)field.rdot_des.z,
            (double)ev.x, (double)ev.y, (double)ev.z);

        // Log the actual force command
        AP::logger().Write("UCMD", "TimeUS,ux,uy,uz,vis", "Qfffi",
            AP_HAL::micros64(),
            (double)u_d.x, (double)u_d.y, (double)u_d.z,
            (int32_t)1);
    }
#endif

    // --- Fallback: AHRS poshold controller ---
    if (!using_vision) {
        float x_ref = 0.0f, y_ref = 0.0f, z_ref = 4.0f;
        float mass = 0.03351f;
        float gravity = 9.81f;
        Vector3f e_z(0.0f, 0.0f, 1.0f);

        Vector3f x_d(x_ref, y_ref, -z_ref);
        Vector3f x_d_dot(0.0f, 0.0f, 0.0f);
        Vector3f x_d_ddot(0.0f, 0.0f, 0.0f);
        Vector3f x_d_dddot(0.0f, 0.0f, 0.0f);

        Matrix3f kp1(-0.5f, 0.0f, 0.0f,
            0.0f, -0.5f, 0.0f,
            0.0f, 0.0f, -0.5f);

        Matrix3f kd1(-0.3f, 0.0f, 0.0f,
            0.0f, -0.3f, 0.0f,
            0.0f, 0.0f, -0.3f);

        if(ahrs.get_relative_position_NED_home(_position) && ahrs.get_velocity_NED(_velocity))
        {
            _acceleration = ahrs.get_accel_ef();

            if(_return_home)
            {
                Vector3f x = _position;
                Vector3f x_dot = _velocity;
                Vector3f x_ddot = _acceleration + e_z * gravity;

                Vector3f xe = x - x_d;
                Vector3f xe_dot = x_dot - x_d_dot;
                Vector3f xe_ddot = x_ddot - x_d_ddot;

                u_d = kp1 * xe + kd1 * xe_dot - e_z * mass * gravity + x_d_ddot * mass;
                u_d_dot = kp1 * xe_dot + kd1 * xe_ddot + x_d_dddot * mass;
            }
        }

        // Log force command from poshold
        AP::logger().Write("UCMD", "TimeUS,ux,uy,uz,vis", "Qfffi",
            AP_HAL::micros64(),
            (double)u_d.x, (double)u_d.y, (double)u_d.z,
            (int32_t)0);

        // Still compute velocity field for observation if estimator is healthy
        if (vision_ok) {
            RelativeFieldOutput field = computeRelativeVelocityField(pos, _field_params);

            AP::logger().Write("VFLD", "TimeUS,rdx,rdy,rdz,rho,rref,erho,mur,muz", "Qffffffff",
                AP_HAL::micros64(),
                (double)field.rdot_des.x, (double)field.rdot_des.y, (double)field.rdot_des.z,
                (double)field.rho,
                (double)field.rho_ref,
                (double)field.e_rho,
                (double)field.mu_r,
                (double)field.mu_z);

            Vector3f ev = vel - field.rdot_des;
            AP::logger().Write("VDBG", "TimeUS,vx,vy,vz,dx,dy,dz,ex,ey,ez", "Qfffffffff",
                AP_HAL::micros64(),
                (double)vel.x, (double)vel.y, (double)vel.z,
                (double)field.rdot_des.x, (double)field.rdot_des.y, (double)field.rdot_des.z,
                (double)ev.x, (double)ev.y, (double)ev.z);
        }
    }

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

