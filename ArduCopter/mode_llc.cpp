#include "Copter.h"
#define IS_SIM true
#define CTRL_TRANSFORMATION 0

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
    
#if IS_SIM
    float x_ref = 0.0f, y_ref = 0.0f, z_ref = 3.0f;
    float x_dot_ref = 0.0f, y_dot_ref = 0.0f, z_dot_ref = 0.0f;
    float x_ddot_ref = 0.0f, y_ddot_ref = 0.0f, z_ddot_ref = 0.0f;
    float x_dddot_ref = 0.0f, y_dddot_ref = 0.0f, z_dddot_ref = 0.0f;

    Vector3f x_d(x_ref, y_ref, -z_ref);
    Vector3f x_d_dot(x_dot_ref, y_dot_ref, -z_dot_ref);
    Vector3f x_d_ddot(x_ddot_ref, y_ddot_ref, -z_ddot_ref);
    Vector3f x_d_dddot(x_dddot_ref, y_dddot_ref, -z_dddot_ref);
    // float psi_d = 3.1416f/4.0f;

    // Parameters
    float mass = 0.03351f;
    float gravity = 9.81f;
    
    Vector3f e_z(0.0f, 0.0f, 1.0f);

    // Drone data initialization
    Vector3f x(0.0f, 0.0f, 0.0f);
    Vector3f x_dot(0.0f, 0.0f, 0.0f);
    Vector3f x_ddot(0.0f, 0.0f, 0.0f);

    // Control gains
    Matrix3f kp1(-0.5f, 0.0f, 0.0f,
        0.0f, -0.5f, 0.0f,
        0.0f, 0.0f, -0.5f);

    Matrix3f kd1(-0.3f, 0.0f, 0.0f,
        0.0f, -0.3, 0.0f,
        0.0f, 0.0f, -0.3f);

    Vector3f u_d(0.0f, 0.0f, 0.0f);
    Vector3f u_d_dot(0.0f, 0.0f, 0.0f);

    // Get position, velocity and acceleration data
    if(ahrs.get_relative_position_NED_home(_position) && ahrs.get_velocity_NED(_velocity)) 
    {   
        _acceleration = ahrs.get_accel_ef(); // Acceleration in NED inertial frame

        if(_return_home)
        {
            x = _position;
            x_dot = _velocity;
            x_ddot = _acceleration + e_z*gravity;
    
            // Errors
            Vector3f xe = x - x_d;
            Vector3f xe_dot = x_dot - x_d_dot;
            Vector3f xe_ddot = x_ddot - x_d_ddot;
            
            // Control law
            u_d = kp1 * xe + kd1 * xe_dot - e_z * mass * gravity + x_d_ddot * mass;
            u_d_dot = kp1 * xe_dot + kd1 * xe_ddot + x_d_dddot * mass;
            gcs().send_text(MAV_SEVERITY_INFO, "Reference calculated");
        }
    }
#endif

 
    // Handle motor spool states
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();  
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_z_controller(0.0f);
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();

        pos_control->relax_z_controller(0.0f);
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        if (_have_new_force_target) { 
            calculateVirtualMap(_force_target, _force_target_derivative, psi_d, psi_d_dot, refQuaternion, refAngularVelocity);
            _return_home = false;
        }
#if IS_SIM
        else
        {
            calculateVirtualMap(u_d, u_d_dot, psi_d, psi_d_dot, refQuaternion, refAngularVelocity);
        }
#else
        else
        {
            // Set desired neutral attitude (null quaternion)
            refQuaternion = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
            refAngularVelocity = Vector3f(0.0f, 0.0f, 0.0f);
        }
#endif
        // Flying - run quaternion controller
        attitude_control->input_quaternion(refQuaternion, refAngularVelocity);
        // pos_control->set_alt_target_with_slew(200.0f);
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Do nothing
        break;
    }
    // Set constant throttle for hover
    attitude_control->set_throttle_out(refThrottle, true, g.throttle_filt);
    // Set throttle directly to the motors
    // motors->set_throttle(0.35f);
    // pos_control->update_z_controller();

    AP::logger().Write("YFTG", "TimeUS,rcvd,fx,fy,fz,fxd,fyd,fzd", "Qbffffff", 
        AP_HAL::micros64(), 
        (bool)_have_new_force_target,
        (float)_force_target.x, 
        (float)_force_target.y, 
        (float)_force_target.z, 
        (float)_force_target_derivative.x, 
        (float)_force_target_derivative.y, 
        (float)_force_target_derivative.z);

    AP::logger().Write("YFRC", "TimeUS,rcvd,fx,fy,fz,fxd,fyd,fzd", "Qbffffff", 
        AP_HAL::micros64(), 
        (bool)_have_new_force_target,
        (float)_force_target_recvd.x, 
        (float)_force_target_recvd.y, 
        (float)_force_target_recvd.z, 
        (float)_force_target_derivative_recvd.x, 
        (float)_force_target_derivative_recvd.y, 
        (float)_force_target_derivative_recvd.z);

    Quaternion bodyQuaternion, qError;
    ahrs.get_quat_body_to_ned(bodyQuaternion);
    bodyQuaternion.normalize();
    qError = refQuaternion.inverse() * bodyQuaternion;

    AP::logger().Write("YQBD", "TimeUS,q1,q2,q3,q4", "Qffff", 
        AP_HAL::micros64(), 
        bodyQuaternion.q1, 
        bodyQuaternion.q2, 
        bodyQuaternion.q3, 
        bodyQuaternion.q4);

    AP::logger().Write("YQRF", "TimeUS,q1,q2,q3,q4", "Qffff", 
        AP_HAL::micros64(), 
        refQuaternion.q1, 
        refQuaternion.q2, 
        refQuaternion.q3, 
        refQuaternion.q4);

    AP::logger().Write("YQER", "TimeUS,q1,q2,q3,q4", "Qffff",
        AP_HAL::micros64(), 
        qError.q1, 
        qError.q2, 
        qError.q3, 
        qError.q4);

    Vector3f angularVelocity = ahrs.get_gyro_latest();
    Vector3f angularError = angularVelocity - refAngularVelocity;
    AP::logger().Write("YWBD", "TimeUS,wx,wy,wz", "Qfff", 
        AP_HAL::micros64(), 
        angularVelocity.x, 
        angularVelocity.y, 
        angularVelocity.z);

    AP::logger().Write("YWRF", "TimeUS,wx,wy,wz", "Qfff", 
        AP_HAL::micros64(), 
        refAngularVelocity.x, 
        refAngularVelocity.y, 
        refAngularVelocity.z);

    AP::logger().Write("YWER", "TimeUS,wx,wy,wz", "Qfff", 
        AP_HAL::micros64(), 
        angularError.x, 
        angularError.y, 
        angularError.z);
    
}

bool ModeLLC::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_FORCE_VECTOR_TARGET: {
            // Verificar que el mensaje es para este sistema
            mavlink_force_vector_target_t packet;
            mavlink_msg_force_vector_target_decode(&msg, &packet);
            
            if (packet.target_system != g.sysid_this_mav) {
                break;
            }
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

