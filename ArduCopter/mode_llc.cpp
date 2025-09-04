#include "Copter.h"
#define IS_SIM true
#define TUNNING_ATTITUDE false

#if IS_SIM
#include <iostream>
#include <fstream>
#include <ctime>
#endif


const AP_Param::GroupInfo ModeLLC::var_info[] = {
    // @Param: HVR_THR
    // @DisplayName: Throttle for hover
    // @Description: Throttle for hover
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("HVR_THR", 1, ModeLLC, _hover_thr, 0.3287331f),
    
    // @Param: TUNE_HVR_THR
    // DisplayName: Activate/deactivate tunning the throttle compensation
    // Description: Activate/deactivate tunning the throttle compensation
    // Range: 0 1
    // User: Standard
    AP_GROUPINFO("TUNE_HVR_THR", 2, ModeLLC, _tune_hover_thr, 0),

    // @ Param: X_REF
    // @DisplayName: X Reference
    // @Description: X Reference
    // @Range: -10.0 10.0
    // @User: Standard
    AP_GROUPINFO("X_REF", 3, ModeLLC, _x_ref, 0.0f),

    // @Param: Y_REF
    // @DisplayName: Y Reference
    // @Description: Y Reference
    // @Range: -10.0 10.0
    // @User: Standard
    AP_GROUPINFO("Y_REF", 4, ModeLLC, _y_ref, 0.0f),

    // @Param: Z_REF
    // @DisplayName: Z Reference
    // @Description: Z Reference
    // @Range: -10.0 10.0
    // @User: Standard
    AP_GROUPINFO("Z_REF", 5, ModeLLC, _z_ref, 3.0f),

    // @Param: X_KP
    // @DisplayName: X Proportional Gain
    // @Description: X Proportional Gain
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("X_KP", 6, ModeLLC, _xkp, 0.2f),

    // @Param: Y_KP
    // @DisplayName: Y Proportional Gain
    // @Description: Y Proportional Gain
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("Y_KP", 7, ModeLLC, _ykp, 0.2f),

    // @Param: Z_KP
    // @DisplayName: Z Proportional Gain
    // @Description: Z Proportional Gain
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("Z_KP", 8, ModeLLC, _zkp, 0.2f),

    // @Param: X_KD
    // @DisplayName: X Derivative Gain
    // @Description: X Derivative Gain
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("X_KD", 9, ModeLLC, _xkd, 0.1f),

    // @Param: Y_KD
    // @DisplayName: Y Derivative Gain
    // @Description: Y Derivative Gain
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("Y_KD", 10, ModeLLC, _ykd, 0.1f),

    // @Param: Z_KD
    // @DisplayName: Z Derivative Gain
    // @Description: Z Derivative Gain
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("Z_KD", 11, ModeLLC, _zkd, 0.125f),

    AP_GROUPEND
};


bool ModeLLC::init(bool ignore_checks)
{
    // // Initialize position controller for Z axis if not already active
    // if (!pos_control->is_active_z()) {
    //     pos_control->init_z_controller();
    // }

    // // Set vertical speed and acceleration limits
    // pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    // pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);


    _force_target = Vector3f(0.0f, 0.0f, 0.0f);
    _force_target_derivative = Vector3f(0.0f, 0.0f, 0.0f);
    _force_target_recvd = Vector3f(0.0f, 0.0f, 0.0f);
    _force_target_derivative_recvd = Vector3f(0.0f, 0.0f, 0.0f);
    refQuaternion = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    refAngularVelocity = Vector3f(0.0f, 0.0f, 0.0f);
    
    init_time = AP_HAL::millis() / 1E3;
    new_file = true;
    _return_home = true;
    _have_new_force_target = false;

    return true;
}

void ModeLLC::exit()
{
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // Set vertical speed and acceleration limits
    // pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    // pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    // pos_control->set_alt_target_with_slew(200.0f);
    // pos_control->update_z_controller();
}

void ModeLLC::run()
{
    float psi_d = 0.0f;
    float psi_d_dot = 0.0f;
    float t = AP_HAL::millis() / 1E3 - init_time;
    mass = (float) _hover_thr / (float) gravity;
    
#if TUNNING_ATTITUDE
// Log attitude data to file        
    float A = 0.17f, omega = 3.5f;
    float roll_d = A*cosf(omega*t);
    float roll_d_dot = -A*omega*sinf(omega*t);
    float pitch_d = A*sinf(omega*t);
    float pitch_d_dot = A*omega*cosf(omega*t);
    Quaternion q_roll(cosf(roll_d/2.0f), sinf(roll_d/2.0f), 0.0f, 0.0f);
    Quaternion q_pitch(cosf(pitch_d/2.0f), 0.0f, sinf(pitch_d/2.0f), 0.0f);
    Quaternion q_d = q_roll * q_pitch;
    Quaternion q_roll_dot(0.0f/2.0f, roll_d_dot*cosf(roll_d/2)/2.0f, roll_d_dot*sinf(roll_d/2)/2.0f, 0.0f/2.0f);
    Quaternion q_pitch_dot(0.0f/2.0f, 0.0f/2.0f, pitch_d_dot*cosf(pitch_d/2)/2.0f, pitch_d_dot*sinf(pitch_d/2)/2.0f);
    Quaternion q_aux1 = q_roll_dot * q_pitch;
    Quaternion q_aux2 = q_roll * q_pitch_dot;
    Quaternion q_d_dot(q_aux1.q1 + q_aux2.q1, q_aux1.q2 + q_aux2.q2, q_aux1.q3 + q_aux2.q3, q_aux1.q4 + q_aux2.q4);
    Quaternion q_aux3 = q_d.inverse() * q_d_dot;
    Quaternion omega_d_quat(2*q_aux3.q1, 2*q_aux3.q2, 2*q_aux3.q3, 2*q_aux3.q4);
    std::cout << "time: " << t << std::endl;

    q_d.normalize();
    // omega_d_quat.normalize();
    Vector3f omega_d(omega_d_quat.q2, omega_d_quat.q3, omega_d_quat.q4);
#endif

#if IS_SIM

    float x_ref = (float)_x_ref, y_ref = (float)_y_ref, z_ref = (float)_z_ref;
    float x_dot_ref = 0.0f, y_dot_ref = 0.0f, z_dot_ref = 0.0f;
    float x_ddot_ref = 0.0f, y_ddot_ref = 0.0f, z_ddot_ref = 0.0f;
    float x_dddot_ref = 0.0f, y_dddot_ref = 0.0f, z_dddot_ref = 0.0f;

    Vector3f x_d(x_ref, y_ref, -z_ref);
    Vector3f x_d_dot(x_dot_ref, y_dot_ref, -z_dot_ref);
    Vector3f x_d_ddot(x_ddot_ref, y_ddot_ref, -z_ddot_ref);
    Vector3f x_d_dddot(x_dddot_ref, y_dddot_ref, -z_dddot_ref);

    // Parameters
    // Drone data initialization
    Vector3f x(0.0f, 0.0f, 0.0f);
    Vector3f x_dot(0.0f, 0.0f, 0.0f);
    Vector3f x_ddot(0.0f, 0.0f, 0.0f);

    Matrix3f kp1(-(float)_xkp, 0.0f, 0.0f,
                0.0f, -(float)_ykp, 0.0f,
                0.0f, 0.0f, -(float)_zkp);

    Matrix3f kd1(-(float)_xkd, 0.0f, 0.0f,
                0.0f, -(float)_ykd, 0.0f,
                0.0f, 0.0f, -(float)_zkd);

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
            x_ddot = _acceleration + _ez*gravity;
    
            // Errors
            Vector3f xe = x - x_d;
            Vector3f xe_dot = x_dot - x_d_dot;
            Vector3f xe_ddot = x_ddot - x_d_ddot;
            
            // Control law
            u_d = kp1 * xe +kd1 * xe_dot - _ez * mass * gravity + x_d_ddot * mass;
            u_d_dot = kp1 * xe_dot + kd1 * xe_ddot + x_d_dddot * mass;
            _ud = u_d;
        }
    }
#endif

    // std::cout << "hover: " << _hover_thr << std::endl;

 
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
#if TUNNING_ATTITUDE 
        refQuaternion = q_d;
        refAngularVelocity = omega_d;
        // refThrottle = (float) _hover_thr;
#else
#endif

        if ((bool) _tune_hover_thr)
        {
            refThrottle = (float) _hover_thr;
            refQuaternion = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
            refAngularVelocity = Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        // Flying - run quaternion controller
        attitude_control->input_quaternion(refQuaternion, refAngularVelocity);
        // pos_control->set_alt_target_with_slew(300.0f);
        // pos_control->update_z_controller();

        motors->set_throttle(refThrottle);
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Do nothing
        break;
    }
    // Set constant throttle for hover
    // attitude_control->set_throttle_out(T, true, g.throttle_filt);
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

    AP::logger().Write("YTHR", "TimeUS,thr", "Qf",
        AP_HAL::micros64(), 
        refThrottle);

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

    AP::logger().Write("YQTG", "TimeUS,q1,q2,q3,q4", "Qffff", 
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
    AP::logger().Write("YWBD", "TimeUS,wxb,wyb,wzb", "Qfff", 
        AP_HAL::micros64(), 
        angularVelocity.x, 
        angularVelocity.y, 
        angularVelocity.z);

    AP::logger().Write("YWTG", "TimeUS,wxt,wyt,wzt", "Qfff", 
        AP_HAL::micros64(), 
        refAngularVelocity.x, 
        refAngularVelocity.y, 
        refAngularVelocity.z);

    AP::logger().Write("YWER", "TimeUS,wx,wy,wz", "Qfff", 
        AP_HAL::micros64(), 
        angularError.x, 
        angularError.y, 
        angularError.z);

    if(this->new_file && _have_new_force_target) {
        // Time stamp
        this->new_file = false;  
        auto td = std::time(nullptr);
        auto tm = *std::localtime(&td);
        char timestamp[20];
        std::strftime(timestamp, sizeof(timestamp), "%m-%d_%H-%M-%S", &tm);
        time_offset = t;

        this->att_filename = "/home/olara/ap_drone_ws/src/target_tracking/plots/attitude/data/attitude_data_" + std::string(timestamp) + ".txt";
    }

    _zb = bodyQuaternion * _ez;
    _zb *= -1.0f;
    _zb.normalize();
    _ud_norm = _ud.normalized();

    if (_have_new_force_target) {
        // Open file to save q_d, q_body, q_error along with time
        std::ofstream attitude_data(this->att_filename, std::ios_base::app);


        if (!attitude_data.is_open()) {
            std::cerr << "Error opening file" << std::endl;
        } else {
            // Write time, q_d, q_body, q_error to file
            attitude_data << t-time_offset << " "; // Time in seconds
            attitude_data << refQuaternion.q1 << " " << refQuaternion.q2 << " " << refQuaternion.q3 << " " << refQuaternion.q4 << " "; // q_d quaternion
            attitude_data << bodyQuaternion.q1 << " " << bodyQuaternion.q2 << " " << bodyQuaternion.q3 << " " << bodyQuaternion.q4 << " "; // q_body quaternion
            attitude_data << qError.q1 << " " << qError.q2 << " " << qError.q3 << " " << qError.q4 << " "; // q_error quaternion
            attitude_data << refAngularVelocity.x << " " << refAngularVelocity.y << " " << refAngularVelocity.z << " "; // omega_d vector
            attitude_data << angularVelocity.x << " " << angularVelocity.y << " " << angularVelocity.z << " "; // omega vector
            attitude_data << _force_target.x << " " << _force_target.y << " " << _force_target.z << " " << _force_target[3] << " "; // Control action
            attitude_data << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " "; // Motor angular velocities
            attitude_data << 0.0 << " " << 0.0 << " " << 0.0 << " "; // Control force
            attitude_data << 0.0 << " " << 0.0 << " " << 0.0 << " "; // Control force derivative
            attitude_data << _ud_norm.x << " " << _ud_norm.y << " " << _ud_norm.z << " "; // Control force normalized
            attitude_data << _zb.x << " " << _zb.y << " " << _zb.z << std::endl; // Body frame z-axis in NED inertial frame
        }

        attitude_data.close();
    }
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
            
            // Actualizar el vector de fuerza
            _force_target_recvd.x = packet.force_x;
            _force_target_recvd.y = packet.force_y;
            _force_target_recvd.z = packet.force_z;
            _force_target_derivative_recvd.x = packet.force_derivative_x;
            _force_target_derivative_recvd.y = packet.force_derivative_y;
            _force_target_derivative_recvd.z = packet.force_derivative_z;

            // Quaternion q_body;
            // ahrs.get_quat_body_to_ned(q_body);
            // q_body.normalize();
            // _force_target_recvd = q_body.inverse() * _force_target_recvd;
            
            _force_target_recvd.z += -mass * gravity; // Adjust for gravity in inertial frame
            _force_target = _force_target_recvd; // Update force target vector
            _force_target_derivative = _force_target_derivative_recvd; // Update desired force derivative vector
        
            _ud = _force_target; // Update desired force vector

            _have_new_force_target = true;
            _last_force_target_ms = AP_HAL::millis();

            return true;
        }
    }
    return false;
}

void ModeLLC::calculateVirtualMap(const Vector3f& u_d, const Vector3f& u_dot_d, 
    float psi_d, float psi_dot_d,
    Quaternion& refQuat, Vector3f& refOmega) {

    Vector3f u_d_norm = u_d.normalized();
    Vector3f u_d_dot_norm = u_dot_d / u_d.length() - u_d * (u_d * u_dot_d) / powf(u_d.length(), 3.0f);  

    Quaternion q_dxy(1.0f/2.0f * sqrtf(-2*u_d_norm.z + 2),
    u_d_norm.y / sqrtf(-2*u_d_norm.z + 2),
    -u_d_norm.x / sqrtf(-2*u_d_norm.z + 2),
    0.0f);

    Quaternion q_dz(cosf(psi_d/2.0f), 
    0.0f, 
    0.0f, 
    sinf(psi_d/2.0f));
    
    refQuat = q_dxy * q_dz;
    refQuat.normalize();

    refOmega = {-sinf(psi_d)*u_d_dot_norm.x + cosf(psi_d)*u_d_dot_norm.y + u_d_dot_norm.z*(sinf(psi_d)*u_d_norm.x - cosf(psi_d)*u_d_norm.y)/(u_d_norm.z - 1.0f),
        -cosf(psi_d)*u_d_dot_norm.x - sinf(psi_d)*u_d_dot_norm.y + u_d_dot_norm.z*(cosf(psi_d)*u_d_norm.x + sinf(psi_d)*u_d_norm.y)/(u_d_norm.z - 1.0f),
        psi_dot_d + (u_d_norm.x*u_d_dot_norm.y - u_d_norm.y*u_d_dot_norm.x)/(u_d_norm.z - 1.0f)};

    refThrottle = u_d.length();
}
