#include "Copter.h"

// class ModeQuaternionTest : public Mode {
// public:
//     ModeQuaternionTest(void);
//     bool init(bool ignore_checks) override;
//     void run() override;

// private:
//     const float HOVER_THROTTLE = 2.5f; // Adjust this based on your vehicle
// };

bool ModeLLC::init(bool ignore_checks)
{
    // Initialize position controller for Z axis if not already active
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // Set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
}

void ModeLLC::run()
{
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
    case AP_Motors::SpoolState::SHUT_DOWN: {
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_z_controller(0.0f);
        break;
    }

    case AP_Motors::SpoolState::GROUND_IDLE: {
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);
        break;
    }

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED: {
        // Get current position and velocity from inertial nav
        Vector3f current_pos = inertial_nav.get_position_neu_cm().tofloat() * 0.01f;  // Convert to meters
        Vector3f current_vel = inertial_nav.get_velocity_neu_cms().tofloat() * 0.01f;
        // Vector3f current_accel = inertial_nav.get_accel_neu().tofloat() * 0.01f;
        
        // Set test target position (in meters)
        Vector3f pos_target(TEST_TARGET_X, TEST_TARGET_Y, TEST_TARGET_Z);
        
        // Calculate velocity control
        Vector3f u, u_dot;
        calculate_velocity_control(current_pos, current_vel, pos_target, u, u_dot);
        
        // Calculate virtual control
        Vector3f target_ang_vel;
        calculate_virtual_control(u, u_dot, 0.0f, target_attitude, target_ang_vel);
        
        // Run quaternion controller for attitude
        attitude_control->input_quaternion(target_attitude, target_ang_vel);
        
        // Set position target for Z axis
        // pos_control->set_alt_target_with_slew(TEST_TARGET_Z * -100.0f); // Convert to cm, negative for NED
        break;
    }

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN: {
        // Do nothing
        break;
    }
    }

    // Update the vertical position controller
    pos_control->update_z_controller();

    // Add logging for debugging
    // log_data();
}

// Optional: Add logging to help with debugging
void ModeLLC::log_data()
{
    // Get current attitude
    Quaternion current_attitude;
    ahrs.get_quat_body_to_ned(current_attitude);
    
    // Calculate attitude error
    Quaternion attitude_error = current_attitude.inverse() * target_attitude;
    Vector3f error_angle;
    attitude_error.to_axis_angle(error_angle);

    // Get current angular rates
    Vector3f gyro = ahrs.get_gyro_latest();

    // Get current altitude and target
    float current_alt = inertial_nav.get_position_z_up_cm();
    float target_alt = pos_control->get_pos_target_z_cm();

    // Use AP::logger() instead of copter.logger
    AP::logger().Write("QUAT", "TimeUS,ErrX,ErrY,ErrZ,GyrX,GyrY,GyrZ,Alt,TAlt",
                       "srrrrrrnn",        // s:microseconds, r:radians, n:meters
                       "F--------",        // F:flight, -:no flags
                       "Qffffffff",        // Q:uint64_t, f:float
                       AP_HAL::micros64(),
                       (float)error_angle.x,
                       (float)error_angle.y,
                       (float)error_angle.z,
                       (float)gyro.x,
                       (float)gyro.y,
                       (float)gyro.z,
                       (float)(current_alt * 0.01f),  
                       (float)(target_alt * 0.01f));
}

void ModeLLC::calculate_virtual_control(const Vector3f& ud, const Vector3f& ud_dot, float psi_d,
                                      Quaternion& quat_target, Vector3f& ang_vel_target) 
{
    // Safety check - don't process if control vector is too small
    if (ud.length() < 0.1f) {
        // Set default attitude (level) if control vector is too small
        quat_target = Quaternion();  // Default identity quaternion
        ang_vel_target.zero();
        
        // AP::logger().Write("VCTE", "TimeUS,ErrCode",
        //                   "s-",        // s:microseconds, -:no units
        //                   "F-",        // F:flight, -:no flags
        //                   "QB",        // Q:uint64_t, B:int8_t
        //                   AP_HAL::micros64(),
        //                   (int8_t)1);  // Error code 1: Control vector too small
        return;
    }
    
    // Normalize ud vector
    Vector3f udg = ud.normalized();
    
    // Safety check for square root arguments
    float sqrt_term = -2.0f * udg.z + 2.0f;
    if (sqrt_term <= 0.0f) {
        // Set default attitude if math would fail
        quat_target = Quaternion();
        ang_vel_target.zero();
        
        // AP::logger().Write("VCTE", "TimeUS,ErrCode",
        //                   "s-",        // s:microseconds, -:no units
        //                   "F-",        // F:flight, -:no flags
        //                   "QB",        // Q:uint64_t, B:int8_t
        //                   AP_HAL::micros64(),
        //                   (int8_t)2);  // Error code 2: Invalid sqrt argument
        return;
    }
    
    // Calculate quaternion components
    float qd_0 = 0.5f * sqrtf(sqrt_term) * cosf(psi_d * 0.5f);
    float qd_1 = (-udg.x * sinf(psi_d * 0.5f) + udg.y * cosf(psi_d * 0.5f)) / sqrtf(sqrt_term);
    float qd_2 = (-udg.x * cosf(psi_d * 0.5f) - udg.y * sinf(psi_d * 0.5f)) / sqrtf(sqrt_term);
    float qd_3 = 0.5f * sqrtf(sqrt_term) * sinf(psi_d * 0.5f);
    
    quat_target.q1 = qd_0;
    quat_target.q2 = qd_1;
    quat_target.q3 = qd_2;
    quat_target.q4 = qd_3;
    
    // Normalize quaternion
    quat_target.normalize();
    
    // For now, set angular velocity to zero for stability
    ang_vel_target.zero();
    
    // Log virtual control info
    // AP::logger().Write("VCTL", "TimeUS,UdgX,UdgY,UdgZ,Q1,Q2,Q3,Q4",
    //                   "s-------",     // s:microseconds, -:no units for the rest
    //                   "F-------",     // F:flight, -:no flags
    //                   "Qfffffff",     // Q:uint64_t, f:float
    //                   AP_HAL::micros64(),
    //                   (float)udg.x,
    //                   (float)udg.y,
    //                   (float)udg.z,
    //                   (float)quat_target.q1,
    //                   (float)quat_target.q2,
    //                   (float)quat_target.q3,
    //                   (float)quat_target.q4);
}

void ModeLLC::calculate_velocity_control(const Vector3f& pos, const Vector3f& vel, 
                                       const Vector3f& pos_target,
                                       Vector3f& u, Vector3f& u_dot) 
{
    // Calculate state error vector [pos_error; vel_error]
    Vector3f pos_error = pos_target - pos;
    Vector3f vel_error = -vel;  // Since desired velocity is [0,0,0]
    
    // PD gains
    // Position gains (stronger in Z)
    const float Kp_xy = 0.0f;
    const float Kp_z = 0.0f;
    Vector3f Kp(Kp_xy, Kp_xy, Kp_z);
    
    // Velocity gains
    const float Kd = 0.0f;
    Vector3f Kd_vec(Kd, Kd, Kd);
    
    // Calculate PD control
    Vector3f pd_output;
    pd_output.x = -(Kp.x * pos_error.x + Kd_vec.x * vel_error.x);
    pd_output.y = -(Kp.y * pos_error.y + Kd_vec.y * vel_error.y);
    pd_output.z = -(Kp.z * pos_error.z + Kd_vec.z * vel_error.z);
    
    // Add gravity compensation
    const float mass = 1.5f; // Estimate of vehicle mass in kg
    const float gravity = GRAVITY_MSS;
    Vector3f gravity_comp(0.0f, 0.0f, mass * gravity);
    
    // Final control input
    u = pd_output - gravity_comp;
    u_dot.zero();  // No acceleration feedforward for now
    
    // Log control info
    AP::logger().Write("VLCL",
                    "TimeUS,PErrX,PErrY,PErrZ,VErrX,VErrY,VErrZ,UX,UY,UZ",
                        "Qfffffffff", // Units\n 
                        "F---------", // Multipliers\n 
                        AP_HAL::micros64(),
                        (float)pos_error.x,
                        (float)pos_error.y,
                        (float)pos_error.z,
                        (float)vel_error.x,
                        (float)vel_error.y,
                        (float)vel_error.z,
                        (float)u.x,
                        (float)u.y,
                        (float)u.z);


}