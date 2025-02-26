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
        // Get current position and velocity from inertial nav
        Vector3f current_pos = inertial_nav.get_position_neu_cm().tofloat() * 0.01f;  // Convert to meters
        Vector3f current_vel = inertial_nav.get_velocity_neu_cms().tofloat() * 0.01f;
        Vector3f current_accel = inertial_nav.get_accel_neu().tofloat() * 0.01f;
        
        // Set test target position (in meters)
        Vector3f pos_target(TEST_TARGET_X, TEST_TARGET_Y, TEST_TARGET_Z);
        
        // Calculate velocity control
        Vector3f u, u_dot;
        calculate_velocity_control(current_pos, current_vel, current_accel, pos_target, u, u_dot);
        
        // Calculate virtual control
        calculate_virtual_control(u, u_dot, 0.0f, target_attitude, target_ang_vel);
        
        // Run quaternion controller for attitude
        attitude_control->input_quaternion(target_attitude, target_ang_vel);
        
        // Set position target for Z axis
        pos_control->set_alt_target_with_slew(TEST_TARGET_Z * -100.0f); // Convert to cm, negative for NED
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Do nothing
        break;
    }

    // Update the vertical position controller
    pos_control->update_z_controller();

    // Add logging for debugging
    log_data();
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
                       "sdddEEEmm",
                       "F000000--",
                       "Qffffffff",
                       AP_HAL::micros64(),
                       (double)error_angle.x,
                       (double)error_angle.y,
                       (double)error_angle.z,
                       (double)gyro.x,
                       (double)gyro.y,
                       (double)gyro.z,
                       (double)current_alt * 0.01f,  
                       (double)target_alt * 0.01f);
}

void ModeLLC::calculate_virtual_control(const Vector3f& ud, const Vector3f& ud_dot, float psi_d,
                                      Quaternion& quat_target, Vector3f& ang_vel_target) 
{
    // Normalize ud vector
    Vector3f udg = ud.normalized();
    
    // Calculate quaternion components
    float qd_0 = 0.5f * sqrtf(-2.0f * udg.z + 2.0f) * cosf(psi_d * 0.5f);
    float qd_1 = (-udg.x * sinf(psi_d * 0.5f) + udg.y * cosf(psi_d * 0.5f)) / sqrtf(-2.0f * udg.z + 2.0f);
    float qd_2 = (-udg.x * cosf(psi_d * 0.5f) - udg.y * sinf(psi_d * 0.5f)) / sqrtf(-2.0f * udg.z + 2.0f);
    float qd_3 = 0.5f * sqrtf(-2.0f * udg.z + 2.0f) * sinf(psi_d * 0.5f);
    
    quat_target.q1 = qd_0;
    quat_target.q2 = qd_1;
    quat_target.q3 = qd_2;
    quat_target.q4 = qd_3;
    // q_target normalize
    quat_target.normalize();
    // For initial testing, set angular velocity to zero
    ang_vel_target.zero();
}

void ModeLLC::calculate_velocity_control(const Vector3f& pos, const Vector3f& vel, 
                                       const Vector3f& accel, const Vector3f& pos_target,
                                       Vector3f& u, Vector3f& u_dot) 
{
    // Calculate position error
    Vector3f pos_error = pos_target - pos;
    
    // Calculate desired velocity (simplified P controller)
    Vector3f vel_desired;
    vel_desired.x = pos_error.x * _vel_xy_p_gain;
    vel_desired.y = pos_error.y * _vel_xy_p_gain;
    vel_desired.z = pos_error.z * _vel_z_p_gain;
    
    // Calculate velocity error
    Vector3f vel_error = vel_desired - vel;
    
    // Calculate control output (simplified)
    const float kv = 15.0f;
    const float mass = 0.8f;
    const float gravity = GRAVITY_MSS;
    
    u = -kv * vel_error + Vector3f(0, 0, mass * gravity);
    u_dot.zero();  // Simplified - no acceleration feedforward
}