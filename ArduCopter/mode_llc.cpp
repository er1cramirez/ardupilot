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
    // Set desired neutral attitude (null quaternion)
    // Quaternion target_attitude;
    target_attitude.initialise(); // This creates identity quaternion (no rotation)

    // Set zero angular velocity
    Vector3f target_ang_vel(0.0f, 0.0f, 0.0f);

    // Handle motor spool states
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Define target height (in cm above home)
    const float target_height_cm = 150.0f; // 1.5 meters above home

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
        // Flying - run quaternion controller for attitude
        attitude_control->input_quaternion(target_attitude, target_ang_vel);
        
        // Set position target for Z axis
        pos_control->set_alt_target_with_slew(target_height_cm);
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Do nothing
        break;
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