#include "Copter.h"

// class ModeQuaternionTest : public Mode {
// public:
//     ModeQuaternionTest(void);
//     bool init(bool ignore_checks) override;
//     void run() override;

// private:
//     const float HOVER_THROTTLE = 2.5f; // Adjust this based on your vehicle
// };

// bool ModeQuaternionTest::init(bool ignore_checks)
// {
//     // Initialize any mode-specific variables here
//     return true;
// }

void ModeLLC::run()
{
    // Set desired neutral attitude (null quaternion)
    Quaternion target_attitude;
    target_attitude.initialise(); // This creates identity quaternion (no rotation)

    // Set zero angular velocity
    Vector3f target_ang_vel(0.0f, 0.0f, 0.0f);

    // Handle motor spool states
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // Flying - run quaternion controller
        attitude_control->input_quaternion(target_attitude, target_ang_vel);
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Do nothing
        break;
    }

    // Set constant throttle for hover
    attitude_control->set_throttle_out(HOVER_THROTTLE, true, g.throttle_filt);

    // Get current attitude for debugging
    Quaternion current_attitude;
    copter.ahrs.get_quat_body_to_ned(current_attitude);

    // Log or print error
    Quaternion attitude_error = current_attitude.inverse() * target_attitude;
    Vector3f error_angle;
    attitude_error.to_axis_angle(error_angle);

    // Get current angular rates
    Vector3f gyro = copter.ahrs.get_gyro();

    // Log debugging info
    copter.logger.Write("QUAT", "TimeUS,ErrX,ErrY,ErrZ,GyrX,GyrY,GyrZ",
                       "sdddEEE",
                       "F000000",
                       "Qffffff",
                       AP_HAL::micros64(),
                       (double)error_angle.x,
                       (double)error_angle.y,
                       (double)error_angle.z,
                       (double)gyro.x,
                       (double)gyro.y,
                       (double)gyro.z);
}