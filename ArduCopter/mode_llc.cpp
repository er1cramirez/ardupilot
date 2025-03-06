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
    float x_ref = 1.0f, y_ref = 1.0f, z_ref = 0.5f;
    // float x_dot_ref = 0.0f, y_dot_ref = 0.0f, z_dot_ref = 0.0f;
    // float x_ddot_ref = 0.0f, y_ddot_ref = 0.0f, z_ddot_ref = 0.0f;
    // float x_dddot_ref = 0.0f, y_dddot_ref = 0.0f, z_dddot_ref = 0.0f;

    Vector3f x_d(x_ref, y_ref, -z_ref);
    Vector3f x_d_dot(0.0f, 0.0f, 0.0f);
    Vector3f x_d_ddot(0.0f, 0.0f, 0.0f);
    Vector3f x_d_dddot(0.0f, 0.0f, 0.0f);

    float psi_d = 0.0f;
    float psi_d_dot = 0.0f;


    // Set desired neutral attitude (null quaternion)
    Quaternion target_attitude;
    target_attitude.initialise(); // This creates identity quaternion (no rotation)
    // Set zero angular velocity
    Vector3f target_ang_vel(0.0f, 0.0f, 0.0f);
    // thrust
    // float T = 0.0f;

    // Parameters
    float mass = 0.05f;
    float grav = 9.81f;
    float T = mass*grav;
    Vector3f e_z(0.0f, 0.0f, 1.0f);

    // Drone data initialization
    Vector3f x(0.0f, 0.0f, 0.0f);
    Vector3f x_dot(0.0f, 0.0f, 0.0f);
    Vector3f x_ddot(0.0f, 0.0f, 0.0f);

    // Control gains
    Matrix3f kp1(-0.1f, 0.0f, 0.0f,
                 0.0f, -0.1f, 0.0f,
                 0.0f, 0.0f, -0.0f);

    Matrix3f kd1(-0.08f, 0.0f, 0.0f,
                0.0f, -0.08f, 0.0f,
                0.0f, 0.0f, -0.0f);

    if(ahrs.get_relative_position_NED_home(x) && ahrs.get_velocity_NED(x_dot)) 
    {   
        x_ddot = ahrs.get_accel_ef(); // Acceleration in NED inertial frame
        x_ddot = x_ddot + e_z*grav;

        // Errors
        Vector3f xe = x - x_d;
        Vector3f xe_dot = x_dot - x_d_dot;
        Vector3f xe_ddot = x_ddot - x_d_ddot;

        // Control law
        Vector3f u_d = kp1 * xe + kd1 * xe_dot - e_z * mass * grav + x_d_ddot * mass;
        // Vector3f u_d_dot = kp1 * xe_dot + x_d_dddot*mass;// + kd1 * xe_ddot + x_d_dddot * mass;
        Vector3f u_d_dot = kp1 * xe_dot + kd1 * xe_ddot + x_d_dddot * mass;

        // Calculate virtual control
        calculate_virtual_control(u_d, u_d_dot, psi_d, T, psi_d_dot, target_attitude, target_ang_vel);
    }
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
        // Flying - run quaternion controller
        attitude_control->input_quaternion(target_attitude, target_ang_vel);

        pos_control->set_alt_target_with_slew(100.0f);
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Do nothing
        break;
    }
    // Set constant throttle for hover
    // attitude_control->set_throttle_out(HOVER_THROTTLE, true, g.throttle_filt);
    pos_control->update_z_controller();
}

void ModeLLC::calculate_virtual_control(const Vector3f& u_d, const Vector3f& u_d_dot, float psi_d,
    float psi_d_dot, float& T, Quaternion& q_d, Vector3f& omega_d) 
{
    // Desired attitude
    Vector3f u_d_norm = u_d.normalized();
    Vector3f u_d_dot_norm = u_d_dot / u_d.length() - u_d * (u_d * u_d_dot) / powf(u_d.length(), 3.0f);
    Quaternion q_dxy(1.0f/2.0f * sqrtf(-2*u_d_norm.z + 2),
                     u_d_norm.y / sqrtf(-2*u_d_norm.z + 2),
                     -u_d_norm.x / sqrtf(-2*u_d_norm.z + 2),
                     0.0f);

    Quaternion q_dz(cosf(psi_d/2.0f), 
                         0.0f, 
                         0.0f, 
                         sinf(psi_d/2.0f));

    q_d = q_dxy * q_dz;
    q_d.normalize();
    omega_d = {-sinf(psi_d)*u_d_dot_norm.x + cosf(psi_d)*u_d_dot_norm.y + u_d_dot_norm.z*(sinf(psi_d)*u_d_norm.x - cosf(psi_d)*u_d_norm.y)/(u_d_norm.z - 1.0f),
                         -cosf(psi_d)*u_d_dot_norm.x - sinf(psi_d)*u_d_dot_norm.y + u_d_dot_norm.z*(cosf(psi_d)*u_d_norm.x + sinf(psi_d)*u_d_norm.y)/(u_d_norm.z - 1.0f),
                         psi_d_dot + (u_d_norm.x*u_d_dot_norm.y - u_d_norm.y*u_d_dot_norm.x)/(u_d_norm.z - 1.0f)};

    // Thrust
    T = u_d.length(); 
}


// void ModeLLC::run()
// {
//     // Set desired neutral attitude (null quaternion)
//     // Quaternion target_attitude;
//     target_attitude.initialise(); // This creates identity quaternion (no rotation)

//     // Set zero angular velocity
//     Vector3f target_ang_vel(0.0f, 0.0f, 0.0f);

//     // Handle motor spool states
//     if (!motors->armed()) {
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        
//         attitude_control->reset_rate_controller_I_terms();
//         attitude_control->reset_yaw_target_and_rate(false);
//         pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
//     } else {
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
//     }

//     // Define target height (in cm above home)
//     const float target_height_cm = 150.0f; // 1.5 meters above home

//     switch (motors->get_spool_state()) {
//     case AP_Motors::SpoolState::SHUT_DOWN:
//         // Motors Stopped
//         attitude_control->reset_yaw_target_and_rate();
//         attitude_control->reset_rate_controller_I_terms();
//         pos_control->relax_z_controller(0.0f);
//         break;

//     case AP_Motors::SpoolState::GROUND_IDLE:
//         // Landed
//         attitude_control->reset_yaw_target_and_rate();
//         attitude_control->reset_rate_controller_I_terms_smoothly();
//         pos_control->relax_z_controller(0.0f);
//         break;

//     case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
//         // Flying - run quaternion controller for attitude
//         attitude_control->input_quaternion(target_attitude, target_ang_vel);
        
//         // Set position target for Z axis
//         pos_control->set_alt_target_with_slew(target_height_cm);
//         break;

//     case AP_Motors::SpoolState::SPOOLING_UP:
//     case AP_Motors::SpoolState::SPOOLING_DOWN:
//         // Do nothing
//         break;
//     }

//     // Update the vertical position controller
//     pos_control->update_z_controller();

//     // Add logging for debugging
//     // log_data();
// }

