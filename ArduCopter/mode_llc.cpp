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
    float x_ref = 2.5f, y_ref = -2.5f, z_ref = 0.5f;
    // float x_dot_ref = 0.0f, y_dot_ref = 0.0f, z_dot_ref = 0.0f;
    // float x_ddot_ref = 0.0f, y_ddot_ref = 0.0f, z_ddot_ref = 0.0f;
    // float x_dddot_ref = 0.0f, y_dddot_ref = 0.0f, z_dddot_ref = 0.0f;

    Vector3f x_d(x_ref, y_ref, -z_ref);
    Vector3f x_d_dot(0.0f, 0.0f, 0.0f);
    Vector3f x_d_ddot(0.0f, 0.0f, 0.0f);
    // Vector3f x_d_dddot(0.0f, 0.0f, 0.0f);

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
    float mass = 0.0385f;
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
                 0.0f, 0.0f, -0.3f);

    Matrix3f kd1(-0.08f, 0.0f, 0.0f,
                0.0f, -0.08f, 0.0f,
                0.0f, 0.0f, -0.1f);

    if(ahrs.get_relative_position_NED_home(x) && ahrs.get_velocity_NED(x_dot)) 
    {   
        x_ddot = ahrs.get_accel_ef(); // Acceleration in NED inertial frame
        x_ddot = x_ddot + e_z*grav;

        // Errors
        Vector3f xe = x - x_d;
        // Vector3f xe_dot = x_dot - x_d_dot;

        Vector3f Ve;
        // Vector3f xe_ddot = x_ddot - x_d_ddot;

        // HLC
        Vector3f u_d, u_d_dot;
        calculate_hlc(x_d, x, x_d_dot, x_dot, x_d_ddot, x_ddot, u_d, u_d_dot, Ve);

        // // Control law
        // Vector3f u_d = kp1 * xe + kd1 * xe_dot - e_z * mass * grav + x_d_ddot * mass;
        // // Vector3f u_d_dot = kp1 * xe_dot + x_d_dddot*mass;// + kd1 * xe_ddot + x_d_dddot * mass;
        // Vector3f u_d_dot = kp1 * xe_dot + kd1 * xe_ddot + x_d_dddot * mass;

        // Calculate virtual control
        calculate_virtual_control(u_d, u_d_dot, psi_d, T, psi_d_dot, target_attitude, target_ang_vel);

        // Log control info
        AP::logger().Write("VLCL",
            "TimeUS,PErrX,PErrY,PErrZ,VErrX,VErrY,VErrZ,T",
                "Qfffffff",
                AP_HAL::micros64(),
                (float)xe.x,
                (float)xe.y,
                (float)xe.z,
                (float)Ve.x,
                (float)Ve.y,
                (float)Ve.z,
                (float)T);
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

        // pos_control->set_alt_target_with_slew(100.0f);
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Do nothing
        break;
    }
    // Set constant throttle for hover
    attitude_control->set_throttle_out(T, true, g.throttle_filt);
    // pos_control->update_z_controller();
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


void ModeLLC::calculate_hlc(const Vector3f& xi_c, const Vector3f& xi, 
    const Vector3f& xi_dot_c, const Vector3f& xi_dot, 
    const Vector3f& xi_ddot_c, const Vector3f& xi_ddot,
    Vector3f& u, Vector3f& u_dot, Vector3f& Ve)
{
    // Calculate position, velocity and acceleration errors
    Vector3f dv = xi_c - xi;
    Vector3f dv_dot = xi_dot_c - xi_dot;
    Vector3f dv_ddot = xi_ddot_c - xi_ddot;

    Vector3f V = xi_dot;
    Vector3f V_dot = xi_ddot;

    // Distance and direction calculations
    float d = dv.length();
    Vector3f R = dv / d;
    float d_dot = dv_dot * R; 
    Vector3f R_dot = (dv_dot * d - dv * d_dot) / (d * d);
    float d_ddot = dv_ddot * R + dv_dot * R_dot;
    Vector3f R_ddot = (dv_ddot * d - dv * d_ddot) / (d * d) - ((dv_dot * d - dv * d_dot) * d_dot * 2.0f) / (d * d * d);

    // Define constant vectors
    Vector3f T(0.0f, 0.0f, 1.0f);
    Vector3f T_dot(0.0f, 0.0f, 0.0f);
    Vector3f T_ddot(0.0f, 0.0f, 0.0f);

    // Membership functions
    float c1 = 1.5f;
    float mu_far = tanhf(c1 * d);
    float mu_close = 1.0f / coshf(c1 * d); // sech(x) = 1/cosh(x)

    // First derivatives of membership functions
    float sech_c1d = 1.0f / coshf(c1 * d);
    float mu_far_dot = c1 * sech_c1d * sech_c1d * d_dot;
    float mu_close_dot = -c1 * sech_c1d * tanhf(c1 * d) * d_dot;

    // Second derivatives of membership functions
    float mu_far_ddot = c1 * sech_c1d * sech_c1d * d_ddot - 
    2.0f * c1 * c1 * sech_c1d * sech_c1d * tanhf(c1 * d) * d_dot * d_dot;
    float mu_close_ddot = -c1 * sech_c1d * tanhf(c1 * d) * d_ddot - 
        c1 * c1 * sech_c1d * (1.0f - 2.0f * tanhf(c1 * d) * tanhf(c1 * d)) * d_dot * d_dot;

    // Height control parameters
    // float c1_t = 1.0f;
    float d_t = -xi.z;
    float d_t_dot = -xi_dot.z;
    float d_t_ddot = -xi_ddot.z;

    // Gain parameters
    float c2_k = 0.4f;
    float c2_T = c2_k * tanhf(c1 * d_t);
    float c2_T_dot = c2_k * (c1 * powf(1.0f / coshf(c1 * d_t), 2.0f) * d_t_dot);
    float c2_T_ddot = c2_k * (c1 * powf(1.0f / coshf(c1 * d_t), 2.0f) * d_t_ddot - 
    2.0f * c1 * c1 * powf(1.0f / coshf(c1 * d_t), 2.0f) * tanhf(c1 * d_t) * d_t_dot * d_t_dot);

    float c2_R = 0.9f;
    float c2_R_dot = 0.0f;
    float c2_R_ddot = 0.0f;

    // Desired velocity vector - changed order of operations
    Vector3f Vd = (R * (mu_far * c2_R) + T * (mu_close * c2_T));

    // First derivative of desired velocity - changed order of operations
    Vector3f Vd_dot = (R * (mu_far * c2_R_dot) + R * (mu_far_dot * c2_R) + R_dot * (mu_far * c2_R)) + 
                      (T * (mu_close * c2_T_dot) + T * (mu_close_dot * c2_T) + T_dot * (mu_close * c2_T));

    // Second derivative of desired velocity - changed order of operations
    Vector3f Vd_ddot = (R * (mu_far * c2_R_ddot) + R * (mu_far_dot * c2_R_dot) + R_dot * (mu_far * c2_R_dot) +
                       R * (mu_far_dot * c2_R_dot) + R * (mu_far_ddot * c2_R) + R_dot * (mu_far_dot * c2_R) +
                       R_dot * (mu_far * c2_R_dot) + R_dot * (mu_far_dot * c2_R) + R_ddot * (mu_far * c2_R)) +
                      (T * (mu_close * c2_T_ddot) + T * (mu_close_dot * c2_T_dot) + T_dot * (mu_close * c2_T_dot) +
                       T * (mu_close_dot * c2_T_dot) + T * (mu_close_ddot * c2_T) + T_dot * (mu_close_dot * c2_T) +
                       T_dot * (mu_close * c2_T_dot) + T_dot * (mu_close_dot * c2_T) + T_ddot * (mu_close * c2_T));
    // Control law
    float kv = 0.21f;
    float m = 0.0385f;
    // float gr = 9.81f;

    // Calculate control outputs
    Ve = V - Vd;
    u = (V - Vd) * (-kv) + Vd_dot * m - Vector3f(0.0f, 0.0f, motors->get_throttle_hover());
    u_dot = (V_dot - Vd_dot) * (-kv) + Vd_ddot * m;
}