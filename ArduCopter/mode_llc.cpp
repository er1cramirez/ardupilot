#include "Copter.h"


bool ModeLLC::init(bool ignore_checks)
{
    // Set different gains for each axis (x, y, z)
    set_3sta_parameters(Vector3f(0.1f, 0.1f, 0.4f),  // k1: higher gain for z-axis
                       Vector3f(5.5f, 5.5f, 5.5f),      // k2: higher gain for z-axis
                       Vector3f(0.02f, 0.02f, 0.02f));  // k3: higher gain for z-axis
    
    reset_3sta();
    // Initialize position controller for Z axis if not already active
    // if (!pos_control->is_active_z()) {
    //     pos_control->init_z_controller();
    // }
    // // Set vertical speed and acceleration limits
    // pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    // pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // Initialize trajectory timing
    _trajectory_start_ms = AP_HAL::millis();
    // Initialize last run time
    last_run_ms = AP_HAL::millis();
    gcs().send_text(MAV_SEVERITY_INFO, "entering_LLC");
    return true;
}


void ModeLLC::run()
{
    // Calculate delta time since last execution
    uint32_t now_ms = AP_HAL::millis();
    dt = (now_ms - last_run_ms) / 1000.0f;  // Convert to seconds
    last_run_ms = now_ms;
    // Generate time-varying trajectory references
    float x_ref, y_ref;
    generate_trajectory_reference(x_ref, y_ref);
    
    // Add debug flags
    static bool debug_output = false;
    static uint32_t last_debug_ms = 0;
    
    // Test different altitude targets - try making z_ref more positive to go down,
    // more negative to go up (since we're in NED frame)

    float psi_d = 0.0f;
    float psi_d_dot = 0.0f;


    // Set desired neutral attitude (null quaternion)
    Quaternion target_attitude;
    target_attitude.initialise(); // This creates identity quaternion (no rotation)
    // Set zero angular velocity
    Vector3f target_ang_vel(0.0f, 0.0f, 0.0f);


    // Parameters
    float mass = 0.039f;
    float grav = 9.81f;
    float T = mass*grav;

    // inertial frame z-axis
    Vector3f e_z(0.0f, 0.0f, 1.0f);

    // Drone data initialization
    Vector3f x(0.0f, 0.0f, 0.0f);
    Vector3f x_dot(0.0f, 0.0f, 0.0f);
    Vector3f x_ddot(0.0f, 0.0f, 0.0f);

    // Get position, velocity and acceleration data
    if(ahrs.get_relative_position_NED_home(x) && ahrs.get_velocity_NED(x_dot)) 
    {   
        float z_offset = 0.3f; // Offset for z position
        x.z = x.z + z_offset; // Add offset to z position
        x_ddot = ahrs.get_accel_ef(); // Acceleration in NED inertial frame
        x_ddot = x_ddot + e_z*grav;

        
        // Desired position
        // Fix: In NED frame, positive z is downward, so we should use positive z_ref directly
        Vector3f x_d(x_ref, y_ref, x.z);
        Vector3f x_d_dot(0.0f, 0.0f, x_dot.z);
        Vector3f x_d_ddot(0.0f, 0.0f, x_ddot.z);
        // Add debug output every 500ms
        uint32_t now = AP_HAL::millis();
        if (debug_output && now - last_debug_ms > 500) {
            last_debug_ms = now;
            gcs().send_text(MAV_SEVERITY_INFO, 
                "POS: x:%.2f y:%.2f z:%.2f | TGT: x:%.2f y:%.2f z:%.2f", 
                (double)x.x, (double)x.y, (double)x.z,
                (double)x_d.x, (double)x_d.y, (double)x_d.z);
        }

        // Errors
        Vector3f xe = x - x_d;
        // Vector3f xe_dot = x_dot - x_d_dot;

        Vector3f Ve;
        // Vector3f xe_ddot = x_ddot - x_d_ddot;

        // HLC
        Vector3f u_d, u_d_dot;
        calculate_hlc(x_d, x, x_d_dot, x_dot, x_d_ddot, x_ddot, u_d, u_d_dot, Ve);

        // Debug control outputs
        if (debug_output && now - last_debug_ms < 50) { // Same update cycle as position
            gcs().send_text(MAV_SEVERITY_INFO, 
                "CTRL: ux:%.2f uy:%.2f uz:%.2f | Vz:%.2f Vz_d:%.2f", 
                (double)u_d.x, (double)u_d.y, (double)u_d.z,
                (double)x_dot.z, (double)(x_d_dot.z));
        }

        // Calculate virtual control
        Vector3f u_aux = {0.0f, 0.0f, 0.0f};
        u_aux = u_d + Vector3f{0.0f, 0.0f, 0.0f};
        calculate_virtual_control(u_aux, u_d_dot, psi_d, T, psi_d_dot, target_attitude, target_ang_vel);

        // Log more detailed control info
        float d_r = Vector2f(xe.x, xe.y).length();
        AP::logger().Write("VLCL",
            "TimeUS,PErrX,PErrY,d_r,VErrX,VErrY,VErrZ,ud_x,ud_y,ud_z",
                "Qfffffffff",
                AP_HAL::micros64(),
                (float)xe.x,
                (float)xe.y,
                (float)d_r,
                (float)Ve.x,
                (float)Ve.y,
                (float)Ve.z,
                (float)u_d.x,
                (float)u_d.y,
                (float)u_d.z);
    }
    // Handle motor spool states
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        
        // attitude_control->reset_rate_controller_I_terms();
        // attitude_control->reset_yaw_target_and_rate(false);
        // pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        // attitude_control->reset_yaw_target_and_rate();
        // attitude_control->reset_rate_controller_I_terms();
        // pos_control->relax_z_controller(0.0f);
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        // attitude_control->reset_yaw_target_and_rate();
        // attitude_control->reset_rate_controller_I_terms_smoothly();

        // pos_control->relax_z_controller(0.0f);
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // Flying - run quaternion controller
        
        attitude_control->input_quaternion(target_attitude, target_ang_vel);

        // pos_control->set_alt_target_with_slew(70.0f);
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
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

// Fix function signature to match call site - IMPORTANT parameter order change!
void ModeLLC::calculate_virtual_control(const Vector3f& u_d, const Vector3f& u_dot_d, float psi_d,
    float& T, float psi_dot_d, Quaternion& q_d, Vector3f& omega_d) 
{

    Vector3f uu, uup;
    float norm = u_d.length();  // This is the thrust magnitude
    if (norm < 4.6416e-04f) norm = 4.6416e-04f;  // Minimum thrust value
    float norm3 = norm * norm * norm;
    float u = u_d.x * u_dot_d.x + u_d.y * u_dot_d.y + u_d.z * u_dot_d.z;

    uu = u_d;
    uu.normalize();  // Unit vector in thrust direction

    // Derivative of the unit thrust vector
    uup.x = u_dot_d.x / norm - u_d.x * u / norm3;
    uup.y = u_dot_d.y / norm - u_d.y * u / norm3;
    uup.z = u_dot_d.z / norm - u_d.z * u / norm3;

    

    float u_3 = sqrtf(-2 * uu.z + 2);

    // Calculate desired quaternion based on thrust direction
    Quaternion refQuaternion;
    refQuaternion.q1 = u_3 * cosf(psi_d / 2) / 2;
    refQuaternion.q2 = (-uu.x * sinf(psi_d / 2) + uu.y * cosf(psi_d / 2)) / u_3;
    refQuaternion.q3 = (-uu.x * cosf(psi_d / 2) - uu.y * sinf(psi_d / 2)) / u_3;
    refQuaternion.q4 = sinf(psi_d / 2) * u_3 / 2;

    // Calculate desired angular velocity
    Vector3f refOmega;
    refOmega.x = -uup.x * sinf(psi_d) + uup.y * cosf(psi_d) + uup.z * (uu.x * sinf(psi_d) - uu.y * cosf(psi_d)) / (1 - uu.z);
    refOmega.y = -uup.x * cosf(psi_d) - uup.y * sinf(psi_d) + uup.z * (uu.x * cosf(psi_d) + uu.y * sinf(psi_d)) / (1 - uu.z);
    refOmega.z = psi_dot_d - (-uu.x * uup.y + uu.y * uup.x) / (1 - uu.z);

    T = norm;
    q_d = refQuaternion;
    omega_d = refOmega;
}


void ModeLLC::calculate_hlc(const Vector3f& xi_c, const Vector3f& xi, 
    const Vector3f& xi_dot_c, const Vector3f& xi_dot, 
    const Vector3f& xi_ddot_c, const Vector3f& xi_ddot,
    Vector3f& u, Vector3f& u_dot, Vector3f& Ve)
{
    // Calculate position, velocity and acceleration errors
    // The z component of the desired position is the height
    // And the z component of the desired velocity is the vertical speed
    // And the z component of the desired acceleration is the vertical acceleration
    Vector3f dv = xi_c - xi;
    Vector3f dv_dot = xi_dot_c - xi_dot;
    // Vector3f dv_ddot = xi_ddot_c - xi_ddot;

    Vector3f V = xi_dot;
    Vector3f V_dot = xi_ddot;

    // Distance and direction calculations
    float d = dv.length();
    
    
    Vector3f R = dv / d;
    float d_dot = dv_dot * R; 
    Vector3f R_dot = (dv_dot * d - dv * d_dot) / (d * d);
    // float d_ddot = dv_ddot * R + dv_dot * R_dot;
    // Vector3f R_ddot = (dv_ddot * d - dv * d_ddot) / (d * d) - ((dv_dot * d - dv * d_dot) * d_dot * 2.0f) / (d * d * d);

    // Define constant vectors
    Vector3f Tv(0.0f, 0.0f, 1.0f);
    Vector3f T_dot(0.0f, 0.0f, 0.0f);
    // Vector3f T_ddot(0.0f, 0.0f, 0.0f);

    // Parameters for navigation functions (similar to MATLAB)
    float a = 1.0f;         // Amplitude parameter
    float b_0 = 1.5f;       // Base slope parameter (equivalent to c1 in original code)
    float k_b = 0.3f;       // Height sensitivity for b parameter
    float c = 0.0f;         // Offset parameter
    
    //Positive scalar value of current height(tangential distance)
    float z = -xi.z;        // Height (positive upward)
    float z_dot = -xi_dot.z; // Height derivative
    // float z_ddot = -xi_ddot.z; // Height second derivative
    
    // Height-dependent b parameter (similar to MATLAB implementation)
    float b = b_0 * (1.0f + k_b * expf(-k_b * z));
    
    // Check for potential overflow in exp calculation
    if (-k_b * z > 88.0f) { // exp(88) is near float max
        gcs().send_text(MAV_SEVERITY_WARNING, "FPE risk: Exp overflow in HLC: z=%.2f", (double)z);
    }
    
    float b_dot = -b_0 * k_b * k_b * expf(-k_b * z) * z_dot;
    
    // Membership functions - check for potential NaN in tanh/cosh functions
    float arg_far = b * (d - c);
    if (fabsf(arg_far) > 88.0f) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FPE risk: Arg far too large in HLC: %.2f", (double)arg_far);
        arg_far = (arg_far > 0) ? 88.0f : -88.0f; // Limit to avoid float issues
    }
    
    float mu_far = a * tanhf(arg_far);
    
    float cosh_arg = b_0 * (d - c);
    if (fabsf(cosh_arg) > 88.0f) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FPE risk: Cosh arg too large in HLC: %.2f", (double)cosh_arg);
        cosh_arg = (cosh_arg > 0) ? 88.0f : -88.0f;
    }
    
    float mu_close = 1.0f / coshf(cosh_arg); 
    
    // First derivatives of membership functions
    // float sech_arg_far = 1.0f / coshf(arg_far);
    float mu_far_dot = a * (1.0f - tanhf(arg_far) * tanhf(arg_far)) * (b_dot * (d - c) + b * d_dot);
    float mu_close_dot = -b_0 * tanhf(b_0 * (d - c)) * (1.0f / coshf(b_0 * (d - c))) * d_dot;
    
    // Gain parameters
    float c2_k = 0.1f;
    float c2_T = c2_k * tanhf(b_0 * z); // Using b_0 as in the original code
    float c2_T_dot = c2_k * (b_0 * powf(1.0f / coshf(b_0 * z), 2.0f) * z_dot);
    float c2_R = 1.1f;
    float c2_R_dot = 0.0f;
    // float c2_R_ddot = 0.0f;

    // Desired velocity vector
    Vector3f Vd = (R * (mu_far * c2_R) + Tv * (mu_close * c2_T));

    // First derivative of desired velocity
    Vector3f Vd_dot = (R * (mu_far * c2_R_dot) + R * (mu_far_dot * c2_R) + R_dot * (mu_far * c2_R)) + 
                      (Tv * (mu_close * c2_T_dot) + Tv * (mu_close_dot * c2_T) + T_dot * (mu_close * c2_T));
    // Control law
    // float kv = 0.3f;
    // float m = 0.035f;
    // float gr = 9.81f;

    // Calculate control outputs
    Ve = V - Vd;
    // u = (V - Vd) * (-kv) - Vector3f(0.0f, 0.0f, m * gr);
    // u_dot = (V_dot - Vd_dot) * (-kv);
    // Initialize u.z
    // u.z = 0.035f*9.81f;
    // Call the control calculation function
    calculate_stsmc_control(V, Vd, V_dot, Vd_dot, u, u_dot);
    // u.z += -0.035f*9.81f;
    // set u_dot to zero for testing
    // u_dot.zero();
}

void ModeLLC::set_3sta_parameters(const Vector3f& new_k1, const Vector3f& new_k2, const Vector3f& new_k3) {
    k1 = new_k1;
    k2 = new_k2;
    k3 = new_k3;
}

/**
 * Utility: Sign function
 */
float ModeLLC::sign(float x) {
    return (x > 0.0) ? 1.0 : ((x < 0.0) ? -1.0 : 0.0);
}

/**
 * Reset the controller state
 */
void ModeLLC::reset_3sta() {
    x3_state.zero();
}

/**
 * Calculate phi1 function for 3-STA
 * 
 * @param x1 Velocity error (v - v_d)
 * @param x2 Acceleration error (a - a_d)
 * @return phi1 function values (3D vector)
 */
Vector3f ModeLLC::calculate_phi1(const Vector3f& x1, const Vector3f& x2) {
    Vector3f phi1 = {0.0, 0.0, 0.0};
    
    for (int i = 0; i < 3; i++) {
        // Calculate phi1 as per 3-STA definition - now using axis-specific k2 gains
        phi1[i] = x2[i] + (k2[i] * x1[i]);
    }
    
    return phi1;
}


/**
 * Calculate the 3-STA control law for velocity tracking
 * 
 * @param v Current velocity [vx,vy,vz]
 * @param v_d Desired velocity [vx,vy,vz]
 * @param a Current acceleration [ax,ay,az]
 * @param a_d Desired acceleration [ax,ay,az]
 * @param dt Time step
 * @param u Output control signal
 * @param u_dot Output control derivative
 */
void ModeLLC::calculate_stsmc_control(const Vector3f& v, const Vector3f& v_d,
    const Vector3f& a, const Vector3f& a_d,
    Vector3f& u, Vector3f& u_dot) {

    // Calculate error states for 3-STA
    Vector3f x1, x2;
    for (int i = 0; i < 3; i++) {
        x1[i] = v[i] - v_d[i];    // Velocity error
        x2[i] = a[i] - a_d[i];    // Acceleration error
    }

    // Calculate phi1
    Vector3f phi1 = calculate_phi1(x1, x2);

    Vector3f x3_dot = {0.0, 0.0, 0.0};

    for (int i = 0; i < 3; i++) {
        // Check for potential issues in pow calculation
        float phi_abs = fabsf(phi1[i]);
        
        // Protect against very small values that might cause precision issues
        if (phi_abs < 0.00001f) {
            phi_abs = 0.00001f;
        }
        
        // Calculate control derivate with protected values
        u_dot[i] = -k1[i] * powf(phi_abs, 0.5f) * sign(phi1[i]) + x3_state[i];

        // Calculate the derivative of x3 (for integration) - now using axis-specific gains
        x3_dot[i] = -k3[i] * sign(phi1[i]);

        // Update the integral state
        x3_state[i] = last_x3[i] + x3_dot[i] * dt;

        // Calculate the control signal
        u[i] = last_u[i] + u_dot[i] * dt;
        last_u[i] = u[i];
        last_x3[i] = x3_state[i];
    }
}

// Generate time-varying XY reference based on trajectory type
void ModeLLC::generate_trajectory_reference(float& x_ref, float& y_ref)
{
    // Get elapsed time in seconds
    uint32_t now = AP_HAL::millis();
    float elapsed_sec = (now - _trajectory_start_ms) / 1000.0f;
    
    // Trajectory parameters
    const float speed = 0.5f;  // m/s
    
    // Define trajectory type (can be expanded with more patterns)
    enum class TrajType {
        LINEAR_X,      // Linear motion along X axis
        LINEAR_Y,      // Linear motion along Y axis
        CIRCLE         // Circular path
    };
    
    // Select trajectory type (can be made configurable)
    TrajType traj_type = TrajType::CIRCLE;
    
    // Starting point for the trajectory
    const float start_x = 0.0f;
    const float start_y = 0.0f;
    
    // Generate reference based on trajectory type
    switch (traj_type) {
        case TrajType::LINEAR_X:
            x_ref = start_x + speed * elapsed_sec;
            y_ref = start_y + speed * elapsed_sec;
            break;
            
        case TrajType::LINEAR_Y:
            x_ref = start_x;
            y_ref = start_y + speed * elapsed_sec;
            break;
            
        case TrajType::CIRCLE: {
            // Circular trajectory
            const float radius = 5.0f;  // meters
            const float angular_speed = speed / radius;  // rad/s
            const float angle = angular_speed * elapsed_sec;
            
            x_ref = start_x + radius * cosf(angle);
            y_ref = start_y + radius * sinf(angle);
            break;
        }
    }
    
}