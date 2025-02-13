#include "Copter.h"

#if MODE_VELCTRL_ENABLED == ENABLED

bool ModeVelCtrl::init(bool ignore_checks) 
{
    if (!copter.position_ok() && !ignore_checks) {
        return false;
    }

    // Initialize position and velocity targets to current position and zero velocity
    _pos_target = inertial_nav.get_position_neu_cm().tofloat();
    _vel_target.zero();
    _accel_target.zero();
    _pos_vel_targets_set = false;

    // Initialize position controller
    pos_control->init_xy_controller();
    pos_control->init_z_controller();

    return true;
}

void ModeVelCtrl::run()
{
    // If not armed or landed, make vehicle safe
    if (!motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        make_safe_ground_handling();
        return;
    }

    // Set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Run velocity controller
    calculate_velocity_control();
}

bool ModeVelCtrl::set_target_position_velocity(const Vector3f& pos_target, 
                                             const Vector3f& vel_target,
                                             const Vector3f& accel_target)
{
    _pos_target = pos_target;
    _vel_target = vel_target;
    _accel_target = accel_target;
    _pos_vel_targets_set = true;
    return true;
}

// Update calculate_velocity_control() method:
void ModeVelCtrl::calculate_velocity_control() 
{
    // Get current position, velocity and acceleration
    Vector3f xi = inertial_nav.get_position_neu_cm().tofloat();
    Vector3f xi_dot = inertial_nav.get_velocity_neu_cms();
    Vector3f xi_ddot = inertial_nav.get_accel_neu_cmss();

    // Calculate xi_c vector [a, b, s] where s = current z position
    Vector3f xi_c(_pos_target.x, _pos_target.y, xi.z);
    Vector3f xi_c_dot(0, 0, xi_dot.z);
    Vector3f xi_c_ddot(0, 0, xi_ddot.z);

    // Calculate position deviation
    Vector3f dv = xi_c - xi;
    Vector3f dv_dot = xi_c_dot - xi_dot;
    Vector3f dv_ddot = xi_c_ddot - xi_ddot;

    // Calculate distance and unit vector
    float d = dv.length();
    Vector3f R = dv / d;
    
    // Calculate derivatives
    float d_dot = dv_dot.dot(R);
    Vector3f R_dot = (d * dv_dot - dv * d_dot) / (d * d);
    
    // Target vector calculations
    Vector3f T(0, 0, 1); // xiv_cs normalized
    
    // Membership functions (similar to MATLAB)
    const float c1 = 2.0f;
    const float c2 = 3.0f;
    float mu_far = tanhf(c1 * d);
    float mu_close = 1.0f / coshf(c1 * d); // sech = 1/cosh

    // Desired velocity calculation
    Vector3f Vd = c2 * (mu_far * R + mu_close * T);
    
    // Control law implementation
    const float kv = 15.0f;
    const float mass = 0.8f;
    const float gravity = GRAVITY_MSS * 100.0f; // Convert to cm/s/s

    // Calculate control vector u (desired acceleration)
    Vector3f u = -kv * (xi_dot - Vd) + Vector3f(0, 0, -mass * gravity);
    
    // Calculate virtual control outputs
    Quaternion att_target;
    Vector3f ang_vel_target;
    calculate_virtual_control(u, u * 0, att_target, ang_vel_target); // Simplified u_dot

    // Command attitude controller
    attitude_control->input_quaternion(att_target, ang_vel_target);
}

void ModeVelCtrl::calculate_virtual_control(const Vector3f& ud, 
                                          const Vector3f& ud_dot,
                                          Quaternion& qd, 
                                          Vector3f& omega_d)
{
    // Normalize thrust vector
    Vector3f udg = ud.normalized();
    
    // Get desired yaw (could be from pilot input or mission)
    float psi_d = wrap_2PI(get_pilot_desired_yaw());
    float psid_dot = get_pilot_desired_yaw_rate();

    // Calculate quaternion components (matching MATLAB implementation)
    float qd_0 = 0.5f * sqrtf(-2.0f * udg.z + 2.0f) * cosf(psi_d * 0.5f);
    float qd_1 = (-udg.x * sinf(psi_d * 0.5f) + udg.y * cosf(psi_d * 0.5f)) / sqrtf(-2.0f * udg.z + 2.0f);
    float qd_2 = (-udg.x * cosf(psi_d * 0.5f) - udg.y * sinf(psi_d * 0.5f)) / sqrtf(-2.0f * udg.z + 2.0f);
    float qd_3 = 0.5f * sqrtf(-2.0f * udg.z + 2.0f) * sinf(psi_d * 0.5f);
    
    qd.q1 = qd_0;
    qd.q2 = qd_1;
    qd.q3 = qd_2;
    qd.q4 = qd_3;

    // Calculate angular velocity (matching MATLAB implementation)
    Vector3f udg_dot = ud_dot * (1.0f/ud.length()) - 
                      ud * (ud.dot(ud_dot)/(2.0f * powf(ud.length(), 3.0f)));

    omega_d.x = -sinf(psi_d) * udg_dot.x + cosf(psi_d) * udg_dot.y + 
                (udg_dot.z * (sinf(psi_d) * udg.x - cosf(psi_d) * udg.y))/(udg.z - 1.0f);
    
    omega_d.y = -cosf(psi_d) * udg_dot.x - sinf(psi_d) * udg_dot.y + 
                (udg_dot.z * (cosf(psi_d) * udg.x + sinf(psi_d) * udg.y))/(udg.z - 1.0f);
    
    omega_d.z = psid_dot + (udg.x * udg_dot.y - udg.y * udg_dot.x)/(udg.z - 1.0f);
}

#endif