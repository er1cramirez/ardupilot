#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_QUAT_ENABLED

#include "AC_CustomControl_Quat.h"

const AP_Param::GroupInfo AC_CustomControl_Quat::var_info[] = {
    // @Param: KP_ATT
    // @DisplayName: Attitude Controller P Gain
    // @Description: Attitude controller proportional gain
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("KP_ATT", 1, AC_CustomControl_Quat, _kp_att, 700.0f),

    // @Param: KD_ATT
    // @DisplayName: Attitude Controller D Gain
    // @Description: Attitude controller derivative gain
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("KD_ATT", 2, AC_CustomControl_Quat, _kd_att, 18.0f),

    // @Param: UD_FF
    // @DisplayName: Feedforward Term
    // @Description: Feedforward term for desired acceleration
    // @Range: 0 20
    // @User: Advanced
    AP_GROUPINFO("UD_FF", 3, AC_CustomControl_Quat, _ud_ff, 9.81f),

    // @Param: PSI_REF
    // @DisplayName: Reference Yaw Angle
    // @Description: Desired yaw angle in degrees
    // @Range: -180 180
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("PSI_REF", 4, AC_CustomControl_Quat, _psi_ref, 0.0f),

    // @Param: DEBUG
    // @DisplayName: Debug Output Enable
    // @Description: Enable debug output over UDP for MATLAB plotting
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("DEBUG", 5, AC_CustomControl_Quat, _debug_enabled, 0),

    AP_GROUPEND
};

AC_CustomControl_Quat::AC_CustomControl_Quat(AC_CustomControl &frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _last_error(0,0,0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_CustomControl_Quat::quaternion_product(const float q1[4], const float q2[4], float result[4]) 
{
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void AC_CustomControl_Quat::calculate_virtual_control(const Vector3f& ud, const Vector3f& ud_dot, float psi_d, float qd[4], Vector3f& omega_d)
{
    // Normalize ud
    float ud_norm = ud.length();
    if (ud_norm < 1e-6f) {
        ud_norm = 1e-6f;
    }
    Vector3f udg = ud / ud_norm;
    
    // Calculate desired quaternion
    float sqrt_term = sqrtf(-2*udg.z + 2);
    if (sqrt_term < 1e-6f) {
        sqrt_term = 1e-6f;
    }
    
    qd[0] = 0.5f * sqrt_term * cosf(psi_d/2);
    qd[1] = (-udg.x*sinf(psi_d/2) + udg.y*cosf(psi_d/2))/sqrt_term;
    qd[2] = (-udg.x*cosf(psi_d/2) - udg.y*sinf(psi_d/2))/sqrt_term;
    qd[3] = 0.5f * sqrt_term * sinf(psi_d/2);
    
    // Calculate udg_dot
    Vector3f udg_dot = ud_dot*(1.0f/ud_norm) - ud*(ud.dot(ud_dot))/(ud_norm*ud_norm*ud_norm);
    
    // Calculate desired angular velocity
    omega_d.x = -sinf(psi_d)*udg_dot.x + cosf(psi_d)*udg_dot.y + 
                (udg_dot.z*(sinf(psi_d)*udg.x - cosf(psi_d)*udg.y))/(udg.z - 1);
    omega_d.y = -cosf(psi_d)*udg_dot.x - sinf(psi_d)*udg_dot.y + 
                (udg_dot.z*(cosf(psi_d)*udg.x + sinf(psi_d)*udg.y))/(udg.z - 1);
    omega_d.z = (udg.x*udg_dot.y - udg.y*udg_dot.x)/(udg.z - 1);
}

void AC_CustomControl_Quat::send_debug_data(const float q_e[4], const Vector3f& omega_e, const Vector3f& tau)
{
    if (!_debug_enabled) {
        return;
    }

    // Pack data into a struct for sending
    struct debug_data {
        float timestamp_us;
        float q_error[4];
        float omega_error[3];
        float control_output[3];
        float current_quaternion[4];
    } data;

    // Fill in the data
    data.timestamp_us = AP_HAL::micros();
    memcpy(data.q_error, q_e, sizeof(float)*4);
    data.omega_error[0] = omega_e.x;
    data.omega_error[1] = omega_e.y;
    data.omega_error[2] = omega_e.z;
    data.control_output[0] = tau.x;
    data.control_output[1] = tau.y;
    data.control_output[2] = tau.z;

    Quaternion current_quat;
    _ahrs->get_quat_body_to_ned(current_quat);
    data.current_quaternion[0] = current_quat.q1;
    data.current_quaternion[1] = current_quat.q2;
    data.current_quaternion[2] = current_quat.q3;
    data.current_quaternion[3] = current_quat.q4;

    // Send data over UDP
    debug_sock.sendto(&data, sizeof(data), "127.0.0.1", debug_port);
}


Vector3f AC_CustomControl_Quat::update(void)
{
    if (_ahrs == nullptr || _att_control == nullptr) {
        return Vector3f(0,0,0);
    }

    // Reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            reset();
            return Vector3f(0,0,0);
        default:
            break;
    }

    // Get current quaternion and angular velocity
    Quaternion current_quat;
    _ahrs->get_quat_body_to_ned(current_quat);
    Vector3f gyro = _ahrs->get_gyro();
    
    // Create test acceleration vector (will be replaced with actual control input)
    Vector3f ud(0, 0, -_ud_ff);
    Vector3f ud_dot(0, 0, 0);
    
    // Calculate desired quaternion and omega using virtual controller
    float qd[4];
    Vector3f omega_d;
    calculate_virtual_control(ud, ud_dot, radians(_psi_ref), qd, omega_d);
    
    // Calculate quaternion error (q_e = conj(q_d)*q)
    float q[4] = {current_quat.q1, current_quat.q2, current_quat.q3, current_quat.q4};
    float conj_qd[4] = {qd[0], -qd[1], -qd[2], -qd[3]};
    float q_e[4];
    quaternion_product(conj_qd, q, q_e);
    
    // Calculate angular velocity error
    Vector3f omega_e = gyro - omega_d;
    
   // Calculate control torques
    Vector3f tau;
    tau.x = -_kp_att * q_e[1] - _kd_att * omega_e.x;
    tau.y = -_kp_att * q_e[2] - _kd_att * omega_e.y;
    tau.z = -_kp_att * q_e[3] - _kd_att * omega_e.z;
    
    // Send debug data
    send_debug_data(q_e, omega_e, tau);
    
    return tau;
}

void AC_CustomControl_Quat::reset(void)
{
    _last_error.zero();
}

#endif  // AP_CUSTOMCONTROL_QUAT_ENABLED