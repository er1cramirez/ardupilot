#pragma once

// #include "AC_CustomControl_Quat_config.h"
#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_QUAT_ENABLED
#include "AC_CustomControl_Backend.h"
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/Socket.h>

class AC_CustomControl_Quat : public AC_CustomControl_Backend {
public:
    AC_CustomControl_Quat(AC_CustomControl &frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);

    Vector3f update(void) override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // Helper functions
    void quaternion_product(const float q1[4], const float q2[4], float result[4]);
    void calculate_virtual_control(const Vector3f& ud, const Vector3f& ud_dot, float psi_d, float qd[4], Vector3f& omega_d);
    void send_debug_data(const float q_e[4], const Vector3f& omega_e, const Vector3f& tau);
    
    // Parameters
    AP_Float _kp_att;        // attitude proportional gain
    AP_Float _kd_att;        // attitude derivative gain
    AP_Float _ud_ff;         // feedforward term for ud
    AP_Float _psi_ref;       // reference yaw angle
    AP_Int8  _debug_enabled; // enable debug output over UDP
    
    // Previous state storage for derivative calculations
    Vector3f _last_error;

    // UDP socket for MATLAB plotting
    SocketAPM debug_sock{true};
    static constexpr uint16_t debug_port = 9002;
};
#endif  // AP_CUSTOMCONTROL_QUAT_ENABLED