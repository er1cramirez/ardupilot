#pragma once

#include "AC_CustomControl_Backend.h"

#if AP_CUSTOMCONTROL_ENABLED
#include <AP_Math/AP_Math.h>

class AC_CustomControl_LLC : public AC_CustomControl_Backend {
public:
    AC_CustomControl_LLC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, 
                        AC_AttitudeControl*& att_control, 
                        AP_MotorsMulticopter*& motors, float dt);
    
    // Main update function - returns motor outputs
    Vector3f update() override;
    
    // Reset controller state
    void reset() override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];
    
    // for setting notch filter sample rates
    void set_notch_sample_rate(float sample_rate) override {};

private:
    // PID controller parameters
    AP_Float _kp_roll;
    AP_Float _kp_pitch;
    AP_Float _kp_yaw;
    AP_Float _kd_roll;
    AP_Float _kd_pitch;
    AP_Float _kd_yaw;
    AP_Float _ki_roll;
    AP_Float _ki_pitch;
    AP_Float _ki_yaw;
    AP_Float _throttle_hover;
    AP_Float _integrator_windup;
    float _integrator_roll = 0.0f;
    float _integrator_pitch = 0.0f;
    float _integrator_yaw = 0.0f;
    float _dt;
    bool _first_run = true;
    Quaternion _prevTargetQuaternion;
    Quaternion _prevBodyQuaternion;
    
    // Quaternion math helper
    void calculate_attitude_error_quaternion(const Quaternion &attitude_body, 
                                           const Quaternion &attitude_target,
                                           Quaternion &error_quaternion);
};

#endif // AP_CUSTOMCONTROL_ENABLED