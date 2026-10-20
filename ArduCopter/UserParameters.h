#pragma once

#include <AP_Param/AP_Param.h>

// Parámetros de ModeLLC ajustables en vuelo (param set USR_xxx), ronda B.
class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    float kp_xy() const { return _kp_xy; }
    float ki_xy() const { return _ki_xy; }
    float kp_z() const { return _kp_z; }
    float tilt_max_deg() const { return _tilt_max; }
    bool angle_boost() const { return _aboost != 0; }
    bool yaw_hold() const { return _yaw_hold != 0; }

private:
    AP_Float _kp_xy;
    AP_Float _ki_xy;
    AP_Float _kp_z;
    AP_Float _tilt_max;
    AP_Int8  _aboost;
    AP_Int8  _yaw_hold;
};
