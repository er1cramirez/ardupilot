#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // @Param: _KPXY
    // @DisplayName: LLC xy velocity P gain
    // @Description: Ganancia P del lazo de velocidad relativa xy [throttle/(m/s)]
    // @User: Advanced
    AP_GROUPINFO("_KPXY", 0, UserParameters, _kp_xy, 0.03f),

    // @Param: _KIXY
    // @DisplayName: LLC xy velocity I gain
    // @Description: Ganancia I del lazo de velocidad relativa xy [throttle/m]
    // @User: Advanced
    AP_GROUPINFO("_KIXY", 1, UserParameters, _ki_xy, 0.015f),

    // @Param: _KPZ
    // @DisplayName: LLC z velocity P gain
    // @Description: Ganancia P del lazo de velocidad relativa z [throttle/(m/s)]
    // @User: Advanced
    AP_GROUPINFO("_KPZ", 2, UserParameters, _kp_z, 0.095f),

    // @Param: _TILTMAX
    // @DisplayName: LLC max tilt
    // @Description: Inclinación máxima por saturación de la norma de u_xy [deg]
    // @Range: 5 45
    // @User: Advanced
    AP_GROUPINFO("_TILTMAX", 3, UserParameters, _tilt_max, 20.0f),

    // @Param: _ABOOST
    // @DisplayName: LLC angle boost
    // @Description: 1 = aplicar angle boost de ArduPilot (doble compensación), 0 = no
    // @User: Advanced
    AP_GROUPINFO("_ABOOST", 4, UserParameters, _aboost, 0),

    // @Param: _YAWHOLD
    // @DisplayName: LLC yaw hold
    // @Description: 1 = mantener el yaw de entrada al modo, 0 = yaw deseado 0
    // @User: Advanced
    AP_GROUPINFO("_YAWHOLD", 5, UserParameters, _yaw_hold, 1),

    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
