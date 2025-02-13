#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_QUATERNION_ENABLED

#include "AC_CustomControl_Backend.h"

class AC_CustomControl_Quaternion : public AC_CustomControl_Backend {
public:
    AC_CustomControl_Quaternion(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, 
                              AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);
    
    Vector3f update() override;
    void reset() override;

    // Tabla de parámetros
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // Parámetros del controlador
    AP_Float _kp_attitude;    // Ganancia proporcional de actitud
    AP_Float _kd_rate;        // Ganancia derivativa de velocidad angular
    AP_Float _kp_thrust;      // Ganancia proporcional para thrust
    
    // Variables internas
    float _dt;
    Vector3f _last_gyro;
    
private:
    // Funciones auxiliares
    Vector3f calculate_torque(const Quaternion &q_error, const Vector3f &omega_error);
};

#endif  // AP_CUSTOMCONTROL_QUATERNION_ENABLED
