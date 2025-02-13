#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_QUATERNION_ENABLED

#include "AC_CustomControl_Quaternion.h"
#include <GCS_MAVLink/GCS.h>

// tabla de parámetros configurables
const AP_Param::GroupInfo AC_CustomControl_Quaternion::var_info[] = {
    // @Param: KP_ATT
    // @DisplayName: Attitude Proportional Gain
    // @Description: Quaternion attitude controller proportional gain
    // @Range: 100 1000
    // @User: Advanced
    AP_GROUPINFO("KP_ATT", 1, AC_CustomControl_Quaternion, _kp_attitude, 700.0f),

    // @Param: KD_RATE
    // @DisplayName: Angular Rate Derivative Gain
    // @Description: Angular rate controller derivative gain
    // @Range: 1 50
    // @User: Advanced
    AP_GROUPINFO("KD_RATE", 2, AC_CustomControl_Quaternion, _kd_rate, 18.0f),

    // @Param: KP_THR
    // @DisplayName: Thrust Proportional Gain
    // @Description: Thrust controller proportional gain
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("KP_THR", 3, AC_CustomControl_Quaternion, _kp_thrust, 1.0f),

    AP_GROUPEND
};

AC_CustomControl_Quaternion::AC_CustomControl_Quaternion(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, 
    AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);
}

Vector3f AC_CustomControl_Quaternion::update()
{
    // Verificar estado del motor
    if (_motors->get_spool_state() <= AP_Motors::SpoolState::GROUND_IDLE) {
        reset();
        return Vector3f();
    }

    // Obtener actitud actual y deseada
    Quaternion attitude_actual, attitude_desired;
    _ahrs->get_quat_body_to_ned(attitude_actual);
    attitude_desired = _att_control->get_attitude_target_quat();

    // Normalizar cuaterniones
    attitude_actual.normalize();
    attitude_desired.normalize();

    // Calcular error de cuaternión (q_e = q_d^* ⊗ q)
    Quaternion q_error = attitude_desired.inverse() * attitude_actual;

    // Obtener velocidades angulares
    Vector3f omega_actual = _ahrs->get_gyro_latest();
    Vector3f omega_desired = _att_control->get_attitude_target_ang_vel();
    Vector3f omega_error = omega_actual - omega_desired;

    // Calcular torques de control
    Vector3f tau = calculate_torque(q_error, omega_error);

    // Limitar torques máximos
    tau.x = constrain_float(tau.x, -1.0f, 1.0f);
    tau.y = constrain_float(tau.y, -1.0f, 1.0f);
    tau.z = constrain_float(tau.z, -1.0f, 1.0f);

    _last_gyro = omega_actual;
    
    return tau;
}

Vector3f AC_CustomControl_Quaternion::calculate_torque(const Quaternion &q_error, const Vector3f &omega_error)
{
    // Extraer vector de error del cuaternión
    Vector3f q_vec_error(q_error.q2, q_error.q3, q_error.q4);
    
    // Ley de control: τ = -Kp * q_vector - Kd * ω_error
    Vector3f tau = -(_kp_attitude * q_vec_error + _kd_rate * omega_error);
    
    return tau;
}

void AC_CustomControl_Quaternion::reset()
{
    _last_gyro.zero();
    
    // Notificar reset del controlador
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CustomControl: quaternion controller reset");
}

#endif  // AP_CUSTOMCONTROL_QUATERNION_ENABLED
