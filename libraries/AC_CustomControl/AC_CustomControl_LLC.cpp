#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ENABLED

#include "AC_CustomControl_LLC.h"
#include <AP_Logger/AP_Logger.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_LLC::var_info[] = {
    // @Param: ROLL_P
    // @DisplayName: Roll P gain
    // @Description: Roll P gain for PD attitude controller
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("ROLL_P", 1, AC_CustomControl_LLC, _kp_roll, 6.5f),

    // @Param: PITCH_P
    // @DisplayName: Pitch P gain
    // @Description: Pitch P gain for PD attitude controller
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("PITCH_P", 2, AC_CustomControl_LLC, _kp_pitch, 6.5f),

    // @Param: YAW_P
    // @DisplayName: Yaw P gain
    // @Description: Yaw P gain for PD attitude controller
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("YAW_P", 3, AC_CustomControl_LLC, _kp_yaw, 6.5f),

    // @Param: ROLL_D
    // @DisplayName: Roll D gain
    // @Description: Roll D gain for PD attitude controller
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("ROLL_D", 4, AC_CustomControl_LLC, _kd_roll, 0.15f),

    // @Param: PITCH_D
    // @DisplayName: Pitch D gain
    // @Description: Pitch D gain for PD attitude controller
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("PITCH_D", 5, AC_CustomControl_LLC, _kd_pitch, 0.15f),

    // @Param: YAW_D
    // @DisplayName: Yaw D gain
    // @Description: Yaw D gain for PD attitude controller
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("YAW_D", 6, AC_CustomControl_LLC, _kd_yaw, 0.15f),

    // @Param: ROLL_I
    // @DisplayName: Roll I gain
    // @Description: Roll I gain for PID attitude controller
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("ROLL_I", 7, AC_CustomControl_LLC, _ki_roll, 0.0f),

    // @Param: PITCH_I
    // @DisplayName: Pitch I gain
    // @Description: Pitch I gain for PID attitude controller
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("PITCH_I", 8, AC_CustomControl_LLC, _ki_pitch, 0.0f),

    // @Param: YAW_I
    // @DisplayName: Yaw I gain
    // @Description: Yaw I gain for PID attitude controller
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("YAW_I", 9, AC_CustomControl_LLC, _ki_yaw, 0.0f),

    // @Param: INTEGRATOR_WINDUP
    // @DisplayName: Integrator windup limit
    // @Description: Maximum windup limit for the integrator
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("INTEGRATOR_WINDUP", 10, AC_CustomControl_LLC, _integrator_windup, 0.1f),

    AP_GROUPEND
};

AC_CustomControl_LLC::AC_CustomControl_LLC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, 
                                     AC_AttitudeControl*& att_control, 
                                     AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    _dt = dt;
}

void AC_CustomControl_LLC::reset()
{
    // Reset any integrators or other controller states here if needed
    _integrator_roll = 0.0f;
    _integrator_pitch = 0.0f;
    _integrator_yaw = 0.0f;
}

void AC_CustomControl_LLC::calculate_attitude_error_quaternion(const Quaternion &attitude_body, 
                                                            const Quaternion &attitude_target,
                                                            Quaternion &error_quaternion)
{
    // Calculate the error quaternion between current and target attitude
    // q_error = q_target * q_body^(-1)
    // error_quaternion = attitude_target * attitude_body.inverse();
    error_quaternion = attitude_target.inverse() * attitude_body;
    
    // Ensure shortest rotation path
    error_quaternion.normalize();
}

Vector3f AC_CustomControl_LLC::update()
{
    // Get current attitude as quaternion
    Quaternion attitude_body;
    Quaternion attitude_body_ned;
    _ahrs->get_body_quat(attitude_body);
    _ahrs->get_quat_body_to_ned(attitude_body_ned);
    
    // Get target attitude from attitude controller
    Quaternion attitude_target;
    attitude_target = _att_control->get_attitude_target_quat();
     
    if(_first_run) {
        // Initialize previous quaternion on first run
        _prevTargetQuaternion = attitude_target;
        _prevBodyQuaternion = attitude_body;
        _first_run = false;
    }

    // Checks for sign changes in quaternion and if it is, flip the quaternion
    if (attitude_body.q1 * _prevBodyQuaternion.q1 + attitude_body.q2 * _prevBodyQuaternion.q2 +
        attitude_body.q3 * _prevBodyQuaternion.q3 + attitude_body.q4 * _prevBodyQuaternion.q4 < 0) {
        attitude_body.q1 = -attitude_body.q1;
        attitude_body.q2 = -attitude_body.q2;
        attitude_body.q3 = -attitude_body.q3;
        attitude_body.q4 = -attitude_body.q4;
    }

    if (attitude_target.q1 * _prevTargetQuaternion.q1 + attitude_target.q2 * _prevTargetQuaternion.q2 +
        attitude_target.q3 * _prevTargetQuaternion.q3 + attitude_target.q4 * _prevTargetQuaternion.q4 < 0) {
        attitude_target.q1 = -attitude_target.q1;
        attitude_target.q2 = -attitude_target.q2;
        attitude_target.q3 = -attitude_target.q3;
        attitude_target.q4 = -attitude_target.q4;
    }

    _prevBodyQuaternion = attitude_body;
    _prevTargetQuaternion = attitude_target;

    // Calculate attitude error quaternion
    Quaternion q_error;
    attitude_target.normalize();
    attitude_body.normalize();
    calculate_attitude_error_quaternion(attitude_body, attitude_target, q_error);
    
    // Extract the vectorial part as the error
    Vector3f rotation_vector_error(q_error.q2, q_error.q3, q_error.q4);
    
    // Get current angular velocity (gyro data)
    Vector3f gyro = _ahrs->get_gyro_latest();
    
    // Get target angular velocity from attitude controller
    Vector3f target_ang_vel = _att_control->get_attitude_target_ang_vel();
    
    // Calculate angular velocity error
    Vector3f ang_vel_error = gyro - target_ang_vel;
    // Apply PD controller gains
    Vector3f torques;
    _integrator_roll = _integrator_roll + (rotation_vector_error.x * _dt);
    _integrator_pitch = _integrator_pitch + (rotation_vector_error.y * _dt);
    _integrator_yaw = _integrator_pitch + (rotation_vector_error.z * _dt);
    _integrator_roll = constrain_float(_integrator_roll, -_integrator_windup, _integrator_windup);
    _integrator_pitch = constrain_float(_integrator_pitch, -_integrator_windup, _integrator_windup);
    _integrator_yaw = constrain_float(_integrator_yaw, -_integrator_windup, _integrator_windup);
    torques.x = - _kp_roll * rotation_vector_error.x - _kd_roll * ang_vel_error.x - _ki_roll * _integrator_roll;
    torques.y = - _kp_pitch * rotation_vector_error.y - _kd_pitch * ang_vel_error.y - _ki_pitch * _integrator_pitch;
    torques.z = - _kp_yaw * rotation_vector_error.z - _kd_yaw * ang_vel_error.z - _ki_yaw * _integrator_yaw;

    AP::logger().Write("ZQBD", "TimeUS,q1,q2,q3,q4", "Qffff", 
        AP_HAL::micros64(), 
        attitude_body.q1, 
        attitude_body.q2, 
        attitude_body.q3, 
        attitude_body.q4);

    AP::logger().Write("ZQTG", "TimeUS,q1,q2,q3,q4", "Qffff", 
        AP_HAL::micros64(), 
        attitude_target.q1, 
        attitude_target.q2, 
        attitude_target.q3, 
        attitude_target.q4);

    AP::logger().Write("ZQER", "TimeUS,q1,q2,q3,q4", "Qffff", 
                       AP_HAL::micros64(), 
                       q_error.q1, 
                       q_error.q2, 
                       q_error.q3, 
                       q_error.q4);

    AP::logger().Write("ZWBD", "TimeUS,x,y,z", "Qfff", 
        AP_HAL::micros64(), 
        gyro.x, 
        gyro.y, 
        gyro.z);

    AP::logger().Write("ZWTG", "TimeUS,x,y,z", "Qfff", 
        AP_HAL::micros64(), 
        target_ang_vel.x, 
        target_ang_vel.y, 
        target_ang_vel.z);

    AP::logger().Write("ZINT", "TimeUS,x,y,z", "Qfff",
        AP_HAL::micros64(), 
        _integrator_roll,
        _integrator_pitch,
        _integrator_yaw);

    AP::logger().Write("ZTOR", "TimeUS,x,y,z", "Qfff",
        AP_HAL::micros64(), 
        torques.x,
        torques.y,
        torques.z);
    

    // Set thrust from the throttle input (this comes from throttle control)
    // This uses the existing throttle system in the vehicle
    
    // Return torque outputs as roll, pitch, yaw commands to be sent to motors
    // Note: This will be scaled inside the AC_CustomControl framework
    return torques;
}

#endif  // AP_CUSTOMCONTROL_ENABLED