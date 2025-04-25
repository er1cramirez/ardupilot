#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ENABLED

#include "AC_CustomControl_LLC.h"
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_LLC::var_info[] = {
    // @Param: ROLL_P
    // @DisplayName: Roll P gain
    // @Description: Roll P gain for PD attitude controller
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("ROLL_P", 1, AC_CustomControl_LLC, _kp_roll, 3.8f),

    // @Param: PITCH_P
    // @DisplayName: Pitch P gain
    // @Description: Pitch P gain for PD attitude controller
    // @Range: 0.0 10.0
    // @User: Standard
    AP_GROUPINFO("PITCH_P", 2, AC_CustomControl_LLC, _kp_pitch, 3.8f),

    // @Param: YAW_P
    // @DisplayName: Yaw P gain
    // @Description: Yaw P gain for PD attitude controller
    // @Range: 0.0 10.0
    // @User: Standard  
    AP_GROUPINFO("YAW_P", 3, AC_CustomControl_LLC, _kp_yaw, 3.8f),

    // @Param: ROLL_D
    // @DisplayName: Roll D gain
    // @Description: Roll D gain for PD attitude controller
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("ROLL_D", 4, AC_CustomControl_LLC, _kd_roll, 0.2f),

    // @Param: PITCH_D
    // @DisplayName: Pitch D gain
    // @Description: Pitch D gain for PD attitude controller
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("PITCH_D", 5, AC_CustomControl_LLC, _kd_pitch, 0.2f),

    // @Param: YAW_D
    // @DisplayName: Yaw D gain
    // @Description: Yaw D gain for PD attitude controller
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("YAW_D", 6, AC_CustomControl_LLC, _kd_yaw, 0.2f),

    // @Param: THROTTLE_HOVER
    // @DisplayName: Throttle hover
    // @Description: Throttle hover value for the vehicle
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("THROTTLE_HOVER", 7, AC_CustomControl_LLC, _throttle_hover, 0.5f),

    AP_GROUPEND
};

AC_CustomControl_LLC::AC_CustomControl_LLC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, 
                                     AC_AttitudeControl*& att_control, 
                                     AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_CustomControl_LLC::reset()
{
    // Reset any integrators or other controller states here if needed
}

void AC_CustomControl_LLC::calculate_attitude_error_quaternion(const Quaternion &attitude_body, 
                                                            const Quaternion &attitude_target,
                                                            Quaternion &error_quaternion)
{
    // Calculate the error quaternion between current and target attitude
    // q_error = q_target * q_body^(-1)
    error_quaternion = attitude_target * attitude_body.inverse();
    
    // Ensure shortest rotation path
    error_quaternion.normalize();
}

Vector3f AC_CustomControl_LLC::update()
{
    // Get current attitude as quaternion
    Quaternion attitude_body;
    _ahrs->get_quat_body_to_ned(attitude_body);
    
    // Get target attitude from attitude controller
    Quaternion attitude_target;
    attitude_target = _att_control->get_attitude_target_quat();

    
    // Calculate attitude error quaternion
    Quaternion q_error;
    calculate_attitude_error_quaternion(attitude_body, attitude_target, q_error);

    AP::logger().Write("XQBT", "TimeUS,q1b,q2b,q3b,q4b,q1t,q2t,q3t,q4t", "Qffffffff", 
                       AP_HAL::micros64(), 
                       attitude_body.q1, 
                       attitude_body.q2, 
                       attitude_body.q3, 
                       attitude_body.q4,
                       attitude_target.q1,
                       attitude_target.q2,
                       attitude_target.q3,
                       attitude_target.q4);

    AP::logger().Write("XQER", "TimeUS,q1error,q2error,q3error,q4error", "Qffff", 
                       AP_HAL::micros64(), 
                       q_error.q1, 
                       q_error.q2, 
                       q_error.q3, 
                       q_error.q4);
    
    // Convert quaternion error to rotation vector (roll, pitch, yaw errors)
    // Calculate error quaternion using requested approach
    // Quaternion q_error_new = attitude_body.inverse() * attitude_target;
    
    // Extract the vectorial part as the error
    Vector3f rotation_vector_error(q_error.q2, q_error.q3, q_error.q4);
    
    // Get current angular velocity (gyro data)
    Vector3f gyro = _ahrs->get_gyro();
    
    // Get target angular velocity from attitude controller
    Vector3f target_ang_vel = _att_control->get_attitude_target_ang_vel();
    
    // Calculate angular velocity error
    Vector3f ang_vel_error = target_ang_vel - gyro;
    
    // Apply PD controller gains
    Vector3f torques;
    torques.x = _kp_roll * rotation_vector_error.x + _kd_roll * ang_vel_error.x;
    torques.y = _kp_pitch * rotation_vector_error.y + _kd_pitch * ang_vel_error.y;
    torques.z = _kp_yaw * rotation_vector_error.z + _kd_yaw * ang_vel_error.z;
    
    // Set thrust from the throttle input (this comes from throttle control)
    // This uses the existing throttle system in the vehicle
    
    // Return torque outputs as roll, pitch, yaw commands to be sent to motors
    // Note: This will be scaled inside the AC_CustomControl framework
    return torques;
}

#endif  // AP_CUSTOMCONTROL_ENABLED