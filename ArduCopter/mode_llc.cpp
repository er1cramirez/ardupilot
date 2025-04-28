#include "Copter.h"

bool ModeLLC::init(bool ignore_checks)
{
    // Initialize position controller for Z axis if not already active
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // Set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
}

void ModeLLC::run()
{
    // Set desired neutral attitude (null quaternion)
    Quaternion target_attitude;
    target_attitude.initialise(); // This creates identity quaternion (no rotation)
    // Set zero angular velocity
    Vector3f target_ang_vel(0.0f, 0.0f, 0.0f);

 
    // Handle motor spool states
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();  
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_z_controller(0.0f);
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();

        pos_control->relax_z_controller(0.0f);
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // Flying - run quaternion controller
        attitude_control->input_quaternion(target_attitude, target_ang_vel);

        pos_control->set_alt_target_with_slew(100.0f);
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Do nothing
        break;
    }
    // Set constant throttle for hover
    // attitude_control->set_throttle_out(T, true, g.throttle_filt);
    // Set throttle directly to the motors
    // motors->set_throttle(0.35f);
    pos_control->update_z_controller();
}

bool ModeLLC::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_FORCE_VECTOR_TARGET: {
            // Verificar que el mensaje es para este sistema
            mavlink_force_vector_target_t packet;
            mavlink_msg_force_vector_target_decode(&msg, &packet);
            
            if (packet.target_system != g.sysid_this_mav) {
                break;
            }
            
            // Actualizar el vector de fuerza
            _force_target.x = packet.force_x;
            _force_target.y = packet.force_y;
            _force_target.z = packet.force_z;
            _force_target_derivative.x = packet.force_derivative_x;
            _force_target_derivative.y = packet.force_derivative_y;
            _force_target_derivative.z = packet.force_derivative_z;

            AP::logger().Write("ZYXW", "TimeUS,fx,fy,fz,fxd,fyd,fzd", "Qffffff", 
                AP_HAL::micros64(), 
                (float)_force_target.x, 
                (float)_force_target.y, 
                (float)_force_target.z, 
                (float)_force_target_derivative.x, 
                (float)_force_target_derivative.y, 
                (float)_force_target_derivative.z);

            gcs().send_text(MAV_SEVERITY_INFO, "Force vector target received");

            Quaternion bodyQuaternion;
            ahrs.get_quat_body_to_ned(bodyQuaternion);
            bodyQuaternion.normalize();
            _force_target = bodyQuaternion * _force_target;
            _force_target.z += -0.03351f*9.81f;
            
            _have_new_force_target = true;
            _last_force_target_ms = AP_HAL::millis();
            return true;
        }
    }
    return false;
}
