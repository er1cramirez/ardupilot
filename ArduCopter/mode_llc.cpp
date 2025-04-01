#include "Copter.h"

#if MODE_LLC_ENABLED

ModeLLC::SubMode ModeLLC::llc_mode = SubMode::FailSafe;
bool ModeLLC::takeoff_complete = false;

bool ModeLLC::init(bool ignore_checks)
{
    // Start in position hold mode until we receive a takeoff command or force commands
    pos_hold_start();
    
    return true;
}


void ModeLLC::run()
{
    // Check if we should transition to a different mode based on current state
    // For example, if we have force data, transition to VelControl mode
    if (llc_mode == SubMode::PosHold && _have_new_force_target) {
        vel_control_start();
    }
    
    // If we're in VelControl but haven't received force data for a while, 
    // consider falling back to PosHold
    if (llc_mode == SubMode::VelControl && 
        (AP_HAL::millis() - _last_force_target_ms > 2000)) {
        gcs().send_text(MAV_SEVERITY_INFO, "LLC: No force data, reverting to position hold");
        pos_hold_start();
    }

    // Run the appropriate controller based on current state
    switch (llc_mode) {
        case SubMode::TakeOff:
            // Run takeoff controller
            takeoff_run();
            break;
        case SubMode::VelControl:
            // Run velocity control
            vel_control_run();
            break;
        case SubMode::PosHold:
            // Run position hold
            pos_hold_run();
            break;
        case SubMode::FailSafe:
            // In failsafe, transition to position hold
            pos_hold_start();
            break;
    }
}

void ModeLLC::takeoff_run()
{
    auto_takeoff.run();
    gcs().send_text(MAV_SEVERITY_INFO, "LLC: Takeoff in progress");
    if (auto_takeoff.complete && !takeoff_complete) {
        takeoff_complete = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Takeoff complete");
        llc_mode = SubMode::PosHold;
#if AP_FENCE_ENABLED
        copter.fence.auto_enable_fence_after_takeoff();
#endif
    }
}

void ModeLLC::vel_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    
    // Run velocity controller
    if (_have_new_force_target) { 
        if (refThrottle > 1.0f) {
            gcs().send_text(MAV_SEVERITY_ERROR, "refThrottle > 1.0f");
            refThrottle = 0.7f;
        }
        attitude_control->input_quaternion(refQuaternion, refAngularVelocity);
        attitude_control->set_throttle_out(refThrottle, false, g.throttle_filt);
        _have_new_force_target = false;
    }
    // Get position, velocity and acceleration data
    if(ahrs.get_relative_position_NED_home(_position) && ahrs.get_velocity_NED(_velocity)) 
    {   
        // float z_offset = 0.3f; // Offset for z position
        // x.z = x.z + z_offset; // Add offset to z position
        _acceleration = ahrs.get_accel_ef(); // Acceleration in NED inertial frame
        AP::logger().Write("VLCL",
            "TimeUS,u_d.x,u_d.y,u_d.z,u_d_dot.x,u_d_dot.y,u_d_dot.z",
            "Qffffff",
                AP_HAL::micros64(),
                (float)_force_target.x,
                (float)_force_target.y,
                (float)_force_target.z,
                (float)_force_target_derivative.x,
                (float)_force_target_derivative.y,
                (float)_force_target_derivative.z);
    }
}

void ModeLLC::vel_control_start()
{
    // set to velocity control mode
    llc_mode = SubMode::VelControl;

    // initialise velocity controller
    refQuaternion.initialise(); // This creates identity quaternion (no rotation)
    // Set zero angular velocity
    refAngularVelocity.zero();
    // Set zero thrust
    refThrottle = 0.039*9.81f;
    gcs().send_text(MAV_SEVERITY_INFO, "entering_LLC");
}

void ModeLLC::pos_hold_run()
{
    if (_have_new_force_target && llc_mode != SubMode::VelControl && !is_taking_off()) {
        vel_control_start();//function to initialise velocity control
    }
    // Set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
                
    // Run position controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();
    
    // Call attitude controller
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

void ModeLLC::pos_hold_start()
{
    // set to position hold mode
    llc_mode = SubMode::PosHold;

    // initialise position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
        pos_control->init_xy_controller();
    }
    
    // Set a clear message
    gcs().send_text(MAV_SEVERITY_INFO, "LLC: Position hold active");
}


bool ModeLLC::do_user_takeoff_start(float takeoff_alt_cm)
{
    // calculate target altitude and frame (either alt-above-ekf-origin or alt-above-terrain)
    int32_t alt_target_cm;
    bool alt_target_terrain = false;
#if AP_RANGEFINDER_ENABLED
    if (wp_nav->rangefinder_used_and_healthy() &&
        wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER &&
        takeoff_alt_cm < 1000) {  // Use a fixed value instead of accessing protected member
        // can't takeoff downwards
        if (takeoff_alt_cm <= copter.rangefinder_state.alt_cm) {
            return false;
        }
        // provide target altitude as alt-above-terrain
        alt_target_cm = takeoff_alt_cm;
        alt_target_terrain = true;
    } else
#endif
    {
        // interpret altitude as alt-above-home
        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);

        // provide target altitude as alt-above-ekf-origin
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
            // this should never happen but we reject the command just in case
            return false;
        }
    }

    // Switch to takeoff mode - this is key
    llc_mode = SubMode::TakeOff;
    
    gcs().send_text(MAV_SEVERITY_INFO, "LLC: Takeoff started");
    
    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // clear i term when we're taking off
    pos_control->init_z_controller();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff.start(alt_target_cm, alt_target_terrain);
    
    // record takeoff has not completed
    takeoff_complete = false;

    return true;
}


bool ModeLLC::is_taking_off() const
{
    return llc_mode == SubMode::TakeOff && !takeoff_complete;
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

            // calculate_virtual_control(_force_target, _force_target_derivative, 0.0f,
            //     refThrottle, 0.0f, refQuaternion, refAngularVelocity);
            calculateVirtualMap(_force_target, _force_target_derivative, 0.0f,
                0.0f, refQuaternion, refAngularVelocity);
            _have_new_force_target = true;
            _last_force_target_ms = AP_HAL::millis();
            
            return true;
        }
    }
    return false;
}

/**
 * Calculates orientation control parameters for a quadrotor controller
 * 
 * @param u_d Desired thrust vector
 * @param u_dot_d Derivative of the desired thrust vector
 * @param psi_d Desired yaw angle in radians
 * @param psi_dot_d Derivative of the desired yaw angle in radians
 * @param refQuat Output reference quaternion
 * @param refOmega Output reference angular velocity
 * @return Thrust magnitude (norm of u_d)
 */
void ModeLLC::calculateVirtualMap(const Vector3f& u_d, const Vector3f& u_dot_d, 
    float psi_d, float psi_dot_d,
    Quaternion& refQuat, Vector3f& refOmega) {
    // Constants to avoid numerical issues
    constexpr float MIN_NORM = 4.6416e-04f;
    constexpr float MIN_DENOMINATOR = 1e-6f;
    
    // Calculate thrust magnitude (norm of u_d)
    float thrust = u_d.length();
    
    // Check for zero thrust
    if (thrust < MIN_NORM) {
        // For very small thrusts, maintain current orientation but zero angular velocity
        thrust = MIN_NORM;
        refOmega.zero();
        refThrottle = thrust;
        return;
    }
    
    // Calculate unit vector of thrust direction
    Vector3f uu = u_d / thrust;
    
    // Calculate the dot product for derivative calculation
    float u_dot = u_d.dot(u_dot_d);
    float thrust_cubed = thrust * thrust * thrust;
    
    // Calculate the derivative of the unit vector
    // uup = u_dot_d/norm_u - u_d*(u_d·u_dot_d)/norm_u^3
    Vector3f uup = u_dot_d / thrust - u_d * (u_dot / thrust_cubed);
    
    // Precalculate values for quaternion and angular velocity with safety checks
    float one_minus_uuz = 1.0f - uu.z;
    one_minus_uuz = one_minus_uuz < MIN_DENOMINATOR ? MIN_DENOMINATOR : one_minus_uuz;
    
    float u_3 = sqrtf(2.0f * one_minus_uuz);
    float inv_u_3 = 1.0f / u_3;
    float inv_one_minus_uuz = 1.0f / one_minus_uuz;
    
    // Precalculate trigonometric values
    float half_psi = psi_d * 0.5f;
    float cos_half_psi = cosf(half_psi);
    float sin_half_psi = sinf(half_psi);
    float cos_psi = cosf(psi_d);
    float sin_psi = sinf(psi_d);
    
    // Calculate the desired quaternion
    // Note: Adapt these component names to match your Quaternion class
    // Here I'm using the convention: w,x,y,z components
    refQuat.q1 = u_3 * cos_half_psi * 0.5f;  // or q1 depending on your implementation
    refQuat.q2 = (-uu.x * sin_half_psi + uu.y * cos_half_psi) * inv_u_3;  // or q2
    refQuat.q3 = (-uu.x * cos_half_psi - uu.y * sin_half_psi) * inv_u_3;  // or q3
    refQuat.q4 = sin_half_psi * u_3 * 0.5f;  // or q4
    
    // Ensure quaternion is normalized
    refQuat.normalize();
    
    // Calculate the desired angular velocity with minimal terms
    float term1 = uu.x * sin_psi - uu.y * cos_psi;
    float term2 = uu.x * cos_psi + uu.y * sin_psi;
    float term3 = uu.x * uup.y - uu.y * uup.x;
    
    refOmega.x = -uup.x * sin_psi + uup.y * cos_psi + uup.z * term1 * inv_one_minus_uuz;
    refOmega.y = -uup.x * cos_psi - uup.y * sin_psi + uup.z * term2 * inv_one_minus_uuz;
    refOmega.z = psi_dot_d - term3 * inv_one_minus_uuz;

    refThrottle = thrust;
}

#endif // MODE_LLC_ENABLED