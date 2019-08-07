#include "mode.h"
#include "Plane.h"

bool ModeManualPlus::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;
//    plane.airspeed.enabled() = true;

    return true;
}

void ModeManualPlus::_exit()
{
    if (plane.g.auto_trim > 0) {
        plane.trim_radio();
    }
}

void ModeManualPlus::update()
{
    // Scale servo end points based on airspeed: plane.smoothed_airspeed
    // goal is:
    //   0m/s to FBW_MIN is 100% of the endpoint
    //   FBW_MIN to 1/4 between FBW_MIN and FBW_MAX is 100% of the end point
    //   FBW_MIN + (1/4 between FBW_MIN and FBW_MAX) gets scaled to 50% of the end point at FBW_MAX
    //   FBW_MAX and above remains at 50% the max endpoint
    // Consider the following: 
    //   FBW_MIN is 12m/s and FBW_MAX is 24m/s
    //   Roll and Pitch max endpoints would have full range up to 15m/s
    //   At 15m/s, Pitch and Roll servo_min and servo_max values would be scaled from 100% to 50% at FBW_MAX
    //   At FBW_MAX, Pitch and Roll servo_min and servo_max values would be at 50%, regardless of speed gt FBW_MAX
    // Would be cool to have params to define different scales for Pitch and Roll, will consider/research 

    //float mplus_scale = smoothed_airspeed / MAX(aparm.airspeed_min, 1);

    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
    plane.steering_control.steering = plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();
}

