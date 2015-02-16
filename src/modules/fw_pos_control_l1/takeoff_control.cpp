#include "takeoff_control.h"
#include <mavlink/mavlink_log.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include "fw_parameters.h"
#include "mtecs/mTecs.h"

using namespace launchdetection;

TakeOffControl::TakeOffControl(ECL_L1_Pos_Controller &l1, struct fw_parameters &param)
  :launch_detection_state(LAUNCHDETECTION_RES_NONE)
  , launchDetector()
  , _l1_control(l1)
  , _parameters(param)
{

}

void TakeOffControl::init()
{

}

void TakeOffControl::control(struct vehicle_attitude_setpoint_s &_att_sp,
                             const position_setpoint_triplet_s &pos_sp_triplet,
                             const struct vehicle_global_position_s &_global_pos)
{
    //mavlink_log_critical(_mavlink_fd, "Takeoff ");
    /* Perform launch detection */
    if (launchDetector.launchDetectionEnabled() &&
           launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {
       /* Inform user that launchdetection is running */
       static hrt_abstime last_sent = 0;
       if(hrt_absolute_time() - last_sent > 4e6) {
           mavlink_log_info(_mavlink_fd, "#audio: Launchdetection running");
           last_sent = hrt_absolute_time();
       }

       /* Detect launch */
       launchDetector.update(_sensor_combined.accelerometer_m_s2[0]);

       /* update our copy of the laucn detection state */
       launch_detection_state = launchDetector.getLaunchDetected();
    } else	{
       /* no takeoff detection --> fly */
       launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
    }

    /* Set control values depending on the detection state */
    if (launch_detection_state != LAUNCHDETECTION_RES_NONE) {
       /* Launch has been detected, hence we have to control the plane. */

       _l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);
       _att_sp.roll_body = _l1_control.nav_roll();
       _att_sp.yaw_body = _l1_control.nav_bearing();
       //warnx("Takeoff params:  nav_roll  %.2f, naw_yaw %.2f",
       //      (double)_att_sp.roll_body, (double)_att_sp.yaw_body);
       //mavlink_log_critical(_mavlink_fd, "Takeoff params:  nav_roll  %.2f, naw_yaw %.2f",
       //                     (double)_att_sp.roll_body, (double)_att_sp.yaw_body);

       /* Select throttle: only in LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS we want to use
        * full throttle, otherwise we use the preTakeOff Throttle */
       float takeoff_throttle = launch_detection_state !=
           LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS ?
           launchDetector.getThrottlePreTakeoff() : _parameters.throttle_max;

       /* select maximum pitch: the launchdetector may impose another limit for the pitch
        * depending on the state of the launch */
       float takeoff_pitch_max_deg = launchDetector.getPitchMax(_parameters.pitch_limit_max);
       float takeoff_pitch_max_rad = math::radians(takeoff_pitch_max_deg);
    } else {
       /* Tell the attitude controller to stop integrating while we are waiting
        * for the launch */
       _att_sp.roll_reset_integral = true;
       _att_sp.pitch_reset_integral = true;
       _att_sp.yaw_reset_integral = true;

       /* Set default roll and pitch setpoints during detection phase */
       _att_sp.roll_body = 0.0f;
       _att_sp.pitch_body = math::max(math::radians(pos_sp_triplet.current.pitch_min),
               math::radians(10.0f));
    }
}

bool TakeOffControl::is_takeoff_complete()
{

}

void TakeOffControl::update_parameters()
{
    /* Update Launch Detector Parameters */
    launchDetector.updateParams();
}

void TakeOffControl::reset()
{
    launch_detection_state = LAUNCHDETECTION_RES_NONE;
    launchDetector.reset();
}

float TakeOffControl::get_throttle()
{
    return launchDetector.getThrottlePreTakeoff();
}

bool TakeOffControl::has_launchdetector()
{
    return launch_detection_state != LAUNCHDETECTION_RES_NONE;
}

bool TakeOffControl::is_launchdetector_engines_started()
{
    return launch_detection_state == LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
}
