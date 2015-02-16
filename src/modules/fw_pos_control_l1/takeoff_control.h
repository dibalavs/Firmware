#ifndef TAKEOFF_CONTROL_H
#define TAKEOFF_CONTROL_H

#include "launchdetection/LaunchDetector.h"
#include <ecl/l1/ecl_l1_pos_controller.h>

struct vehicle_attitude_setpoint_s;
struct fw_parameters;

class TakeOffControl
{
public:
    TakeOffControl(ECL_L1_Pos_Controller &l1, struct fw_parameters &param);
    void init();

    bool is_takeoff_complete();

    void update_parameters();

    void control(struct vehicle_attitude_setpoint_s &_att_sp,
                 const struct position_setpoint_triplet_s &pos_sp_triplet,
                 const struct vehicle_global_position_s &_global_pos);

    void reset();

    float get_throttle();

    bool has_launchdetector();

    bool is_launchdetector_engines_started();

private:
    /* takeoff/launch states */
    launchdetection::LaunchDetectionResult launch_detection_state;

    /* Launch detection */
    launchdetection::LaunchDetector launchDetector;

    ECL_L1_Pos_Controller &_l1_control;
    struct fw_parameters &_parameters;
};

#endif // TAKEOFF_CONTROL_H

