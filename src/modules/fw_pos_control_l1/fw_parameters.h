#ifndef FW_PARAMETERS_H
#define FW_PARAMETERS_H

struct fw_parameters {
    float l1_period;
    float l1_damping;

    float time_const;
    float time_const_throt;
    float min_sink_rate;
    float max_sink_rate;
    float max_climb_rate;
    float climbout_diff;
    float heightrate_p;
    float heightrate_ff;
    float speedrate_p;
    float throttle_damp;
    float integrator_gain;
    float vertical_accel_limit;
    float height_comp_filter_omega;
    float speed_comp_filter_omega;
    float roll_throttle_compensation;
    float speed_weight;
    float pitch_damping;

    float airspeed_min;
    float airspeed_trim;
    float airspeed_max;

    float pitch_limit_min;
    float pitch_limit_max;
    float roll_limit;
    float throttle_min;
    float throttle_max;
    float throttle_cruise;
    float throttle_slew_max;

    float throttle_land_max;

    float land_slope_angle;
    float land_H1_virt;
    float land_flare_alt_relative;
    float land_thrust_lim_alt_relative;
    float land_heading_hold_horizontal_distance;
    float land_flare_pitch_min_deg;
    float land_flare_pitch_max_deg;
    int land_use_terrain_estimate;
};

#endif // FW_PARAMETERS_H

