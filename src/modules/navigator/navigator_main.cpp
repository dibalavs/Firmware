/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	Lorenz Meier
 *              Jean Cyr
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file navigator_main.c
 * Implementation of the main navigation state machine.
 *
 * Handles missions, geo fencing and failsafe navigation behavior.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_set_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/fence.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

/**
 * navigator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int navigator_main(int argc, char *argv[]);

class Navigator
{
public:
	/**
	 * Constructor
	 */
	Navigator();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~Navigator();

	/**
	 * Start the navigator task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display the navigator status.
	 */
	void		status();

	/**
	 * Load fence parameters.
	 */
	void		load_fence(const char *file_name);

	/**
	 * Save fence parameters.
	 */
	void		save_fence(const char *file_name);

	/**
	 * Specify fence vertex parameter.
	 */
	void		fence_point(int argc, char *argv[]);

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_navigator_task;			/**< task handle for sensor task */

	int		_global_pos_sub;
	int		_att_sub;			/**< vehicle attitude subscription */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_vstatus_sub;			/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_control_sub;		/**< notification of manual control updates */
	int		_mission_sub;
	int		_fence_sub;
	int		_fence_pub;

	orb_advert_t	_triplet_pub;			/**< position setpoint */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct airspeed_s				_airspeed;		/**< airspeed */
	struct vehicle_status_s				_vstatus;		/**< vehicle status */
	struct vehicle_global_position_s		_global_pos;		/**< global vehicle position */
	struct vehicle_global_position_set_triplet_s	_global_triplet;	/**< triplet of global setpoints */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	unsigned	_mission_items_maxcount;				/**< maximum number of mission items supported */
	struct	mission_item_s 				* _mission_items;	/**< storage for mission items */
	bool		_mission_valid;						/**< flag if mission is valid */

	struct	fence_s 				_fence;			/**< storage for fence vertices */
	bool						_fence_valid;		/**< flag if fence is valid */
	bool						_inside_fence;		/**< vehicle is inside fence */	
	bool						_fence_needs_save;	/**< save fence at next opportunity */

	/** manual control states */
	float		_seatbelt_hold_heading;		/**< heading the system should hold in seatbelt mode */
	float		_loiter_hold_lat;
	float		_loiter_hold_lon;
	float		_loiter_hold_alt;
	bool		_loiter_hold;

	struct {
		float throttle_cruise;
	}		_parameters;			/**< local copies of interesting parameters */

	struct {
		param_t throttle_cruise;

	}		_parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle status.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for position updates.
	 */
	void		vehicle_attitude_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		mission_poll();

	/**
	 * Control throttle.
	 */
	float		control_throttle(float energy_error);

	/**
	 * Control pitch.
	 */
	float		control_pitch(float altitude_error);

	void calculate_airspeed_errors();
	void calculate_gndspeed_undershoot();
	void calculate_altitude_error();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));

	void		publish_fence(const struct fence_s *fence);

	bool		fence_valid(const struct fence_s *fence);
};

namespace navigator
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

Navigator	*g_navigator;
}

Navigator::Navigator() :

	_task_should_exit(false),
	_navigator_task(-1),

/* subscriptions */
	_global_pos_sub(-1),
	_att_sub(-1),
	_airspeed_sub(-1),
	_vstatus_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),
	_fence_sub(-1),
	_fence_pub(-1),

/* publications */
	_triplet_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),
/* states */
	_mission_items_maxcount(20),
	_mission_valid(false),
	_loiter_hold(false),
	_fence_valid(false),
	_inside_fence(true),
	_fence_needs_save(false)
{
	_global_pos.valid = false;
	memset(&_fence, 0, sizeof(_fence));
	_mission_items = (mission_item_s*)malloc(sizeof(mission_item_s) * _mission_items_maxcount);
	if (!_mission_items) {
		_mission_items_maxcount = 0;
		warnx("no free RAM to allocate mission, rejecting any waypoints");
	}

	_parameter_handles.throttle_cruise = param_find("NAV_DUMMY");

	/* fetch initial values */
	parameters_update();
}

Navigator::~Navigator()
{
	if (_navigator_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_navigator_task);
				break;
			}
		} while (_navigator_task != -1);
	}

	navigator::g_navigator = nullptr;
}

int
Navigator::parameters_update()
{

	//param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));

	return OK;
}

void
Navigator::vehicle_status_poll()
{
	bool vstatus_updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_vstatus_sub, &vstatus_updated);

	if (vstatus_updated)
		orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus);
}

void
Navigator::vehicle_attitude_poll()
{
	/* check if there is a new position */
	bool att_updated;
	orb_check(_att_sub, &att_updated);

	if (att_updated)
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);
}

void
Navigator::mission_poll()
{
	/* check if there is a new setpoint */
	bool mission_updated;
	orb_check(_mission_sub, &mission_updated);

	if (mission_updated) {

		struct mission_s mission;
		orb_copy(ORB_ID(mission), _mission_sub, &mission);

		// XXX this is not optimal yet, but a first prototype /
		// test implementation

		if (mission.count <= _mission_items_maxcount) {
			/*
			 * Perform an atomic copy & state update
			 */
			irqstate_t flags = irqsave();

			memcpy(_mission_items, mission.items, mission.count * sizeof(struct mission_item_s));
			_mission_valid = true;

			irqrestore(flags);
		} else {
			warnx("mission larger than storage space");
		}
	}
}

void
Navigator::task_main_trampoline(int argc, char *argv[])
{
	navigator::g_navigator->task_main();
}

void
Navigator::task_main()
{

	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_mission_sub = orb_subscribe(ORB_ID(mission));
	_fence_sub = orb_subscribe(ORB_ID(fence));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	// Load initial states
	if (orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus) != OK)
		_vstatus.arming_state = ARMING_STATE_STANDBY; // for testing... commander may not be running
	orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);
	struct mission_s mission;
	orb_copy(ORB_ID(mission), _mission_sub, &mission);
	if (mission.count <= _mission_items_maxcount) {
		/*
		 * Perform an atomic copy & state update
		 */
		irqstate_t flags = irqsave();
		memcpy(_mission_items, mission.items, mission.count * sizeof(struct mission_item_s));
		_mission_valid = true;
		irqrestore(flags);
	}

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vstatus_sub, 200);
	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	parameters_update();

	load_fence(nullptr);

	/* wakeup source(s) */
	struct pollfd fds[3];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _fence_sub;
	fds[2].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* check vehicle status for changes to publication state */
		vehicle_status_poll();

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only update fence if it has changed */
		if (fds[2].revents & POLLIN) {
			/* read from fence to clear updated flag */
			struct fence_s fence;
			orb_copy(ORB_ID(fence), _fence_sub, &fence);
			if (fence_valid(&fence)) {
				// Only update and save if it has changed
				unsigned len = sizeof(fence.count) + (fence.count * sizeof(struct fence_item_s));
				if (memcmp(&fence, &_fence, len)) {
					memcpy(&_fence, &fence, len);
					_fence_valid = _fence.count != 0;
					_fence_needs_save = true;
				}
			}
		}

		/* only run controller if position changed */
		if (fds[1].revents & POLLIN) {


			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

			vehicle_attitude_poll();

			mission_poll();

			if (_fence_valid && _global_pos.valid) {
				_inside_fence = inside_geofence(&_global_pos, &_fence);
			}

			math::Vector2f ground_speed(_global_pos.vx, _global_pos.vy);
			// Current waypoint
			math::Vector2f next_wp(_global_triplet.current.lat / 1e7f, _global_triplet.current.lon / 1e7f);
			// Global position
			math::Vector2f current_position(_global_pos.lat / 1e7f, _global_pos.lon / 1e7f);

			/* AUTONOMOUS FLIGHT */

			if (1 /* autonomous flight */) {

				/* execute navigation once we have a setpoint */
				if (_mission_valid) {

					// Next waypoint
					math::Vector2f prev_wp;

					if (_global_triplet.previous_valid) {
						prev_wp.setX(_global_triplet.previous.lat / 1e7f);
						prev_wp.setY(_global_triplet.previous.lon / 1e7f);

					} else {
						/*
						 * No valid next waypoint, go for heading hold.
						 * This is automatically handled by the L1 library.
						 */
						prev_wp.setX(_global_triplet.current.lat / 1e7f);
						prev_wp.setY(_global_triplet.current.lon / 1e7f);

					}



					/********   MAIN NAVIGATION STATE MACHINE   ********/

					// XXX to be put in its own class

					if (_global_triplet.current.nav_cmd == NAV_CMD_WAYPOINT) {
						/* waypoint is a plain navigation waypoint */
						

					} else if (_global_triplet.current.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
						_global_triplet.current.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
						_global_triplet.current.nav_cmd == NAV_CMD_LOITER_UNLIMITED) {

						/* waypoint is a loiter waypoint */
						
					}

					// XXX at this point we always want no loiter hold if a
					// mission is active
					_loiter_hold = false;

				} else {

					if (!_loiter_hold) {
						_loiter_hold_lat = _global_pos.lat / 1e7f;
						_loiter_hold_lon = _global_pos.lon / 1e7f;
						_loiter_hold_alt = _global_pos.alt;
						_loiter_hold = true;
					}

					//_parameters.loiter_hold_radius
				}

			} else if (0/* seatbelt mode enabled */) {

				/** SEATBELT FLIGHT **/
				continue;

			} else {

				/** MANUAL FLIGHT **/

				/* no flight mode applies, do not publish an attitude setpoint */
				continue;
			}

			/********   MAIN NAVIGATION STATE MACHINE   ********/

			if (_global_triplet.current.nav_cmd == NAV_CMD_RETURN_TO_LAUNCH) {
				// XXX define launch position and return

			} else if (_global_triplet.current.nav_cmd == NAV_CMD_LAND) {
				// XXX flared descent on final approach

			} else if (_global_triplet.current.nav_cmd == NAV_CMD_TAKEOFF) {

				/* apply minimum pitch if altitude has not yet been reached */
				if (_global_pos.alt < _global_triplet.current.altitude) {
					_att_sp.pitch_body = math::max(_att_sp.pitch_body, _global_triplet.current.param1);
				}
			}

			/* lazily publish the setpoint only once available */
			if (_triplet_pub > 0) {
				/* publish the attitude setpoint */
				orb_publish(ORB_ID(vehicle_global_position_set_triplet), _triplet_pub, &_global_triplet);

			} else {
				/* advertise and publish */
				_triplet_pub = orb_advertise(ORB_ID(vehicle_global_position_set_triplet), &_global_triplet);
			}

		}

		perf_end(_loop_perf);

		// Only do file io if in standby mode
		if (_fence_needs_save && (_vstatus.arming_state == ARMING_STATE_STANDBY)) {
			_fence_needs_save = false;
			save_fence(nullptr);
		}
	}

	warnx("exiting.");

	_navigator_task = -1;
	_exit(0);
}

int
Navigator::start()
{
	ASSERT(_navigator_task == -1);

	/* start the task */
	_navigator_task = task_spawn_cmd("navigator",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&Navigator::task_main_trampoline,
				       nullptr);

	if (_navigator_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
Navigator::status()
{
	warnx("Global position is %svalid", _global_pos.valid ? "" : "in");
	if (_global_pos.valid) {
		warnx("Longitude %5.5f degrees, latitude %5.5f degrees", _global_pos.lon / 1e7, _global_pos.lat / 1e7);
		warnx("Altitude %5.5f meters, altitude above home %5.5f meters",
			(double)_global_pos.alt, (double)_global_pos.relative_alt);
		warnx("Ground velocity in m/s, x %5.5f, y %5.5f, z %5.5f",
			(double)_global_pos.vx, (double)_global_pos.vy, (double)_global_pos.vz);
		warnx("Compass heading in degrees %5.5f", (double)_global_pos.yaw * 57.2957795);
	}
	if (_fence_valid) {
		warnx("Geofence is valid");
		warnx("Vertex longitude latitude");
		for (unsigned i = 0; i < _fence.count; i++) {
			const char *ret_point = "";
			if (i == 0)
				ret_point = "(return point)";
			warnx("%6u %9.5f %8.5f %s", i, (double)_fence.items[i].lon, (double)_fence.items[i].lat, ret_point);
		}
		if (_global_pos.valid)
			warnx("Craft is %sside fence", _inside_fence ? "in" : "out");
	} else
		warnx("Geofence not set");
}

void
Navigator::publish_fence(const struct fence_s *fence)
{
	if (_fence_pub == -1)
		_fence_pub = orb_advertise(ORB_ID(fence), fence);
	else
		orb_publish(ORB_ID(fence), _fence_pub, fence);
}

bool
Navigator::fence_valid(const struct fence_s *fence)
{
	struct vehicle_global_position_s pos;
	// NULL fence is valid
	if (fence->count == 0)
		return true;
	// Otherwise
	if ((fence->count < 4) || (fence->count > GEOFENCE_MAX_VERTICES)) {
		warnx("Fence must have at least 3 sides and not more than %d", GEOFENCE_MAX_VERTICES - 1);
		return false;
	}
	pos.lat = (int)(fence->items[0].lat * 1e7f);
	pos.lon = (int)(fence->items[0].lon * 1e7f);
	if (!inside_geofence(&pos, fence)) {
		warnx("Return point must be inside fence");
		return false;
	}
	return true;
}

void
Navigator::load_fence(const char *fname)
{
	const char *file_name = GEOFENCE_DEFAULT_FILE_NAME;
	int fence_file, result;
	struct fence_s fence;

	if (fname)
		file_name = fname;

	warnx("Loading fence parameters from '%s'", file_name);
	fence_file = open(file_name, O_RDONLY);
	if (fence_file < 0)
		errx(1, "Could not open file '%s'", file_name);

	result = read(fence_file, &fence, sizeof(fence));

	close(fence_file);

	if (result < sizeof(fence.count))
		errx(1, "error reading file");
	else if (result < (sizeof(fence.count) + (fence.count * sizeof(struct fence_item_s))))
		errx(1, "error reading file");

	if (fence_valid(&fence)) {
		memcpy(&_fence, &fence, sizeof(_fence));
		_fence_valid = _fence.count != 0;
	}
	else
		errx(1, "Invalid fence loaded, ignored!\n");
}

void
Navigator::save_fence(const char *fname)
{
	const char *file_name = GEOFENCE_DEFAULT_FILE_NAME;
	int fence_file, result;
	unsigned ix;

	if (!fence_valid(&_fence))
		errx(1, "Not saving invalid fence");

	if (fname)
		file_name = fname;

	warnx("Saving fence parameters to '%s'", file_name);
	/* delete the parameter file in case it exists */
	unlink(file_name);
	fence_file = open(file_name,  O_WRONLY | O_CREAT | O_EXCL);
	if (fence_file < 0)
		errx(1, "opening '%s' failed", file_name);

	int len = sizeof(_fence.count) + (_fence.count * sizeof(struct fence_item_s));

	result = write(fence_file, &_fence, len);

	close(fence_file);

	if (result < len) {
		unlink(file_name);
		errx(1, "error saving to '%s'", file_name);
	}
}

void
Navigator::fence_point(int argc, char *argv[])
{
	int ix, num_points;
	double lon, lat;
	char *end;
	static struct fence_s fence;

	if (argc < 4)
		errx(1, "Specify: index num_points latitude longitude");

	ix = atoi(argv[0]);
	num_points = atoi(argv[1]);
	lat = strtod(argv[2], &end);
	lon = strtod(argv[3], &end);
	// Number of points = 0 means clear the fence
	if ((num_points != 0) && ((ix < 0) || (ix >= num_points)))
		errx(1, "Index must be greater than 0 and less than num_points\n");
	fence.count = num_points;
	fence.items[ix].lat = (float)lat;
	fence.items[ix].lon = (float)lon;
	if (num_points == 0)
		publish_fence(&fence);
	else if (ix == (num_points - 1))
		publish_fence(&fence);
}

int navigator_main(int argc, char *argv[])
{
	if (argc < 2)
		errx(1, "usage: navigator {start|stop|status|loadfence|savefence|fencepoint}");

	if (!strcmp(argv[1], "start")) {

		if (navigator::g_navigator != nullptr)
			errx(1, "already running");

		navigator::g_navigator = new Navigator;

		if (navigator::g_navigator == nullptr)
			errx(1, "alloc failed");

		if (OK != navigator::g_navigator->start()) {
			delete navigator::g_navigator;
			navigator::g_navigator = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (navigator::g_navigator == nullptr)
		errx(1, "not running");

	if (!strcmp(argv[1], "stop")) {

		delete navigator::g_navigator;
		navigator::g_navigator = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		navigator::g_navigator->status();
		exit(0);
	}

	if (!strcmp(argv[1], "loadfence")) {
		char *fname = nullptr;
		if (argc > 2)
			fname = argv[2];
		navigator::g_navigator->load_fence(fname);
		exit(0);
	}

	if (!strcmp(argv[1], "savefence")) {
		char *fname = nullptr;
		if (argc > 2)
			fname = argv[2];
		navigator::g_navigator->save_fence(fname);
		exit(0);
	}

	if (!strcmp(argv[1], "fencepoint")) {
		navigator::g_navigator->fence_point(argc - 2, argv + 2);
		exit(0);
	}

	warnx("unrecognized command\nusage: navigator {start|stop|status|loadfence|savefence|fencepoint}");
	return 1;
}
