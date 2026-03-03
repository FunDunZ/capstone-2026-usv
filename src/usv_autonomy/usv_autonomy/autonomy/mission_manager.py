# ================================================================
# File: mission_manager.py
# Description: High level state machine for adapting mission plan
# Author: Andrew Thomas
# Co-Authors: N/A
# Created: 01-15-2026
# Last Modified: 02-20-2026
# Version: 2.0.0
# Dependencies: buoy_tasks, mission, obstacle_avoidance, object_tracker, math
# Notes:
# ================================================================
from usv_autonomy.autonomy.buoy_tasks import gen_gate_wps
from usv_autonomy.mavlink.mission import mission_upload, set_current_waypoint
from usv_autonomy.autonomy.obstacle_avoidance import ObstacleAvoider, is_obstacle_in_path
from usv_autonomy.autonomy.object_tracker import ObjectTracker
from math import cos, radians, sqrt

TASK_MODE = "GATE"                           # user selection
GO_TO_COLOR = "RED"                          # ^^^
GRID_RESOLUTION = 2.0                        # meters per grid cell for D* Lite
TRACKER_MIN_HITS = 3                         # detections required to confirm a track
TRACKER_STALE_TIMEOUT = 30.0
TRACKER_DELETE_TIMEOUT = 60.0
CLEARANCE_MARGIN = 8.0                      # meters past the obstacle along path

class MissionManager:
    def __init__(self, usv):
        self.usv = usv
        self.current_mission = []
        self.state = "MISSION_FOLLOW"
        self.current_wp = 0                 # index
        self.task_mode = TASK_MODE          # task selection
        self.task_color = GO_TO_COLOR
        self.last_task_wp = None
        self.last_avoid_wp = None
        self.active_gate = None
        self.pos = None
        self.current_wp_coord = None
        self.last_obstacle_check_wp = None
        self.last_obstacle_check_dest = None
        self.known_track_ids = set()
        
        # D* Lite obstacle avoidance
        self.avoider = None
        self.detection_range = 20.0  # meters
        # tracker keeps obstacle memory across updates
        self.object_tracker = ObjectTracker(
            min_hits_to_confirm=TRACKER_MIN_HITS,
            stale_timeout_s=TRACKER_STALE_TIMEOUT,
            delete_timeout_s=TRACKER_DELETE_TIMEOUT,
        )


    def _update(self, vessel_pos, current_wp, detections):
        self.current_wp = current_wp
        self.pos = vessel_pos

        if self.current_wp < len(self.current_mission):
            self.current_wp_coord = self.current_mission[self.current_wp]

        tracked_objects = self.object_tracker.update(detections)
        obstacles = [obj for obj in tracked_objects if obj["type"] == "AVOID"]
        buoys = [obj for obj in detections if obj["type"] == "BUOY"]
        if detections:
            print(f"Detections raw {len(detections)} tracked {len(tracked_objects)}")

        check_obstacles_now = self._should_check_obstacles_now(obstacles)

        if self.state == "AVOID":
            self._check_obstacle_cleared()
            return

        if self.state == "MISSION_FOLLOW":
            self._check_for_overrides(obstacles, buoys, check_obstacles_now)
                
        elif self.state == "GATE":
            self._check_gate_completion()

        self.last_obstacle_check_wp = self.current_wp
        self.last_obstacle_check_dest = self._current_leg_destination_key()

    def _check_for_overrides(self, obstacles, buoys, check_obstacles_now):
        # Priority 1: Obstacle avoidance
        if check_obstacles_now and obstacles and self.current_wp < len(self.current_mission):
            for obs in obstacles:
                if self._obstacle_blocks_current_leg(obs, threshold_meters=10):
                    self._handle_avoidance(obstacles)
                    return

        # Priority 2: Gate handling
        if self.task_mode == "GATE":
            gate = self._gate_builder(buoys)
            if gate:
                self.active_gate = gate
                if self._gate_blocking_path(gate) and self.state != "GATE":
                    self._handle_gate(gate)
                    return

        # Priority 3: Go-to-buoy
        elif self.task_mode == "GO_TO_COLOR":
            target_buoy = self._find_target_buoy(buoys, self.task_color)
            if target_buoy:
                self._handle_go_to_buoy(target_buoy)
                return
            else:
                if buoys:
                    self._handle_avoidance(buoys)

    def _obstacle_blocks_current_leg(self, obstacle, threshold_meters=10):
        # check only vessel to current wp leg
        if self.pos is None or self.current_wp >= len(self.current_mission):
            return False

        return is_obstacle_in_path(
            self.pos,
            self.current_mission[self.current_wp],
            obstacle,
            threshold_meters,
        )

    def _current_leg_destination_key(self):
        if self.current_wp >= len(self.current_mission):
            return None
        wp = self.current_mission[self.current_wp]
        return (wp.get("lat"), wp.get("lon"), wp.get("alt"))

    def _should_check_obstacles_now(self, obstacles):
        leg_changed = self.current_wp != self.last_obstacle_check_wp
        dest_changed = self._current_leg_destination_key() != self.last_obstacle_check_dest

        new_track_detected = False
        for obs in obstacles:
            track_id = obs.get("id")
            if track_id is None:
                continue
            if track_id not in self.known_track_ids:
                new_track_detected = True
                self.known_track_ids.add(track_id)

        return leg_changed or dest_changed or new_track_detected


    def _compute_clearance_point(self, obstacles):
        next_waypoint = self.current_mission[self.current_wp]

        meters_per_deg_lat = 111320
        avg_lat = (self.pos["lat"] + next_waypoint["lat"]) / 2
        meters_per_deg_lon = meters_per_deg_lat * cos(radians(avg_lat))

        # vessel -> next_waypoint vector in meters
        vx = (next_waypoint["lon"] - self.pos["lon"]) * meters_per_deg_lon
        vy = (next_waypoint["lat"] - self.pos["lat"]) * meters_per_deg_lat
        path_len = sqrt(vx**2 + vy**2)

        if path_len < 0.1:
            return next_waypoint

        # unit vector along path
        ux, uy = vx / path_len, vy / path_len

        # project each obstacle onto path, take the furthest
        max_proj = 0.0
        for obs in obstacles:
            ox = (obs["lon"] - self.pos["lon"]) * meters_per_deg_lon
            oy = (obs["lat"] - self.pos["lat"]) * meters_per_deg_lat
            proj = ox * ux + oy * uy
            if proj > max_proj:
                max_proj = proj

        # clearance point is furthest obstacle projection + margin
        clearance_dist = max_proj + CLEARANCE_MARGIN
        clearance_dist = min(clearance_dist, path_len)  # don't overshoot WP

        cp_lon = self.pos["lon"] + (ux * clearance_dist) / meters_per_deg_lon
        cp_lat = self.pos["lat"] + (uy * clearance_dist) / meters_per_deg_lat

        print(f"Clearance point: {cp_lat:.6f}, {cp_lon:.6f} "
            f"({clearance_dist:.1f}m along path)")

        return {"lat": cp_lat, "lon": cp_lon, "alt": 0}


    def _handle_avoidance(self, obstacles):
        print("OBSTACLE AVOIDANCE TRIGGERED")

        if self.current_wp >= len(self.current_mission):
            print("ERROR: No next waypoint")
            return

        next_waypoint     = self.current_mission[self.current_wp]
        clearance_point   = self._compute_clearance_point(obstacles)

        if self.avoider is None:
            self.avoider = ObstacleAvoider(
                mission_waypoints=self.current_mission,
                grid_resolution=GRID_RESOLUTION,
                obstacle_buffer=3
            )

        avoidance_waypoints = self.avoider.plan_avoidance(
            vessel_pos=self.pos,
            next_waypoint=next_waypoint,
            obstacles=obstacles,
            clearance_point=clearance_point
        )

        if avoidance_waypoints is None:
            print("WARNING: Could not find avoidance path")
            return

        print(f"Avoidance path: {len(avoidance_waypoints)} waypoints")

        avoidance_waypoints = avoidance_waypoints[1:-1]

        if len(avoidance_waypoints) == 0:
            print("WARNING: Avoidance path empty after stripping — obstacle may be too close")
            return

        self._insert_wps(self.current_wp, avoidance_waypoints)

        success = mission_upload(self.usv, self.current_mission)

        if not success:
            print("ERROR: Failed to upload avoidance mission")
            del self.current_mission[self.current_wp:self.current_wp + len(avoidance_waypoints)]
            return

        set_current_waypoint(self.usv, self.current_wp)

        self.last_avoid_wp = self.current_wp + len(avoidance_waypoints)
        self.state = "AVOID"

        print(f"Avoidance active. Resume at WP {self.last_avoid_wp}")


    def _check_obstacle_cleared(self):
        if self.last_avoid_wp is not None:
            if self.current_wp >= self.last_avoid_wp:
                self.state = "MISSION_FOLLOW"
                self.last_avoid_wp = None
                print("Avoidance complete, resuming mission")


    def _gate_builder(self, buoys):
        if buoys:
            if len(buoys) == 2:
                if buoys[0]["color"] == "RED":
                    buoy_right = buoys[0]
                    buoy_left = buoys[1]
                else:
                    buoy_right = buoys[1]
                    buoy_left = buoys[0]

                return buoy_left, buoy_right
            else:
                return None


    def _which_side_of_gate(self, a, b, p):
        a_lat = a["lat"]
        a_lon = a["lon"]
        b_lat = b["lat"]
        b_lon = b["lon"]
        p_lat = p["lat"]
        p_lon = p["lon"]

        side = (b_lon - a_lon) * (p_lat - a_lat) - (b_lat - a_lat) * (p_lon - a_lon)
        return side


    def _gate_blocking_path(self, gate):
        if self.pos is None or self.current_wp_coord is None:
            return False
        buoy_left, buoy_right = gate
        side_current = self._which_side_of_gate(buoy_left, buoy_right, self.pos)
        side_next = self._which_side_of_gate(buoy_left, buoy_right, self.current_wp_coord)

        if (side_current * side_next < 0):
            print("Gate Found in Path")
            return True
        else:
            return False


    def _handle_gate(self, gate):
        buoy_left, buoy_right = gate

        gate_wps = gen_gate_wps(buoy_left, buoy_right)

        gate_wps_formatted = [
            {"lat": gate_wps[0][0], "lon": gate_wps[0][1], "alt": 0},
            {"lat": gate_wps[1][0], "lon": gate_wps[1][1], "alt": 0},
            {"lat": gate_wps[2][0], "lon": gate_wps[2][1], "alt": 0}
        ]

        self._insert_wps(self.current_wp, gate_wps_formatted)
        mission_upload(self.usv, self.current_mission)
        set_current_waypoint(self.usv, self.current_wp)
        self.last_task_wp = self.current_wp + 2
        self.state = "GATE"

        print("Gate WPs Inserted and Uploaded")


    def _check_gate_completion(self):
        if self.last_task_wp is not None and self.current_wp > self.last_task_wp:
            self.state = "MISSION_FOLLOW"
            self.last_task_wp = None
            print("Gate passage complete; resuming mission")


    def _find_target_buoy(self, buoys, target_color):
        for buoy in buoys:
            if buoy["color"] == target_color:
                return buoy
        return None


    def _handle_go_to_buoy(self, buoy):
        buoy_wp = {
            "lat": buoy["lat"],
            "lon": buoy["lon"],
            "alt": 0
        }

        self._insert_wps(self.current_wp, [buoy_wp])
        mission_upload(self.usv, self.current_mission)
        set_current_waypoint(self.usv, self.current_wp)

        self.state = "GO_TO_BUOY"
        print(f"Heading to {buoy['color']} buoy")


    def _insert_wps(self, current_wp, new_wps):
        self.current_mission[current_wp:current_wp] = new_wps


    def set_mission(self, mission):
        self.current_mission = mission
        self.last_obstacle_check_wp = None
        self.last_obstacle_check_dest = None
        self.known_track_ids.clear()
        self.object_tracker.reset()
        
        # Initialize avoider with mission area
        if len(mission) > 0:
            self.avoider = ObstacleAvoider(
                mission_waypoints=mission,
                grid_resolution=GRID_RESOLUTION,
                obstacle_buffer=3
            )
