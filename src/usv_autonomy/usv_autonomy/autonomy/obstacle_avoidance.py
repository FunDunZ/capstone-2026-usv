# ================================================================
# File: obstacle_avoidance.py
# Description: Obstacle avoidance using D* Lite path planning
# Author: Andrew Thomas
# Co-Authors: N/A
# Created: 02-12-2026
# Last Modified: 02-18-2026
# Version: 1.0.0
# Dependencies: DStarLite, math
# Notes:
# ================================================================

from .dstar_lite import (
    DStarLite,
    create_grid_from_mission_area,
    path_to_waypoints,
    simplify_path
)
from math import sqrt, cos, radians


class ObstacleAvoider:
    

    def __init__(self, mission_waypoints, grid_resolution=1.0, obstacle_buffer=3):
        # build a grid that covers the mission area
        self.grid, self.converter = create_grid_from_mission_area(
            mission_waypoints,
            grid_resolution=grid_resolution,
            margin=50
        )

        self.grid_resolution = grid_resolution
        # planner radius is in cells convert from meters
        self.obstacle_buffer = int(obstacle_buffer / grid_resolution)

        # planner is created on first avoidance request
        self.planner = None

        # keep obstacle cells so repeats are ignored
        self.known_obstacles = set()

        # store latest path for telemetry and debug
        self.current_path = None

    def plan_avoidance(self, vessel_pos, next_waypoint, obstacles, clearance_point=None):

        start_cell = self.converter.latlon_to_grid(vessel_pos["lat"], vessel_pos["lon"])
        
        # Use clearance point as goal if provided, otherwise fall back to next_waypoint
        goal_wp    = clearance_point if clearance_point is not None else next_waypoint
        goal_cell  = self.converter.latlon_to_grid(goal_wp["lat"], goal_wp["lon"])

        new_obstacles = self._mark_obstacles(obstacles)

        if not new_obstacles and not self.known_obstacles:
            print("No obstacles in grid, nothing to plan around")
            return None

        if self.planner is None:
            print(f"Initializing D* Lite planner: Start {start_cell} -> Goal {goal_cell}")
            self.planner = DStarLite(self.grid, start_cell, goal_cell)
            path_cells = self.planner.plan()
        else:
            if self.planner.goal != goal_cell:
                print(f"Goal changed to {goal_cell}, reinitializing planner")
                self.planner = DStarLite(self.grid, start_cell, goal_cell)
                path_cells = self.planner.plan()
            elif new_obstacles:
                print(f"Replanning with {len(new_obstacles)} new obstacles")
                path_cells = self.planner.replan(start_cell, new_obstacles)
            else:
                print("No new obstacles, replanning from new start position")
                path_cells = self.planner.replan(start_cell)

        if path_cells is None:
            print("ERROR: No path found around obstacles!")
            return None

        print(f"Path found with {len(path_cells)} cells")
        self.current_path = path_cells

        waypoints = path_to_waypoints(path_cells, self.converter)
        waypoints = simplify_path(waypoints, min_distance=3.0)

        print(f"Simplified to {len(waypoints)} waypoints")
        return waypoints

    def _mark_obstacles(self, obstacles):
        
        new_obstacle_cells = []

        for obs in obstacles:
            obs_cell = self.converter.latlon_to_grid(obs["lat"], obs["lon"])

            if obs_cell in self.known_obstacles:
                continue

            # inflate each obstacle with a safety radius
            self.grid.set_obstacle(
                obs_cell[0],
                obs_cell[1],
                radius=self.obstacle_buffer
            )

            self.known_obstacles.add(obs_cell)
            new_obstacle_cells.append(obs_cell)

            print(f"Marked obstacle at {obs_cell} with {self.obstacle_buffer}-cell buffer")

        return new_obstacle_cells

    def clear_obstacles(self):
        
        for row in range(self.grid.rows):
            for col in range(self.grid.cols):
                self.grid.set_cost(row, col, 1.0)

        self.known_obstacles.clear()
        self.planner = None
        print("Cleared all obstacles from navigation grid")

    def get_path_length(self):
        
        if self.current_path is None:
            return 0

        total_length = 0
        for i in range(len(self.current_path) - 1):
            cell1 = self.current_path[i]
            cell2 = self.current_path[i + 1]

            dr = cell2[0] - cell1[0]
            dc = cell2[1] - cell1[1]
            distance = sqrt(dr * dr + dc * dc) * self.grid_resolution
            total_length += distance

        return total_length


def calculate_distance(pos1, pos2):
    
    dlat = (pos2["lat"] - pos1["lat"]) * 111320
    avg_lat = (pos1["lat"] + pos2["lat"]) / 2
    dlon = (pos2["lon"] - pos1["lon"]) * 111320 * cos(radians(avg_lat))

    return sqrt(dlat ** 2 + dlon ** 2)


def is_obstacle_in_path(vessel_pos, next_waypoint, obstacle, threshold_meters=10):
    
    meters_per_deg_lat = 111320
    avg_lat = (vessel_pos["lat"] + next_waypoint["lat"]) / 2
    meters_per_deg_lon = meters_per_deg_lat * cos(radians(avg_lat))

    # use vessel as origin so vector math stays simple
    v_x = 0
    v_y = 0

    w_x = (next_waypoint["lon"] - vessel_pos["lon"]) * meters_per_deg_lon
    w_y = (next_waypoint["lat"] - vessel_pos["lat"]) * meters_per_deg_lat

    o_x = (obstacle["lon"] - vessel_pos["lon"]) * meters_per_deg_lon
    o_y = (obstacle["lat"] - vessel_pos["lat"]) * meters_per_deg_lat

    path_length = sqrt(w_x ** 2 + w_y ** 2)
    if path_length < 0.1:
        return False

    dot = (o_x * w_x + o_y * w_y) / (path_length * path_length)

    if dot < 0:
        closest_x, closest_y = v_x, v_y
    elif dot > 1:
        closest_x, closest_y = w_x, w_y
    else:
        closest_x = dot * w_x
        closest_y = dot * w_y

    distance = sqrt((o_x - closest_x) ** 2 + (o_y - closest_y) ** 2)
    return distance < threshold_meters

