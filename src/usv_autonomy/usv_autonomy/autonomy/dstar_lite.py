# ================================================================
# File: dstar_lite.py
# Description: Path Planning Algorithm (D* Lite) and related grid utilities
# Author: Andrew Thomas
# Co-Authors: N/A
# Created: 02-12-2026
# Last Modified: 02-18-2026
# Version: 1.0.0
# Dependencies: heapq, math
# Notes:
# ================================================================

import heapq
from math import sqrt, cos, radians

class PriorityQueue:
    

    def __init__(self):
        self.heap = []  # entries [key tie_breaker item]
        self.item_map = {}  # latest heap index for each live item
        self.counter = 0

    def insert(self, item, key):
        
        if item in self.item_map:
            self.update(item, key)
            return

        entry = [key, self.counter, item]
        self.counter += 1
        self.item_map[item] = len(self.heap)
        heapq.heappush(self.heap, entry)
        self._rebuild_map()
    
    def update(self, item, key):
        
        if item not in self.item_map:
            self.insert(item, key)
            return

        # heapq cannot remove arbitrary entries directly
        # mark old entry and push replacement
        old_idx = self.item_map[item]
        self.heap[old_idx][2] = None

        entry = [key, self.counter, item]
        self.counter += 1
        heapq.heappush(self.heap, entry)
        self._rebuild_map()
    
    def remove(self, item):
        
        if item in self.item_map:
            idx = self.item_map[item]
            self.heap[idx][2] = None
            del self.item_map[item]
    
    def top(self):
        
        self._clean()
        if self.heap:
            return self.heap[0][2]
        return None
    
    def top_key(self):
        
        self._clean()
        if self.heap:
            return self.heap[0][0]
        return [float("inf"), float("inf")]
    
    def pop(self):
        
        self._clean()
        if self.heap:
            _key, _counter, item = heapq.heappop(self.heap)
            if item is not None:
                del self.item_map[item]
                self._rebuild_map()
            return item
        return None
    
    def empty(self):
        
        self._clean()
        return len(self.heap) == 0
    
    def _clean(self):
        
        while self.heap and self.heap[0][2] is None:
            heapq.heappop(self.heap)
        self._rebuild_map()
    
    def _rebuild_map(self):
        
        self.item_map.clear()
        for idx, entry in enumerate(self.heap):
            if entry[2] is not None:
                self.item_map[entry[2]] = idx


class CoordinateConverter:
    

    def __init__(self, origin_lat, origin_lon, grid_resolution_meters=1.0):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.resolution = grid_resolution_meters

        # approximation good enough for our scale
        self.meters_per_deg_lat = 111320

    def latlon_to_grid(self, lat, lon):
        
        dlat = lat - self.origin_lat
        dlon = lon - self.origin_lon

        # longitude scale changes with latitude
        meters_per_deg_lon = self.meters_per_deg_lat * cos(radians(lat))
        meters_north = dlat * self.meters_per_deg_lat
        meters_east = dlon * meters_per_deg_lon

        row = int(meters_north / self.resolution)
        col = int(meters_east / self.resolution)

        return (row, col)
    
    def grid_to_latlon(self, row, col):
        
        meters_north = (row + 0.5) * self.resolution
        meters_east = (col + 0.5) * self.resolution

        dlat = meters_north / self.meters_per_deg_lat
        lat = self.origin_lat + dlat
        meters_per_deg_lon = self.meters_per_deg_lat * cos(radians(lat))
        dlon = meters_east / meters_per_deg_lon
        lon = self.origin_lon + dlon

        return (lat, lon)


class NavigationGrid:
    

    def __init__(self, rows, cols, default_cost=1.0):
        self.rows = rows
        self.cols = cols
        # cost to move into each cell
        self.grid = [[default_cost for _ in range(cols)] for _ in range(rows)]

    def is_valid(self, row, col):
        
        return 0 <= row < self.rows and 0 <= col < self.cols
    
    def get_cost(self, row, col):
        
        if not self.is_valid(row, col):
            return float("inf")
        return self.grid[row][col]
    
    def set_cost(self, row, col, cost):
        
        if self.is_valid(row, col):
            self.grid[row][col] = cost
    
    def set_obstacle(self, row, col, radius=0):
        
        obstacle_cost = 1e6
        self.set_cost(row, col, obstacle_cost)

        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if dr * dr + dc * dc <= radius * radius:
                    self.set_cost(row + dr, col + dc, obstacle_cost)
    
    def get_neighbors(self, row, col, allow_diagonal=True):
        
        neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        if allow_diagonal:
            directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if self.is_valid(new_row, new_col):
                neighbors.append((new_row, new_col))

        return neighbors
    
    def get_edge_cost(self, from_cell, to_cell):
        
        from_r, from_c = from_cell
        to_r, to_c = to_cell

        dr = abs(to_r - from_r)
        dc = abs(to_c - from_c)
        distance = sqrt(dr * dr + dc * dc)
        cell_cost = self.get_cost(to_r, to_c)

        return distance * cell_cost


class DStarLite:
    

    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal

        # cost tables for the planner
        self.g = {}
        self.rhs = {}
        self.U = PriorityQueue()

        self.k_m = 0
        self.s_last = start
        self._initialize()
    
    def _initialize(self):
        
        self.g.clear()
        self.rhs.clear()
        self.U = PriorityQueue()
        self.k_m = 0

        for row in range(self.grid.rows):
            for col in range(self.grid.cols):
                cell = (row, col)
                self.g[cell] = float("inf")
                self.rhs[cell] = float("inf")

        # search expands backward from goal
        self.rhs[self.goal] = 0
        key = self._calculate_key(self.goal)
        self.U.insert(self.goal, key)
    
    def _calculate_key(self, s):
        
        g_val = self.g.get(s, float("inf"))
        rhs_val = self.rhs.get(s, float("inf"))
        min_val = min(g_val, rhs_val)
        h = self._heuristic(s, self.start)
        return [min_val + h + self.k_m, min_val]
    
    def _heuristic(self, a, b):
        
        r1, c1 = a
        r2, c2 = b
        return sqrt((r2 - r1) ** 2 + (c2 - c1) ** 2)
    
    def _update_vertex(self, u):
        
        if u != self.goal:
            neighbors = self.grid.get_neighbors(u[0], u[1])

            min_rhs = float("inf")
            for neighbor in neighbors:
                cost = self.grid.get_edge_cost(u, neighbor)
                g_neighbor = self.g.get(neighbor, float("inf"))
                min_rhs = min(min_rhs, cost + g_neighbor)

            self.rhs[u] = min_rhs

        self.U.remove(u)
        g_val = self.g.get(u, float("inf"))
        rhs_val = self.rhs.get(u, float("inf"))
        if g_val != rhs_val:
            key = self._calculate_key(u)
            self.U.insert(u, key)
    
    def _compute_shortest_path(self):
        
        while True:
            if self.U.empty():
                break

            top_key = self.U.top_key()
            start_key = self._calculate_key(self.start)
            g_start = self.g.get(self.start, float("inf"))
            rhs_start = self.rhs.get(self.start, float("inf"))

            if top_key >= start_key and g_start == rhs_start:
                break

            u = self.U.pop()
            k_old = top_key
            k_new = self._calculate_key(u)

            if k_old < k_new:
                self.U.insert(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                self.U.remove(u)

                neighbors = self.grid.get_neighbors(u[0], u[1])
                for s in neighbors:
                    self._update_vertex(s)
            else:
                self.g[u] = float("inf")
                self._update_vertex(u)
                neighbors = self.grid.get_neighbors(u[0], u[1])
                for s in neighbors:
                    self._update_vertex(s)
    
    def plan(self):
        
        self._compute_shortest_path()
        return self._extract_path()
    
    def _extract_path(self):
        
        if self.g.get(self.start, float("inf")) == float("inf"):
            return None

        path = [self.start]
        current = self.start

        while current != self.goal:
            neighbors = self.grid.get_neighbors(current[0], current[1])
            best_neighbor = None
            best_cost = float("inf")

            for neighbor in neighbors:
                edge_cost = self.grid.get_edge_cost(current, neighbor)
                g_neighbor = self.g.get(neighbor, float("inf"))
                total_cost = edge_cost + g_neighbor

                if total_cost < best_cost:
                    best_cost = total_cost
                    best_neighbor = neighbor

            if best_neighbor is None:
                return None

            current = best_neighbor
            path.append(current)

        return path
    
    def update_edge_costs(self, changed_edges):
        
        if self.s_last != self.start:
            self.k_m += self._heuristic(self.s_last, self.start)
            self.s_last = self.start

        for change in changed_edges:
            if isinstance(change, tuple) and len(change) == 2:
                cell, new_cost = change
                self.grid.set_cost(cell[0], cell[1], new_cost)

                neighbors = self.grid.get_neighbors(cell[0], cell[1])
                affected_cells = [cell] + neighbors
            else:
                (from_cell, to_cell), _new_cost = change
                affected_cells = [from_cell, to_cell]

            for u in affected_cells:
                if self.grid.is_valid(u[0], u[1]):
                    self._update_vertex(u)

        self._compute_shortest_path()
    
    def replan(self, new_start, obstacles=None):
        
        self.start = new_start

        if obstacles:
            changed_edges = []
            for obs in obstacles:
                changed_edges.append((obs, 1e6))
            self.update_edge_costs(changed_edges)
        else:
            if self.s_last != self.start:
                self.k_m += self._heuristic(self.s_last, self.start)
                self.s_last = self.start
            self._compute_shortest_path()

        return self._extract_path()


def create_grid_from_mission_area(waypoints, grid_resolution=1.0, margin=50):
    
    lats = [wp["lat"] for wp in waypoints]
    lons = [wp["lon"] for wp in waypoints]
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)

    meters_per_deg_lat = 111320
    avg_lat = (min_lat + max_lat) / 2
    meters_per_deg_lon = meters_per_deg_lat * cos(radians(avg_lat))
    lat_range_m = (max_lat - min_lat) * meters_per_deg_lat
    lon_range_m = (max_lon - min_lon) * meters_per_deg_lon

    lat_range_m += 2 * margin
    lon_range_m += 2 * margin

    rows = int(lat_range_m / grid_resolution) + 1
    cols = int(lon_range_m / grid_resolution) + 1

    margin_lat = margin / meters_per_deg_lat
    margin_lon = margin / meters_per_deg_lon
    origin_lat = min_lat - margin_lat
    origin_lon = min_lon - margin_lon

    grid = NavigationGrid(rows, cols)
    converter = CoordinateConverter(origin_lat, origin_lon, grid_resolution)

    return grid, converter


def path_to_waypoints(path, converter):
    
    waypoints = []
    for cell in path:
        lat, lon = converter.grid_to_latlon(cell[0], cell[1])
        waypoints.append({"lat": lat, "lon": lon, "alt": 0})  # matches mavlink waypoint dict shape
    
    return waypoints


def simplify_path(waypoints, min_distance=5.0):
    
    if len(waypoints) <= 2:
        return waypoints

    simplified = [waypoints[0]]
    for wp in waypoints[1:-1]:
        last_wp = simplified[-1]

        dlat = (wp["lat"] - last_wp["lat"]) * 111320
        avg_lat = (wp["lat"] + last_wp["lat"]) / 2
        dlon = (wp["lon"] - last_wp["lon"]) * 111320 * cos(radians(avg_lat))
        distance = sqrt(dlat ** 2 + dlon ** 2)

        if distance >= min_distance:
            simplified.append(wp)

    simplified.append(waypoints[-1])
    return simplified

