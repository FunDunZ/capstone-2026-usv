# ================================================================
# File: buoy_tasks.py
# Description: Waypoint calculations for buoy specific tasking
# Author: Andrew Thomas
# Co-Authors: N/A
# Created: 01-15-2026
# Last Modified: 01-22-2026
# Version: 1.0.1
# Dependencies: None
# Notes: only gate has been implemented
# ================================================================


from math import cos, radians, sqrt


PRE_GATE_OFFSET = 3 # meters (parameter we can change; maybe I'll make this into a startup option or smth so user doesn't have to touch the code to change this)
POST_GATE_OFFSET = 3 #^

M_PER_DEG = 111320 # meters per degree




def gen_gate_wps(buoy_left, buoy_right):

    mid_lat = (buoy_left["lat"] + buoy_right["lat"]) / 2
    mid_lon = (buoy_left["lon"] + buoy_right["lon"]) / 2


    # yellow = buoy_left, red = buoy_right (for now)
    delta_lat = buoy_right["lat"] - buoy_left["lat"] # degrees
    delta_lon = buoy_right["lon"] - buoy_left["lon"]

    # convert to meters
    delta_lat_m = delta_lat * M_PER_DEG
    delta_lon_m = delta_lon * (M_PER_DEG * cos(radians(mid_lat))) # lon depends on lat

    # normalize
    magnitude = sqrt(delta_lat_m ** 2 + delta_lon_m ** 2)
    offset_dir_lat_m = delta_lat_m / magnitude
    offset_dir_lon_m = delta_lon_m / magnitude

    # perpendicular to gate
    perp_lat_m = -offset_dir_lon_m
    perp_lon_m = offset_dir_lat_m

    # apply offsets
    pre_offset_lat_m = perp_lat_m * PRE_GATE_OFFSET # meters
    pre_offset_lon_m = perp_lon_m * PRE_GATE_OFFSET

    post_offset_lat_m = perp_lat_m * POST_GATE_OFFSET
    post_offset_lon_m = perp_lon_m * POST_GATE_OFFSET

    # convert back to deg.
    pre_offset_lat_deg = pre_offset_lat_m / M_PER_DEG
    pre_offset_lon_deg = pre_offset_lon_m / (M_PER_DEG * cos(radians(mid_lat)))

    post_offset_lat_deg = post_offset_lat_m / M_PER_DEG
    post_offset_lon_deg = post_offset_lon_m / (M_PER_DEG * cos(radians(mid_lat)))

    # set wps
    pre_wp = mid_lat + pre_offset_lat_deg, mid_lon + pre_offset_lon_deg
    mid_wp = mid_lat, mid_lon
    post_wp = mid_lat - post_offset_lat_deg, mid_lon - post_offset_lon_deg

    return pre_wp, mid_wp, post_wp