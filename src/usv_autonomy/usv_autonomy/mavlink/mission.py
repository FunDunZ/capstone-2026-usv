# ================================================================
# File: mission.py
# Description: Mission commands -> Download, Upload, Get Current
# Author: Andrew Thomas
# Co-Authors: N/A
# Created: 10-20-2025
# Last Modified: 2-19-2026
# Version: 1.1.0
# Dependencies: pymavlink
# Notes: N/A
# ================================================================

from pymavlink import mavutil

_cached_wp  = 0
_cached_pos = None

def mission_download(usv):
    usv.mav.mission_request_list_send(usv.target_system, usv.target_component)
    count_msg = usv.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)

    if count_msg:
        wp_count = count_msg.count
        print(f"Total Waypoints: {wp_count}")
    else:
        print(f"Timeout on Waypoint Count Request")
        return 0, []

    wp_list = [None] * wp_count

    for i in range(wp_count):
        usv.mav.mission_request_int_send(usv.target_system, usv.target_component, i)
        wp_msg = usv.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)

        if wp_msg:
            wp_list[i] = {
                "lat": wp_msg.x / 1e7,
                "lon": wp_msg.y / 1e7,
                "alt": wp_msg.z / 1e7
            }
            print(f"Waypoint {i} received")
        else:
            print(f"Timeout on Waypoint {i} Request")

    return wp_count, wp_list

def mission_upload(usv, new_mission):
    wp_count = len(new_mission)
    print(f"Uploading {wp_count} waypoints...")

    usv.mav.mission_count_send(usv.target_system, usv.target_component, wp_count)

    for i in range(wp_count):
        req_msg = usv.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5)
        if req_msg:
            wp = new_mission[i]
            usv.mav.mission_item_int_send(
                usv.target_system,
                usv.target_component,
                i,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1, 0, 0, 0, 0,
                int(wp["lat"] * 1e7),
                int(wp["lon"] * 1e7),
                wp["alt"]
            )
            print(f"New waypoint {i} uploaded")
        else:
            print(f"Timeout on WP request {i}")
            return False

    ack = usv.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if ack:
        print("Mission Upload Complete")
        return True
    else:
        print("No mission ack received")
        return False

def get_current_wp(usv):
    global _cached_wp
    msg = usv.recv_match(type="MISSION_CURRENT", blocking=False)
    if msg:
        _cached_wp = msg.seq
    return _cached_wp

def set_current_waypoint(usv, seq):
    usv.mav.mission_set_current_send(usv.target_system, usv.target_component, seq)
    ack = usv.recv_match(type='MISSION_CURRENT', blocking=True, timeout=5)
    if ack and ack.seq == seq:
        global _cached_wp
        _cached_wp = seq
        print(f"Current waypoint set to {seq}")
        return True
    else:
        print(f"Failed to confirm waypoint set to {seq}")
        return False

def get_vessel_pos(usv):
    global _cached_pos
    msg = usv.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        _cached_pos = {
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "alt": msg.alt / 1000.0,
            "heading": msg.hdg / 100.0  # centidegrees -> degrees
        }
    if _cached_pos is None:
        print("WARNING: No vessel position received yet")
    return _cached_pos


def pump_messages(usv):
    global _cached_wp, _cached_pos
    while True:
        msg = usv.recv_match(blocking=False)
        if msg is None:
            break
        msg_type = msg.get_type()
        if msg_type == 'MISSION_CURRENT':
            _cached_wp = msg.seq
        elif msg_type == 'GLOBAL_POSITION_INT':
            _cached_pos = {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "alt": msg.alt / 1000.0,
                "heading": msg.hdg / 100.0
            }
