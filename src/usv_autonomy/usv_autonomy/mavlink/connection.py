# ================================================================
# File: connection.py
# Description: Script to establish connection and confirm heartbeat
# Author: Andrew Thomas
# Co-Authors: N/A
# Created: 10-16-2025
# Last Modified: 02-19-2026
# Version: 2.0.0
# Dependencies: pymavlink, time
# Notes: N/A
# ================================================================

from pymavlink import mavutil
import time


def connect_usv():

    # COMM_PORT = 'udpin:192.168.2.1:14550'
    COMM_PORT = 'udpin:0.0.0.0:14550'  # sim
    print(f"Establishing Link Over {COMM_PORT}")

    usv = mavutil.mavlink_connection(COMM_PORT)
    print("Waiting for heartbeat...")

    usv.wait_heartbeat()
    print(f"Heartbeat Received (sys={usv.target_system} comp={usv.target_component})")

    # Request GLOBAL_POSITION_INT at 10 Hz
    usv.mav.request_data_stream_send(
        usv.target_system,
        usv.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        10,  # Hz
        1    # 1 = start
    )
    print("Position stream requested at 10 Hz")

    # Request MISSION_CURRENT at 2 Hz
    # Interval is in microseconds: 500000 us = 2 Hz
    usv.mav.command_long_send(
        usv.target_system,
        usv.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,
        500000,
        0, 0, 0, 0, 0
    )
    print("MISSION_CURRENT stream requested at 2 Hz")

    # Give the vehicle a moment to start streaming before the main loop
    # tries to read from it
    time.sleep(0.5)

    return usv
