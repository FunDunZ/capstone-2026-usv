from math import radians, cos, sin, sqrt, atan2, degrees

CAMERA_HEIGHT = 0.6
TILT_ANGLE    = 0.0

def localize_object(bbox_cx, bbox_cy, depth_m, image_width, image_height,
                    focal_length_px, vessel_pos, obj_type, confidence):
    if depth_m <= 0 or depth_m > 100:
        return None

    cx = image_width  / 2.0
    cy = image_height / 2.0
    nx = (bbox_cx - cx) / focal_length_px
    ny = (bbox_cy - cy) / focal_length_px
    cam_x = depth_m * nx
    cam_y = depth_m * ny
    cam_z = depth_m

    tilt_rad = radians(TILT_ANGLE)
    world_x  =  cam_x
    world_z  =  cam_z * cos(tilt_rad) - cam_y * sin(tilt_rad)

    horizontal_dist    = sqrt(world_x**2 + world_z**2)
    bearing_offset_deg = degrees(atan2(world_x, world_z))

    heading     = vessel_pos.get("heading", 0.0)
    abs_bearing = (heading + bearing_offset_deg) % 360
    bearing_rad = radians(abs_bearing)

    meters_per_deg_lat = 111320
    meters_per_deg_lon = meters_per_deg_lat * cos(radians(vessel_pos["lat"]))

    delta_north = horizontal_dist * cos(bearing_rad)
    delta_east  = horizontal_dist * sin(bearing_rad)

    obj_lat = vessel_pos["lat"] + (delta_north / meters_per_deg_lat)
    obj_lon = vessel_pos["lon"] + (delta_east  / meters_per_deg_lon)

    return {
        "type":       obj_type,
        "lat":        obj_lat,
        "lon":        obj_lon,
        "confidence": confidence,
    }
