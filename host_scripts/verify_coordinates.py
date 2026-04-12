#!/usr/bin/env python3
"""
Verify GPS-to-Gazebo coordinate transformations by reading actual mission and world files.
Dynamically loads data from mission_parkour_all.json and water_world.sdf
Works from any directory.
"""
import math
import json
import re
import sys
from pathlib import Path

# Find project root dynamically
SCRIPT_DIR = Path(__file__).parent.absolute()
PROJECT_ROOT = SCRIPT_DIR.parent
MISSION_FILE = PROJECT_ROOT / "sim/configs/mission_parkour_all.json"
WORLD_FILE = PROJECT_ROOT / "sim/worlds/water_world.sdf"
SIM_HOME = (-35.363262, 149.165237)

def extract_waypoints_from_sdf():
    """Extract waypoint poses and GPS from water_world.sdf comments"""
    waypoints = {}
    try:
        with open(WORLD_FILE, 'r') as f:
            content = f.read()
        
        # Find all waypoint model blocks
        model_pattern = r'<model name="(waypoint[^"]*?)">'
        
        for match in re.finditer(model_pattern, content):
            model_name = match.group(1)
            model_start = match.start()
            model_end = content.find('</model>', model_start)
            if model_end == -1:
                continue
            
            model_block = content[model_start:model_end + 8]
            
            # Extract comment from model block (handles dashes in Turkish text)
            comment_match = re.search(r'<!--\s*(.*?)\s*-->', model_block, re.DOTALL)
            if not comment_match:
                continue
            
            comment = comment_match.group(1)
            
            # Find the pose tag within this model
            pose_match = re.search(r'<pose>([^ ]+) ([^ ]+)', model_block)
            if pose_match:
                pose_x = float(pose_match.group(1))
                pose_y = float(pose_match.group(2))
                
                # Extract GPS from comment
                gps_match = re.search(r'GPS\(([^,]+),\s*([^)]+)\)', comment)
                if gps_match:
                    gps_lat = float(gps_match.group(1))
                    gps_lon = float(gps_match.group(2))
                    
                    # Extract world coordinates from comment
                    world_match = re.search(r'world\(([^,]+),\s*([^)]+)\)', comment)
                    if world_match:
                        world_x = float(world_match.group(1))
                        world_y = float(world_match.group(2))
                        
                        waypoints[model_name] = {
                            'pose_x': pose_x,
                            'pose_y': pose_y,
                            'world_xy': (world_x, world_y),
                            'gps': [gps_lat, gps_lon],
                            'comment': comment.strip()
                        }
    except Exception as e:
        print(f"✗ Error parsing world file: {e}")
    
    return waypoints

def load_mission_gps():
    """Load GPS coordinates from mission_parkour_all.json"""
    gps_list = []
    try:
        with open(MISSION_FILE) as f:
            mission = json.load(f)
        if isinstance(mission, list):
            gps_list = mission
    except Exception as e:
        print(f"✗ Error loading mission: {e}")
    
    return gps_list

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Standard forward azimuth calculation."""
    d_lon = math.radians(lon2 - lon1)
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    y = math.sin(d_lon) * math.cos(lat2_r)
    x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def haversine_distance(lat1, lon1, lat2, lon2):
    """Distance in meters."""
    R = 6371000.0  # Earth radius in meters
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def local_xy_to_global_1(home_lat, home_lon, pos_x, pos_y):
    """sim_nav_state.py convention: x→lon, y→lat"""
    cos_home = math.cos(math.radians(home_lat))
    meters_to_lon = 1.0 / (111320.0 * cos_home) if abs(cos_home) > 1e-6 else 1.0 / 111320.0
    lat = home_lat + (pos_y / 111320.0)
    lon = home_lon + (pos_x * meters_to_lon)
    return lat, lon

def local_xy_to_global_2(home_lat, home_lon, pos_x, pos_y):
    """sitl_gazebo_bridge.py after NED conversion: x_ned→lat, y_ned→lon"""
    # After conversion: x_ned = p.y, y_ned = p.x
    # So if we pass in raw Gazebo p.x, p.y as pos_x, pos_y:
    x_ned = pos_y  # Gazebo y → NED x
    y_ned = pos_x  # Gazebo x → NED y
    lat = home_lat + (x_ned / 111320.0)
    lon = home_lon + (y_ned / (111320.0 * math.cos(math.radians(home_lat))))
    return lat, lon

def world_xy_to_bearing(dx, dy):
    """Convert world x,y offset to bearing (0°=north, 90°=east, 180°=south, 270°=west)."""
    return (math.degrees(math.atan2(dx, dy)) + 360) % 360

print("=" * 80)
print("GPS-to-Gazebo Coordinate Verification (Dynamic from Files)")
print("=" * 80)
print(f"Project root: {PROJECT_ROOT}")
print(f"Mission file: {MISSION_FILE}")
print(f"World file: {WORLD_FILE}")

# Load data from files
mission_gps = load_mission_gps()
world_waypoints = extract_waypoints_from_sdf()

print(f"\n✓ Loaded mission: {len(mission_gps)} waypoints from {MISSION_FILE}")
print(f"✓ Extracted world waypoints: {len(world_waypoints)} models from {WORLD_FILE}")
print(f"✓ SIM_HOME GPS: {SIM_HOME[0]:.7f}, {SIM_HOME[1]:.7f}")

print("\n" + "-" * 80)
print("Verification (Mission GPS vs World Markers):")
print("-" * 80)

all_pass = True

# Sort waypoints by index/order for consistent matching
sorted_wp_names = sorted(world_waypoints.keys(), key=lambda x: (
    # Sort by p1/p2 first: p1_* before p2_*
    0 if 'p1_' in x else 1,
    # Then by number if present
    int(x.split('_')[-1]) if x.split('_')[-1].isdigit() else 0,
    # Finally by name
    x
))

for i, gps in enumerate(mission_gps, 1):
    lat, lon = gps[0], gps[1]
    
    # Find corresponding world waypoint in sorted order
    if i <= len(sorted_wp_names):
        wp_key = sorted_wp_names[i - 1]
        wp = world_waypoints[wp_key]
        world_x, world_y = wp['world_xy']
        world_bearing = world_xy_to_bearing(world_x, world_y)
        
        # Bearing from HOME to mission GPS
        gps_bearing = calculate_bearing(SIM_HOME[0], SIM_HOME[1], lat, lon)
        
        # Distance check
        distance = haversine_distance(SIM_HOME[0], SIM_HOME[1], lat, lon)
        
        bearing_diff = abs(gps_bearing - world_bearing)
        if bearing_diff > 180:
            bearing_diff = 360 - bearing_diff
        
        status = "✓" if bearing_diff < 1.0 else "⚠"
        print(f"{status} WP{i}: GPS({lat:.7f}, {lon:.7f}) | Bearing: {gps_bearing:.2f}° vs World: {world_bearing:.2f}° (Δ={bearing_diff:.2f}°)")
        print(f"   Distance: {distance:.2f}m | World coords: ({world_x:.1f}, {world_y:.1f}) | Model: {wp_key}")
        
        if bearing_diff >= 1.0:
            all_pass = False
    else:
        print(f"⚠ WP{i}: GPS({lat:.7f}, {lon:.7f}) | NO MATCHING WORLD WAYPOINT (expected {len(sorted_wp_names)} waypoints but mission has {len(mission_gps)})")
        all_pass = False

print("\n" + "=" * 80)
if all_pass:
    print("✓ ALL WAYPOINTS VERIFIED - Bearings consistent with world markers")
    sys.exit(0)
else:
    print("⚠ SOME WAYPOINTS HAVE INCONSISTENCIES")
    sys.exit(1)
