#!/usr/bin/env python3
"""
Quick validation that mission coordinates are correct by reading actual files.
Dynamically loads from mission_parkour_all.json and water_world.sdf
Works from any directory.
"""
import json
import math
import re
import sys
from pathlib import Path

# Find project root dynamically
SCRIPT_DIR = Path(__file__).parent.absolute()
PROJECT_ROOT = SCRIPT_DIR.parent
MISSION_FILE = PROJECT_ROOT / "sim/configs/mission_parkour_all.json"
WORLD_FILE = PROJECT_ROOT / "sim/worlds/water_world.sdf"
SIM_HOME = [-35.363262, 149.165237]

def mission_waypoints(payload):
    """Return waypoint list from flat or structured mission payload."""
    if isinstance(payload, list):
        return payload
    if isinstance(payload, dict):
        for key in ("waypoints", "mission", "coordinates"):
            value = payload.get(key)
            if isinstance(value, list):
                return value
    return []

def mission_profile(payload):
    """Return optional mission_profile object from structured mission payload."""
    if isinstance(payload, dict) and isinstance(payload.get("mission_profile"), dict):
        return payload["mission_profile"]
    return {}

def extract_world_markers():
    """Extract world coordinates from water_world.sdf waypoint comments"""
    markers = []
    try:
        with open(WORLD_FILE, 'r') as f:
            content = f.read()
        
        # Find all waypoint comments with world coordinates
        pattern = r'<!-- ([^-]*?world\(([^,]+),\s*([^)]+)\))'
        
        for match in re.finditer(pattern, content):
            comment = match.group(1)
            world_x = float(match.group(2))
            world_y = float(match.group(3))
            
            # Extract waypoint name from comment
            wp_match = re.search(r'(WP\d+)', comment)
            wp_name = wp_match.group(1) if wp_match else f"WP{len(markers)+1}"
            markers.append(((world_x, world_y), wp_name))
    except Exception as e:
        print(f"Error parsing world: {e}")
    
    return markers

def extract_target_markers():
    """Extract target buoy positions and convert Gazebo pose to mission ENU world xy."""
    targets = {}
    try:
        with open(WORLD_FILE, 'r') as f:
            content = f.read()
        pattern = r'<include><uri>model://target_buoy_([^<]+)</uri><name>(target_buoy_[^<]+)</name><pose>([^ ]+) ([^ ]+)'
        for match in re.finditer(pattern, content):
            color = match.group(1).strip().upper()
            name = match.group(2).strip()
            pose_x = float(match.group(3))
            pose_y = float(match.group(4))
            targets[color] = ((-pose_y, pose_x), name)
    except Exception as e:
        print(f"Error parsing targets: {e}")
    return targets

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate bearing from point 1 to point 2"""
    d_lon = math.radians(lon2 - lon1)
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    y = math.sin(d_lon) * math.cos(lat2_r)
    x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def world_xy_to_bearing(x, y):
    """Convert world x,y to bearing from origin"""
    return (math.degrees(math.atan2(x, y)) + 360) % 360

print("=" * 80)
print("MISSION COORDINATE VALIDATION (Dynamic from Files)")
print("=" * 80)
print(f"Project root: {PROJECT_ROOT}")

# Load mission file
try:
    with open(MISSION_FILE) as f:
        mission_payload = json.load(f)
    mission = mission_waypoints(mission_payload)
    profile = mission_profile(mission_payload)
    if not mission:
        raise ValueError("mission payload has no waypoint list")
    print(f"✓ Mission file loaded: {len(mission)} waypoints")
except Exception as e:
    print(f"✗ FAILED to load mission: {e}")
    sys.exit(1)

# Extract world markers
world_markers = extract_world_markers()
target_markers = extract_target_markers()
print(f"✓ World markers extracted: {len(world_markers)} waypoints")
print(f"✓ Target markers extracted: {len(target_markers)} buoys")

print("\n" + "-" * 80)

# Validate bearing consistency
all_valid = True
for i, wp_gps in enumerate(mission, 1):
    mission_lat, mission_lon = wp_gps
    
    # Get world marker coordinates
    if i <= len(world_markers):
        world_x, world_y = world_markers[i-1][0]
        label = f"WP{i}"
    else:
        target_color = str(profile.get("target_color", "RED") or "RED").upper()
        target = target_markers.get(target_color)
        if not target:
            print(f"⚠ WP{i}: no target marker for target_color={target_color}")
            all_valid = False
            continue
        world_x, world_y = target[0]
        label = f"P3 Target {target_color}"
        
    # Calculate bearings
    mission_bearing = calculate_bearing(SIM_HOME[0], SIM_HOME[1], mission_lat, mission_lon)
    world_bearing = world_xy_to_bearing(world_x, world_y)
    
    bearing_diff = abs(mission_bearing - world_bearing)
    if bearing_diff > 180:
        bearing_diff = 360 - bearing_diff
    
    if bearing_diff < 1.0:
        print(f"✓ {label}: Bearing {mission_bearing:.2f}° (Mission) vs {world_bearing:.2f}° (World) - Δ {bearing_diff:.3f}°")
    else:
        print(f"⚠ {label}: Bearing MISMATCH {mission_bearing:.2f}° vs {world_bearing:.2f}° - Δ {bearing_diff:.3f}°")
        all_valid = False

print("\n" + "=" * 80)
if all_valid:
    print("✓ VALIDATION PASSED - All waypoints have consistent bearings")
    sys.exit(0)
else:
    print("✗ VALIDATION FAILED - Some waypoints have inconsistent bearings")
    sys.exit(1)
