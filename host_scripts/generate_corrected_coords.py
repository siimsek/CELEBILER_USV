#!/usr/bin/env python3
"""
Generate corrected GPS coordinates by reading actual world.sdf waypoint positions.
Dynamically extracts waypoints from water_world.sdf
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
WORLD_FILE = PROJECT_ROOT / "sim/worlds/water_world.sdf"
SIM_HOME = [-35.363262, 149.165237]
EARTH_RADIUS_M = 6371000.0

def extract_world_waypoints():
    """Extract waypoint poses from water_world.sdf"""
    waypoints = []
    try:
        with open(WORLD_FILE, 'r') as f:
            content = f.read()
        
        # Find all waypoint models
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
            
            # Find the pose tag
            pose_match = re.search(r'<pose>([^ ]+) ([^ ]+)', model_block)
            if pose_match:
                pose_x = float(pose_match.group(1))
                pose_y = float(pose_match.group(2))
                
                # Extract world coordinates
                world_match = re.search(r'world\(([^,]+),\s*([^)]+)\)', comment)
                if world_match:
                    world_x = float(world_match.group(1))
                    world_y = float(world_match.group(2))
                    
                    # Extract GPS from comment
                    gps_match = re.search(r'GPS\(([^,]+),\s*([^)]+)\)', comment)
                    if gps_match:
                        gps_lat = float(gps_match.group(1))
                        gps_lon = float(gps_match.group(2))
                        waypoints.append({
                            'name': model_name,
                            'world_xy': (world_x, world_y),
                            'pose_xy': (pose_x, pose_y),
                            'gps': [gps_lat, gps_lon]
                        })
    except Exception as e:
        print(f"Error extracting waypoints from {WORLD_FILE}: {e}", file=sys.stderr)
    
    return waypoints

def world_xy_to_gps(home_lat, home_lon, x_m, y_m):
    """Convert Gazebo world x,y (meters) to GPS lat,lon."""
    lat = home_lat + (y_m / 111320.0)
    cos_lat = math.cos(math.radians(home_lat))
    meters_to_lon = 1.0 / (111320.0 * cos_lat) if abs(cos_lat) > 1e-6 else 1.0 / 111320.0
    lon = home_lon + (x_m * meters_to_lon)
    return [lat, lon]

print("=" * 80, file=sys.stderr)
print("CORRECTED COORDINATES GENERATOR", file=sys.stderr)
print("=" * 80, file=sys.stderr)
print(f"Project root: {PROJECT_ROOT}", file=sys.stderr)
print(f"World file: {WORLD_FILE}", file=sys.stderr)
print(f"SIM_HOME: {SIM_HOME[0]:.7f}, {SIM_HOME[1]:.7f}", file=sys.stderr)

waypoints = extract_world_waypoints()

if not waypoints:
    print("✗ ERROR: No waypoints found in world file", file=sys.stderr)
    sys.exit(1)

print(f"✓ Extracted {len(waypoints)} waypoints", file=sys.stderr)

# Generate mission coordinates
mission_gps = []
for wp in waypoints:
    gps = wp['gps']
    mission_gps.append(gps)
    print(f"  WP: {wp['name']:20} | GPS: [{gps[0]:.7f}, {gps[1]:.7f}] | World: ({wp['world_xy'][0]:6.1f}, {wp['world_xy'][1]:6.1f})", file=sys.stderr)

print("\n" + "=" * 80, file=sys.stderr)
print("✓ Mission JSON ready (output below):", file=sys.stderr)
print("=" * 80, file=sys.stderr)

# Output valid JSON array to stdout (pipe-friendly)
print(json.dumps(mission_gps, indent=2))
