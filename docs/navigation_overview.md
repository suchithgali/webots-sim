# Drone Navigation & Dynamic Routing Overview

The simulation is designed to be fully dynamic, meaning it does not rely on a list of hard-coded flight coordinates (like "fly forward 5 meters, turn left"). Instead, it translates an abstract warehouse configuration (`default.json`) into physical simulation waypoints on the fly, and uses dynamic physics calculations to reach them.

Here is a breakdown of how the navigation system works end-to-step without hardcoded paths.

## 1. Dynamic Coordinate Generation (`waypoint_planner.py`)
At startup, the planner reads the JSON configuration which dictates how many aisles there are, how long they are, and the number of bays per aisle. It then maps the abstract JSON inches into the Webots physical 3D grid.

Instead of writing `[[-5.4, -10.7], [-4.2, -10.7], ...]`, the code calculates these physical locations mathematically using fractions:

```python
# waypoint_planner.py
for i in range(num_stops):
    # Distribute stops perfectly from 0.0 to 1.0 along the aisle length
    frac  = i / (num_stops - 1)
    
    # Calculate the physical X position based on this fraction
    x_sim = x_start_sim + frac * (x_end_sim - x_start_sim)
```

It calculates the `Y` (Aisle Depth) by automatically stepping down the warehouse by exactly 3.35 meters per aisle defined in the config:
```python
# Webots physical world aisles are spaced exactly 3.35m apart.
y_mid_sim = -10.70 + (aisle_idx * 3.35)
```

## 2. The Serpentine "S-Pattern" & Alternating Elevations
To minimize battery drain and save time, the planner alternates scanning the bottom and top racks. If the drone is at the top rack, instead of flying back down to the bottom rack to move to the next pallet, it simply slides over to the next pallet and scans the *top* rack first, then moves down.

```python
# waypoint_planner.py
# To minimize wasted diagonal flight, we alternate scanning up and down
level_order = scan_levels if i % 2 == 0 else list(reversed(scan_levels))

for curr_z in level_order:
    waypoints.append([x_sim, y_mid_sim, curr_z, True]) # Mark as a scanning stop
```

## 3. Physics-Based Flight Execution (`drone_controller.py`)
Once the 3D waypoints are generated, the drone determines how to reach them using a continuous feedback loop (PID controller) rather than scripted timing. It measures the Euclidean `<X, Y>` distance to the target at every frame. 

It calculates the angle to the target and precisely scales the engine pitch (forward speed) based on how far away the target is. We recently updated this braking logic to prevent the drone from crawling too slowly:

```python
# drone_controller.py
dx = self.target_position[0] - self.current_pose[0]
dy = self.target_position[1] - self.current_pose[1]
xy_dist = np.sqrt(dx ** 2 + dy ** 2)

if xy_dist > 0.1:
    # Add distance-based braking: if we are close to the target, slow down!
    dist_factor = clamp(xy_dist / 1.5, 0.8, 1.0)
    pitch_disturbance = clamp(np.log10(abs(angle_left) + 0.001), self.MAX_PITCH_DISTURBANCE, 0.1) * dist_factor
```

## 4. Transit Waypoints vs. Scan Waypoints
The planner generates two types of waypoints (represented by the 4th boolean parameter in the generated array):
- `False`: **Transit Waypoints.** The drone flies fast and uses wide, loose tolerances to round corners in the hallway outside the racks.
- `True`: **Scan Waypoints.** The drone must stop precisely in front of a bay, hold altitude, spin 360 degrees to allow the OpenCV thread to capture the barcode, and then proceed.

```python
# drone_controller.py
current_wp_is_scan = waypoints[self.target_index][3]
precision = self.target_precision if current_wp_is_scan else self.transit_precision

# Must precisely reach the target 3D coordinate before advancing to the next waypoint
if xy_dist < precision and z_dist < 0.2:
    if current_wp_is_scan:
        self._set_state(DroneState.CAPTURING)
    else:
        self.target_index += 1
```

By passing a configuration file to the simulator, the drone calculates the physical space, generates an efficient serpentine grid, and manages its own physics to navigate from point to point seamlessly.