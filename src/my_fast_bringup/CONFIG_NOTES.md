# Configuration & Optimization Notes

## 1. Disabled Nav2 Servers (`navigation_launch.py`)
To save ~30% CPU on the Raspberry Pi 4, two unused servers were removed from `navigation_launch.py`:
- `route_server` (nav2_route)
- `docking_server` (opennav_docking)

**How to re-enable in the future if needed:**
In `launch/navigation_launch.py`:
1. Re-add `'route_server'` and `'docking_server'` to `lifecycle_nodes`.
2. Re-add the `Node(...)` definitions for `nav2_route` and `opennav_docking` to `load_nodes` (or composable list).

## 2. Collision Monitor Polygon (`config/nav2_params.yaml`)
- `PolygonStop`: Configured in `base_link` frame as `[[0.18, 0.18], [0.18, -0.18], [-0.18, -0.18], [-0.18, 0.18]]`.
- The physical robot chassis is 30x30 cm centered at the origin `base_link` (bounds: +/- 0.15 m).
- The 0.18 m boundary provides a 3 cm safety buffer around the perimeter.
- Sensor frames:
  - LiDAR is mounted at `x: 0.02 m`, `y: -0.04 m` (4 cm to the right), `z: 0.21 m`.
  - IMU is mounted at `x: 0.03 m`, `y: -0.06 m` (6 cm to the right), `z: 0.07 m`.
  - The `collision_monitor` uses TF to automatically transform laser scans into `base_link`, so the LiDAR offset is automatically accounted for.

## 3. Nav2 Dynamic Limits
- Controller: `RegulatedPurePursuitController` (RPP)
- `rotate_to_heading_angular_vel`: 1.0 rad/s (matches Lichtblick [-1, 1] limits)
- `max_velocity`: `[0.28, 0.0, 1.0]`
- `min_velocity`: `[-0.25, 0.0, -1.0]`
