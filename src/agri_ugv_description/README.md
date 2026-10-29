# agri_ugv_description

A 4-wheel skid-steer UGV inspired by the Clearpath Husky A300, for
**ROS 2 Jazzy + Gazebo Sim 8 (Harmonic)** on Ubuntu 24.04. It comes with two
worlds: a flat test world (`empty_flat`) and an olive orchard (`olive_orchard`).
The model is based on `docs/Pasted image.png` (tech-spec drawing) and
`docs/Pasted image (2).png` (render).

## Quick start

```bash
cd ~/agri_robot_ws
colcon build --symlink-install
source install/setup.bash

# Flat test world (default), with Gazebo GUI
ros2 launch agri_ugv_description sim.launch.py
# Olive orchard world
ros2 launch agri_ugv_description sim.launch.py world:=olive_orchard
# Headless (server only, lower CPU/RAM). Works with either world.
ros2 launch agri_ugv_description sim.launch.py world:=olive_orchard headless:=true
# Optional: rviz:=true, camera:=false / lidar:=false (save CPU/RAM),
#           custom spawn pose x:= y:= z:= yaw:=, or world:=/path/to/file.sdf

# Drive with the keyboard (second terminal, after `source install/setup.bash`)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# ALL automated tests in one command: starts/stops its own headless
# simulations (isolated ROS domain, so an open simulation is not disturbed)
ros2 run agri_ugv_description run_tests.sh                 # motion + orchard + sensor + rtk
ros2 run agri_ugv_description run_tests.sh sensor          # just one suite

# Or one test by hand (the simulation for that world must be running)
ros2 run agri_ugv_description motion_test.py    # flat world, 20 checks
ros2 run agri_ugv_description orchard_test.py   # orchard world, 11 checks
ros2 run agri_ugv_description sensor_test.py    # orchard world, 22 sensor checks
ros2 run agri_ugv_description rtk_test.py       # orchard world, 11 RTK GPS checks

# Look at the sensors
ros2 run rqt_image_view rqt_image_view /camera/image_raw        # colour
ros2 run rqt_image_view rqt_image_view /camera/depth/image_raw  # depth
ros2 topic echo /gps/fix                 # RTK position (status 2 = RTK)
ros2 topic echo /gps/raw                 # same receiver without RTK (metres off)
ros2 topic echo /rtk_base/corrections    # what the base station sends (E/N/U metres)
ros2 param set /rtk_gnss rtk_enabled false   # RTK off (demo); true to switch on

# Inspect the model in RViz only
ros2 launch agri_ugv_description display.launch.py
```

**Run only one simulation at a time** in the same ROS domain. Two simulations
share `/cmd_vel` and `/clock`, so a teleop or test would drive both robots.
To run a second, isolated one, export different values in that terminal first:
`export ROS_DOMAIN_ID=77 GZ_PARTITION=agri_test`.

`~/.bashrc` also sources `~/ros2_ws`. This package uses a unique name, so the
two do not conflict.

## Robot dimensions

Tags: **[D]** printed on the reference drawing, **[E]** estimated by measuring
pixels against the printed dimensions, **[A]** assumption (not in the references).

| Parameter | Value | Tag |
|---|---|---|
| Overall length (bumper to bumper) | 0.990 m | D |
| Overall width (outside of wheels) | 0.698 m | D (model: track + wheel width = 0.699 m) |
| Overall height (top plate) | 0.381 m | D |
| Top plate length | 0.740 m | D |
| Chassis width | 0.420 m | D |
| Ground clearance | 0.139 m | D |
| Bumper underside height | 0.160 m | D |
| Total mass | 80 kg | D |
| Max speed | 2.0 m/s | D |
| Wheel radius / width | 0.165 m / 0.114 m | E |
| Wheelbase / track | 0.512 m / 0.585 m | E |
| Chassis body length | 0.700 m | E |
| Mass split | wheels 4 × 2.6 kg, tyre-spring carriers 4 × 0.3 kg, bumpers 2 × 0.8 kg, chassis 66.8 kg | A |
| Tyre compliance | 25 000 N/m, 600 N·s/m per wheel, ±25 mm travel (about 7 mm static deflection) | A |
| Tyre friction μ / μ2 | 1.0 / 1.0 | A |
| Wheel effort / velocity limit | 100 N·m / 15 rad/s | A |
| Max angular velocity | 2.0 rad/s | A |
| Linear / angular acceleration limit | 1.0 m/s² / 3.0 rad/s² | A |
| Skid-steer separation multiplier | 1.55 | tuned in simulation |

All of these are xacro properties at the top of `urdf/agri_ugv.urdf.xacro`
and `urdf/agri_ugv.gazebo.xacro`.

## Model structure

- Frames follow REP-103 (x forward, y left, z up).
- `base_footprint` is on the ground. `base_link` is at axle height (0.165 m).
- `base_link` holds the yellow body, black top plate, e-stop and box collision.
  The front and rear bumpers are fixed links.
- Each wheel hangs from a short **tyre-compliance prismatic joint**
  (`*_tyre_spring_joint`, axis Z). It has a spring and damper, and its preload
  keeps it at 0 under static load, so ride height is unchanged. These were added
  in Phase 2, for the reason given below.
- Four `continuous` wheel joints (`*_wheel_joint`) with axis **+Y**, so a
  positive joint velocity drives the robot forward.
- Visuals and collisions are simple primitives (boxes and cylinders), not CAD meshes.

### Why the tyre springs

Gazebo's DART physics engine treats contacts as rigid and ignores `kp`/`kd`.
On uneven ground, a rigid 4-wheel chassis therefore rests on 3 wheels. The
orchard terrain has about 3 mm of twist under the footprint, and that was
enough to lift a wheel. Rotation in place reached only 63–73° of a commanded
90°, with 0.26–0.41 m drift. On the flat spawn pad of the same terrain it was
accurate. The springs model pneumatic tyre deflection. With them, rotation on
the uneven path improved to about 81° with about 0.11 m drift. Softer springs
(12 000 N/m) were tried and gave no further gain.

## Drive system

Gazebo's built-in `gz::sim::systems::DiffDrive` runs in skid-steer mode, with
two `left_joint` and two `right_joint` entries. ros2_control is not used,
because `gz_ros2_control` and `controller_manager` are not installed.

A skid-steer vehicle slides its wheels sideways when it turns. With the
geometric track (0.585 m), the robot only turned 232.6° for a commanded 360°.
The DiffDrive `wheel_separation` is therefore `track × 1.55 = 0.907 m`, the same
idea as the Husky driver's `wheel_separation_multiplier`.

### ROS interface (via `ros_gz_bridge`, `config/ros_gz_bridge.yaml`)

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | ROS → Gazebo |
| `/odom` | nav_msgs/Odometry (wheel odometry, `odom` → `base_footprint`) | Gazebo → ROS |
| `/tf` | odom → base_footprint | Gazebo → ROS |
| `/joint_states` | sensor_msgs/JointState (wheels and tyre springs) | Gazebo → ROS |
| `/ground_truth/odom` | nav_msgs/Odometry (true pose in `world`) | Gazebo → ROS |
| `/imu` | sensor_msgs/Imu, 100 Hz, frame `imu_link` | Gazebo → ROS |
| `/gps/navsat`, `/rtk_base/navsat` | sensor_msgs/NavSatFix, 10 Hz: robot and base-station receivers (inputs of `rtk_gnss.py`) | Gazebo → ROS |
| `/gps/raw` | sensor_msgs/NavSatFix, 10 Hz, standalone GPS (~1.5 m error), status FIX | ROS node `rtk_gnss.py` |
| `/rtk_base/corrections` | geometry_msgs/Vector3Stamped, 1 Hz, correction in metres (x east, y north, z up) | ROS node `rtk_gnss.py` |
| `/gps/fix` | sensor_msgs/NavSatFix, 10 Hz, frame `gps_link`, RTK position (~2 cm), status GBAS_FIX (2) | ROS node `rtk_gnss.py` |
| `/points` | sensor_msgs/PointCloud2, 3D LiDAR, 16 × 900 points, 10 Hz, frame `lidar_link` | Gazebo → ROS |
| `/scan` | sensor_msgs/LaserScan, 10 Hz, frame `lidar_link`, derived from `/points` by `cloud_to_scan.py` | ROS node |
| `/camera/image_raw`, `/camera/camera_info` | sensor_msgs/Image (rgb8) + CameraInfo, 15 Hz, frame `camera_optical_frame` | Gazebo → ROS |
| `/camera/depth/image_raw` | sensor_msgs/Image (32FC1, metres, `inf` = no return), 15 Hz, same frame and intrinsics as the colour image | Gazebo → ROS |
| `/clock` | rosgraph_msgs/Clock | Gazebo → ROS |

## Sensors (`urdf/agri_ugv.sensors.xacro`, Phase 3)

All sensor values are **[A]** assumptions typical of a Husky research setup.
The sensor links have no mass, so the robot is still 80 kg.

| Sensor | Position (from ground, robot centre) | Settings |
|---|---|---|
| IMU | top plate centre, z = 0.39 m | 100 Hz, gyro noise 0.002 rad/s, accel noise 0.02 m/s², orientation in ENU |
| GPS (NavSat) + RTK | 0.28 m behind centre, z = 0.39 m | 10 Hz, receiver noise 1 cm horizontal / 2 cm vertical; satellite/atmosphere error and RTK corrections added by `rtk_gnss.py` (see below) |
| 3D LiDAR (`gpu_lidar`, VLP-16 class) | on a 0.25 m mast 0.20 m ahead of centre, optical centre z = 0.67 m | 16 rings from −15° to +15° (2° apart), 900 points per ring (0.4°), 0.4–100 m, 10 Hz, 3 cm noise. Publishes `/points` |
| 2D scan (derived) | same as LiDAR | `cloud_to_scan.py` keeps points within ±0.25 m of the sensor's height and takes the closest one per 0.4° bin, up to 30 m. Publishes `/scan` for slam_toolbox / Nav2 |
| RGB-D camera (`rgbd_camera`, RealSense D435 class) | front edge of the top plate, z = 0.42 m, tilted 5.7° down | colour + aligned depth, 640 × 480, 69.4° horizontal FOV (D435 colour), depth 0.3–10 m, 15 Hz. For visual SLAM (e.g. RTAB-Map) |

- **GPS origin:** both worlds use the same origin, an olive-growing area near
  Sfax, Tunisia: 34.74° N, 10.76° E, 20 m elevation, ENU (x = east, y = north).
- **Switches:** `camera:=false` and `lidar:=false` remove those sensors
  (`lidar:=false` also stops `cloud_to_scan.py`).
- **Why the taller mast (Task 1):** the −15° ring must clear the top plate,
  GPS puck and camera. At 0.29 m above the plate, the closest return is the
  ground at about 2.5 m, so there are no self-hits.
- **Why `/scan` is derived:** the `pointcloud_to_laserscan` package is not
  installed, so a small Python node does the same job.
- **RTK GPS (Task 3):** an RTK base station (`models/rtk_base_station`, tripod
  with antenna and radio) stands on a known point in both worlds (orchard:
  x = −29.5, y = −14.0; flat world: −8, −8). `rtk_gnss.py` models the
  satellite/atmosphere error as a smooth common error of about 1.5 m
  horizontal and 2–3 m vertical, the same for both receivers. The base station
  knows its true position, so `measured − known` is the error; it sends this
  correction at 1 Hz. The robot subtracts it: **standalone ≈ 1.5 m, RTK ≈ 2 cm**.
  If RTK is switched off (`rtk_enabled false`) or corrections are older than
  5 s, `/gps/fix` falls back to the standalone position with status FIX (0).
- **GPS noise units:** Gazebo applies NavSat horizontal noise to
  latitude/longitude **in degrees**, so the xacro converts metres to degrees.
- **Camera QoS:** subscribe to `/camera/image_raw` with **reliable** QoS.
  Best-effort subscribers dropped about 57 % of the 0.9 MB frames here
  (4.3 Hz received of 10 Hz published). `config/view.rviz` uses reliable.

Example output from the orchard (robot 6 m into the path between rows 2 and 3):
`docs/phase3_camera_view.png` and `docs/phase3_lidar_scan.png`.

## Olive orchard world (`worlds/olive_orchard.sdf`)

| Feature | Value |
|---|---|
| Layout | 5 rows × 8 trees (40 trees), rows along x |
| Spacing | 6 m between rows, 5 m between trees (intensive olive orchard), ±0.15 m planting jitter |
| Paths | bare-dirt paths between rows, about 2.5–2.8 m clear between canopies, with faint wheel ruts. Headlands extend about 14 m past the last trees (4 m dirt, then grass) |
| Tree | trunk 0.9–1.2 m, 3 scaffold branches, ellipsoid canopy about 3.4 m wide and 3.5 m tall. 2 variants with random yaw |
| Tree collision | trunk cylinder only. The canopy starts about 1.5 m up, above the 0.38 m robot |
| Terrain | 64 × 64 m heightmap (129 × 129 px, 0.5 m/px). Gentle undulation plus 6 cm berms under the rows. Height range 0.307 m, max slope 3.5° |
| Ground texture | one 1024 × 1024 image over the whole terrain: dirt paths, weedy strips under rows, grass outside |
| Robot spawn | x = −23.0, y = −3.0, z = 0.155, yaw = 0. On a flattened pad at the headland, at the entrance of the path between rows 2 and 3, facing down the path. 5.4 m from the nearest trunk |
| Weight | static primitives only, no meshes. Measured RAM: about 340 MB server, about 580 MB GUI |

The world and models are generated by `scripts/generate_orchard.py`
(deterministic, seed 7). To change the layout, edit the constants at the top
of that script, run it, and rebuild:

```bash
cd ~/agri_robot_ws/src/agri_ugv_description && python3 scripts/generate_orchard.py
cd ~/agri_robot_ws && colcon build --symlink-install
```

It writes `worlds/olive_orchard.sdf`, `worlds/olive_orchard.json` (layout and
spawn metadata used by the test) and `models/{orchard_terrain,olive_tree_a,olive_tree_b}`.
If you change the spawn point, also update `WORLDS` in `launch/sim.launch.py`.
The launch file adds `share/agri_ugv_description/models` to `GZ_SIM_RESOURCE_PATH`.

## Test results

Tests run headless on sim time and are measured against Gazebo's ground-truth
pose. The real-time factor was 0.6–0.9, which doesn't affect sim-time results.

### Task 3, RTK GPS: `rtk_test.py` (orchard), **11/11 passed** (2026-10-29)

| Check | Result |
|---|---|
| Base station receiver on the known point | (−29.501, −13.998, 1.808) m vs (−29.500, −14.000, 1.810) m in the world file |
| Corrections rate | 1.00 Hz |
| Common error in the corrections | 1.48 m horizontal, +1.69 m vertical |
| Standalone GPS (`/gps/raw`) at rest | mean error 1.55 m |
| RTK at rest (`/gps/fix`, status 2) | horizontal RMS 1.9 cm, vertical RMS 3.1 cm |
| RTK while driving 5.9 m | RMS 1.8 cm, max 3.7 cm (standalone 1.54 m) |
| RTK off → fallback | status 0, error 1.51 m |
| RTK on again | status 2, RMS 1.7 cm |

The driving trace is written to `~/.ros/agri_ugv_tests/rtk_trace.csv`.

### Task 2, RGB-D camera: `sensor_test.py` (orchard), **22/22 passed** (2026-10-21)

New checks (the 18 earlier checks also pass):

| Check | Result |
|---|---|
| Depth rate (sim time) | 15.2 Hz |
| Depth format | 640 × 480 32FC1, frame `camera_optical_frame`, same size and frame as the colour image |
| Depth vs true distance to known trunks (projected with `camera_info`) | olive_r2_t1: 5.07 m measured / 5.07 m true; olive_r3_t1: 4.86 / 4.86 m |
| Sky and near ground | sky patch 100 % "no return"; ground just ahead 0.74 m |

Regression after Task 2: `motion_test` 20/20, `orchard_test` 11/11.

### Task 1, 3D LiDAR: `sensor_test.py` (orchard), **18/18 passed** (2026-10-21)

New checks (the 13 earlier checks also pass):

| Check | Result |
|---|---|
| `/points` and `/scan` rates (sim time) | 10.0 Hz and 10.0 Hz |
| Cloud layout | 16 rings × 900 points, fields x, y, z, intensity, ring, frame `lidar_link` |
| No self-hits | 8266 valid points, closest 2.45 m (lowest ring meets the ground at about 2.5 m) |
| Ground at the right height | 2535 ground points within 3.5 m, median 0.666 m below the sensor (expected 0.667) |
| Canopies above the trunks | 1611 points more than 1.52 m above ground, 100 % within 2 m of a known tree |
| `/scan` (from the cloud) vs known trunks | 11 trunks matched, median error 0.020 m (3 cm sensor noise). Max 23.4 m = an occluded trunk |

Regression after Task 1: `motion_test` 20/20, `orchard_test` 11/11 (same values as before).

**Cost:** the 16 × 900 cloud lowers the headless real-time factor in the
orchard from about 0.6 to about 0.3–0.5 on this laptop. Sim-time results are
not affected.

### Sensors: `sensor_test.py` (orchard), **13/13 passed** (2026-10-21, 2D LiDAR version)

| Check | Result |
|---|---|
| All topics publishing | imu, gps, scan, camera |
| Rates (sim time) | IMU 100.0 Hz, GPS 10.0 Hz, LiDAR 10.0 Hz, camera 15.2 Hz |
| IMU at rest | gravity 9.79 m/s², yaw matches ground truth (0.00°) |
| IMU while spinning | 0.766 rad/s measured vs 0.765 true |
| GPS at spawn (31 fixes averaged) | 0.07 m from the true antenna position |
| GPS while driving 3.00 m | moved 3.08 m |
| LiDAR vs known trunk positions | 14 trunks matched, median range error 0.009 m. Max 11.8 m = a trunk hidden behind a nearer one, which is correct |
| LiDAR self-hits | none (closest return 5.87 m) |
| Camera | 640 × 480 rgb8, non-blank image (pixel std 56) |

Failures found and fixed on the way:
1. GPS was off by 3.3 km, because the noise was given in metres but Gazebo reads degrees.
2. The camera seemed to run at 4–8 Hz. It was publishing at full rate, but the
   test's best-effort subscription dropped frames; now it subscribes reliably.

After adding the sensors, the flat test (20/20) and the orchard test (11/11)
were re-run with the same results as below.

### Flat world: `motion_test.py`, **20/20 passed**

Re-run after the tyre springs were added (a shared robot file changed):

| Test | Command | Result | Pass criterion |
|---|---|---|---|
| Rest on ground | – | z = −0.002 m | abs(z) < 0.02 m |
| Forward | 0.5 m/s × 4 s | +1.846 m, lateral 0.000 m, yaw 0.0° | 1.875 m ± 10 %, straight |
| Stop | 0 | v = 0.000 m/s | abs(v) < 0.02 |
| Backward | −0.5 m/s × 4 s | −1.870 m, straight | −1.875 m ± 10 % |
| Arc left / right | 0.5 m/s, ±0.5 rad/s × 4 s | +109.4° / −112.6° | ±112.2° ± 15 % |
| Spin in place left / right | ±1.0 rad/s, one turn | +350.9° / −350.7°, drift ≤ 0.002 m | ±360° ± 10 % |
| Max speed | 2.0 m/s | 2.000 m/s, braking distance 2.009 m | 2.0 ± 0.1 m/s |

Phase 1 results without the springs were also 20/20: arcs ±101–103°, spins
±356–357°.

### Orchard world: `orchard_test.py`, **11/11 passed**

| Check | Result | Pass criterion |
|---|---|---|
| Spawned at safe pose | (−23.000, −3.000), 5.43 m trunk clearance | within 0.10 m of spawn |
| Resting level and still | roll 0.00°, pitch 0.00°, v = w = 0 | tilt < 5° |
| On terrain surface | z = 0.103 m, heightmap 0.105 m | abs(error) < 0.04 m |
| `/cmd_vel` forward 0.5 m/s × 16 s along path | 7.852 m (expected 7.875) | ± 10 % |
| Stays on path, clear of trunks | lateral deviation 0.000 m, min clearance 2.20 m | < 0.40 m, > 0 |
| Follows uneven terrain | traversed 0.105–0.149 m terrain, max abs(z − heightmap) 0.003 m, max tilt 1.0° | < 0.05 m, < 8° |
| Stop | v = w = 0.000 | < 0.02 |
| Rotate in place +90° / −90° (0.5 rad/s) | +81.0° / −81.2°, drift 0.11 m | ±90° ± 15 %, drift < 0.15 m |
| `/cmd_vel` reverse 0.5 m/s × 4 s | −1.999 m | < −1.6 m |
| Stop after reverse | v = 0.000 | < 0.02 |

**Failure found and fixed:** before the tyre springs, the orchard test was
10/11. Rotation left reached +76.4° against a required ≥ 76.5°, with 0.148 m drift.

**Gazebo GUI:** visually checked by the user on 2026-10-21 in the Gazebo
window and in RViz: tree rows, dirt paths with ruts, grass strips, robot and
camera image all render as intended.

## Known limitations

- Sensor rendering (3D LiDAR + RGB-D camera) raises the Gazebo server to about
  510 MB in the orchard, and the headless real-time factor drops to about 0.2–0.4
  on this laptop. Use `camera:=false` when the camera is not needed.
- **Depth is noise-free.** A real D435 has about 2 % depth error at 4 m. Gazebo's
  `rgbd_camera` noise setting applies to the colour image only, so the depth
  image is perfect. Add noise in software if a SLAM evaluation needs it.
- `cloud_to_scan.py` (Python) uses about 35–40 % of one CPU core, mostly to
  receive the 460 KB cloud 10 times a second. A C++ node (or the
  `pointcloud_to_laserscan` package, not installed) would be cheaper.

- **Rotation on uneven ground** reaches about 90 % of the commanded angle
  (81° for 90°) with about 0.1 m drift, because skid-steer slip varies with
  uneven wheel loads. Wheel odometry (`/odom`) still reports the full
  commanded angle. Use ground truth, an IMU or closed-loop heading control
  for accurate turns.
- **Arc turns** on flat ground are within 3 % after the spring change.
  Instantaneous yaw rate during spins oscillates (stick-slip); the average is correct.
- Only one friction value (μ = 1.0) is used for all ground. Dirt, grass and
  wet soil are not distinguished.
- With the GUI on this 4-core machine, the real-time factor is about 0.6.
  Use `headless:=true` (plus `rviz:=true` if needed) for faster runs.
