# ros2-mavlink-nav-bridge

Interactive MAVLink-style GCS shell connected to a ROS 2 drone simulator with RViz2 visualization.

## Architecture

```
┌─────────────────────────────┐         ROS 2          ┌──────────────────────────┐
│  gcs_shell  (Terminal A)    │ ──/rc_channels ──────► │  drone_sim  (Terminal B) │
│  interactive stdin          │ ──/vehicle_command ──► │  50 Hz physics loop      │
│                             │ ◄─/drone_status ─────  │  publishes TF + odom     │
└─────────────────────────────┘                        └───────────┬──────────────┘
                                                                   │ /drone_marker
                                                       ┌───────────▼──────────────┐
                                                       │  RViz2   (Terminal C)    │
                                                       │  drone moves in 3D       │
                                                       └──────────────────────────┘
```

## RC Channel Reference (ArduPilot Mode 2)

| Channel | Control | 1000 | 1500 | 2000 |
|---|---|---|---|---|
| RC1 | Roll | left | neutral | right |
| RC2 | Pitch | **forward** | neutral | backward |
| RC3 | Throttle | cut motors | hover | full climb |
| RC4 | Yaw | rotate left | neutral | rotate right |

> **Important:** In ArduPilot convention, *below* neutral on pitch (RC2 < 1500) means nose down = forward.

## Build

```bash
make build
source install/setup.bash
```

## Run

```bash
# Terminal A — GCS shell (interactive)
ros2 run mavlink_nav_bridge gcs_shell

# Terminal B — drone simulator + physics
ros2 run mavlink_nav_bridge drone_sim

# Terminal C — RViz2
rviz2 -d config/drone_view.rviz
```

## Interactive Session Example

```
> arm
[ARM]
> rc 3 1500         ← throttle to hover, drone starts climbing
RC3 = 1500
> rc 2 1400         ← pitch forward (below neutral = nose down = forward)
RC2 = 1400
> rc 4 1600         ← yaw right
RC4 = 1600
> rc 4 1500         ← stop yaw
RC4 = 1500
> rc 2 1500         ← stop forward movement
RC2 = 1500
> status
armed=YES  pos=(2.34, 0.00, 4.87)  yaw=12.3 deg  vel=(0.21, 0.00, 0.01)
> goto 10 5 8       ← autonomous fly to (x=10, y=5, z=8m)
GOTO (10, 5, 8)
> land
[LAND]
> status
armed=NO  pos=(10.0, 5.0, 0.0)  yaw=12.3 deg  vel=(0.00, 0.00, 0.00)
```

## RViz2

- **Green cube** = drone armed and flying
- **Red cube** = drone disarmed
- **Text label** = `ARMED z=4.9m`
- **TF frame** = `base_link` moves in 3D relative to `map`
- **Odometry trail** = path the drone has flown

## Tests

```bash
make test

# or directly
./build/mavlink_nav_bridge/test_drone_physics
```

Tests verify the physics model: hover stability, climb on throttle > 1500, forward motion on pitch below neutral, ground clamp, yaw rate change.
