# SAE EletroQuad 2025 - Black Bee Drones

This repository contains the autonomous mission code developed by [Black Bee Drones](https://github.com/Black-Bee-Drones) for the Competição EletroQuad SAE BRASIL – Eletrobras 2025 (18–22 June 2025, Univap, São José dos Campos).

## Competition Overview

The first EletroQuad edition required every team to fly three outdoor GPS missions with a standardized quadrotor. A camera supplies the targets; the flight controller executes the motion. Allowed stacks were ArduPilot and PX4. This repository uses ArduPilot through MAVROS.

- **Slalom** — Fly at low altitude (~2.5 m), identify four colored posts, pass them on alternating sides, then land.
- **Hang the Hook** — Find a red hose, release a carried hook onto it, return and land on the blue circular base.
- **Bouncing** — Identify geometric figures on ground pads and land with the gear inside the assigned figure.

## Technical Stack

### Software

- **[ROS 2](https://docs.ros.org/)** Humble
- **[MAVROS](https://github.com/mavlink/mavros)** — ArduPilot bridge
- **[Mirela SDK](https://github.com/Black-Bee-Drones/mirela-sdk)** — drone, vision, and process helpers used by these packages (later evolved into [Nectar SDK](https://github.com/Black-Bee-Drones/nectar-sdk))
- **[Yasmin](https://github.com/uleroboticsgroup/yasmin)** — state machines (`slalom`, `hook`)
- **[OpenCV](https://opencv.org/)** — HSV color filters
- **[Ultralytics YOLO](https://docs.ultralytics.com/)** — ONNX detection in `bouncing`

Camera input is a USB webcam (`IMAGE_SOURCE="webcam"`). Image-center constants in `slalom` and `hook` are 320×240.

## Missions

### Slalom (Mission 1)

**Objective**: Take off, locate four vertical posts (pink, red, blue, black; order and first pass side given on the day), pass left/right in alternation, land after the last post.

This repository has **two independent implementations** of the same mission.

**`slalom`** — Yasmin + Mirela HSV line detection.

- Top machine: INITIALIZE → TAKEOFF (1.3 m) → START_LINE_DETECTION → BEAM_NAVIGATION → LAND → END
- Nested beam loop: search → center → approach (1.56 m) → side pass, then flip side
- Color sequence in [`constants.py`](slalom/slalom/constants.py): `red_sl`, `blue_sl`, `black_sl`, `pink_sl`. First side: `left`
- Black posts use a dedicated detector node (`black_line_detection_node`)

Run: `ros2 run slalom slalom_mission`

**`slalon`** — HSV color filter plus range estimate, separate from Yasmin.

- `depth_st` sequences the color filter (black → pink → red → blue) and publishes distance / image-side
- Two mission drivers: `movement` (search, align, approach, pass, land) or `slalom_mission` (enum machine on the same topics: `depth_topic`, `where_is_it`, `switch_state`)
- Takeoff in `slalom_mission`: 1.5 m. Pass side starts left and alternates

Run:

```bash
ros2 launch slalon slalon_launch.py              # depth_st + movement
ros2 launch slalon slalom_mission_launch.py      # depth_st + slalom_mission
```

**Documentation**: [slalon/README.md](slalon/README.md)

### Hang the Hook (Mission 2)

**Objective**: Take off carrying a hook, find the red hose (12.7 mm, ~2 m high), hang the hook, RTL and land on the blue circular takeoff base. An optional 25 cm blue ground line may be used as a cue.

**Implementation** (`hook`):

- Yasmin machine in [`hook/mangalarga.py`](hook/hook/mangalarga.py): INITIALIZE → TAKEOFF (4.87 m) → SEARCH_BLUE_LINE → CENTER_RED_BLOB → DESCEND_TO_HOOK → RELEASE_HOOK → RETURN_TO_LAUNCH → END
- Mirela line detection (HSV) and PID nodes for centering / descent
- Servo RC AUX 3: `HOCK_HOLD_PWM=1000`, `HOCK_RELEASE_PWM=2000`
- `FOLLOW_BLUE_LINE` is implemented in-package but is not on the main transition path from `SEARCH_BLUE_LINE`

**Documentation**: [hook/README.md](hook/README.md)

### Bouncing (Mission 3)

**Objective**: From the takeoff pad, recognize geometric figures on eight ground bases and land inside the assigned figure.

**Implementation** (`bouncing`):

- `MavDrone` + GPS geofence; search waypoints interpolated from arena corners
- YOLOv11n ONNX (`bouncing/ai/yolo/yolov11n.onnx`). Classes: circle, square, triangle, hexagon, pentagon, star, cross, house
- Entry `main()` currently instantiates `BouncingNode("star")`
- Launch starts `bouncing_node` and `camera_node`

**Documentation**: [bouncing/README.md](bouncing/README.md)

## File Structure

```
SAE-Eletroquad/
├── slalom/          # Slalom, Yasmin + HSV line detection
├── slalon/          # Slalom, color filter + range estimate
├── hook/            # Hang the Hook
├── bouncing/        # Bouncing
└── README.md
```

Each directory is an `ament_python` ROS 2 package.

## Running Missions

```bash
cd ~/ros2_ws
colcon build --packages-select slalom slalon hook bouncing
source install/setup.bash
```

`slalom` and `hook` also need `mirela_sdk` and `mirela_interfaces` in the workspace.

| Mission | Command |
|---|---|
| Slalom (`slalom`) | `ros2 run slalom slalom_mission` |
| Slalom (`slalon`) | `ros2 launch slalon slalon_launch.py` |
| Hang the Hook | `ros2 run hook mangalarga` |
| Bouncing | `ros2 launch bouncing bouncing_launch.py` |

Optional interrupt for bouncing: `ros2 run bouncing land_command_node`. Package READMEs cover install, extra nodes, and (for hook) Yasmin Viewer.

## References

### Competition

- [EletroQuad SAE BRASIL](https://saebrasil.org.br/programas-estudantis/eletroquad/)
- [2025 announcement (missions)](https://saebrasil.org.br/competicao-eletroquad-sae-brasil-eletrobras-de-drones-autonomos-estreia-18-de-junho/)

### Software

- [Mirela SDK](https://github.com/Black-Bee-Drones/mirela-sdk)
- [Nectar SDK](https://github.com/Black-Bee-Drones/nectar-sdk) (successor used in later years)
- [ROS 2](https://docs.ros.org/)
- [MAVROS](https://github.com/mavlink/mavros)
- [Yasmin](https://github.com/uleroboticsgroup/yasmin)
- [PID controller](https://github.com/Black-Bee-Drones/pid-controller)

## Team

**Black Bee Drones** - Latin America's first academic autonomous drone team  
Federal University of Itajubá (UNIFEI), Brazil

---

*This documentation describes the technical implementation used during SAE EletroQuad 2025. For questions or contributions, please refer to the [Black Bee Drones GitHub organization](https://github.com/Black-Bee-Drones).*
