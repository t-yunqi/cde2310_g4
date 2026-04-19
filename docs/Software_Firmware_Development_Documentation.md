---
title: Soft/Firmware Development Documentation
---

## Navigation

- [Requirements Specification](Requirement_specifications.md)
- [Con Ops](Con_Ops.md)

### Frontier Detection 
The frontier detection module enables the robot to autonomously explore unknown areas of the maze using the occupancy grid map. A cell is considered a frontier if it is a free cell and at least one of its neighbours is unknown. The frontier detection module groups neighbouring frontier cells into frontier regions and chooses the centroid of a region as the navigation destination. If it fails to identify a suitable frontier region, it selects a fallback destination by choosing a free-space point near the explored unexplored boundary that is sufficiently far from obstacles and recently visited fallback points.

### Coordinator
The coordinator orchestrates the overall mission logic, including maze exploration, docking, payload delivery triggering, and undocking. Actual navigation is handled by the Nav2 `NavigateToPose` action, while docking and undocking are handled using the Nav2 docking actions. When the robot is in the `EXPLORE` state, it uses frontier detection to search the maze. If the stationary or moving station ArUco tag is detected, the coordinator transitions to the corresponding docking state. Once docking succeeds, the coordinator triggers payload delivery and waits for the payload node to report completion before commanding the robot to undock and resume exploration.

![Alt Text](Coordinator_state_diagram.png)

| State | Function |
|---|---|
| EXPLORE | The robot explores the maze using frontier detection. |
| GO_TO_STATIONARY | The robot has detected the ArUco tag of the stationary station and is attempting to dock to it. |
| GO_TO_MIDPOINT | The robot has detected the ArUco tag associated with the moving-station mission and is attempting to dock. |
| WAIT_A_COMPLETE | The robot has triggered payload delivery for Station A and is waiting for the payload sequence to finish. |
| WAIT_B_COMPLETE | The robot has triggered payload delivery for Station B and is waiting for the payload sequence to finish. |

### Servo Code
The `payload.py` program controls the two servos used for payload release. It subscribes to `/station_cmd` for mission commands and publishes `/mission_complete` when dispensing is done. For Station A, the payload sequence is executed immediately after receiving `START_A`. For Station B, the node is first armed by `START_B`, and payload release is then triggered step-by-step based on ArUco detections from the right camera. This allows the moving station payload to be dispensed only when the target is in the correct position.

### Interface Diagram