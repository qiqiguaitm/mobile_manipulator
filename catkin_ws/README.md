# AgileX Robot Demo Project

This project implements a mobile manipulation robot system based on the AgileX Tracer chassis and Piper arm, following the design specification in `design.md`.

## Project Structure

The project is organized into the following ROS packages:

1. **sensor_calibration** - Sensor calibration module
2. **inference_abstraction** - AI inference abstraction layer
3. **slam** - Simultaneous Localization and Mapping module
4. **perception** - Object detection and perception module
5. **object_tracker** - Object tracking module using EKF
6. **target_filter** - Target filtering and post-processing
7. **task_router** - Path planning for multi-target picking
8. **task_mgr** - Task management and coordination
9. **chassis_planner** - Chassis motion planning
10. **arm_planner** - Arm trajectory planning
11. **chassis_controller** - Chassis hardware control
12. **arm_controller** - Arm hardware control
13. **dag_framework** - DAG execution framework

## System Architecture

The system follows a layered architecture as specified in the design document:

1. **Hardware Layer** - Physical sensors and actuators
2. **Driver Layer** - Hardware drivers
3. **Middleware Layer** - ROS message communication
4. **Algorithm Layer** - Core algorithms and processing
5. **Application Layer** - High-level task execution

## Dependencies

- ROS Noetic
- Python 3
- NetworkX (for DAG framework)

## Building the Project

```bash
./build_all.sh
```

## Running the Project

```bash
./run_all.sh
```

## Module Communication Flow

The modules communicate through ROS topics following the data flow specified in the design document:

```
SensorCalibration -> SLAM
SensorCalibration -> Perception
SensorCalibration -> ObjectTracker
Perception -> ObjectTracker
ObjectTracker -> TargetFilter
TargetFilter -> TaskRouter
TaskRouter -> TaskManager
SLAM -> TaskManager
TaskManager -> ChassisPlanner
TaskManager -> ArmPlanner
ChassisPlanner -> ChassisController
ArmPlanner -> ArmController
InferenceAbstraction -> Perception
InferenceAbstraction -> ObjectTracker
```

## DAG Execution

The system uses a DAG (Directed Acyclic Graph) execution framework to ensure proper module initialization and execution order based on dependencies.