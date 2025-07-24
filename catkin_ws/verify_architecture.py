#!/usr/bin/env python3

import yaml
import json
from collections import defaultdict

def analyze_architecture_data_flow():
    """Analyze the architecture data flow based on configuration files"""
    
    print("=== Architecture Data Flow Analysis ===\n")
    
    # Read the topics configuration
    try:
        with open('src/arm_planner/config/topics.yaml', 'r') as f:
            topics_config = yaml.safe_load(f)
        print("✓ Loaded topics configuration")
    except Exception as e:
        print(f"✗ Failed to load topics configuration: {e}")
        return
    
    # Display input topics
    print("\n--- Input Topics ---")
    input_topics = topics_config.get('topics', {}).get('inputs', {})
    for name, config in input_topics.items():
        print(f"  {name}:")
        print(f"    Topic: {config.get('name')}")
        print(f"    Type: {config.get('type')}")
    
    # Display output topics
    print("\n--- Output Topics ---")
    output_topics = topics_config.get('topics', {}).get('outputs', {})
    for name, config in output_topics.items():
        print(f"  {name}:")
        print(f"    Topic: {config.get('name')}")
        print(f"    Type: {config.get('type')}")
    
    # Analyze node connections
    print("\n--- Node Connections Analysis ---")
    
    # Based on the launch file, we can infer connections
    nodes = [
        "sensor_calibration",
        "inference_abstraction", 
        "slam",
        "perception",
        "object_tracker",
        "target_filter",
        "task_router",
        "task_mgr",
        "chassis_planner",
        "arm_planner",
        "chassis_controller",
        "arm_controller",
        "dag_framework"
    ]
    
    print(f"Identified {len(nodes)} nodes in the system:")
    for i, node in enumerate(nodes, 1):
        print(f"  {i}. {node}")
    
    # Expected data flow
    print("\n--- Expected Data Flow ---")
    expected_flows = [
        "sensor_calibration → inference_abstraction",
        "inference_abstraction → slam",
        "slam → perception",
        "perception → object_tracker",
        "object_tracker → target_filter",
        "target_filter → task_router",
        "task_router → task_mgr",
        "task_mgr → chassis_planner & arm_planner",
        "chassis_planner → chassis_controller",
        "arm_planner → arm_controller",
        "chassis_controller & arm_controller → dag_framework"
    ]
    
    for flow in expected_flows:
        print(f"  {flow}")
    
    # Verify message types
    print("\n--- Message Type Verification ---")
    message_types = set()
    for config in list(input_topics.values()) + list(output_topics.values()):
        msg_type = config.get('type')
        if msg_type:
            message_types.add(msg_type)
    
    print(f"Found {len(message_types)} unique message types:")
    for msg_type in sorted(message_types):
        print(f"  • {msg_type}")
    
    # Check for potential issues
    print("\n--- Potential Issues ---")
    issues = []
    
    # Check if all required topics have publishers/subscribers
    required_input_topics = ["/map/octomap", "/chassis/pose", "/task_mgr/target_object/pose", "/arm/joint_states"]
    for topic in required_input_topics:
        found = False
        for config in input_topics.values():
            if config.get('name') == topic:
                found = True
                break
        if not found:
            issues.append(f"Missing configuration for required input topic: {topic}")
    
    # Check output topics
    required_output_topics = ["/arm/trajectory", "/arm/end_pose"]
    for topic in required_output_topics:
        found = False
        for config in output_topics.values():
            if config.get('name') == topic:
                found = True
                break
        if not found:
            issues.append(f"Missing configuration for required output topic: {topic}")
    
    if issues:
        print("Found potential issues:")
        for issue in issues:
            print(f"  ✗ {issue}")
    else:
        print("✓ No configuration issues found")
    
    print("\n=== Analysis Complete ===")

def create_test_messages():
    """Create sample input and output messages for testing"""
    
    print("\n=== Creating Test Messages ===")
    
    # Sample input messages based on the topics configuration
    input_messages = {
        "/map/octomap": {
            "type": "octomap_msgs/Octomap",
            "sample_data": {
                "header": {
                    "seq": 1,
                    "stamp": {"secs": 1234567890, "nsecs": 123456789},
                    "frame_id": "map"
                },
                "binary": True,
                "id": "OcTree",
                "resolution": 0.05,
                "data": "base64_encoded_octomap_data"
            }
        },
        "/chassis/pose": {
            "type": "geometry_msgs/PoseStamped",
            "sample_data": {
                "header": {
                    "seq": 1,
                    "stamp": {"secs": 1234567890, "nsecs": 123456789},
                    "frame_id": "odom"
                },
                "pose": {
                    "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        },
        "/task_mgr/target_object/pose": {
            "type": "geometry_msgs/PoseStamped",
            "sample_data": {
                "header": {
                    "seq": 1,
                    "stamp": {"secs": 1234567890, "nsecs": 123456789},
                    "frame_id": "map"
                },
                "pose": {
                    "position": {"x": 3.0, "y": 4.0, "z": 0.5},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        },
        "/arm/joint_states": {
            "type": "sensor_msgs/JointState",
            "sample_data": {
                "header": {
                    "seq": 1,
                    "stamp": {"secs": 1234567890, "nsecs": 123456789},
                    "frame_id": "arm_base"
                },
                "name": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                "position": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                "velocity": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "effort": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            }
        }
    }
    
    # Sample output messages
    output_messages = {
        "/arm/trajectory": {
            "type": "trajectory_msgs/JointTrajectory",
            "sample_data": {
                "header": {
                    "seq": 1,
                    "stamp": {"secs": 1234567890, "nsecs": 123456789},
                    "frame_id": "arm_base"
                },
                "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                "points": [
                    {
                        "positions": [0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
                        "velocities": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                        "accelerations": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "effort": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "time_from_start": {"secs": 1, "nsecs": 0}
                    }
                ]
            }
        },
        "/arm/end_pose": {
            "type": "geometry_msgs/PoseStamped",
            "sample_data": {
                "header": {
                    "seq": 1,
                    "stamp": {"secs": 1234567890, "nsecs": 123456789},
                    "frame_id": "map"
                },
                "pose": {
                    "position": {"x": 3.1, "y": 4.1, "z": 0.6},
                    "orientation": {"x": 0.1, "y": 0.1, "z": 0.1, "w": 0.9}
                }
            }
        }
    }
    
    # Save test messages to files
    with open('test_input_messages.json', 'w') as f:
        json.dump(input_messages, f, indent=2)
    print("✓ Created test_input_messages.json")
    
    with open('test_output_messages.json', 'w') as f:
        json.dump(output_messages, f, indent=2)
    print("✓ Created test_output_messages.json")
    
    print("\n=== Test Messages Creation Complete ===")

if __name__ == '__main__':
    analyze_architecture_data_flow()
    create_test_messages()