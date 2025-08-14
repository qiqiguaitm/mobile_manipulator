#!/usr/bin/env python3

import sys
import time
import threading
from collections import defaultdict

# Mock ROS classes for testing without ROS
class MockROS:
    def __init__(self):
        self.topics = defaultdict(list)  # topic -> [callbacks]
        self.published_messages = []  # [(topic, message)]
        
    def Publisher(self, topic, msg_type, queue_size=10):
        return MockPublisher(topic, msg_type, self)
        
    def Subscriber(self, topic, msg_type, callback):
        return MockSubscriber(topic, msg_type, callback, self)
        
    def init_node(self, name):
        print(f"Initialized mock node: {name}")
        
    def loginfo(self, msg):
        print(f"[INFO] {msg}")
        
    def logdebug(self, msg):
        print(f"[DEBUG] {msg}")
        
    def is_shutdown(self):
        return False
        
    def Rate(self, hz):
        return MockRate(hz)

class MockPublisher:
    def __init__(self, topic, msg_type, mock_ros):
        self.topic = topic
        self.msg_type = msg_type
        self.mock_ros = mock_ros
        
    def publish(self, msg):
        print(f"Published to {self.topic}: {msg}")
        self.mock_ros.published_messages.append((self.topic, msg))

class MockSubscriber:
    def __init__(self, topic, msg_type, callback, mock_ros):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback
        self.mock_ros = mock_ros
        mock_ros.topics[topic].append(callback)
        
    def simulate_message(self, msg):
        """Simulate receiving a message"""
        for callback in self.mock_ros.topics[self.topic]:
            callback(msg)

class MockRate:
    def __init__(self, hz):
        self.interval = 1.0 / hz
        
    def sleep(self):
        time.sleep(self.interval)

# Mock message classes
class MockPoseStamped:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z
        self.header = MockHeader()
        
    def __str__(self):
        return f"PoseStamped(x={self.x}, y={self.y}, z={self.z})"

class MockJointState:
    def __init__(self, name=None, position=None):
        self.name = name or []
        self.position = position or []
        self.header = MockHeader()
        
    def __str__(self):
        return f"JointState(name={self.name}, position={self.position})"

class MockArmTrajectory:
    def __init__(self, joints=None, points=None):
        self.joint_names = joints or []
        self.points = points or []
        self.header = MockHeader()
        
    def __str__(self):
        return f"ArmTrajectory(joints={self.joint_names}, points={len(self.points)} points)"

class MockHeader:
    def __init__(self):
        self.stamp = time.time()
        self.frame_id = ""

# Mock rospy
rospy = MockROS()

# Import the actual node code
sys.path.insert(0, 'src/arm_planner/scripts')
sys.path.insert(0, 'src/arm_planner/msg')

# Mock the message types
import sys
sys.modules['sensor_msgs.msg'] = type('sensor_msgs.msg', (), {
    'JointState': MockJointState
})
sys.modules['geometry_msgs.msg'] = type('geometry_msgs.msg', (), {
    'PoseStamped': MockPoseStamped
})
sys.modules['arm_planner.msg'] = type('arm_planner.msg', (), {
    'ArmTrajectory': MockArmTrajectory
})

# Now we can import the node
try:
    from arm_planner_node import ArmPlannerNode
    print("Successfully imported ArmPlannerNode")
except Exception as e:
    print(f"Error importing ArmPlannerNode: {e}")
    sys.exit(1)

def test_data_flow():
    """Test the data flow between modules"""
    print("Starting data flow test...")
    
    # Create the arm planner node
    node = ArmPlannerNode()
    
    # Simulate receiving messages on input topics
    print("\n--- Testing Input Messages ---")
    
    # Simulate octomap data
    map_msg = MockPoseStamped(1.0, 2.0, 3.0)
    print("Simulating map data...")
    for sub in node.map_sub.mock_ros.topics['/map/octomap']:
        sub(map_msg)
    
    # Simulate chassis pose
    chassis_msg = MockPoseStamped(0.5, 1.0, 0.0)
    print("Simulating chassis pose...")
    for sub in node.chassis_pose_sub.mock_ros.topics['/chassis/pose']:
        sub(chassis_msg)
    
    # Simulate target pose
    target_msg = MockPoseStamped(2.0, 3.0, 1.0)
    print("Simulating target pose...")
    for sub in node.target_pose_sub.mock_ros.topics['/task_mgr/target_object/pose']:
        sub(target_msg)
    
    # Simulate joint states
    joint_msg = MockJointState(['joint1', 'joint2'], [0.1, 0.2])
    print("Simulating joint states...")
    for sub in node.joint_state_sub.mock_ros.topics['/arm/joint_states']:
        sub(joint_msg)
    
    print("\n--- Checking Output Messages ---")
    
    # Check if messages were published to output topics
    output_messages = node.trajectory_pub.mock_ros.published_messages + node.end_pose_pub.mock_ros.published_messages
    
    if output_messages:
        print(f"Found {len(output_messages)} output messages:")
        for topic, msg in output_messages:
            print(f"  {topic}: {msg}")
    else:
        print("No output messages found")
    
    print("\nData flow test completed!")

if __name__ == '__main__':
    test_data_flow()