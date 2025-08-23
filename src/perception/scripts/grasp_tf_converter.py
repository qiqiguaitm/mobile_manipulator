#!/usr/bin/python3
"""
Grasp TF Converter - Transforms camera frame grasps to arm base frame
Publishes 6DoF poses that can be directly used by MoveIt
"""


import rospy
import numpy as np
import tf2_ros
from perception.msg import GraspDetectionArray3D, GraspPose3D
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion, TransformStamped
from moveit_msgs.msg import Grasp
from trajectory_msgs.msg import JointTrajectoryPoint
import tf.transformations as tf_trans


class GraspTFConverter:
    def __init__(self):
        rospy.init_node('grasp_tf_converter')
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Frame names
        self.camera_frame = rospy.get_param('~camera_frame', 'camera/hand_color_optical_frame')
        self.arm_base_frame = rospy.get_param('~arm_base_frame', 'piper_link0')
        self.mobile_base_frame = rospy.get_param('~mobile_base_frame', 'base_link')
        self.world_frame = rospy.get_param('~world_frame', 'map')  # or 'odom'
        self.end_effector_frame = rospy.get_param('~end_effector_frame', 'piper_gripper')
        
        # Target frame mode: 'arm_base', 'mobile_base', or 'world'
        self.target_frame_mode = rospy.get_param('~target_frame_mode', 'mobile_base')
        
        # Grasp parameters
        self.approach_distance = rospy.get_param('~approach_distance', 0.10)  # 10cm
        self.retreat_distance = rospy.get_param('~retreat_distance', 0.10)
        self.grasp_offset = rospy.get_param('~grasp_offset', 0.02)  # 2cm from surface
        
        # Publishers
        self.pose_pub = rospy.Publisher(
            '/arm_planner/grasp_poses',
            PoseArray,
            queue_size=1
        )
        
        self.moveit_grasp_pub = rospy.Publisher(
            '/arm_planner/moveit_grasps',
            Grasp,
            queue_size=10
        )
        
        # Subscriber
        self.grasp_sub = rospy.Subscriber(
            '/perception/hand/grasps_3d',
            GraspDetectionArray3D,
            self.grasp_callback
        )
        
        # Select target frame based on mode
        if self.target_frame_mode == 'world':
            self.target_frame = self.world_frame
        elif self.target_frame_mode == 'mobile_base':
            self.target_frame = self.mobile_base_frame
        else:  # arm_base
            self.target_frame = self.arm_base_frame
        
        rospy.loginfo(f"Grasp TF converter initialized")
        rospy.loginfo(f"  Camera frame: {self.camera_frame}")
        rospy.loginfo(f"  Target frame mode: {self.target_frame_mode}")
        rospy.loginfo(f"  Target frame: {self.target_frame}")
        
    def grasp_callback(self, msg):
        """Process 3D grasps and transform to target frame"""
        try:
            # Get transform from camera to target frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            # Create pose array for visualization
            pose_array = PoseArray()
            pose_array.header.frame_id = self.target_frame
            pose_array.header.stamp = rospy.Time.now()
            
            # Process each detection
            for detection in msg.detections:
                # Get best grasp pose (highest confidence)
                if not detection.grasp_poses_3d:
                    continue
                    
                best_grasp = max(detection.grasp_poses_3d, 
                               key=lambda g: g.confidence)
                
                # Transform to arm frame
                arm_pose = self.transform_grasp(best_grasp, transform)
                if arm_pose:
                    pose_array.poses.append(arm_pose)
                    
                    # Create MoveIt grasp message
                    moveit_grasp = self.create_moveit_grasp(
                        arm_pose, best_grasp.width, best_grasp.confidence
                    )
                    self.moveit_grasp_pub.publish(moveit_grasp)
            
            # Publish pose array for visualization
            if pose_array.poses:
                self.pose_pub.publish(pose_array)
                rospy.loginfo(f"Published {len(pose_array.poses)} grasp poses in {self.target_frame}")
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF error: {e}")
    
    def transform_grasp(self, grasp_3d, transform):
        """Transform grasp from camera to arm frame"""
        try:
            # Manual transform without tf2_geometry_msgs
            # Extract translation and rotation from transform
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            # Convert quaternion to rotation matrix
            q_rot = [rot.x, rot.y, rot.z, rot.w]
            rot_matrix = tf_trans.quaternion_matrix(q_rot)
            
            # Apply transform to position
            pos_camera = np.array([grasp_3d.position.x, grasp_3d.position.y, grasp_3d.position.z, 1])
            pos_transformed = rot_matrix.dot(pos_camera)
            
            # Create transformed pose
            arm_pose = Pose()
            arm_pose.position.x = pos_transformed[0] + trans.x
            arm_pose.position.y = pos_transformed[1] + trans.y
            arm_pose.position.z = pos_transformed[2] + trans.z
            
            # Transform orientation
            q_grasp = [grasp_3d.orientation.x, grasp_3d.orientation.y, 
                      grasp_3d.orientation.z, grasp_3d.orientation.w]
            q_combined = tf_trans.quaternion_multiply(q_rot, q_grasp)
            arm_pose.orientation = Quaternion(x=q_combined[0], y=q_combined[1],
                                             z=q_combined[2], w=q_combined[3])
            
            # Adjust grasp approach
            arm_pose = self.adjust_grasp_approach(arm_pose)
            
            return arm_pose
            
        except Exception as e:
            rospy.logerr(f"Transform error: {e}")
            return None
    
    def adjust_grasp_approach(self, pose):
        """Adjust grasp to approach from above with proper orientation"""
        # Get current orientation as matrix
        q = [pose.orientation.x, pose.orientation.y, 
             pose.orientation.z, pose.orientation.w]
        rot_matrix = tf_trans.quaternion_matrix(q)
        
        # Adjust for top-down grasp (gripper pointing down)
        # This depends on your gripper mounting
        approach_rotation = tf_trans.euler_matrix(0, np.pi, 0)  # Rotate 180 around Y
        
        # Combine rotations
        final_rotation = np.dot(rot_matrix, approach_rotation)
        final_quat = tf_trans.quaternion_from_matrix(final_rotation)
        
        # Update pose
        pose.orientation = Quaternion(
            x=final_quat[0], y=final_quat[1], 
            z=final_quat[2], w=final_quat[3]
        )
        
        # Add vertical offset for approach
        pose.position.z += self.grasp_offset
        
        return pose
    
    def create_moveit_grasp(self, pose, width, confidence):
        """Create MoveIt grasp message"""
        grasp = Grasp()
        
        # Grasp pose (MoveIt always needs arm_base frame)
        # If target is not arm_base, we need additional transform
        grasp.grasp_pose.header.frame_id = self.arm_base_frame if self.target_frame_mode == 'arm_base' else self.target_frame
        grasp.grasp_pose.header.stamp = rospy.Time.now()
        grasp.grasp_pose.pose = pose
        
        # Pre-grasp approach
        grasp.pre_grasp_approach.direction.header.frame_id = self.arm_base_frame
        grasp.pre_grasp_approach.direction.vector.z = -1.0  # Approach from above
        grasp.pre_grasp_approach.min_distance = 0.05
        grasp.pre_grasp_approach.desired_distance = self.approach_distance
        
        # Post-grasp retreat
        grasp.post_grasp_retreat.direction.header.frame_id = self.arm_base_frame
        grasp.post_grasp_retreat.direction.vector.z = 1.0  # Retreat upward
        grasp.post_grasp_retreat.min_distance = 0.05
        grasp.post_grasp_retreat.desired_distance = self.retreat_distance
        
        # Gripper posture
        grasp.pre_grasp_posture.joint_names = ['piper_gripper_finger_joint']
        pre_grasp_point = JointTrajectoryPoint()
        pre_grasp_point.positions = [width]  # Open to object width
        pre_grasp_point.time_from_start = rospy.Duration(1.0)
        grasp.pre_grasp_posture.points.append(pre_grasp_point)
        
        grasp.grasp_posture.joint_names = ['piper_gripper_finger_joint']
        grasp_point = JointTrajectoryPoint()
        grasp_point.positions = [0.0]  # Close gripper
        grasp_point.time_from_start = rospy.Duration(1.0)
        grasp.grasp_posture.points.append(grasp_point)
        
        # Quality
        grasp.grasp_quality = confidence
        
        # Allowed touch objects (if scene is known)
        grasp.allowed_touch_objects = []
        
        return grasp


def main():
    try:
        converter = GraspTFConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()