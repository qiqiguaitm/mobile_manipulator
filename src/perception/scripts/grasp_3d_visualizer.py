#!/usr/bin/python3
"""
3D Grasp Visualizer - Visualizes grasp detections in RViz
Shows bounding boxes, grasp poses, and touching points
"""


import rospy
import numpy as np
from perception.msg import GraspDetectionArray3D
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
import tf.transformations as tf_trans


class Grasp3DVisualizer:
    def __init__(self):
        rospy.init_node('grasp_3d_visualizer')
        
        # Visualization parameters
        self.show_bbox = rospy.get_param('~show_bbox', True)
        self.show_grasp_poses = rospy.get_param('~show_grasp_poses', True)
        self.show_touching_points = rospy.get_param('~show_touching_points', True)
        self.show_labels = rospy.get_param('~show_labels', True)
        self.max_grasps_per_object = rospy.get_param('~max_grasps_per_object', 3)
        
        # Publishers
        self.marker_pub = rospy.Publisher(
            '/perception/grasp_3d_markers',
            MarkerArray,
            queue_size=1
        )
        
        # Subscriber
        self.grasp_sub = rospy.Subscriber(
            '/perception/hand/grasps_3d',
            GraspDetectionArray3D,
            self.grasp_callback
        )
        
        self.marker_id = 0
        rospy.loginfo("3D Grasp visualizer started")
    
    def grasp_callback(self, msg):
        """Generate visualization markers for grasps"""
        marker_array = MarkerArray()
        self.marker_id = 0
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Process each detection
        for i, detection in enumerate(msg.detections):
            # 1. Bounding box
            if self.show_bbox:
                bbox_marker = self.create_bbox_marker(
                    detection, msg.header, i
                )
                marker_array.markers.append(bbox_marker)
            
            # 2. Object center
            center_marker = self.create_center_marker(
                detection, msg.header, i
            )
            marker_array.markers.append(center_marker)
            
            # 3. Grasp poses (gripper visualization)
            if self.show_grasp_poses:
                grasp_markers = self.create_grasp_markers(
                    detection, msg.header, i
                )
                marker_array.markers.extend(grasp_markers)
            
            # 4. Touching points
            if self.show_touching_points:
                touch_markers = self.create_touching_markers(
                    detection, msg.header, i
                )
                marker_array.markers.extend(touch_markers)
            
            # 5. Text labels
            if self.show_labels:
                label_marker = self.create_label_marker(
                    detection, msg.header, i
                )
                marker_array.markers.append(label_marker)
        
        # Publish all markers
        self.marker_pub.publish(marker_array)
        rospy.loginfo(f"Published {len(marker_array.markers)} visualization markers")
    
    def create_bbox_marker(self, detection, header, obj_id):
        """Create 3D bounding box marker"""
        marker = Marker()
        marker.header = header
        marker.ns = "bbox"
        marker.id = self.get_marker_id()
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        # Box corners
        center = detection.center_3d
        dims = detection.dimensions
        
        # Create wireframe cube using lines
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.002  # Line width
        
        # Define 8 corners of the box
        half_x = dims.x / 2
        half_y = dims.y / 2
        half_z = dims.z / 2
        
        corners = [
            [center.x - half_x, center.y - half_y, center.z - half_z],
            [center.x + half_x, center.y - half_y, center.z - half_z],
            [center.x + half_x, center.y + half_y, center.z - half_z],
            [center.x - half_x, center.y + half_y, center.z - half_z],
            [center.x - half_x, center.y - half_y, center.z + half_z],
            [center.x + half_x, center.y - half_y, center.z + half_z],
            [center.x + half_x, center.y + half_y, center.z + half_z],
            [center.x - half_x, center.y + half_y, center.z + half_z]
        ]
        
        # Define edges
        edges = [
            (0,1), (1,2), (2,3), (3,0),  # Bottom face
            (4,5), (5,6), (6,7), (7,4),  # Top face
            (0,4), (1,5), (2,6), (3,7)   # Vertical edges
        ]
        
        for edge in edges:
            p1 = Point(x=corners[edge[0]][0], y=corners[edge[0]][1], z=corners[edge[0]][2])
            p2 = Point(x=corners[edge[1]][0], y=corners[edge[1]][1], z=corners[edge[1]][2])
            marker.points.append(p1)
            marker.points.append(p2)
        
        # Color based on confidence
        marker.color = self.get_confidence_color(detection.score)
        
        marker.lifetime = rospy.Duration(0.5)
        return marker
    
    def create_center_marker(self, detection, header, obj_id):
        """Create sphere at object center"""
        marker = Marker()
        marker.header = header
        marker.ns = "center"
        marker.id = self.get_marker_id()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position = detection.center_3d
        marker.pose.orientation.w = 1.0
        
        marker.scale = Vector3(x=0.02, y=0.02, z=0.02)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        
        marker.lifetime = rospy.Duration(0.5)
        return marker
    
    def create_grasp_markers(self, detection, header, obj_id):
        """Create gripper visualization for grasp poses"""
        markers = []
        
        # Sort grasps by confidence
        sorted_grasps = sorted(detection.grasp_poses_3d, 
                              key=lambda g: g.confidence, reverse=True)
        
        # Show top N grasps
        for i, grasp in enumerate(sorted_grasps[:self.max_grasps_per_object]):
            # Gripper frame (use arrow to show grasp pose)
            gripper_marker = Marker()
            gripper_marker.header = header
            gripper_marker.ns = f"gripper_{obj_id}"
            gripper_marker.id = self.get_marker_id()
            gripper_marker.type = Marker.ARROW
            gripper_marker.action = Marker.ADD
            
            # Set pose
            gripper_marker.pose.position = grasp.position
            gripper_marker.pose.orientation = grasp.orientation
            
            # Scale (arrow size)
            gripper_marker.scale = Vector3(x=0.05, y=0.01, z=0.01)
            
            # Color by rank
            if i == 0:  # Best grasp
                gripper_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)
            else:
                gripper_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5)
            
            gripper_marker.lifetime = rospy.Duration(0.5)
            markers.append(gripper_marker)
            
            # Grasp approach arrow
            arrow_marker = Marker()
            arrow_marker.header = header
            arrow_marker.ns = f"approach_{obj_id}"
            arrow_marker.id = self.get_marker_id()
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            
            # Arrow points from approach position to grasp position
            approach_point = Point(
                x=grasp.position.x,
                y=grasp.position.y,
                z=grasp.position.z + 0.1  # 10cm above
            )
            arrow_marker.points = [approach_point, grasp.position]
            
            arrow_marker.scale = Vector3(x=0.005, y=0.01, z=0.01)  # Shaft, head width, head length
            arrow_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)
            
            arrow_marker.lifetime = rospy.Duration(0.5)
            markers.append(arrow_marker)
            
            # Gripper width visualization
            width_marker = Marker()
            width_marker.header = header
            width_marker.ns = f"width_{obj_id}"
            width_marker.id = self.get_marker_id()
            width_marker.type = Marker.LINE_STRIP
            width_marker.action = Marker.ADD
            
            # Create line showing gripper opening
            half_width = grasp.width / 2
            q = [grasp.orientation.x, grasp.orientation.y, 
                 grasp.orientation.z, grasp.orientation.w]
            rot_matrix = tf_trans.quaternion_matrix(q)
            
            # Local gripper coordinates
            left_point_local = np.array([0, -half_width, 0, 1])
            right_point_local = np.array([0, half_width, 0, 1])
            
            # Transform to world
            left_point_world = rot_matrix.dot(left_point_local)
            right_point_world = rot_matrix.dot(right_point_local)
            
            width_marker.points = [
                Point(x=grasp.position.x + left_point_world[0],
                      y=grasp.position.y + left_point_world[1],
                      z=grasp.position.z + left_point_world[2]),
                Point(x=grasp.position.x + right_point_world[0],
                      y=grasp.position.y + right_point_world[1],
                      z=grasp.position.z + right_point_world[2])
            ]
            
            width_marker.scale.x = 0.003
            width_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
            
            width_marker.lifetime = rospy.Duration(0.5)
            markers.append(width_marker)
        
        return markers
    
    def create_touching_markers(self, detection, header, obj_id):
        """Create markers for touching points"""
        markers = []
        
        for i, tp in enumerate(detection.touching_points_3d):
            # Line between touching points
            line_marker = Marker()
            line_marker.header = header
            line_marker.ns = f"touching_{obj_id}"
            line_marker.id = self.get_marker_id()
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            line_marker.points = [tp.point1, tp.point2]
            line_marker.scale.x = 0.002
            line_marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)
            
            line_marker.lifetime = rospy.Duration(0.5)
            markers.append(line_marker)
            
            # Spheres at contact points
            for point in [tp.point1, tp.point2]:
                sphere_marker = Marker()
                sphere_marker.header = header
                sphere_marker.ns = f"contact_{obj_id}"
                sphere_marker.id = self.get_marker_id()
                sphere_marker.type = Marker.SPHERE
                sphere_marker.action = Marker.ADD
                
                sphere_marker.pose.position = point
                sphere_marker.pose.orientation.w = 1.0
                
                sphere_marker.scale = Vector3(x=0.005, y=0.005, z=0.005)
                sphere_marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)
                
                sphere_marker.lifetime = rospy.Duration(0.5)
                markers.append(sphere_marker)
        
        return markers
    
    def create_label_marker(self, detection, header, obj_id):
        """Create text label for object"""
        marker = Marker()
        marker.header = header
        marker.ns = "labels"
        marker.id = self.get_marker_id()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position above object
        marker.pose.position.x = detection.center_3d.x
        marker.pose.position.y = detection.center_3d.y
        marker.pose.position.z = detection.center_3d.z + detection.dimensions.z/2 + 0.05
        marker.pose.orientation.w = 1.0
        
        # Text content
        marker.text = f"Obj {obj_id+1}\nScore: {detection.score:.2f}\nGrasps: {len(detection.grasp_poses_3d)}"
        
        marker.scale.z = 0.02  # Text height
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        marker.lifetime = rospy.Duration(0.5)
        return marker
    
    def get_confidence_color(self, score):
        """Get color based on confidence score"""
        if score > 0.8:
            return ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6)  # Green
        elif score > 0.5:
            return ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.6)  # Yellow
        else:
            return ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.6)  # Red
    
    def get_marker_id(self):
        """Get unique marker ID"""
        self.marker_id += 1
        return self.marker_id


def main():
    try:
        visualizer = Grasp3DVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()