# 3D Grasp Detection
float32[4] bbox_2d                # Original 2D bounding box [x1,y1,x2,y2]
float32 score                     # Detection confidence

# 3D information
geometry_msgs/Point center_3d     # 3D center position in camera frame
geometry_msgs/Point dimensions    # 3D bounding box dimensions (width, height, depth)

# Multiple 3D grasp poses
GraspPose3D[] grasp_poses_3d

# 3D touching points
TouchingPoint3D[] touching_points_3d  # Pairs of 3D contact points

# Optional fields (copied from 2D)
string rle                        # RLE encoded mask
sensor_msgs/CompressedImage mask # Segmentation mask
float32[] reid_feature           # Re-identification feature