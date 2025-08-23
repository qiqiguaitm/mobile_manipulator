# Grasp detection result for hand camera
float32[4] bbox                    # [x1, y1, x2, y2]
float32 score                      # Object detection confidence
sensor_msgs/CompressedImage mask   # Segmentation mask (compressed)
string rle                         # RLE encoded mask (COCO format)
float32[] reid_feature             # 256-dim feature vector
perception/GraspPose[] grasp_poses # Multiple grasp poses for this object
perception/TouchingPoint[] touching_points # Grasp touching points