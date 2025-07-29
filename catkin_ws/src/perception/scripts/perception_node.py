#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

# Try to import cv_bridge and cv2 with comprehensive error handling
cv_bridge_available = False
cv2_available = False
CvBridge = None

# Isolate library loading to prevent cross-environment conflicts
def safe_import_cv_bridge():
    """Safely import cv_bridge with environment isolation"""
    global cv_bridge_available, CvBridge
    
    # Log current Python paths for debugging
    rospy.logdebug("Current Python paths: %s", sys.path)
    
    try:
        # Try importing with current environment first
        from cv_bridge import CvBridge
        cv_bridge_available = True
        rospy.loginfo("Successfully imported cv_bridge")
        return CvBridge
    except ImportError as e:
        rospy.logerr("Failed to import cv_bridge: %s", str(e))
        rospy.logerr("This may be due to missing dependencies or library conflicts.")
    except Exception as e:
        rospy.logerr("Failed to initialize cv_bridge due to library issues: %s", str(e))
        rospy.logerr("This may be due to library version conflicts. The perception node will run but image processing will be disabled.")
    
    return None

# Add system path to find cv2 and isolate from conda environments
def setup_library_paths():
    """Setup library paths to prioritize system libraries"""
    # Log initial paths
    rospy.logdebug("Initial Python paths: %s", sys.path)
    
    # Remove conda paths if they exist to prevent library conflicts
    sys_paths_to_remove = [path for path in sys.path if '/miniconda' in path or '/anaconda' in path]
    for path in sys_paths_to_remove:
        if path in sys.path:
            sys.path.remove(path)
            rospy.loginfo("Removed conda path from sys.path: %s", path)
    
    # Add system paths to find libraries
    system_paths = [
        '/usr/lib/python3/dist-packages',
        '/usr/local/lib/python3.8/dist-packages',
        '/opt/ros/noetic/lib/python3/dist-packages'
    ]
    
    for path in system_paths:
        if path not in sys.path:
            sys.path.insert(0, path)  # Insert at beginning to prioritize
            rospy.logdebug("Added system path to sys.path: %s", path)

setup_library_paths()

# Try to import cv_bridge
CvBridge = safe_import_cv_bridge()

if cv_bridge_available:
    try:
        import cv2
        import numpy as np
        cv2_available = True
        rospy.loginfo("Successfully imported cv2 and numpy")
    except ImportError as e:
        rospy.logerr("Failed to import cv2 or numpy: %s", str(e))
        rospy.logerr("This may be due to missing dependencies or library conflicts.")
        cv_bridge_available = False
    except Exception as e:
        rospy.logerr("Failed to initialize cv2 due to library issues: %s", str(e))
        rospy.logerr("This may be due to library version conflicts. The perception node will run but image processing will be disabled.")
        cv_bridge_available = False

class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node')
        
        # Initialize class variables for cv_bridge availability
        self.cv_bridge_available = cv_bridge_available
        self.cv2_available = cv2_available
        
        # Load topic configurations with error handling
        try:
            topics = rospy.get_param('topics')
        except KeyError:
            rospy.logerr("Failed to get 'topics' parameter from parameter server")
            rospy.signal_shutdown("Missing 'topics' parameter")
            return
        
        # Publishers for perception outputs
        try:
            self.object_pub = rospy.Publisher(topics['outputs']['object_list']['name'], PoseStamped, queue_size=10)
        except KeyError as e:
            rospy.logerr("Failed to get topic name from topics parameter: %s", str(e))
            rospy.signal_shutdown("Invalid topics parameter structure")
            return
        
        # Initialize bridge to None
        self.bridge = None
        
        # Subscribers for camera inputs
        if self.cv_bridge_available and self.cv2_available:
            try:
                self.bridge = CvBridge()
                self.top_camera_sub = rospy.Subscriber(topics['inputs']['top_camera']['name'], Image, self.top_camera_callback)
                self.hand_camera_sub = rospy.Subscriber(topics['inputs']['hand_camera']['name'], Image, self.hand_camera_callback)
                self.chassis_camera_sub = rospy.Subscriber(topics['inputs']['chassis_camera']['name'], Image, self.chassis_camera_callback)
                rospy.loginfo("Camera subscribers created successfully")
            except KeyError as e:
                rospy.logerr("Failed to get camera topic names from topics parameter: %s", str(e))
                rospy.signal_shutdown("Invalid topics parameter structure")
                return
            except ImportError as e:
                rospy.logerr("Import error during camera subscriber initialization: %s", str(e))
                rospy.logerr("This may be due to library version conflicts.")
                self.cv_bridge_available = False
                self.cv2_available = False
                self.bridge = None
            except Exception as e:
                rospy.logerr("Failed to initialize camera subscribers: %s", str(e))
                # Check if this is a symbol lookup error
                if "undefined symbol" in str(e) or "symbol lookup error" in str(e):
                    rospy.logerr("Library symbol conflict detected during initialization. This is likely due to mixed conda/system libraries.")
                rospy.logwarn("Image processing will be disabled due to initialization errors")
                self.cv_bridge_available = False
                self.cv2_available = False
                self.bridge = None
        else:
            rospy.logwarn("Image processing is disabled due to missing cv_bridge or cv2. Camera subscribers will not be created.")
            
        # Create a fixed pose message for continuous publishing
        self.fixed_pose = PoseStamped()
        self.fixed_pose.header.frame_id = "fixed_frame"
        self.fixed_pose.pose.position.x = 1.0
        self.fixed_pose.pose.position.y = 2.0
        self.fixed_pose.pose.position.z = 3.0
        
        # Check if cv2 is available
        if not self.cv2_available:
            rospy.logwarn("Image processing is disabled due to cv2 library issues")
        
        rospy.loginfo("Perception node initialized")

    def top_camera_callback(self, msg):
        """Process top camera images"""
        if not self.cv_bridge_available or not self.cv2_available or self.bridge is None:
            rospy.logdebug("Image processing skipped - cv_bridge or cv2 not available")
            return
            
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Process image (placeholder for actual perception logic)
            rospy.logdebug("Received top camera image: %dx%d", cv_image.shape[1], cv_image.shape[0])
        except ImportError as e:
            rospy.logerr("Import error in top camera callback: %s", str(e))
            rospy.logerr("This may be due to library version conflicts.")
        except Exception as e:
            rospy.logerr("Error processing top camera image: %s", str(e))
            # Check if this is a symbol lookup error
            if "undefined symbol" in str(e) or "symbol lookup error" in str(e):
                rospy.logerr("Library symbol conflict detected. This is likely due to mixed conda/system libraries.")
            rospy.logerr("Disabling image processing due to persistent errors")
            # Disable image processing to prevent continuous errors
            self.cv_bridge_available = False
            self.cv2_available = False
            self.bridge = None
            import traceback
            rospy.logerr("Traceback: %s", traceback.format_exc())

    def hand_camera_callback(self, msg):
        """Process hand camera images"""
        if not self.cv_bridge_available or not self.cv2_available or self.bridge is None:
            rospy.logdebug("Image processing skipped - cv_bridge or cv2 not available")
            return
            
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Process image (placeholder for actual perception logic)
            rospy.logdebug("Received hand camera image: %dx%d", cv_image.shape[1], cv_image.shape[0])
        except ImportError as e:
            rospy.logerr("Import error in hand camera callback: %s", str(e))
            rospy.logerr("This may be due to library version conflicts.")
        except Exception as e:
            rospy.logerr("Error processing hand camera image: %s", str(e))
            # Check if this is a symbol lookup error
            if "undefined symbol" in str(e) or "symbol lookup error" in str(e):
                rospy.logerr("Library symbol conflict detected. This is likely due to mixed conda/system libraries.")
            rospy.logerr("Disabling image processing due to persistent errors")
            # Disable image processing to prevent continuous errors
            self.cv_bridge_available = False
            self.cv2_available = False
            self.bridge = None
            import traceback
            rospy.logerr("Traceback: %s", traceback.format_exc())

    def chassis_camera_callback(self, msg):
        """Process chassis camera images"""
        if not self.cv_bridge_available or not self.cv2_available or self.bridge is None:
            rospy.logdebug("Image processing skipped - cv_bridge or cv2 not available")
            return
            
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Process image (placeholder for actual perception logic)
            rospy.logdebug("Received chassis camera image: %dx%d", cv_image.shape[1], cv_image.shape[0])
        except ImportError as e:
            rospy.logerr("Import error in chassis camera callback: %s", str(e))
            rospy.logerr("This may be due to library version conflicts.")
        except Exception as e:
            rospy.logerr("Error processing chassis camera image: %s", str(e))
            # Check if this is a symbol lookup error
            if "undefined symbol" in str(e) or "symbol lookup error" in str(e):
                rospy.logerr("Library symbol conflict detected. This is likely due to mixed conda/system libraries.")
            rospy.logerr("Disabling image processing due to persistent errors")
            # Disable image processing to prevent continuous errors
            self.cv_bridge_available = False
            self.cv2_available = False
            self.bridge = None
            import traceback
            rospy.logerr("Traceback: %s", traceback.format_exc())

    def run(self):
        """Main loop - continuously publish fixed output"""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Update timestamp
            self.fixed_pose.header.stamp = rospy.Time.now()
            # Publish fixed pose
            self.object_pub.publish(self.fixed_pose)
            rospy.logdebug("Published fixed pose: x=%f, y=%f, z=%f", 
                          self.fixed_pose.pose.position.x,
                          self.fixed_pose.pose.position.y,
                          self.fixed_pose.pose.position.z)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = PerceptionNode()
        # Check if node was properly initialized
        if hasattr(node, 'object_pub'):
            node.run()
        else:
            rospy.logerr("Perception node failed to initialize properly")
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Unhandled exception in perception node: %s", str(e))