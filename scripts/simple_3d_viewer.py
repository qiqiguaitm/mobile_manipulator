#!/usr/bin/env python3
"""
Simple text-based 3D viewer - no GUI needed
"""

import os
os.environ['LD_PRELOAD'] = '/usr/lib/aarch64-linux-gnu/libffi.so.7'

import rospy
from perception.msg import GraspDetectionArray3D
import sys

def grasp_callback(msg):
    # Clear screen
    print("\033[2J\033[H")  # Clear screen and move cursor to top
    
    print("=" * 70)
    print("                    3D GRASP VISUALIZATION")
    print("=" * 70)
    print(f"Time: {rospy.Time.now().to_sec():.2f}")
    print(f"Objects Detected: {len(msg.detections)}")
    print("-" * 70)
    
    if not msg.detections:
        print("\n⚠️  No objects detected. Place objects in front of camera.")
        return
    
    for i, det in enumerate(msg.detections):
        print(f"\n📦 OBJECT {i+1}")
        print(f"├─ Score: {det.score:.2%}")
        
        if det.center_3d:
            print(f"├─ 3D Position:")
            print(f"│  ├─ X: {det.center_3d.x:+7.3f} m {'←' if det.center_3d.x < 0 else '→'}")
            print(f"│  ├─ Y: {det.center_3d.y:+7.3f} m {'↓' if det.center_3d.y > 0 else '↑'}")
            print(f"│  └─ Z: {det.center_3d.z:+7.3f} m (depth)")
        
        if det.dimensions:
            print(f"├─ Size: {det.dimensions.x:.2f} × {det.dimensions.y:.2f} × {det.dimensions.z:.2f} m")
        
        if det.grasp_poses_3d:
            print(f"└─ Grasps: {len(det.grasp_poses_3d)} poses")
            best_grasp = max(det.grasp_poses_3d, key=lambda g: g.confidence)
            print(f"   └─ Best: {best_grasp.confidence:.2%} confidence, {best_grasp.width:.3f}m width")
    
    print("\n" + "=" * 70)
    print("Press Ctrl+C to exit")

def main():
    rospy.init_node('simple_3d_viewer')
    rospy.Subscriber('/perception/hand/grasps_3d', GraspDetectionArray3D, grasp_callback)
    
    print("Waiting for 3D grasp data...")
    print("Make sure the system is running:")
    print("  roslaunch perception grasp_3d.launch")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\nViewer stopped.")

if __name__ == '__main__':
    main()