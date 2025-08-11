# IMU Topic Unification Documentation

## Overview
This document describes the changes made to unify all IMU-related topics to use the standard `/imu/data` topic instead of the previous `/sensors/imu/data` topic.

## Changes Made

### 1. Configuration Files
- Updated `config/topics.yaml` to use `/imu/data` instead of `/sensors/imu/data`

### 2. Source Code Files
- Modified `scripts/imu_driver_node.py` to publish to `/imu/data`
- Modified `src/imu_driver_complete.cpp` to publish to `/imu/data`
- Modified `src/imu_driver.cpp` to publish to `/imu/data`
- Updated `scripts/imu_test.py` to subscribe to `/imu/data`
- Updated `scripts/test_imu_topic.py` to subscribe to `/imu/data`

### 3. Launch Files
- Updated `test_imu.launch` to echo `/imu/data` instead of `/sensors/imu/data`
- Created new `launch/imu_test_unified.launch` for testing the unified topic

### 4. Build System
- Updated `CMakeLists.txt` to properly install all Python scripts

## Verification
The changes have been compiled successfully and are ready for testing with actual hardware.

## Affected Components
- IMU driver (both Python and C++ versions)
- IMU test scripts
- Any downstream components that consume IMU data

## Migration Notes
All components that previously subscribed to `/sensors/imu/data` should now subscribe to `/imu/data`.