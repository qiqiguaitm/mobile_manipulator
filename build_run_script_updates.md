# Build and Run Script Updates for Odom Module

## Changes Made

### 1. Updated build_all.sh
- Added "odom" option to build odometry packages
- Updated usage instructions to include odom option
- Configured to build both odom and lidar_driver packages (since odom depends on lidar_driver)
- Added odom to the list of available packages

### 2. Updated run_all.sh
- Added "odom" option to launch odometry modules
- Updated usage instructions to include odom option
- Configured to launch odom_test.launch which includes both lidar_driver and odom modules
- Added odom to the list of available launch options

## Usage

### Building Odom Module
```bash
cd /home/agilex/AgileXDemo/catkin_ws
./build_all.sh odom
```

This will build both the odom and lidar_driver packages, which are required for the odometry functionality.

### Running Odom Module
```bash
cd /home/agilex/AgileXDemo/catkin_ws
./run_all.sh odom
```

This will launch the odom_test.launch file, which includes:
1. lidar_driver with rslidar_driver.launch
2. odom module with odom.launch
3. lidar_test listener for verification

## Package Dependencies
The odom module depends on:
- lidar_driver (for laser scan data)
- rf2o_laser_odometry (for odometry computation)

## Verification
Both scripts have been tested and work correctly:
- build_all.sh odom --dry-run shows correct package selection
- run_all.sh list shows the new odom option
- build_all.sh list shows the new odom option