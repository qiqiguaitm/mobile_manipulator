#!/bin/bash
# test_imu_driver.sh

echo "Testing IMU driver..."

# Check if IMU device is available
if [ ! -e /dev/ttyUSB0 ]; then
    echo "Error: IMU device /dev/ttyUSB0 not found"
    exit 1
fi

echo "IMU device /dev/ttyUSB0 found"

# Check if the IMU driver executable exists
if [ ! -f /home/agilex/AgileXDemo/catkin_ws/devel/lib/imu_driver/imu_driver ]; then
    echo "Error: IMU driver executable not found"
    exit 1
fi

echo "IMU driver executable found"

# Try to run the IMU driver with a short timeout
echo "Testing IMU driver connection (5 seconds)..."
timeout 5s /home/agilex/AgileXDemo/catkin_ws/devel/lib/imu_driver/imu_driver 2>&1 | grep -E "(opened|Error|error)" || echo "Test completed or no output"

echo "IMU driver test finished"