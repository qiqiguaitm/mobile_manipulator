// imu_driver.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sensor_msgs/Imu.h>
#include <signal.h>

#ifdef __cplusplus 
extern "C"{
#endif

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
}
#endif

#define IMU_SERIAL   "/dev/ttyUSB0"
#define BAUD         (115200)
#define GRA_ACC      (9.8)
#define DEG_TO_RAD   (0.01745329)
#define BUF_SIZE     1024

// Simple IMU data structure for parsing data
typedef struct {
    float acc[3];
    float gyr[3];
    float mag[3]; 
    float quat[4];
} imu_data_t;

static uint8_t buf[2048];
static imu_data_t current_imu_data;
static bool data_available = false;

// Simple parser for IMU data (this is a simplified version)
// In a real implementation, you would need to implement the full packet parsing
// based on the IMU protocol documentation
bool parse_imu_data(uint8_t* buffer, size_t len, imu_data_t* data) {
    // This is a placeholder implementation
    // In a real implementation, you would parse the actual IMU protocol packets
    // For now, we'll just simulate receiving data
    static int counter = 0;
    counter++;
    
    // Simulate some data changes
    data->acc[0] = sin(counter * 0.1) * 0.1;
    data->acc[1] = cos(counter * 0.1) * 0.1;
    data->acc[2] = 9.8;
    
    data->gyr[0] = sin(counter * 0.05) * 0.05;
    data->gyr[1] = cos(counter * 0.05) * 0.05;
    data->gyr[2] = 0.0;
    
    data->mag[0] = sin(counter * 0.02) * 0.2;
    data->mag[1] = cos(counter * 0.02) * 0.2;
    data->mag[2] = 0.5;
    
    // Simple quaternion from euler angles (assuming small angles)
    data->quat[0] = 1.0;  // w
    data->quat[1] = 0.0;  // x
    data->quat[2] = 0.0;  // y
    data->quat[3] = 0.0;  // z
    
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_driver");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    // Get parameters
    std::string port;
    int baud;
    private_nh.param<std::string>("port", port, IMU_SERIAL);
    private_nh.param<int>("baud", baud, BAUD);

    // Publisher for IMU data
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/sensors/imu/data", 20);

    serial::Serial sp;

    serial::Timeout to = serial::Timeout::simpleTimeout(100);

    sp.setPort(port);
    sp.setBaudrate(baud);
    sp.setTimeout(to);
    
    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port " << port);
        return -1;
    }
    
    if(sp.isOpen())
    {
        ROS_INFO_STREAM(port << " is opened.");
    }
    else
    {
        return -1;
    }
    
    ros::Rate loop_rate(100); // 100 Hz
    sensor_msgs::Imu imu_msg;

    while(ros::ok())
    {
        size_t num = sp.available();
        if(num != 0)
        {
            uint8_t buffer[BUF_SIZE]; 

            if(num > BUF_SIZE)
                num = BUF_SIZE;
            
            num = sp.read(buffer, num);
            if(num > 0)
            {
                // Parse IMU data from buffer
                if(parse_imu_data(buffer, num, &current_imu_data))
                {
                    // Fill IMU message
                    imu_msg.header.stamp = ros::Time::now();
                    imu_msg.header.frame_id = "imu_link";
                    
                    // Orientation (quaternion)
                    imu_msg.orientation.x = current_imu_data.quat[1];
                    imu_msg.orientation.y = current_imu_data.quat[2];
                    imu_msg.orientation.z = current_imu_data.quat[3];
                    imu_msg.orientation.w = current_imu_data.quat[0];
                    
                    // Angular velocity
                    imu_msg.angular_velocity.x = current_imu_data.gyr[0] * DEG_TO_RAD;
                    imu_msg.angular_velocity.y = current_imu_data.gyr[1] * DEG_TO_RAD;
                    imu_msg.angular_velocity.z = current_imu_data.gyr[2] * DEG_TO_RAD;
                    
                    // Linear acceleration
                    imu_msg.linear_acceleration.x = current_imu_data.acc[0] * GRA_ACC;
                    imu_msg.linear_acceleration.y = current_imu_data.acc[1] * GRA_ACC;
                    imu_msg.linear_acceleration.z = current_imu_data.acc[2] * GRA_ACC;
                    
                    // Publish IMU data
                    imu_pub.publish(imu_msg);
                }
            }
        }
        loop_rate.sleep();
    }
    
    sp.close();
 
    return 0;
}