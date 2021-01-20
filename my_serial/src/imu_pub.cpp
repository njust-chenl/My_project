// Step 1:  Include Library Headers:
//发布imu的数据到IMU_data
#include <ros/ros.h> 
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


int main(int argc, char** argv)
{
    // Step 2: Initialization:
    ros::init(argc, argv, "imu");     
    ros::NodeHandle n;  
    
    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("IMU_data", 20);  

    ros::Rate loop_rate(50);  
    while(ros::ok())
    {
                   
            sensor_msgs::Imu imu_data;
            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = "imu";
          
            //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
           imu_data.orientation.x = -0;
            imu_data.orientation.y = -0;
            imu_data.orientation.z = 0;
            imu_data.orientation.w = 0;
            //线加速度
            imu_data.linear_acceleration.x = 0.00; 
            imu_data.linear_acceleration.y = 0.00;
            imu_data.linear_acceleration.z = 9.8;
	    //角速度
            imu_data.angular_velocity.x = 0.00; 
            imu_data.angular_velocity.y = 0.00; 
            imu_data.angular_velocity.z = 0;
 
 
 
            IMU_pub.publish(imu_data);
       
 
       
        ros::spinOnce();  
        loop_rate.sleep();  
    }
 
    return 0;
}
