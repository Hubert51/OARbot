//
// Created by ruijie on 2/11/20.
//

// tutorial to create a CPP package
// https://www.jetbrains.com/help/clion/ros-setup-tutorial.html

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    // Do data processing here...
    output = *input;
    std::cout << input->width << " " << input->height << std::endl;

    // Publish the data.
    pub.publish (output);
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin ();
}