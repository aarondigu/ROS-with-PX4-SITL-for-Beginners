#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


sensor_msgs::PointCloud point_cloud;


void pcl_cam_call(const sensor_msgs::PointCloud2ConstPtr& msg){

  sensor_msgs::convertPointCloud2ToPointCloud(*msg, point_cloud);

}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_pcl2_to_pcl_converter");

  ros::NodeHandle node;

  ros::Subscriber pcl2_cam_sub = node.subscribe("transformed_pcl2", 1, pcl_cam_call);


  ros::Publisher pcl_pub = 
    node.advertise<sensor_msgs::PointCloud>("pcl_base", 1);

  ros::Rate rate(10.0);



  //ros::Rate rate(10.0);
  while (node.ok()){

    pcl_pub.publish(point_cloud);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};