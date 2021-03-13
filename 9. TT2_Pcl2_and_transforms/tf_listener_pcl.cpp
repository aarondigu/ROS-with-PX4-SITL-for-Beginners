#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
//#include "sensor_msgs/point_cloud_conversion.h"   
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
//#include <pcl/ros/conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>


//sensor_msgs::PointCloud point_cloud;


sensor_msgs::PointCloud2 pcl2_camera;

void pcl2_cam_call(const sensor_msgs::PointCloud2ConstPtr& msg){
  pcl2_camera = *msg;
 // sensor_msgs::convertPointCloud2ToPointCloud(*msg, point_cloud);
  //ROS_INFO("inside callback");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::Subscriber pcl2_cam_sub = node.subscribe("camera/depth/points", 1, pcl2_cam_call);
  //<sensor_msgs::PointCloud2>
  ros::Publisher pcl2_tf = 
    node.advertise<sensor_msgs::PointCloud2>("transformed_pcl2", 1);

  //ros::Publisher pcl_pub = 
  //  node.advertise<sensor_msgs::PointCloud>("pcl_base", 1);

  ros::Rate rate(10.0);

  tf::TransformListener listener;

  ros::Duration(5.0).sleep();
  ROS_INFO("Ready...");

  //ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("camera_link", "base_link",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    sensor_msgs::PointCloud2 pcl2_base;
    //pcl2_base = pcl2_camera;
    pcl_ros::transformPointCloud("base_link", pcl2_camera, pcl2_base, listener); 
    //sensor_msgs::convertPointCloudToPointCloud2(point_cloud, point_cloud2);
    
    pcl2_tf.publish(pcl2_base);
    //pcl_pub.publish(point_cloud);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};