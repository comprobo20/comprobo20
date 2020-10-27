#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <math.h>
#include <sensor_msgs/point_cloud_conversion.h>

laser_geometry::LaserProjection* projector_ = 0;
tf::TransformListener* listener_ = 0;
sensor_msgs::PointCloud* prev_cloud = 0;
ros::Publisher pub;
ros::Publisher pub_cloud;



void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  std::cout << "got a message!!!" <<std::endl;
  if (listener_ == 0 || projector_ == 0) {
    return;
  }
  std::cout << "GOT A SCAN!!!" << std::endl;
  sensor_msgs::LaserScan scan_mod(*scan_in);
  if(!listener_->waitForTransform(
        scan_in->header.frame_id,
        "odom",
        scan_in->header.stamp,
        ros::Duration(1.0))){
     return;
  }
  try {
    sensor_msgs::PointCloud2 final_cloud;
    projector_->transformLaserScanToPointCloud("odom",scan_mod,
              final_cloud, *listener_);
    std::string error;
    listener_->getLatestCommonTime("odom", "base_link", final_cloud.header.stamp, &error);
    final_cloud.header.stamp -= ros::Duration(0.05);		// need some slop
    pub_cloud.publish(final_cloud);
  } catch (...) {}
}

int main(int argc, char** argv){
  ros::init(argc, argv, "scan_to_pc2");

  listener_ = new tf::TransformListener();
  projector_ = new laser_geometry::LaserProjection();
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("scan",10,scanCallback);
  pub_cloud = node.advertise<sensor_msgs::PointCloud2>("projected_stable_scan",10);

  ros::Rate rate(10.0);
  while (node.ok()){
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};
