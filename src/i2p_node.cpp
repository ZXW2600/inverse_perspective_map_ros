/**
 * @file icp_node.cpp
 * @author zxw2600 (zhaoxinwei74@gmail.com)
 * @brief inverse perspective transform a image to a pointcloud
 * @version 0.1
 * @date 2022-04-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

// global val
// static transform to do inverse perspective
tf::StampedTransform tf_camera_to_car;
sensor_msgs::CameraInfo camera_info;

void callback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
  // if camera info is not inited
  if(camera_info.width==0 && camera_info.height==0)
  {
    camera_info=*info_msg.get();
  }

  try
  {
    auto image = cv_bridge::toCvShare(img_msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  // init ros node
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  // get parameters
  string car_baselink, cam_link;
  ros::param::get("~car_link", car_baselink);
  ros::param::get("~cam_link", cam_link);

  // lookup static transforma
  tf::TransformListener listener;
  try
  {
    listener.lookupTransform(car_baselink, cam_link,
                             ros::Time(0), tf_camera_to_car);
    ROS_INFO("get Transform from camera to car successfully!");
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  // subscribe to image, register callback
  image_transport::ImageTransport it(nh);
  image_transport::CameraSubscriber sub = it.subscribeCamera("/prius/front_camera", 10, callback);

  // register the publisher of pointcloud

  ros::spin();
}