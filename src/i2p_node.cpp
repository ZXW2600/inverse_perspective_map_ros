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
#include <functional>

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>

#include "pcl_ros/publisher.h"

#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>

#include <iostream>
#include <memory>
#include <mutex>

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pcl;

// global val
// static transform to do inverse perspective
geometry_msgs::TransformStamped tf_cam_to_car;

Matrix3d camera_in_car_RT;
Matrix3d camera_in_car_R;
Vector3d camera_in_car_T;

// static camera matrix
sensor_msgs::CameraInfo camera_info;
Matrix3d camera_K;

// publisher
ros::Publisher pointcloud_pub;
ros::Publisher car_pub;
ros::Publisher cam_pub;

// parameters
string car_baselink, cam_link, pointcloud_topic;

typedef cv::Point3_<uint8_t> Pixel;



/**
 * @brief 预处理，提取有效范围的mask
 *
 * @param image
 * @return Mat&&
 */
Mat getmask(Mat image)
{
  return Mat::ones(image.rows, image.cols, CV_8U);
}

void callback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
  static bool map_init = false;

  static Matrix3d KInv = camera_K.inverse();
  static Matrix3d RTInv ;
  static Matrix3d InvPerspectiveMatrix ;

  static size_t img_w;
  static size_t img_h;

  static Eigen::Matrix3Xd u;
  static Eigen::Matrix3Xd u_ground;

  sensor_msgs::PointCloud2 cameraCloud_msg;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cameraCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::mutex pointcloud_lck;

  // init camera info
  if (camera_info.width == 0 && camera_info.height == 0)
  {
    camera_info = *info_msg.get();

    camera_K << camera_info.K[0], camera_info.K[1], camera_info.K[2],
        camera_info.K[3], camera_info.K[4], camera_info.K[5],
        camera_info.K[6], camera_info.K[7], camera_info.K[8];
    cout << "K:" << endl
         << camera_K << endl;
    ROS_INFO("get camera info successfully : %f %f %f %f %f %f %f %f %f", camera_info.K[0], camera_info.K[1], camera_info.K[2], camera_info.K[3], camera_info.K[4], camera_info.K[5], camera_info.K[6], camera_info.K[7], camera_info.K[8]);

    img_w = img_msg->width;
    img_h = img_msg->height;
    u.resize(3, img_w * img_h);
    u_ground.resize(3, img_w * img_h);
  }

  // if publish needed
  else if (pointcloud_pub.getNumSubscribers() >= 0)
  {

    // convert image format
    cv_bridge::CvImageConstPtr image;
    try
    {
      image = cv_bridge::toCvShare(img_msg, "rgb8");
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
    }

    // get mask, add rcnn segmentation later
    Mat mask =getmask(image->image);

    ROS_INFO("start project image point");

    // init lookup table   
    if (!map_init)
    {
      KInv = camera_K.inverse();
      RTInv = camera_in_car_RT.inverse();
      InvPerspectiveMatrix = RTInv * KInv;
      ROS_INFO_STREAM("IP matrix"<<InvPerspectiveMatrix<<std::endl);

      size_t index = 0;
      for (int i = 0; i < img_w; ++i)
      {
        for (int j = 0; j < img_h; ++j)
        {
          u(0, index) = j;
          u(1, index) = i;
          u(2, index++) = 1;
        }
      }
      u_ground = InvPerspectiveMatrix * u;
      ROS_INFO_STREAM(u_ground.rightCols<1>()<<std::endl);

      u_ground.topRows<2>().array().rowwise() /= u_ground.bottomRows<1>().array();
      ROS_INFO_STREAM(u_ground.rightCols<1>()<<std::endl);

      ROS_INFO("lookup table inited");
      map_init=true;
    }

    ROS_INFO("for each image scan");

    // do inverse perspective map
    image->image.forEach<Pixel>([&cameraCloud, &mask,&pointcloud_lck](Pixel &pixel, const int *position)
                                {  

    // if mask is not zero, perspective this point
    if (mask.at<uint8_t>(position[0], position[1]))
    {
      size_t index = position[0] * img_w + position[1];


      // x=cols j, y=rows i
      Eigen::Vector3d uo;
      uo = u_ground.col(index);

      if (uo.z() > 0)
      {
        auto temp=PointXYZRGB(pixel.x, pixel.y, pixel.z);
        temp.x=uo.x();
        temp.y=uo.y();
        temp.z=0;
        pointcloud_lck.lock();
        cameraCloud->points.push_back(temp);
        pointcloud_lck.unlock();

      }
    } });
  }

  // publish msg
  {
    ROS_INFO("Publishing %ld point to %d subscribers.", cameraCloud->size(), pointcloud_pub.getNumSubscribers());

    toROSMsg<PointXYZRGB>(*cameraCloud.get(), cameraCloud_msg);
    cameraCloud_msg.header.frame_id = car_baselink;
    cameraCloud_msg.header.stamp = ros::Time(0);
    pointcloud_pub.publish(cameraCloud_msg);
    ROS_INFO("Publish finished!");
  }
}

int main(int argc, char **argv)
{
  // init ros node
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  // get parameters
  string camera_topic;
  ros::param::get("~car_link", car_baselink);
  ros::param::get("~cam_link", cam_link);
  ros::param::get("~pointcloud_topic", pointcloud_topic);
  ros::param::get("~camera_topic", camera_topic);

  ROS_INFO("car_link: %s cam_link: %s pointcloud: %s", car_baselink.c_str(), cam_link.c_str(), pointcloud_topic.c_str());
  // lookup static transforma

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);

  tf_cam_to_car = tf_buffer.lookupTransform(cam_link, car_baselink, ros::Time(0), ros::Duration(5.0));

  Isometry3d camera_extrinsic;
  tf2::doTransform(Isometry3d::Identity(), camera_extrinsic, tf_cam_to_car); // robot_pose is the PoseStamped I want to transform

  camera_in_car_T = camera_extrinsic.translation();
  camera_in_car_R = camera_extrinsic.rotation();

  camera_in_car_RT.col(0) = camera_in_car_R.col(0);
  camera_in_car_RT.col(1) = camera_in_car_R.col(1);
  camera_in_car_RT.col(2) = camera_in_car_T.col(0);
  cout << camera_in_car_RT << endl;
  ROS_INFO("get Transform from camera to car successfully!");


  // subscribe to image, register callback
  image_transport::ImageTransport it(nh);
  image_transport::CameraSubscriber sub = it.subscribeCamera(camera_topic, 10, callback);

  // register the publisher of pointcloud
  pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1);
  car_pub = nh.advertise<sensor_msgs::PointCloud2>("car", 1);
  cam_pub = nh.advertise<sensor_msgs::PointCloud2>("cam", 1);

  ros::spin();
}