#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

// ROS Messages
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

class PointCloudPublisher{
private:
  ros::NodeHandle nh;
  ros::Subscriber pcl_subscriber;
  ros::Publisher pcl_publisher;

  tf::TransformBroadcaster broadcaster;
  tf::TransformListener listener;

  pcl::PointCloud<pcl::PointXYZ> subscribed_pcl; // subscribeしたpoint cloudを保存する変数

  std::string subscribe_pcl_name = "/camera/depth/points";
  std::string publish_pcl_name;

  std::string camera_link_name = "camera_link";
  std::string base_link_name = "camera_link";

  bool is_get_pcl = false; // PointCloudをSubscribeしたらTrue

public:
  PointCloudPublisher(){
    this->pcl_subscriber = nh.subscribe(subscribe_pcl_name, 1, &PointCloudPublisher::point_cloud_CB, this);
    this->pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>("/pcl_tutorial/result_point_cloud", 1);

    ROS_INFO("### Point Cloud Publisher initialize OK ###");
  }

  void point_cloud_CB(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, subscribed_pcl);

    // SubscribeしたPoint Cloudが空の場合
    if(subscribed_pcl.points.size() == 0){
      ROS_WARN("No Point Cloud");
      this->is_get_pcl = false;
      return;
    }
    else{
      //ROS_INFO("Get Point Cloud");
      this->is_get_pcl = true;
      // ###########################################################
      // camera_link基準からbase_footprint基準のPoint Cloudに座標変換する
      // ###########################################################

      pcl::PointCloud<pcl::PointXYZ> transformed_pcl;

      //座標変換ができるかどうかを確認
      bool is_possible_transform = listener.canTransform(base_link_name, camera_link_name, ros::Time(0));
      if(is_possible_transform){//座標変換成功
        pcl_ros::transformPointCloud(base_link_name, ros::Time(0), subscribed_pcl, camera_link_name, transformed_pcl, listener);
      }
      else{//座標変換失敗
        ROS_WARN("Failure Transform");
        base_link_name = camera_link_name; //座標変換をしないようにフレーム名を統一
      }

      printf("%d\n", transformed_pcl.size());

    }
  }// point_cloud_CB
};



int main(int argc, char** argv){
  ros::init(argc, argv, "pcl_publisher");
  PointCloudPublisher pcp;
  ros::spin();
  return 0;
}
