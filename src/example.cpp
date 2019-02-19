#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCLを使うときに必要なやつ
// #include <pcl/ros/conversions.h> // 現在は使えない
#include <pcl_conversions/pcl_conversions.h> // 代わりにこれを使う

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud2 output;

  output = *input;

  pub.publish(output);
}//cloud_cb

int main(int argc, char** argv)
{
  //初期化
  ros::init(argc, argv, "pcl_tutorial_example");
  ros::NodeHandle nh;

  ros::Subscriber sub;
  sub = nh.subscribe("input", 1, cloud_cb);

  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  ros::spin();
}//main
