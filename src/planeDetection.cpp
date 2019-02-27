#include <iostream>
#include <vector>
#include <string>

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
#include <pcl_ros/point_cloud.h>

class plane_detection{
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud;
    ros::Publisher pub_cloud;

    //parameters
    double dist_th;

    std::string sub_cloud_name;
    std::string pub_cloud_name;

  public:
    plane_detection(){
      //this->sub_cloud_name = "input";
      this->sub_cloud_name = "/camera/depth/points";
      this->pub_cloud_name = "plane_detection_pcl";

      this->sub_cloud = nh.subscribe(this->sub_cloud_name, 1, &plane_detection::cloud_cb, this);
      this->pub_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >(this->pub_cloud_name, 1);

      this->dist_th = 0.1;
    }//down_sampling

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input_data)
    {
      // sensor_msgs/PointCloud2のデータをpcl/PointCloudに変換
      pcl::PointCloud<pcl::PointXYZRGBA> cloud;
      pcl::fromROSMsg(*input_data, cloud);

      // alpha値の設定
      for (size_t i = 0; i < cloud.points.size(); ++i){
        cloud.points[i].a = 255;
      }

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      // segmentationの設定
      //pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
      pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

      // オプションの設定
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);

      seg.setDistanceThreshold(this->dist_th);

      seg.setInputCloud(cloud.makeShared());
      seg.segment(*inliers, *coefficients);

      if(inliers->indices.size() == 0){
        std::cerr << "Could not detect a plane in input data." << std::endl;
      }
      else{
        for (size_t i = 0; i < inliers->indices.size(); ++i){
          cloud.points[inliers->indices[i]].r = 0;
          cloud.points[inliers->indices[i]].g = 255;
          cloud.points[inliers->indices[i]].b = 0;
          cloud.points[inliers->indices[i]].a = 255;
        }
      }

      //データをpublish
      this->pub_cloud.publish(cloud);

    }//cloud_cb

};// plane_detection

int main(int argc, char** argv){
  //ROSの初期化
  ros::init(argc, argv, "plane_detection");
  ROS_INFO("pcl_tutorial : plane_detection");

  plane_detection pd;

  ros::spin();

}//main
