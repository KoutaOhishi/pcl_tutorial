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

class down_sampling{
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud;
    ros::Publisher pub_cloud;

    //parameters
    double leaf_x;
    double leaf_y;
    double leaf_z;

    std::string sub_cloud_name;
    std::string pub_cloud_name;

  public:
    down_sampling(){
      //this->sub_cloud_name = "input";
      this->sub_cloud_name = "/camera/depth/points";
      this->pub_cloud_name = "down_sampling_pcl";

      this->sub_cloud = nh.subscribe(this->sub_cloud_name, 1, &down_sampling::cloud_cb, this);
      this->pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(this->pub_cloud_name, 1);

      this->leaf_x = 0.01;
      this->leaf_y = 0.01;
      this->leaf_z = 0.01;
    }//down_sampling

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input_data)
    {
      //元のデータとフィルターをかけたデータの格納
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
      pcl::PCLPointCloud2 cloud_filtered;

      //PCL型への変換
      pcl_conversions::toPCL(*input_data, *cloud);

      //ダウンサンプリングの実行
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud(cloudPtr);
      sor.setLeafSize(this->leaf_x,this->leaf_y,this->leaf_z);
      sor.filter(cloud_filtered);

      //ROSで扱えるPCLの形式に変換
      sensor_msgs::PointCloud2 output;
      pcl_conversions::fromPCL(cloud_filtered, output);

      //ダウンサンプリングしたデータをpublish
      this->pub_cloud.publish(output);
    }//cloud_cb

};// down_sampling

int main(int argc, char** argv){
  //ROSの初期化
  ros::init(argc, argv, "down_sampling");
  ROS_INFO("pcl_tutorial : down_sampling");

  down_sampling ds;

  ros::spin();

}//main
