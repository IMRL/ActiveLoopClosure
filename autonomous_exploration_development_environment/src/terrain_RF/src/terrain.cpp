#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <mlpack/core.hpp>
#include <mlpack/methods/random_forest/random_forest.hpp>

using namespace std;
using namespace mlpack;
using namespace pcl;

typedef PointXYZINormal PointT;

class Terrain_RF
{
public:
    Terrain_RF()
        : nh_(),nh_private_()
    {
        initialize();    
    }

    bool setParams()
    {
        nh_private_.getParam("/terrain_RF/sub_pcl_topic",sub_pcl_topic);
        nh_private_.getParam("/terrain_RF/pub_terrain_cloud_topic",pub_terrain_cloud_topic);
        nh_private_.getParam("/terrain_RF/model_file",model_file);
        nh_private_.getParam("/terrain_RF/pcd_saveOrnot",pcd_save);
        nh_private_.getParam("/terrain_RF/pcd_save_path",pcd_save_path);
        nh_private_.getParam("/terrain_RF/DSleafsize",DSleafsize);

        return true;
    }
    bool initialize()
    {
        if(!setParams())
            return false;
        
        // Load the trained model.
        // RandomForest<> rf;
         file_count = 0;
        data::Load(model_file, "random_forest_model", rf);
        ROS_INFO("Load model success!!!");
        cloud_sub = nh_.subscribe(sub_pcl_topic, 1, &Terrain_RF::cloudCb, this);
        terrain_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(pub_terrain_cloud_topic, 1);

        return true;
        
    }

    

private:
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_label (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    // Extract features.
    arma::mat X(4, cloud->points.size());
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      X(0, i) = cloud->points[i].intensity;
      X(1, i) = cloud->points[i].normal_x;
      X(2, i) = cloud->points[i].normal_y;
      X(3, i) = cloud->points[i].curvature;
    }
    ROS_INFO("cloud_RF________");
    // Perform the prediction.
    arma::Row<size_t> predictions;
    auto start = std::chrono::high_resolution_clock::now();
    rf.Classify(X, predictions);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    ROS_INFO("Prediction time: %f seconds", diff.count());

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrain_RF_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        pcl::PointXYZINormal point1;
        pcl::PointXYZINormal point2;
        point1.x = cloud->points[i].x;
        point1.y = cloud->points[i].y;
        point1.z = cloud->points[i].z;
        point1.intensity = predictions(i);
        point1.curvature = cloud->points[i].intensity;
        point1.normal_x = 0;
        point1.normal_y = 0;
        point1 .normal_z = 0;
        terrain_RF_cloud->points.push_back(point1);

        point2.x = cloud->points[i].x;
        point2.y = cloud->points[i].y;
        point2.z = cloud->points[i].z;
        point2.intensity = predictions[i];
        point2.normal_x = cloud->points[i].intensity; //disZ
        point2.normal_y = cloud->points[i].normal_x; //gradm
        point2.normal_z = cloud->points[i].normal_y; //std_dev
        point2.curvature = cloud->points[i].curvature; //intensity
        cloud_label->points.push_back(point2);
    }
   
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*terrain_RF_cloud, output);
    output.header.frame_id = cloud_msg->header.frame_id;
    output.header.stamp = cloud_msg->header.stamp;
    terrain_cloud_pub.publish(output);
    if (pcd_save)
    {   
      cloud_label->width = cloud_label->points.size();
      cloud_label->height = 1;
      cloud_label->is_dense = false; 
      std::string filename = "cloud_label_" + std::to_string(file_count) + ".pcd";
      std::string full_path = pcd_save_path + filename;

      pcl::io::savePCDFileASCII (full_path, *cloud_label);

      file_count++;
    }
    
  }
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber cloud_sub;
  ros::Publisher terrain_cloud_pub;
  std::string sub_pcl_topic;
  std::string model_file;
  std::string pcd_save_path;
  std::string pub_terrain_cloud_topic;
  int file_count;
  double DSleafsize;
  bool pcd_save;
  RandomForest<> rf;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Terrain_RF");

  Terrain_RF t_rf;

  ros::spin();

  return 0;
}