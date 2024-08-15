#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include "livox_ros_driver2/CustomMsg.h"

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

class RotationNode
{
public:
  RotationNode()
  {

    pointcloud_sub_ = nh_.subscribe("/livox/lidar", 1, &RotationNode::pointcloudCallback, this);
    imu_sub_ = nh_.subscribe("/livox/imu", 1, &RotationNode::imuCallback, this);


    rotated_pointcloud_pub_ = nh_.advertise<livox_ros_driver2::CustomMsg>("/rotated_pointcloud", 1);
    // rotated_pointcloud_pub_sensormsg = nh_.advertise<sensor_msgs::PointCloud2>("/rotated_pointcloud_ros", 1);
    rotated_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/rotated_imu", 1);

    // 计算Livox点云和IMU的旋转矩阵

    double angle_y = 25*M_PI / 180;//绕y轴旋转90°
    rotation_matrix_(0, 0) = cos(angle_y);
    rotation_matrix_(0, 2) = sin(angle_y);
    rotation_matrix_(2, 0) = -sin(angle_y);
    rotation_matrix_(2, 2) = cos(angle_y);
    // rotation_matrix_(2, 3) = 0.0; //离地面高度 z轴平移量
  }

  void pointcloudCallback(const livox_ros_driver2::CustomMsgConstPtr &msg)
  {

    livox_ros_driver2::CustomMsg livoxCloud = *msg;
    for(int i = 0; i < msg->point_num; i++) {
        Eigen::Vector3d point;
        point[0] = livoxCloud.points[i].x;
        point[1] = livoxCloud.points[i].y;
        point[2] = livoxCloud.points[i].z;
        point = rotation_matrix_* point;
        livoxCloud.points[i].x = point[0];
        livoxCloud.points[i].y = point[1];
        livoxCloud.points[i].z = point[2];
    }
    rotated_pointcloud_pub_.publish(livoxCloud);

    // pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

    // for(int i = 0; i < msg->point_num; i++) {
    //     Eigen::Vector3d point(msg->points[i].x, msg->points[i].y, msg->points[i].z);
    //     point = rotation_matrix_ * point;
    //     pcl::PointXYZI pcl_point;
    //     pcl_point.x = point[0];
    //     pcl_point.y = point[1];
    //     pcl_point.z = point[2];
    //     pcl_point.intensity = msg->points[i].reflectivity; 
    //     pcl_cloud.push_back(pcl_point);
    // }
    // sensor_msgs::PointCloud2 sensor_msgs_cloud;
    // pcl::toROSMsg(pcl_cloud, sensor_msgs_cloud);
    // sensor_msgs_cloud.header.frame_id = "body";
    // sensor_msgs_cloud.header.stamp = pcl_conversions::fromPCL(pcl_cloud.header.stamp);
    // rotated_pointcloud_pub_sensormsg.publish(sensor_msgs_cloud);

    // // 将Livox点云消息转换为PCL点云格式
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // std::vector<livox_ros_driver2::CustomMsgConstPtr> livox_data;
    // ROS_INFO("!!!!!!!!");

    // sensor_msgs::PointCloud2 livox_cloud;
    // livox_data.push_back(msg);
    // for (size_t j=0; j < livox_data.size(); j++)
    // {
    //     auto& livox_msg = livox_data[j];
    //     auto time_end = livox_msg->points.back().offset_time;
    //     for (unsigned int i=0; i < livox_msg->point_num; ++i)
    //     {
    //         pcl::PointXYZ p;
    //         // PointType p;
    //         p.x = livox_msg->points[i].x;
    //         p.y = livox_msg->points[i].y;
    //         p.z = livox_msg->points[i].z;
    //         // p.intensity = livox_msg->points[i].line  // 整数是线数
    //         //                + livox_msg->points[i].reflectivity /
    //         //                      10000.0;  // ??? 小数是反射率?
    //         // p.curvature = (livox_msg->points[i].offset_time /
    //         //                 (float)time_end);  // 该点时刻在该次扫描时间段的位置
    //         pcl_cloud->push_back(p);    
    //         // }  
    //     }
    // } 
    

    // // 进行Livox点云旋转
    // pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::transformPointCloud(*pcl_cloud, *rotated_cloud, rotation_matrix_);

    // livox_ros_driver2::CustomMsg livox_repub; 
    // for (size_t j=0; j < livox_data.size(); j++)
    // {
    //     auto& livox_msg = livox_data[j];
    //     for (unsigned int i=0; i < livox_msg->point_num; ++i)
    //     {
    //         livox_repub.points[i].x = rotated_cloud -> points[i].x;
    //         livox_repub.points[i].y = rotated_cloud -> points[i].y;
    //         livox_repub.points[i].z = rotated_cloud -> points[i].z; 
    //         livox_repub.points[i].line = livox_msg->points[i].line;
    //         livox_repub.points[i].offset_time = livox_msg->points[i].offset_time;
    //         livox_repub.points[i].reflectivity = livox_msg->points[i].reflectivity;
    //         livox_repub.points[i].tag = livox_msg->points[i].tag;
    //     }
    // }
    // livox_repub.header = msg->header;
    // livox_repub.point_num = msg ->point_num;
    // livox_repub.lidar_id = msg->lidar_id;
    // livox_repub.rsvd = msg->rsvd;
    // livox_repub.timebase = msg->timebase;
    

    // // 发布旋转后的Livox点云
    // rotated_pointcloud_pub_.publish(livox_repub);
  }

  // void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  // {
  //   // 进行IMU旋转
  //   sensor_msgs::Imu rotated_imu(*msg);

  //   // 读取线性加速度和角速度信息
  //   Eigen::Vector3d linear_acceleration;
  //   Eigen::Vector3d angular_velocity;


  //   // 进行线性加速度和角速度旋转
  //   linear_acceleration.x() = (rotation_matrix_(0,0)* msg->linear_acceleration.x)+(rotation_matrix_(0,1)*msg->linear_acceleration.y)+(rotation_matrix_(0,2)*msg->linear_acceleration.z);
  //   linear_acceleration.y() = (rotation_matrix_(1,0)* msg->linear_acceleration.x)+(rotation_matrix_(1,1)*msg->linear_acceleration.y)+(rotation_matrix_(1,2)*msg->linear_acceleration.z);
  //   linear_acceleration.z() = (rotation_matrix_(2,0)* msg->linear_acceleration.x)+(rotation_matrix_(2,1)*msg->linear_acceleration.y)+(rotation_matrix_(2,2)*msg->linear_acceleration.z);
  //   angular_velocity.x() = (rotation_matrix_(0,0)* msg->angular_velocity.x)+(rotation_matrix_(0,1)*msg->angular_velocity.y)+(rotation_matrix_(0,2)*msg->angular_velocity.z);
  //   angular_velocity.y() = (rotation_matrix_(1,0)* msg->angular_velocity.x)+(rotation_matrix_(1,1)*msg->angular_velocity.y)+(rotation_matrix_(1,2)*msg->angular_velocity.z);
  //   angular_velocity.z() = (rotation_matrix_(2,0)* msg->angular_velocity.x)+(rotation_matrix_(2,1)*msg->angular_velocity.y)+(rotation_matrix_(2,2)*msg->angular_velocity.z);
    

  //   // 将旋转后的线性加速度和角速度写回IMU消息
  //   rotated_imu.linear_acceleration.x = linear_acceleration.x();
  //   rotated_imu.linear_acceleration.y = linear_acceleration.y();
  //   rotated_imu.linear_acceleration.z = linear_acceleration.z();

  //   rotated_imu.angular_velocity.x = angular_velocity.x();
  //   rotated_imu.angular_velocity.y = angular_velocity.y();
  //   rotated_imu.angular_velocity.z = angular_velocity.z();

  //   // 发布旋转后的IMU数据
  //   rotated_imu_pub_.publish(rotated_imu);
  // }
  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  sensor_msgs::Imu rotated_imu(*msg);

  const Eigen::Vector3d linear_acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  const Eigen::Vector3d rotated_linear_acceleration = rotation_matrix_ * linear_acceleration;
  const Eigen::Vector3d rotated_angular_velocity = rotation_matrix_ * angular_velocity;

  rotated_imu.linear_acceleration.x = rotated_linear_acceleration.x();
  rotated_imu.linear_acceleration.y = rotated_linear_acceleration.y();
  rotated_imu.linear_acceleration.z = rotated_linear_acceleration.z();

  rotated_imu.angular_velocity.x = rotated_angular_velocity.x();
  rotated_imu.angular_velocity.y = rotated_angular_velocity.y();
  rotated_imu.angular_velocity.z = rotated_angular_velocity.z();

  rotated_imu_pub_.publish(rotated_imu);
}


private:
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher rotated_pointcloud_pub_,rotated_pointcloud_pub_sensormsg;
    ros::Publisher rotated_imu_pub_;
    Eigen::Matrix3d rotation_matrix_ = Eigen::Matrix3d::Identity();
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotation_node");

  RotationNode node;

  ros::spin();

  return 0;
}
