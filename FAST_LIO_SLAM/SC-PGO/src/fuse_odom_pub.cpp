// #include "utility.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <pcl/point_types.h>
typedef pcl::PointXYZINormal PointType;
// typedef pcl::PointCloud<PointType> PointCloudXYZI;
// typedef pcl::PointXYZI PointType;
struct Pose6D {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

/* 将最终优化过的里程计信息添加上后面imu里程计增加的里程计信息构成最新的imu里程计信息 */
class TransformFusion 
{
public:
    std::mutex mtx;
    ros::NodeHandle nh;

    ros::Subscriber regularOdomSub;   // 通过imu积分估计的雷达里程计信息订阅器
    ros::Subscriber loopClosureOdomSub; // 最终优化后的里程计信息订阅器
    ros::Subscriber pclPGOSub;

    ros::Publisher fusedOdomPub; // imu里程计信息发布器
    ros::Publisher fusedOdomPath;     // imu路径发布器

    ros::Publisher pclPGOPub;
    Eigen::Affine3f loopOdomAffine;
    Eigen::Affine3f regularOdomAffineFront;
    Eigen::Affine3f regularOdomAffineBack;

    Pose6D odomCur{0.0,0.0,0.0,0.0,0.0,0.0};
    Pose6D odomFastlio{0.0,0.0,0.0,0.0,0.0,0.0};
    // tf::TransformListener tfListener;
    // tf::StampedTransform lidar2Baselink;

    double loopOdomTime = -1;
    std::deque<nav_msgs::Odometry> regularOdomQueue;

    TransformFusion()
    {
        // 如果雷达坐标系和基座标系不一致，则获取雷达坐标系相对于基座标系的转换关系
        // if (lidarFrame != baselinkFrame)
        // {
        //     try
        //     {
        //         tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
        //         tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
        //     }
        //     catch (tf::TransformException ex)
        //     {
        //         ROS_ERROR("%s", ex.what());
        //     }
        // }

        loopClosureOdomSub = nh.subscribe<nav_msgs::Odometry>("/aft_pgo_odom", 5, &TransformFusion::loopOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        regularOdomSub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 2000, &TransformFusion::regularOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        pclPGOSub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered",20,&TransformFusion::laserHandler, this, ros::TransportHints().tcpNoDelay());

        // TransformFusion这个类产生的数据没有被其它节点使用，只是单纯的为了rviz显示用，所以这个类可以去掉，不影响最后的建图结果
        fusedOdomPub = nh.advertise<nav_msgs::Odometry>("/fusedOdom", 100000); // 该话题没有被任何其它节点利用
        fusedOdomPath = nh.advertise<nav_msgs::Path>("/fusedPath", 1);   // 该话题只为显示用

        pclPGOPub = nh.advertise<sensor_msgs::PointCloud2>("aft_pgo_pcl",100);
    }


    Pose6D inversePose6D(const Pose6D& pose) 
    {
        double x_inv = -pose.x;
        double y_inv = -pose.y;
        double z_inv = -pose.z;
        Eigen::AngleAxisd rollAngle(pose.roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pose.pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(pose.yaw, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d rotation = (yawAngle * pitchAngle * rollAngle).matrix();
        Eigen::Matrix3d rotation_inv = rotation.transpose(); // 对于旋转矩阵，逆矩阵就是转置矩阵
        Eigen::Vector3d euler_inv = rotation_inv.eulerAngles(2, 1, 0); // Z-Y-X顺序
        return {x_inv, y_inv, z_inv, euler_inv[2], euler_inv[1], euler_inv[0]};
    }

    pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
        pcl::transformPointCloud(*cloudIn, *cloudOut, transCur);
        return cloudOut;
    }

    pcl::PointCloud<PointType>::Ptr global2local(const  pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& pose) 
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new  pcl::PointCloud<PointType>());
        Eigen::Affine3f transToLocal = pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
        pcl::transformPointCloud(*cloudIn, *cloudOut, transToLocal.inverse());
        
        return cloudOut;
    }

    Pose6D getOdom(nav_msgs::Odometry _odom)
    {
        auto tx = _odom.pose.pose.position.x;
        auto ty = _odom.pose.pose.position.y;
        auto tz = _odom.pose.pose.position.z;

        double roll, pitch, yaw;
        geometry_msgs::Quaternion quat = _odom.pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

        return Pose6D{tx, ty, tz, roll, pitch, yaw}; 
    } // getOdom

    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void laserHandler(const sensor_msgs::PointCloud2ConstPtr &lasermsg)
    {
        // pcl::PointCloud<PointType>::Ptr laserCurLocal(new pcl::PointCloud<PointType>());
        // pcl::fromROSMsg(*lasermsg,*laserCurLocal);
        // pcl::PointCloud<PointType>::Ptr laserCurGlobal(new pcl::PointCloud<PointType>());
        // std::cout<<"odomCur:" << odomCur.x <<","<<odomCur.y<< " , " <<odomCur.z<<std::endl;
        // *laserCurGlobal = *local2global(laserCurLocal,odomCur);

        pcl::PointCloud<PointType>::Ptr laserCurLocal(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*lasermsg,*laserCurLocal);
        pcl::PointCloud<PointType>::Ptr laserCurfastlio_local(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr laserCurGlobal(new pcl::PointCloud<PointType>());
        Pose6D odomFastlio_inv = inversePose6D(odomFastlio);
        *laserCurfastlio_local = *global2local(laserCurLocal,odomFastlio);
        *laserCurGlobal =*local2global(laserCurfastlio_local,odomCur);

        sensor_msgs::PointCloud2 lasermsgGlobal;
        pcl::toROSMsg(*laserCurGlobal,lasermsgGlobal);
        lasermsgGlobal.header.frame_id = "camera_init";
        pclPGOPub.publish(lasermsgGlobal);
    
    }

    void loopOdometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        // std::lock_guard<std::mutex> lock(mtx);

        loopOdomAffine = odom2affine(*odomMsg);

        loopOdomTime = odomMsg->header.stamp.toSec();
    }

    void regularOdometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        odomFastlio = getOdom(*odomMsg);
        regularOdomQueue.push_back(*odomMsg); //

        // 当没有订阅到最终优化后的里程计信息时，直接返回
        if (loopOdomTime == -1)
        {
            nav_msgs::Odometry fastlioOdom;
            fastlioOdom =*odomMsg;
            fusedOdomPub.publish(fastlioOdom);
            odomCur = getOdom(fastlioOdom);
        }
        else
        {
        while (!regularOdomQueue.empty())
        {       
            // if (regularOdomQueue.front().header.stamp.toSec() <= loopOdomTime)
            if (regularOdomQueue.front().header.stamp.toSec() < loopOdomTime)
                regularOdomQueue.pop_front();
            else
                break;
        }

        Eigen::Affine3f regularOdomAffineFront = odom2affine(regularOdomQueue.front());               
        Eigen::Affine3f regularOdomAffineBack = odom2affine(regularOdomQueue.back());                  
        Eigen::Affine3f regularOdomAffineIncre = regularOdomAffineFront.inverse() * regularOdomAffineBack;
        Eigen::Affine3f regularOdomAffineLast = loopOdomAffine * regularOdomAffineIncre;             
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(regularOdomAffineLast, x, y, z, roll, pitch, yaw); 
        // publish latest odometry
        // 发布最新的imu里程计信息
        nav_msgs::Odometry laserOdometry = regularOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        fusedOdomPub.publish(laserOdometry);
        odomCur = getOdom(laserOdometry);

        }
        // 当订阅到最终优化后的里程计信息时，剔除掉比该帧还老的imu里程计信息帧
      
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboat_loam");


    TransformFusion TF;

    ROS_INFO("\033[1;32m----> Odom fused Started.\033[0m");

    // ros::MultiThreadedSpinner spinner(4);
    ros::spin();

    return 0;
}
