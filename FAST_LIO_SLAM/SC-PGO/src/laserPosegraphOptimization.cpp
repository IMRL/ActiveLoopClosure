//维护一个全局因子图作为计算边缘化不确定性
//将不确定性融合到位姿里发布
#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>

#include <signal.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

#include "scancontext/Scancontext.h"

#include "pcl/io/pcd_io.h"

#include <std_msgs/Float32.h>
using namespace gtsam;

using std::cout;
using std::endl;


// struct Pose6D_Uncertain{
//     double x;
//     double y;
//     double z;
//     double roll;
//     double pitch;
//     double yaw;
//     gtsam::Vector6 uncertain;
// };
struct Pose6D_Uncertain {
    struct Pose {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    } pose;
    gtsam::Vector6 uncertain;
    double d_opt;
    double d_opt_cov; //协方差的行列式
};
double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0; // large value means must add the first given frame.
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.

bool isNowKeyFrame = false; 

bool is_shutdown =false;

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init 
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero 
Pose6D odom_pose_fusion {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<sensor_msgs::NavSatFix::ConstPtr> gpsBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;
std::vector<std::pair<std::pair<int, int>, bool> > closureLoop;
std::vector<pcl::PointCloud<PointType>::Ptr> cloudSlideWindow;
std::vector<Pose6D> odomSlideWindow;
int scNum = 0;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

double firstFrameTime = -1; // 第一帧的时间戳

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; 
std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserCloudsCur;
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;

std::vector<Pose6D_Uncertain> keyframePose_Uncertain;
std::vector<Pose6D> fastlioPoses;
std::vector<double> fastlioTimes;
std::vector<int> keyframe2fastlio;
std::vector<double> keyframeTimes;
int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
gtsam::NonlinearFactorGraph global_gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;
gtsam::Marginals marginals;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Diagonal::shared_ptr esekfNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres, scMaximumRadius;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr Pathcolor(new pcl::PointCloud<PointType>());

// pcl::PointCloud<PointType>::Ptr laserCloudMapPGOCur(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = true;

// bool useGPS = true;
bool useGPS = false;
sensor_msgs::NavSatFix::ConstPtr currGPS;
bool hasGPSforThisKF = false;
bool gpsOffsetInitialized = false; 
double gpsAltitudeInitOffset = 0.0;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;


int frameCount = 0;
// int interval = 10;
std::string savePath ="/home/gw/exploration/nav/nav_test_ws2/src/FAST_LIO_SLAM/SC-PGO/pcd/";
bool savePCD;
double loopfitness;

ros::Publisher pubMapAftPGO, pubOdomAftPGO, pubPathAftPGO,pubCurPointcloudPGO,pubPathColor;
ros::Publisher pubLoopScanLocal, pubLoopSubmapLocal,pubLoopConstraintEdge;
ros::Publisher pubOdomRepubVerifier;
ros::Publisher pose_uncertainty_pub;
ros::Publisher current_pose_det_cov_pub; //当前位姿的不确定性的d-opt发布
ros::Publisher TravelDistance_pub; //总的走过的距离

std::string save_directory;
std::string pgKITTIformat, pgScansDirectory;
std::string odomKITTIformat;
std::string odomTumformat;
std::fstream pgTimeSaveStream;

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

void saveOdometryVerticesKITTIformat(std::string _filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}


void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);

    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &_laserOdometry)
{
	mBuf.lock();
    if (firstFrameTime < 0) {
    firstFrameTime = _laserOdometry->header.stamp.toSec();
    }
	odometryBuf.push(_laserOdometry);
	mBuf.unlock();
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &_laserCloudFullRes)
{
	mBuf.lock();
	fullResBuf.push(_laserCloudFullRes);
	mBuf.unlock();
} // laserCloudFullResHandler

void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr &_gps)
{
    if(useGPS) {
        mBuf.lock();
        gpsBuf.push(_gps);
        mBuf.unlock();
    }
} // gpsHandler

void initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3) );

} // initNoises

Pose6D getOdom(nav_msgs::Odometry::ConstPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion quat = _odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw}; 
} // getOdom

Matrix6 getOdomCov(nav_msgs::Odometry::ConstPtr _odom)
{   
    Matrix6 cov;
    for (int i = 0; i < 6; i++)
        for(int j = 0; j < 6; j++) {
            cov(i,j) = _odom->pose.covariance[i*6+j];
        }

    return cov; 
} // getOdomCov

Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
    // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    // return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
    return Pose6D{double(dx), double(dy), double(dz), double(droll), double(dpitch), double(dyaw)};
} // SE3Diff

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
    
    int numberOfCores = 16;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void pubLoopPair( void ) {
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker markerEdge_false;
    markerEdge_false.header.frame_id = "camera_init";
    markerEdge_false.header.stamp = ros::Time::now();
    markerEdge_false.action = visualization_msgs::Marker::ADD;
    markerEdge_false.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge_false.ns = "loop_edges";
    markerEdge_false.id = 0;
    markerEdge_false.pose.orientation.w = 1;
    markerEdge_false.scale.x = 0.5;
    markerEdge_false.color.r = 1.0;
    markerEdge_false.color.g = 0.0;
    markerEdge_false.color.b = 0.0;
    markerEdge_false.color.a = 1;

    visualization_msgs::Marker markerEdge_true;
    markerEdge_true.header.frame_id = "camera_init";
    markerEdge_true.header.stamp = ros::Time::now();
    markerEdge_true.action = visualization_msgs::Marker::ADD;
    markerEdge_true.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge_true.ns = "loop_edges";
    markerEdge_true.id = 1;
    markerEdge_true.pose.orientation.w = 1;
    markerEdge_true.scale.x = 0.5;
    markerEdge_true.color.r = 0.9;
    markerEdge_true.color.g = 0.9;
    markerEdge_true.color.b = 0;
    markerEdge_true.color.a = 1;

    // for (int it = 0; it < closureLoop.size(); ++it)
    for(auto it = closureLoop.begin(); it != closureLoop.end(); ++it)
    {
      int key_cur = it->first.first;
      int key_pre = it->first.second;
      bool icpPass = it->second;
      geometry_msgs::Point p;
      p.x = keyframePosesUpdated[key_cur].x;
      p.y = keyframePosesUpdated[key_cur].y;
      p.z = keyframePosesUpdated[key_cur].z;
      icpPass ? markerEdge_true.points.push_back(p) : markerEdge_false.points.push_back(p);
      
      p.x = keyframePosesUpdated[key_pre].x;
      p.y = keyframePosesUpdated[key_pre].y;
      p.z = keyframePosesUpdated[key_pre].z;
      icpPass ? markerEdge_true.points.push_back(p) : markerEdge_false.points.push_back(p);
    }

    markerArray.markers.push_back(markerEdge_true);
    markerArray.markers.push_back(markerEdge_false);
    pubLoopConstraintEdge.publish(markerArray);
}

void pubPath( void )
{
    // pub odom and path 
    nav_msgs::Odometry odomAftPGO;
    nav_msgs::Path pathAftPGO;
    pathAftPGO.header.frame_id = "camera_init";
    mKF.lock(); 
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    {
        // std::cout<< "node_idx:"<<node_idx<<std::endl;
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // upodated poses
        const Pose6D_Uncertain pose_est_uncetain = keyframePose_Uncertain.at(node_idx);
        nav_msgs::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "camera_init";
        odomAftPGOthis.child_frame_id = "/aft_pgo";
        odomAftPGOthis.header.stamp = ros::Time().fromSec(keyframeTimes.at(node_idx));
        // odomAftPGOthis.header.stamp = ros::Time::now();
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;
        odomAftPGOthis.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_est.roll, pose_est.pitch, pose_est.yaw);
        // // 将不确定度信息添加到odom消息中
        odomAftPGOthis.pose.covariance[0] = pose_est_uncetain.uncertain[0];
        odomAftPGOthis.pose.covariance[1] = pose_est_uncetain.d_opt_cov; 
        odomAftPGOthis.pose.covariance[7] = pose_est_uncetain.uncertain[1];
        odomAftPGOthis.pose.covariance[14] = pose_est_uncetain.uncertain[2];
        odomAftPGOthis.pose.covariance[21] = pose_est_uncetain.uncertain[3];
        odomAftPGOthis.pose.covariance[28] = pose_est_uncetain.uncertain[4];
        odomAftPGOthis.pose.covariance[35] = pose_est_uncetain.uncertain[5];
        odomAftPGO = odomAftPGOthis;

        geometry_msgs::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "camera_init";
        pathAftPGO.poses.push_back(poseStampAftPGO);

        Eigen::Affine3f regularOdomAffineFront = pcl::getTransformation(fastlioPoses[keyframe2fastlio[node_idx]].x, fastlioPoses[keyframe2fastlio[node_idx]].y, fastlioPoses[keyframe2fastlio[node_idx]].z, fastlioPoses[keyframe2fastlio[node_idx]].roll, fastlioPoses[keyframe2fastlio[node_idx]].pitch, fastlioPoses[keyframe2fastlio[node_idx]].yaw);
        Eigen::Affine3f loopOdomAffine = pcl::getTransformation(pose_est.x, pose_est.y, pose_est.z, pose_est.roll, pose_est.pitch, pose_est.yaw);
        for(int i = keyframe2fastlio[node_idx]+1; i < ((node_idx+1 < recentIdxUpdated) ? keyframe2fastlio[node_idx+1] : fastlioPoses.size()); i++) {
            
            Eigen::Affine3f regularOdomAffineBack = pcl::getTransformation(fastlioPoses[i].x, fastlioPoses[i].y, fastlioPoses[i].z, fastlioPoses[i].roll, fastlioPoses[i].pitch, fastlioPoses[i].yaw);
            Eigen::Affine3f regularOdomAffineIncre = regularOdomAffineFront.inverse() * regularOdomAffineBack; 
            Eigen::Affine3f regularOdomAffineLast = loopOdomAffine * regularOdomAffineIncre;              
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(regularOdomAffineLast, x, y, z, roll, pitch, yaw);

            geometry_msgs::PoseStamped poseStampfastlio;
            poseStampfastlio.header.stamp = ros::Time().fromSec(fastlioTimes[i]);
            poseStampfastlio.pose.position.x = x;
            poseStampfastlio.pose.position.y = y;
            poseStampfastlio.pose.position.z = z;
            poseStampfastlio.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            pathAftPGO.poses.push_back(poseStampfastlio);
        }
    }
    mKF.unlock(); 
    double total_distance = 0.0;
    geometry_msgs::Pose last_pose;
    bool first_pose = true;
    Pathcolor->clear();
    for(const auto& pose_stamped : pathAftPGO.poses) {
        if(first_pose) {
            last_pose = pose_stamped.pose;
            first_pose = false;
        } else {
            double dx = pose_stamped.pose.position.x - last_pose.position.x;
            double dy = pose_stamped.pose.position.y - last_pose.position.y;
            double dz = pose_stamped.pose.position.z - last_pose.position.z;
            total_distance += sqrt(dx*dx + dy*dy + dz*dz);
            // Update the last_pose with the current pose for the next iteration
            last_pose = pose_stamped.pose;
            pcl::PointXYZI point;
            point.x = pose_stamped.pose.position.x;
            point.y = pose_stamped.pose.position.y;
            point.z = pose_stamped.pose.position.z;
            point.intensity = total_distance;
            Pathcolor->push_back(point);
        }
    }
    

    sensor_msgs::PointCloud2 trajectory2;
    pcl::toROSMsg(*Pathcolor, trajectory2);
    trajectory2.header.stamp = ros::Time::now();
    trajectory2.header.frame_id = "camera_init";

    pubPathColor.publish(trajectory2);

    std_msgs::Float32 total_distance_msg;
    total_distance_msg.data = total_distance;
    TravelDistance_pub.publish(total_distance_msg);
    
    pubOdomAftPGO.publish(odomAftPGO); // last pose 
    pubPathAftPGO.publish(pathAftPGO); // poses 

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftPGO.pose.pose.position.x, odomAftPGO.pose.pose.position.y, odomAftPGO.pose.pose.position.z));
    q.setW(odomAftPGO.pose.pose.orientation.w);
    q.setX(odomAftPGO.pose.pose.orientation.x);
    q.setY(odomAftPGO.pose.pose.orientation.y);
    q.setZ(odomAftPGO.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftPGO.header.stamp, "camera_init", "/aft_pgo"));
} // pubPath

double calculateDOptimality(const gtsam::Matrix6& poseInformationMatrix) {
    Eigen::MatrixXd eigenMatrix = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(poseInformationMatrix.data());
    Eigen::EigenSolver<Eigen::MatrixXd> solver(eigenMatrix);
    Eigen::VectorXd eigenvalues = solver.eigenvalues().real();
    double sum_log_eigenvalues = 0.0;
    for (int i = 0; i < eigenvalues.size(); ++i) {
        sum_log_eigenvalues += std::log(eigenvalues[i]);
    }
    double D_opt = std::exp(sum_log_eigenvalues / eigenvalues.size());

    return D_opt;
}

void updatePoses(void)
{
    mKF.lock(); 
    visualization_msgs::MarkerArray marker_array;
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        
        gtsam::Matrix6 poseCovariance = marginals.marginalCovariance(node_idx);
        gtsam::Matrix6 poseInformationMatrix = marginals.marginalInformation(node_idx);

        double D_opt = calculateDOptimality(poseInformationMatrix);
        double D_opt_cov = calculateDOptimality(poseCovariance);
                
        // gtsam::Matrix6 informationMatrix =  poseCovariance.inverse();
        Pose6D& p =keyframePosesUpdated[node_idx];
        Pose6D_Uncertain& p_u = keyframePose_Uncertain[node_idx];

        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();

        p_u.pose.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p_u.pose.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p_u.pose.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p_u.pose.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p_u.pose.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p_u.pose.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
        p_u.uncertain[0] = poseCovariance(0, 0);
        p_u.uncertain[1] = poseCovariance(1, 1);
        p_u.uncertain[2] = poseCovariance(2, 2);
        p_u.uncertain[3] = poseCovariance(3, 3);
        p_u.uncertain[4] = poseCovariance(4, 4);
        p_u.uncertain[5] = poseCovariance(5, 5);
        p_u.d_opt = D_opt;
        p_u.d_opt_cov = D_opt_cov;
        
        //可视化位姿协方差。XYZ
        visualization_msgs::Marker pose_marker;
        pose_marker.header.frame_id = "camera_init";
        pose_marker.header.stamp = ros::Time().fromSec(keyframeTimes.at(node_idx));
        pose_marker.ns = "poses";
        pose_marker.id = node_idx;
        pose_marker.type = visualization_msgs::Marker::ARROW;
        pose_marker.pose.position.x = p.x;
        pose_marker.pose.position.y = p.y;
        pose_marker.pose.position.z = p.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(p.roll, p.pitch, p.yaw);
        tf::quaternionTFToMsg(q, pose_marker.pose.orientation);
        pose_marker.scale.x = 1.0;
        pose_marker.scale.y = 0.1;
        pose_marker.scale.z = 0.1;
        pose_marker.color.a = 1.0;
        pose_marker.color.r = 1.0;
        pose_marker.color.g = 0.0;
        pose_marker.color.b = 0.0;

        marker_array.markers.push_back(pose_marker);

        // Create a covariance marker
        // if (node_idx % 5 == 0) {
        visualization_msgs::Marker cov_marker;
        cov_marker.header.frame_id = "camera_init";
        cov_marker.header.stamp = ros::Time().fromSec(keyframeTimes.at(node_idx));
        cov_marker.ns = "covariances";
        cov_marker.id = node_idx;
        cov_marker.type = visualization_msgs::Marker::SPHERE;
        cov_marker.pose.position.x = p.x;
        cov_marker.pose.position.y = p.y;
        cov_marker.pose.position.z = p.z;
        tf::quaternionTFToMsg(q, cov_marker.pose.orientation);
        cov_marker.scale.x = 50*sqrt(poseCovariance(0, 0)); // variance in x
        cov_marker.scale.y = 50*sqrt(poseCovariance(1, 1)); // variance in y
        cov_marker.scale.z = 50*sqrt(poseCovariance(2, 2)); // variance in z

        cov_marker.color.a = 0.5;
        cov_marker.color.r = 0.0;
        cov_marker.color.g = 1.0;
        cov_marker.color.b = 0.0;

        marker_array.markers.push_back(cov_marker);

        visualization_msgs::Marker rot_cov_marker;
        rot_cov_marker.header.frame_id = "camera_init";
        rot_cov_marker.header.stamp = ros::Time().fromSec(keyframeTimes.at(node_idx));
        rot_cov_marker.ns = "rotation_covariances";
        rot_cov_marker.id = node_idx;
        rot_cov_marker.type = visualization_msgs::Marker::SPHERE;
        rot_cov_marker.pose.position.x = p.x;
        rot_cov_marker.pose.position.y = p.y;
        rot_cov_marker.pose.position.z = p.z;
        tf::quaternionTFToMsg(q, rot_cov_marker.pose.orientation);
        rot_cov_marker.scale.x =  0.5*sqrt(poseCovariance(3, 3)); // variance in roll
        rot_cov_marker.scale.y = 0.5*sqrt(poseCovariance(4, 4)); // variance in pitch
        rot_cov_marker.scale.z = 0.5*sqrt(poseCovariance(5, 5)); // variance in yaw
        rot_cov_marker.color.a = 0.5;
        rot_cov_marker.color.r = 1.0;
        rot_cov_marker.color.g = 0.0;
        rot_cov_marker.color.b = 0.0;

        marker_array.markers.push_back(rot_cov_marker);
        // }

        visualization_msgs::Marker d_opt_marker;
        d_opt_marker.header.frame_id = "camera_init";
        d_opt_marker.header.stamp = ros::Time().fromSec(keyframeTimes.at(node_idx));
        d_opt_marker.ns = "d_optimality";
        d_opt_marker.id = node_idx;
        d_opt_marker.type = visualization_msgs::Marker::CYLINDER;
        d_opt_marker.pose.position.x = p.x;
        d_opt_marker.pose.position.y = p.y;
        d_opt_marker.pose.position.z = p.z;
        d_opt_marker.scale.x = D_opt; 
        d_opt_marker.scale.y = D_opt; 
        d_opt_marker.scale.z = 0.1; 
        d_opt_marker.color.a = 0.5;
        d_opt_marker.color.r = 0.0;
        d_opt_marker.color.g = 0.0;
        d_opt_marker.color.b = 1.0; // 蓝色表示D-optimality

        marker_array.markers.push_back(d_opt_marker);
    }

    Pose6D& p =keyframePosesUpdated[int(isamCurrentEstimate.size())-1];
    Pose6D_Uncertain& p_u = keyframePose_Uncertain[int(isamCurrentEstimate.size())-1];

    std_msgs::Float32 d_opt_cov_msg;
    d_opt_cov_msg.data = p_u.d_opt_cov;
    current_pose_det_cov_pub.publish(d_opt_cov_msg);
    pose_uncertainty_pub.publish(marker_array);

    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();

    recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;

    mtxRecentPose.unlock();
} // updatePoses

void runISAM2opt(void)
{
    // called when a variable added 

    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();
    isamCurrentEstimate = isam->calculateEstimate();

    marginals = gtsam::Marginals(global_gtSAMgraph, isamCurrentEstimate);
    updatePoses();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
                                    transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(), 
                                    transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );
    
    int numberOfCores = 8; // TODO move to yaml 
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

// void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;

        mKF.lock(); 
        // *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[root_idx]);
        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
        mKF.unlock(); 
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud


std::optional<gtsam::Pose3> doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx )
{
    // parse pointclouds
    frameCount++;
    int historyKeyframeSearchNum = 25; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());

    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 25); // use same root of loop kf idx 
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum); 

    // loop verification 
    sensor_msgs::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopScanLocal.publish(cureKeyframeCloudMsg);

    sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopSubmapLocal.publish(targetKeyframeCloudMsg);

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    
// pcl save unused_result
 
    // float loopFitnessScoreThreshold = 0.5; // user parameter but fixed low value is safe. 
    float loopFitnessScoreThreshold =loopfitness;
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    } else {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }
// cout << icp.getFitnessScore()
    ROS_INFO("\033[1;32m----> icp.getFitnessScore()!!!!!!!!!!!:%f.\033[0m",icp.getFitnessScore());

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    
    Eigen::Affine3f tWrong = pcl::getTransformation(keyframePosesUpdated[_curr_kf_idx].x,
                                keyframePosesUpdated[_curr_kf_idx].y,
                                keyframePosesUpdated[_curr_kf_idx].z,
                                keyframePosesUpdated[_curr_kf_idx].roll,
                                keyframePosesUpdated[_curr_kf_idx].pitch,
                                keyframePosesUpdated[_curr_kf_idx].yaw);
    Eigen::Affine3f
        tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(double(keyframePosesUpdated[_loop_kf_idx].roll), double(keyframePosesUpdated[_loop_kf_idx].pitch), double(keyframePosesUpdated[_loop_kf_idx].yaw)),
                        gtsam::Point3(double(keyframePosesUpdated[_loop_kf_idx].x), double(keyframePosesUpdated[_loop_kf_idx].y), double(keyframePosesUpdated[_loop_kf_idx].z)));


    return poseFrom.between(poseTo);
} // doICPVirtualRelative

void process_pg()
{
    while(1)
    {
		while ( !odometryBuf.empty() && !fullResBuf.empty() )
        {
            //
            // pop and check keyframe is or not  
            // 
			mBuf.lock();       
            while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < fullResBuf.front()->header.stamp.toSec())
                odometryBuf.pop();
            if (odometryBuf.empty())
            {
                mBuf.unlock();
                break;
            }


            // Time equal check
            // timeLaserOdometry = odometryBuf.front()->header.stamp.toSec() - firstFrameTime;
            timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
            timeLaser = fullResBuf.front()->header.stamp.toSec();
            // TODO


            laserCloudFullRes->clear();
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr fusionSc (new pcl::PointCloud<PointType>);
            pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            fullResBuf.pop();

            Pose6D pose_curr = getOdom(odometryBuf.front());
            Matrix6 cov_curr = getOdomCov(odometryBuf.front());
            gtsam::Vector6 uncertain;
            for (int i = 0; i < 6; ++i) {
                // uncertain(i) = std::sqrt(cov_curr(i, i));
                uncertain(i) = cov_curr(i, i);
            }
            Pose6D_Uncertain pose_curr_uncerain;
            pose_curr_uncerain.pose.x = pose_curr.x;
            pose_curr_uncerain.pose.y = pose_curr.y;
            pose_curr_uncerain.pose.z = pose_curr.z;
            pose_curr_uncerain.pose.roll = pose_curr.roll;
            pose_curr_uncerain.pose.pitch = pose_curr.pitch;
            pose_curr_uncerain.pose.yaw = pose_curr.yaw;
            pose_curr_uncerain.uncertain = uncertain;
            odometryBuf.pop();

            fastlioPoses.push_back(pose_curr); 
            fastlioTimes.push_back(timeLaserOdometry);
            cloudSlideWindow.push_back(thisKeyFrame);
            odomSlideWindow.push_back(pose_curr);
            while (cloudSlideWindow.size() > 1) {
                cloudSlideWindow.erase(cloudSlideWindow.begin());
                odomSlideWindow.erase(odomSlideWindow.begin());
            }

            // find nearest gps 
            double eps = 0.1; // find a gps topioc arrived within eps second 
            while (!gpsBuf.empty()) {
                auto thisGPS = gpsBuf.front();
                auto thisGPSTime = thisGPS->header.stamp.toSec();
                if( abs(thisGPSTime - timeLaserOdometry) < eps ) {
                    currGPS = thisGPS;
                    hasGPSforThisKF = true; 
                    break;
                } else {
                    hasGPSforThisKF = false;
                }
                gpsBuf.pop();
            }
            mBuf.unlock(); 

            // Early reject by counting local delta movement (for equi-spereated kf drop)
            // 
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;
            
            Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform
            
            double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value. 
            translationAccumulated += delta_translation;
            // rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.  
            rotaionAccumulated += (abs(dtf.roll) + abs(dtf.pitch) + abs(dtf.yaw)); // sum just naive approach. 
            // odom_pose_fusion = odom_pose_prev+dtf;

            if( translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap ) {
                isNowKeyFrame = true;
                translationAccumulated = 0.0; // reset 
                rotaionAccumulated = 0.0; // reset 
            } else {
                isNowKeyFrame = false;
            }

            if( ! isNowKeyFrame ) 
            {
                continue; 
            }
                

            if( !gpsOffsetInitialized ) {
                if(hasGPSforThisKF) { // if the very first frame 
                    gpsAltitudeInitOffset = currGPS->altitude;
                    gpsOffsetInitialized = true;
                } 
            }

            //
            // Save data and Add consecutive node 
            //
            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr fusionScDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext.setInputCloud(thisKeyFrame);
            downSizeFilterScancontext.filter(*thisKeyFrameDS);
            // downSizeFilterScancontext.setInputCloud(fusionSc);
            // downSizeFilterScancontext.filter(*fusionScDS);

            mKF.lock(); 
            keyframeLaserClouds.push_back(thisKeyFrameDS);
            keyframeLaserCloudsCur.push_back(thisKeyFrame);
            keyframePoses.push_back(pose_curr);
            // FusionPose.pushback(odom_pose_fusion);
            keyframePosesUpdated.push_back(pose_curr); // init
            keyframePose_Uncertain.push_back(pose_curr_uncerain);
            
            keyframeTimes.push_back(timeLaserOdometry);
            keyframe2fastlio.push_back(fastlioPoses.size()-1);
            
            scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);
            // scManager.makeAndSaveScancontextAndKeys(*fusionScDS);
            laserCloudMapPGORedraw = true;
            mKF.unlock(); 

            const int prev_node_idx = keyframePoses.size() - 2; 
            const int curr_node_idx = keyframePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
            if( ! gtSAMgraphMade /* prior node */) {
                const int init_node_idx = 0; 
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
                // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

                mtxPosegraph.lock();
                {
                    // prior factor 
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    global_gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    initialEstimate.insert(init_node_idx, poseOrigin);
                    runISAM2opt();          
                }   
                mtxPosegraph.unlock();

                gtSAMgraphMade = true; 

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            }
             else /* consecutive node (and odom factor) after the prior added */ { // == keyframePoses.size() > 1 
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                mtxPosegraph.lock();
                {
                    
                    gtsam::Vector esekfNoiseVector6(6);
                    esekfNoiseVector6 << cov_curr(0,0), cov_curr(1,1), cov_curr(2,2), cov_curr(3,3), cov_curr(4,4), cov_curr(5,5);
                    esekfNoise = noiseModel::Diagonal::Variances(esekfNoiseVector6);
                    // odom factor
                    // gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise));
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), esekfNoise));
                    global_gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), esekfNoise));

                    // gps factor 
                    if(hasGPSforThisKF) {
                        double curr_altitude_offseted = currGPS->altitude - gpsAltitudeInitOffset;
                        mtxRecentPose.lock();
                        gtsam::Point3 gpsConstraint(recentOptimizedX, recentOptimizedY, curr_altitude_offseted); // in this example, only adjusting altitude (for x and y, very big noises are set) 
                        mtxRecentPose.unlock();
                        gtSAMgraph.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise));
                        global_gtSAMgraph.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise));
                        cout << "GPS factor added at node " << curr_node_idx << endl;
                    }
                    initialEstimate.insert(curr_node_idx, poseTo);  
                    runISAM2opt();
                }
                mtxPosegraph.unlock();

                if(curr_node_idx % 100 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }
            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");

            // save utility 
            std::string curr_node_idx_str = padZeros(curr_node_idx);
            // pcl::io::savePCDFileBinary(pgScansDirectory + curr_node_idx_str + ".pcd", *thisKeyFrame); // scan 
            pgTimeSaveStream << timeLaser << std::endl; // path 
        }

        // ps. 
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_pg

void performSCLoopClosure(void)
{
    if( int(keyframePoses.size()) < scManager.NUM_EXCLUDE_RECENT) // do not try too early 
        return;

    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 ) { 
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts 0 and ends n-1
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        scNum++;
        // addding actual 6D constraints in the other thread, icp_calculation.
        mBuf.unlock();
    }
} // performSCLoopClosure

void process_lcd(void)
{
    float loopClosureFrequency = 1.0; // can change szz 0.5
    ros::Rate rate(loopClosureFrequency);
    while (ros::ok())
    {
        rate.sleep();
        performSCLoopClosure();
        // performRSLoopClosure(); // TODO
    }
} // process_lcd

void process_icp(void)
{
    while(1)
    {
		while ( !scLoopICPBuf.empty() )
        {
            if( scLoopICPBuf.size() > 30 ) {
                ROS_WARN("Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            mBuf.lock(); 
            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mBuf.unlock(); 

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx);
            bool icpPass = false;
            if(relative_pose_optional) {
                gtsam::Pose3 relative_pose = relative_pose_optional.value();
                mtxPosegraph.lock();
                std::cout<< "pre" <<keyframePosesUpdated[prev_node_idx].x << " " << keyframePosesUpdated[prev_node_idx].y << " " << keyframePosesUpdated[prev_node_idx].z << std::endl;
                std::cout<< "cur"<< keyframePosesUpdated[curr_node_idx].x <<  " " << keyframePosesUpdated[curr_node_idx].y << " " << keyframePosesUpdated[curr_node_idx].z<<std::endl;
                // gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(curr_node_idx,prev_node_idx, relative_pose, robustLoopNoise));
                global_gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(curr_node_idx,prev_node_idx, relative_pose, robustLoopNoise));

                runISAM2opt();
                std::cout<< "pre" <<keyframePosesUpdated[prev_node_idx].x << " " << keyframePosesUpdated[prev_node_idx].y << " " << keyframePosesUpdated[prev_node_idx].z << std::endl;
                std::cout<< "cur"<< keyframePosesUpdated[curr_node_idx].x <<  " " << keyframePosesUpdated[curr_node_idx].y << " " << keyframePosesUpdated[curr_node_idx].z<<std::endl;
                icpPass=true;
                mtxPosegraph.unlock();
            }

            auto loopPair = std::make_pair(loop_idx_pair, icpPass);
            closureLoop.push_back(loopPair);
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_icp



void process_viz_path(void)
{
    float hz = 10.0; 
    ros::Rate rate(hz);
    while (ros::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubPath();
            pubLoopPair();
        }
    }
}

void process_isam(void)
{
    float hz = 1; 
    ros::Rate rate(hz);
    while (ros::ok()) {
        rate.sleep();
        if( gtSAMgraphMade ) {
            mtxPosegraph.lock();
            runISAM2opt();
            cout << "running isam2 optimization ..." << endl;
            mtxPosegraph.unlock();

            saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat); // pose
            saveOdometryVerticesKITTIformat(odomKITTIformat); // pose
            // saveOdomTum(odomTumformat);
        }
    }
}

void pubMap(void)
{
    int SKIP_FRAMES = 1; // sparse map visulalization to save computations 
    int counter = 0;

    laserCloudMapPGO->clear();
    // laserCloudMapPGOCur->clear();

    mKF.lock(); 
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
        if(counter % SKIP_FRAMES == 0) {
            *laserCloudMapPGO += *local2global(keyframeLaserCloudsCur[node_idx], keyframePosesUpdated[node_idx]);
            // *laserCloudMapPGOCur = *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
        }
        counter++;
    }
    mKF.unlock(); 

    // downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    // downSizeFilterMapPGO.filter(*laserCloudMapPGO);


    sensor_msgs::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "camera_init";
    pubMapAftPGO.publish(laserCloudMapPGOMsg);

}

void process_viz_map(void)
{
    float vizmapFrequency = 10; // 0.1 means run onces every 10s
    ros::Rate rate(vizmapFrequency);
    while (ros::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubMap();
        }
    }
} // pointcloud_viz

void saveFinalPoseInfo(const std::vector<Pose6D>& keyframePoses, const std::string& filename) {
    std::ofstream out_file(filename.c_str(), std::ios::out);
    if (!out_file) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return;
    }
    
    for (const auto& pose : keyframePoses) {
        out_file << pose.x << " "
                 << pose.y << " "
                 << pose.z << " "
                 << pose.roll << " "
                 << pose.pitch << " "
                 << pose.yaw << std::endl;
    }

    out_file.close();
}

void signalHandler(int signum) {
    is_shutdown = true; // 设置退出标志
    ROS_WARN("Signal %d received, preparing to shutdown...", signum);
    ros::shutdown(); // 请求ROS关闭
}
void saveRemainingData() {
    if(savePCD)
    {
        std::string filename4 = savePath + "PgoMap.pcd";
        cout << "pcl_map_save" <<endl;
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(filename4, *laserCloudMapPGO);
    }
}
int main(int argc, char **argv)
{
	// ros::init(argc, argv, "laserPGO");
    ros::init(argc, argv, "laserPGO", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
    signal(SIGINT, signalHandler);
    ros::Rate rate(5000);
    bool status = ros::ok();

	nh.param<std::string>("save_directory", save_directory, "/"); // pose assignment every k m move 
    pgKITTIformat = save_directory + "optimized_poses.txt";
    odomKITTIformat = save_directory + "odom_poses.txt";
    // odomTumformat = save_directory + "odom_tum.txt";
    pgTimeSaveStream = std::fstream(save_directory + "times.txt", std::fstream::out); 
    pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);
    pgScansDirectory = save_directory + "Scans/";
    auto unused = system((std::string("exec rm -r ") + pgScansDirectory).c_str());
    unused = system((std::string("mkdir -p ") + pgScansDirectory).c_str());

	nh.param<double>("keyframe_meter_gap", keyframeMeterGap, 2.0); // pose assignment every k m move 
	nh.param<double>("keyframe_deg_gap", keyframeDegGap, 10.0); // pose assignment every k deg rot 
    keyframeRadGap = deg2rad(keyframeDegGap);

	nh.param<double>("sc_dist_thres", scDistThres, 0.2);  
	nh.param<double>("sc_max_radius", scMaximumRadius, 80.0); // 80 is recommended for outdoor, and lower (ex, 20, 40) values are recommended for indoor 

    nh.param<bool>("savePCD",savePCD,true);
    nh.param<double>("loopfitness",loopfitness,2.0);

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;

    isam = new ISAM2(parameters);
    initNoises();

    scManager.setSCdistThres(scDistThres);
    scManager.setMaximumRadius(scMaximumRadius);

    float filter_size = 0.4; 
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    double mapVizFilterSize;
	nh.param<double>("mapviz_filter_size", mapVizFilterSize, 0.4); // pose assignment every k frames 
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);
	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);
	ros::Subscriber subGPS = nh.subscribe<sensor_msgs::NavSatFix>("/gnss", 100, gpsHandler);

	pubOdomAftPGO = nh.advertise<nav_msgs::Odometry>("/aft_pgo_odom", 100);
	pubOdomRepubVerifier = nh.advertise<nav_msgs::Odometry>("/repub_odom", 100);
	pubPathAftPGO = nh.advertise<nav_msgs::Path>("/aft_pgo_path", 100);
	pubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/aft_pgo_map", 100);
    // pubCurPointcloudPGO = nh.advertise<sensor_msgs::PointCloud2>("/aft_pgo_pcl",100);

	pubLoopScanLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_scan_local", 100);
	pubLoopSubmapLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_submap_local", 100);
    pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/loop_constraint_edge", 100);

    pose_uncertainty_pub = nh.advertise<visualization_msgs::MarkerArray>("/pose_with_uncertainty", 100);
    current_pose_det_cov_pub = nh.advertise<std_msgs::Float32>("/Current_pose_det_cov",100);
    TravelDistance_pub = nh.advertise<std_msgs::Float32>("/Total_distance",100);

    pubPathColor = nh.advertise<sensor_msgs::PointCloud2>("/PathColor", 100);


	std::thread posegraph_slam {process_pg}; // pose graph construction
	std::thread lc_detection {process_lcd}; // loop closure detection 
	std::thread icp_calculation {process_icp}; // loop constraint calculation via icp 
	// std::thread isam_update {process_isam}; // if you want to call less isam2 run (for saving redundant computations and no real-time visulization is required), uncommment this and comment all the above runisam2opt when node is added. 
	std::thread viz_map {process_viz_map}; // visualization - map (low frequency because it is heavy)
	std::thread viz_path {process_viz_path}; // visualization - path (high frequency)

 	// ros::spin();
    // while (ros::ok() && !is_shutdown) {
    //     ros::spinOnce();
    //     ros::Duration(0.01).sleep(); // 控制主循环的速率
    // }
    while (status)
    {
        if (is_shutdown) break;
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    saveRemainingData();
    // Pathcolor
    if(savePCD)
    {
        std::string filename5 = savePath + "Pathcolor.pcd";
        cout << "pcl_map_save" <<endl;
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(filename5, *Pathcolor);
    }

	return 0;
}
