#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <stack>
#include <cmath>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/pca.h>

#include <fstream>

using namespace std;

const double PI = 3.1415926;

/*定义控制参数及状态变量*/
double scanVoxelSize = 0.05;
double decayTime = 2.0;
double noDecayDis = 4.0;
double clearingDis = 8.0;
bool clearingCloud = false;
bool useSorting = true;
double quantileZ = 0.25;
bool considerDrop = false;
bool limitGroundLift = false;
double maxGroundLift = 0.15;
bool clearDyObs = false;
double minDyObsDis = 0.3;
double minDyObsAngle = 0;
double minDyObsRelZ = -0.5;
double minDyObsVFOV = -16.0;
double maxDyObsVFOV = 16.0;
int minDyObsPointNum = 1;
bool noDataObstacle = false;
int noDataBlockSkipNum = 0;
int minBlockPointNum = 10;
double vehicleHeight = 1.5;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double minRelZ = -1.5;
double maxRelZ = 0.2;
double disRatioZ = 0.2;

// terrain voxel parameters
/*定义地形体素和平面体素参数，地形体素和平面体素用于对点云进行分割和处理*/
float terrainVoxelSize = 1.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 21;
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

// planar voxel parameters
float planarVoxelSize = 0.2;
const int planarVoxelWidth = 51;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth;

/*点云指针对象，用于存储激光点云数据和分割后的点云数据*/
// pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
 pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZINormal>()); //修剪后的点云，intensity表示距离初始时间
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCropIn(new pcl::PointCloud<pcl::PointXYZI>()); //修剪之后的点云，intennsity代表反射强度
pcl::PointCloud<pcl::PointXYZINormal>::Ptr
    laserCloudDwz(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr
    terrainCloud(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr
    terrainCloudElev(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainVoxelCloud[terrainVoxelNum];

pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainCloudVar(new pcl::PointCloud<pcl::PointXYZINormal>()); //包含方差信息点云

pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainCloudGrad(new pcl::PointCloud<pcl::PointXYZINormal>()); //包含梯度信息点云

/*存储地形和平面体素信息的数组和向量*/
int terrainVoxelUpdateNum[terrainVoxelNum] = {0}; //地形体素更新次数
float terrainVoxelUpdateTime[terrainVoxelNum] = {0}; //地形体素更新时间
float planarVoxelElev[planarVoxelNum] = {0}; //平面体素的高程信息（一个体素内点的高度最小值）
int planarVoxelEdge[planarVoxelNum] = {0}; //标记平面体素边缘信息
int planarVoxelDyObs[planarVoxelNum] = {0}; //标记平面体素动态障碍物
vector<float> planarPointElev[planarVoxelNum]; //向量数组，用于存储每个平面体素内的点的高程信息
/*声明用于时间和系统状态管理变量*/
double laserCloudTime = 0; //存储激光点云时间戳
bool newlaserCloud = false; //标志新点云数据到来

double systemInitTime = 0; //取lidar第一帧时间为系统初始时间
bool systemInited = false; //标记系统初始化
int noDataInited = 0; //状态控制变量

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float vehicleXRec = 0, vehicleYRec = 0; //车辆历史位置 及noDataInited==0时的位置，若不考虑joy和clearing则为起始位置

float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;

pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilter; //体素下采样滤波


//保存pcd相关：
int frameCount = 0;
int interval = 10;
std::string savePath ="/home/gw/exploration/nav/nav_ws2/src/autonomous_exploration_development_environment/src/terrain_analysis/disZ_pcd/";
bool savePCD = false;
// bool Var_or_elev = true; //true代表根据Var计算 false代表根据Elev计算
int Var_elev_grad = 2; // 0表示根据标准差，1 高程，2，梯度；
// state estimation callback function
void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom) {
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  sinVehicleRoll = sin(vehicleRoll);
  cosVehicleRoll = cos(vehicleRoll);
  sinVehiclePitch = sin(vehiclePitch);
  cosVehiclePitch = cos(vehiclePitch);
  sinVehicleYaw = sin(vehicleYaw);
  cosVehicleYaw = cos(vehicleYaw);

  if (noDataInited == 0) {
    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    noDataInited = 1;
  }
  if (noDataInited == 1) {
    float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                     (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
    if (dis >= noDecayDis)
      noDataInited = 2;
  }
}

// registered laser scan callback function
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2) {
  laserCloudTime = laserCloud2->header.stamp.toSec();

  if (!systemInited) {
    systemInitTime = laserCloudTime;
    systemInited = true;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);

  pcl::PointXYZINormal point;

  laserCloudCrop->clear();
  laserCloudCropIn->clear();
  int laserCloudSize = laserCloud->points.size();
  /*筛选点云*/
  for (int i = 0; i < laserCloudSize; i++) {
    pcl::PointXYZINormal point = laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;
    float pointI = point.intensity;

    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                     (pointY - vehicleY) * (pointY - vehicleY));
    //一定距离内的扇形区域，z轴上原点处-2.5 —1m，最远11m -4.7——3.2m,
    //角度在-23度到16度之间的点云（从原点出发）
    if (pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
        pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.curvature = laserCloudTime - systemInitTime;
      point.intensity = pointI;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      laserCloudCrop->push_back(point);
      // point.intensity = pointI;
      // laserCloudCropIn->push_back(pointI);
    }
  }

  newlaserCloud = true;
}

// joystick callback function
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy) {
  if (joy->buttons[5] > 0.5) {
    noDataInited = 0;
    clearingCloud = true;
  }
}

// cloud clearing callback function
void clearingHandler(const std_msgs::Float32::ConstPtr &dis) {
  noDataInited = 0;
  clearingDis = dis->data;
  clearingCloud = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "terrainAnalysis");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("scanVoxelSize", scanVoxelSize);
  nhPrivate.getParam("decayTime", decayTime);
  nhPrivate.getParam("noDecayDis", noDecayDis);
  nhPrivate.getParam("clearingDis", clearingDis);
  nhPrivate.getParam("useSorting", useSorting);
  nhPrivate.getParam("quantileZ", quantileZ);
  nhPrivate.getParam("considerDrop", considerDrop);
  nhPrivate.getParam("limitGroundLift", limitGroundLift);
  nhPrivate.getParam("maxGroundLift", maxGroundLift);
  nhPrivate.getParam("clearDyObs", clearDyObs);
  nhPrivate.getParam("minDyObsDis", minDyObsDis);
  nhPrivate.getParam("minDyObsAngle", minDyObsAngle);
  nhPrivate.getParam("minDyObsRelZ", minDyObsRelZ);
  nhPrivate.getParam("minDyObsVFOV", minDyObsVFOV);
  nhPrivate.getParam("maxDyObsVFOV", maxDyObsVFOV);
  nhPrivate.getParam("minDyObsPointNum", minDyObsPointNum);
  nhPrivate.getParam("noDataObstacle", noDataObstacle);
  nhPrivate.getParam("noDataBlockSkipNum", noDataBlockSkipNum);
  nhPrivate.getParam("minBlockPointNum", minBlockPointNum);
  nhPrivate.getParam("vehicleHeight", vehicleHeight);
  nhPrivate.getParam("voxelPointUpdateThre", voxelPointUpdateThre);
  nhPrivate.getParam("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nhPrivate.getParam("minRelZ", minRelZ);
  nhPrivate.getParam("maxRelZ", maxRelZ);
  nhPrivate.getParam("disRatioZ", disRatioZ);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/registered_scan", 5, laserCloudHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

  ros::Subscriber subClearing = nh.subscribe<std_msgs::Float32>("/map_clearing", 5, clearingHandler);

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/terrain_map", 2);

  for (int i = 0; i < terrainVoxelNum; i++) {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZINormal>());
  }

  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newlaserCloud) {
      newlaserCloud = false;

      // terrain voxel roll over
      float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

      /*实现了将地形体素云数据在水平方向上向右移动一个体素的功能保持地形体素云数据与车辆位置的对齐，以便在处理和更新数据时保持一致*/
      while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                indY];
          for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
          }
          terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[indY]->clear();
        }
        terrainVoxelShiftX--;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[indY];
          for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
          }
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                            indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
              ->clear();
        }
        terrainVoxelShiftX++;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX +
                                (terrainVoxelWidth - 1)];
          for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
        }
        terrainVoxelShiftY--;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX];
          for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX +
                            (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
              ->clear();
        }
        terrainVoxelShiftY++;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      // stack registered laser scans
      pcl::PointXYZINormal point;
      int laserCloudCropSize = laserCloudCrop->points.size();
      /*将点云数据按照其相对于车辆位置的偏移量，存储到地形体素云数据中的对应位置，用于后续的地形建模或分析*/
      for (int i = 0; i < laserCloudCropSize; i++) 
      {
        pcl::PointXYZINormal point =  ->points[i];

        int indX = int((point.x - vehicleX + terrainVoxelSize / 2) /
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth;
        int indY = int((point.y - vehicleY + terrainVoxelSize / 2) /
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth;

        if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
          indY--;

        if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
            indY < terrainVoxelWidth) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
          terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
        }
      }
/*对地形体素云数据进行更新和筛选，根据一些条件对体素中的点云数据进行处理，保留符合条件的点，并更新相应的计数和时间信息*/
      for (int ind = 0; ind < terrainVoxelNum; ind++) {
        if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
            laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
                voxelTimeUpdateThre ||
            clearingCloud) {
          pcl::PointCloud<pcl::PointXYZINormal>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[ind];
          laserCloudDwz->clear();
          downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
          downSizeFilter.filter(*laserCloudDwz);

          terrainVoxelCloudPtr->clear();
          int laserCloudDwzSize = laserCloudDwz->points.size();
          for (int i = 0; i < laserCloudDwzSize; i++) {
            pcl::PointXYZINormal point = laserCloudDwz->points[i];
            float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                             (point.y - vehicleY) * (point.y - vehicleY));
            if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
                point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
                (laserCloudTime - systemInitTime - point.curvature <
                     decayTime ||
                 dis < noDecayDis) &&
                !(dis < clearingDis && clearingCloud)) {
              terrainVoxelCloudPtr->push_back(point);
            }
          }
          terrainVoxelUpdateNum[ind] = 0;
          terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;

           pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_points(new pcl::PointCloud<pcl::PointXYZ>);

          // 填充voxel_points
          for (const auto& point : *terrainVoxelCloud[ind]) 
          {
              voxel_points->push_back(pcl::PointXYZ(point.x, point.y, point.z));
          }

          if(voxel_points->size()>=3){
           
          // 执行PCA分析
          pcl::PCA<pcl::PointXYZ> pca;
          pca.setInputCloud(voxel_points);
          Eigen::Vector3f eigenValues = pca.getEigenValues();

          for ( auto& point : *terrainVoxelCloud[ind]){
            // point.normal_x= eigenValues[2]/(eigenValues[0]+eigenValues[1]+eigenValues[2]); 
              point.normal_x= eigenValues[2]; 
          }
          //  ROS_INFO( " Voxel point  have Size : %zu ,and have eigenValues: %f ,%f ,%f" , voxel_points->size(),eigenValues[0],eigenValues[1],eigenValues[2]);
          }
          else{
            
            for ( auto& point : *terrainVoxelCloud[ind])
            {
            point.normal_x = 0; 
            }
            // point.normal_x= 10;
          }
        }
      }

      /*将指定范围内的地形体素云数据合并到总的地形云数据中*/
      terrainCloud->clear();
      for (int indX = terrainVoxelHalfWidth - 5;
           indX <= terrainVoxelHalfWidth + 5; indX++) {
        for (int indY = terrainVoxelHalfWidth - 5;
             indY <= terrainVoxelHalfWidth + 5; indY++) {
          *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
        }
      }

      // estimate ground and compute elevation for each point
      for (int i = 0; i < planarVoxelNum; i++) {
        planarVoxelElev[i] = 0;
        planarVoxelEdge[i] = 0;
        planarVoxelDyObs[i] = 0;
        planarPointElev[i].clear();
      }

      int terrainCloudSize = terrainCloud->points.size();
      for (int i = 0; i < terrainCloudSize; i++) {
        pcl::PointXYZINormal point = terrainCloud->points[i];

        int indX =
            int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;
        int indY =
            int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;

        if (point.x - vehicleX + planarVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0)
          indY--;

        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
          for (int dX = -1; dX <= 1; dX++) {
            for (int dY = -1; dY <= 1; dY++) {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                    .push_back(point.z);
              }
            }
          }
        }
        if (clearDyObs) {
          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            if (dis1 > minDyObsDis) {
              float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
              if (angle1 > minDyObsAngle) {
                float pointX2 =
                    pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                float pointY2 =
                    -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                float pointZ2 = pointZ1;

                float pointX3 =
                    pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
                float pointY3 = pointY2;
                float pointZ3 =
                    pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

                float pointX4 = pointX3;
                float pointY4 =
                    pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
                float pointZ4 =
                    -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

                float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
                float angle4 = atan2(pointZ4, dis4) * 180.0 / PI;
                if (angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV) {
                  planarVoxelDyObs[planarVoxelWidth * indX + indY]++;
                }
              }
            } else {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] +=
                  minDyObsPointNum;
            }
          }
        }
      }
/*该段代码清除了在平面体素中被识别为动态障碍物的点，并将相应的平面体素 planarVoxelDyObs 的计数设置为 0 */
      if (clearDyObs) {
        for (int i = 0; i < laserCloudCropSize; i++) {
          pcl::PointXYZINormal point = laserCloudCrop->points[i];

          int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
            if (angle1 > minDyObsAngle) {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
            }
          }
        }
      }
/*计算了平面体素的高度。根据 useSorting 的取值不同，分别采用了排序和查找最小值的方式来获取高度值*/
      if (useSorting) {
        for (int i = 0; i < planarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0) {
            sort(planarPointElev[i].begin(), planarPointElev[i].end());

            int quantileID = int(quantileZ * planarPointElevSize);
            if (quantileID < 0)
              quantileID = 0;
            else if (quantileID >= planarPointElevSize)
              quantileID = planarPointElevSize - 1;

            if (planarPointElev[i][quantileID] >
                    planarPointElev[i][0] + maxGroundLift &&
                limitGroundLift) {
              planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
            } else {
              planarVoxelElev[i] = planarPointElev[i][quantileID];
            }
          }
        }
      } else {
        for (int i = 0; i < planarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0) {
            float minZ = 1000.0;
            int minID = -1;
            for (int j = 0; j < planarPointElevSize; j++) {
              if (planarPointElev[i][j] < minZ) {
                minZ = planarPointElev[i][j];
                minID = j;
              }
            }

            if (minID != -1) {
              planarVoxelElev[i] = planarPointElev[i][minID];
            }
          }
        }
      }

    /*计算voxel内点的方差并筛选*/
    if(Var_elev_grad == 0)
    {
      ROS_INFO("begin ground detect with VAR");
      terrainCloudVar->clear();
      int terrainCloudVarSize =0;
      float max_std_dev = -std::numeric_limits<float>::max();
      float min_std_dev = std::numeric_limits<float>::max();
      for (int i =0;i<terrainCloudSize;i++)
      {
          pcl::PointXYZINormal point = terrainCloud->points[i];
          if(point.z - vehicleZ > minRelZ && point.z -vehicleZ < maxRelZ)
          {
              int indX = int((point.x - vehicleX +planarVoxelSize /2) / planarVoxelSize)+planarVoxelHalfWidth;

              int indY = int((point.y -vehicleY + planarVoxelSize /2 ) /planarVoxelSize) +planarVoxelHalfWidth;

              if(point.x -vehicleX +planarVoxelSize /2 <0)
                  indX--;
              if(point.y -vehicleY +planarVoxelSize /2 <0)
                  indY--;
              
              if(indX >= 0 && indX < planarVoxelWidth && indY >=0 && indY < planarVoxelWidth)
              {
                  if (planarVoxelDyObs[planarVoxelWidth*indX +indY]< minDyObsPointNum || !clearDyObs)
                  {
                      float disZ = point.z - planarVoxelElev[planarVoxelWidth*indX +indY];
                      if (considerDrop)
                          disZ =fabs(disZ);
                      int planarPointElevSize = planarPointElev[planarVoxelWidth*indX + indY].size();

                      if(disZ >= 0 && disZ < vehicleHeight && planarPointElevSize >=minBlockPointNum)
                      {
                          float sum = 0.0;
                          float sumSquared = 0.0;
                          for(const auto& p: planarPointElev[planarVoxelWidth*indX +indY])
                          {
                              sum +=p;
                              sumSquared += p*p;
                          }

                          float mean = sum / planarPointElevSize;
                          float variance = (sumSquared / planarPointElevSize) - (mean * mean);
                          float std_dev = sqrt(variance);
                          terrainCloudVar -> push_back(point);
                          terrainCloudVar ->points[terrainCloudVarSize].intensity =std_dev;
                          // terrainCloudVar ->points[terrainCloudVarSize].intensity =variance;

                            // 更新最大最小标准差值
                          if (std_dev > max_std_dev)
                              max_std_dev = std_dev;
                          if (std_dev < min_std_dev)
                              min_std_dev = std_dev;
                          //  if (variance > max_std_dev)
                          //     max_std_dev = variance;
                          // if (variance < min_std_dev)
                          //     min_std_dev = variance;

                          terrainCloudVarSize++;
                      }
                  }
              }

          }

      }
      //归一化处理
      /*【0，1】*/
      for (int i = 0; i < terrainCloudVarSize; i++) 
      {
      float normalized_std_dev = (terrainCloudVar->points[i].intensity - min_std_dev) / (max_std_dev - min_std_dev);
      terrainCloudVar->points[i].intensity = normalized_std_dev;
      }
      // 将范围映射到 [-1, 1]
      // for (int i = 0; i < terrainCloudVarSize; i++) 
      // {
      // float normalized_std_dev = (terrainCloudVar->points[i].intensity - min_std_dev) / (max_std_dev - min_std_dev);
      // normalized_std_dev = 2 * normalized_std_dev - 1; 
      // terrainCloudVar->points[i].intensity = normalized_std_dev;
      // }
    }
    

  /*生成了一个具有高度信息的地形点云 terrainCloudElev*/
    if(Var_elev_grad == 1)
    {
     ROS_INFO("begin ground detect with ELEV");  
     terrainCloudElev->clear();
      int terrainCloudElevSize = 0;
      for (int i = 0; i < terrainCloudSize; i++) {
        pcl::PointXYZINormal point = terrainCloud->points[i];
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
          int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            if (planarVoxelDyObs[planarVoxelWidth * indX + indY] <
                    minDyObsPointNum ||
                !clearDyObs) {
              float disZ =
                  point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
              if (considerDrop)
                disZ = fabs(disZ);
              int planarPointElevSize =
                  planarPointElev[planarVoxelWidth * indX + indY].size();
              if (disZ >= 0 && disZ < vehicleHeight &&
                  planarPointElevSize >= minBlockPointNum) {
                terrainCloudElev->push_back(point);
                terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;

                terrainCloudElevSize++;
              }
            }
          }
        }
      }
    }
    if(Var_elev_grad == 2)
    {
      ROS_INFO("begin ground detect with GRAD");
      terrainCloudGrad->clear();
      int terrainCloudGradSize = 0;
      float max_std_dev = -std::numeric_limits<float>::max();
      float min_std_dev = std::numeric_limits<float>::max();
      for (int i = 0; i < terrainCloudSize; i++) {
        pcl::PointXYZINormal point = terrainCloud->points[i];
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) 
        {
          int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
              indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
              indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth) 
          {
            if (planarVoxelDyObs[planarVoxelWidth * indX + indY] < minDyObsPointNum || !clearDyObs) 
            {
              float disZ = point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
              if (considerDrop)
                  disZ = fabs(disZ);
              int planarPointElevSize = planarPointElev[planarVoxelWidth * indX + indY].size();
              if (disZ >= 0 && disZ < vehicleHeight && planarPointElevSize >= minBlockPointNum) 
              {
                float sum = 0.0;
                float sumSquared = 0.0;
                // // 计算梯度
                // float gradX = 0.0;
                // float gradY = 0.0;
                // for (int i = 0; i < planarPointElevSize; i++) 
                // {
                //     gradX += (planarPointElev[planarVoxelWidth * indX + indY][i] - point.z) * (point.x - vehicleX);
                //     gradY += (planarPointElev[planarVoxelWidth * indX + indY][i] - point.z) * (point.y - vehicleY);
                // }
                // // 计算梯度的模长
                // float gradientMagnitude = sqrt(gradX * gradX + gradY * gradY);
                // 计算梯度
                // float gradX = 0.0;
                // float gradY = 0.0;
                // for (int i = 0; i < planarPointElevSize; i++) 
                // {
                //     float dx = point.x - vehicleX;
                //     float dy = point.y - vehicleY;
                //     float dz = planarPointElev[planarVoxelWidth * indX + indY][i] - point.z;
                //     if (dx != 0) {
                //         gradX += dz / dx;
                //     }
                //     if (dy != 0) {
                //         gradY += dz / dy;
                //     }
                // }

                // // 平均化梯度
                // gradX /= planarPointElevSize;
                // gradY /= planarPointElevSize;

                // // 计算梯度的模长
                // float gradientMagnitude = sqrt(gradX * gradX + gradY * gradY);
                //  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_points(new pcl::PointCloud<pcl::PointXYZ>);

                //   // 填充voxel_points
                //   for (const auto& point : *terrainVoxelCloud[planarVoxelWidth * indX + indY]) {
                //       voxel_points->push_back(pcl::PointXYZ(point.x, point.y, point.z));
                //   }

                //   // 执行PCA分析
                //   pcl::PCA<pcl::PointXYZ> pca;
                //   pca.setInputCloud(voxel_points);
                //   Eigen::Vector3f eigenValues = pca.getEigenValues();
                //   float smallestEigenvalue = eigenValues[2];
                terrainCloudGrad->push_back(point);
                float pointI = point.intensity;
                // if (gradientMagnitude<=12 && pointI <=20) //针对非油漆平坦路面
                // {
                //   terrainCloudGrad->points[terrainCloudGradSize].intensity =0.5*(gradientMagnitude+pointI);
                // }
                // else if(gradientMagnitude<=12 && 45 >= pointI >=20)
                // {
                //   terrainCloudGrad->points[terrainCloudGradSize].intensity = 0.2*(pointI + gradientMagnitude);
                // }
                // else 
                // {
                //   terrainCloudGrad->points[terrainCloudGradSize].intensity = pointI + gradientMagnitude;
                // }
                for(const auto& p: planarPointElev[planarVoxelWidth*indX +indY])
                {
                    sum +=p;
                    sumSquared += p*p;
                }
                float mean = sum / planarPointElevSize;
                float variance = (sumSquared / planarPointElevSize) - (mean * mean);
                float std_dev = sqrt(variance);
                  // 更新最大最小标准差值
                if (std_dev > max_std_dev)
                    max_std_dev = std_dev;
                if (std_dev < min_std_dev)
                    min_std_dev = std_dev;

              // float dx = point.x - vehicleX;
              // float dy = point.y - vehicleY;
              // // 计算梯度的模长
              // float gradientMagnitude = abs(point.z/sqrt(dx * dx + dy * dy));
                terrainCloudGrad->points[terrainCloudGradSize].curvature = pointI;
                terrainCloudGrad->points[terrainCloudGradSize].intensity = disZ;
                terrainCloudGrad->points[terrainCloudGradSize].normal_x = point.normal_x;
                terrainCloudGrad->points[terrainCloudGradSize].normal_y = std_dev;
                // terrainCloudGrad->points[terrainCloudGradSize].normal_z = gradientMagnitude;

                //  terrainCloudGrad->points[terrainCloudGradSize].intensity = pointI + gradientMagnitude;

                terrainCloudGradSize++;
              }
            }
          }
        }
      }
    }
    if(Var_elev_grad == 3) //考虑区域宽度
    {
      ROS_INFO("begin ground detect with GRAD");
      terrainCloudGrad->clear();
      int terrainCloudGradSize = 0;
      for (int i = 0; i < terrainCloudSize; i++) {
        pcl::PointXYZINormal point = terrainCloud->points[i];
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) 
        {
          int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
              indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
              indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth) 
          {
            if (planarVoxelDyObs[planarVoxelWidth * indX + indY] < minDyObsPointNum || !clearDyObs) 
            {
              float disZ = point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
              if (considerDrop)
                  disZ = fabs(disZ);
              int planarPointElevSize = planarPointElev[planarVoxelWidth * indX + indY].size();
              if (disZ >= 0 && disZ < vehicleHeight && planarPointElevSize >= minBlockPointNum) 
              {
                // 计算梯度
                float gradX = 0.0;
                float gradY = 0.0;
                for (int i = 0; i < planarPointElevSize; i++) 
                {
                    gradX += (planarPointElev[planarVoxelWidth * indX + indY][i] - point.z) * (point.x - vehicleX);
                    gradY += (planarPointElev[planarVoxelWidth * indX + indY][i] - point.z) * (point.y - vehicleY);
                }
                // 计算梯度的模长
                float gradientMagnitude = sqrt(gradX * gradX + gradY * gradY);


                terrainCloudGrad->push_back(point);
                float pointI = point.intensity;
                // //算出区域
                float regionWidth = 0.0;
                float regionLength = 0.0;
                float regionsize = 0.0;
                float regionArea = 1.0;
                int adjacentIndex = i + 1;
                while (adjacentIndex < terrainCloudSize) {
                    float adjacentIntensity = terrainCloud->points[adjacentIndex].intensity;
                    float distanceDiff = std::sqrt(std::pow(point.x - terrainCloud->points[adjacentIndex].x, 2) + std::pow(point.y - terrainCloud->points[adjacentIndex].y, 2));
                    float intensityDiff = std::abs(pointI - adjacentIntensity);
                    if (distanceDiff <= 0.5,intensityDiff <= 15 && pointI >= 20) //非路面区域
                    {
                      regionArea ++;
                      regionWidth += std::abs(terrainCloud->points[adjacentIndex].y - terrainCloud->points[adjacentIndex - 1].y);
                      regionLength += std::abs(terrainCloud->points[adjacentIndex].x - terrainCloud->points[adjacentIndex - 1].x);
                   
                        adjacentIndex++;
                    } else {
                        break;
                    }
                }
                regionsize = regionWidth*regionLength;
                // float regionArea = 1.0;  // 初始面积为1，考虑当前点本身
                // std::vector<bool> visited(terrainCloudSize, false);  // 记录点是否被访问过
                // std::stack<int> stack;
                // stack.push(i);  // 将当前点入栈
                // while (!stack.empty()) 
                // {
                //     int currentIndex = stack.top();
                //     stack.pop();
                //     visited[currentIndex] = true;

                //     // 处理当前点
                //     float pointI = terrainCloud->points[currentIndex].intensity;
                //     float pointX = terrainCloud->points[currentIndex].x;
                //     float pointY = terrainCloud->points[currentIndex].y;

                //     // 遍历相邻点
                //     for (int j = 0; j < terrainCloudSize; j++) {
                //         if (!visited[j]) {
                //             float adjacentIntensity = terrainCloud->points[j].intensity;
                //             float adjacentX = terrainCloud->points[j].x;
                //             float adjacentY = terrainCloud->points[j].y;

                //             float intensityDiff = std::abs(pointI - adjacentIntensity);
                //             float distance = std::sqrt(std::pow(pointX - adjacentX, 2) + std::pow(pointY - adjacentY, 2));

                //             if (intensityDiff <= 15 && distance <= 0.2) {
                //                 regionArea++;
                //                 stack.push(j);  // 将相邻点入栈
                //                 visited[j] = true;  // 标记相邻点为已访问
                //             }
                //         }
                //     }
                // }
                ROS_INFO("regionAreaSize: %f" ,regionArea);
                if (regionArea > 20) //此时为草坪路段
                {
                  ROS_INFO("graff increasing");
                  // terrainCloudGrad->points[terrainCloudGradSize].intensity =(gradientMagnitude+pointI);
                  terrainCloudGrad->points[terrainCloudGradSize].intensity =1;
                }
                else 
                {
                  // terrainCloudGrad->points[terrainCloudGradSize].intensity = 0.1*pointI + gradientMagnitude;
                  terrainCloudGrad->points[terrainCloudGradSize].intensity = -1;
                }

                terrainCloudGradSize++;
              }
            }
          }
        }
      }
    }
     
/*用于处理没有数据的障碍物情况*/
      if (noDataObstacle && noDataInited == 2) {
        for (int i = 0; i < planarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize < minBlockPointNum) {
            planarVoxelEdge[i] = 1;
          }
        }

        for (int noDataBlockSkipCount = 0;
             noDataBlockSkipCount < noDataBlockSkipNum;
             noDataBlockSkipCount++) {
          for (int i = 0; i < planarVoxelNum; i++) {
            if (planarVoxelEdge[i] >= 1) {
              int indX = int(i / planarVoxelWidth);
              int indY = i % planarVoxelWidth;
              bool edgeVoxel = false;
              for (int dX = -1; dX <= 1; dX++) {
                for (int dY = -1; dY <= 1; dY++) {
                  if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                      indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                    if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                        dY] < planarVoxelEdge[i]) {
                      edgeVoxel = true;
                    }
                  }
                }
              }

              if (!edgeVoxel)
                planarVoxelEdge[i]++;
            }
          }
        }

        for (int i = 0; i < planarVoxelNum; i++) {
          if (planarVoxelEdge[i] > noDataBlockSkipNum) {
            int indX = int(i / planarVoxelWidth);
            int indY = i % planarVoxelWidth;

            point.x =
                planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
            point.y =
                planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
            point.z = vehicleZ;
            point.curvature = vehicleHeight;

            point.x -= planarVoxelSize / 4.0;
            point.y -= planarVoxelSize / 4.0;
            terrainCloudElev->push_back(point);

            point.x += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.y += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.x -= planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);
          }
        }
      }

      clearingCloud = false;
      //保存pcd文件：
      if(savePCD)
      {
        if(Var_elev_grad == 0)
        {
          if(frameCount%interval == 0)
          {
          std::string filename = savePath + "frame_" +std::to_string(frameCount) ;
          pcl::io::savePCDFileASCII(filename+".pcd", *terrainCloudVar);
          std::ofstream intensityFile(filename+".txt");
          if (intensityFile.is_open())
          {
            for(const auto& point : terrainCloudVar->points)
            {
              intensityFile << point.intensity << std::endl;
            }
            intensityFile.close();
          }
          }
          frameCount++;
        }
        else if(Var_elev_grad == 1)
        {
          if(frameCount%interval == 0)
          {
          std::string filename = savePath + "frame_" +std::to_string(frameCount) ;
          pcl::io::savePCDFileASCII(filename+".pcd", *terrainCloudElev);
          std::ofstream intensityFile(filename+".txt");
          if (intensityFile.is_open())
          {
            for(const auto& point : terrainCloudElev->points)
            {
              intensityFile << point.x <<" "<< point.y<<" "<<point.z<<" "<<point.curvature<<" "<<point.intensity << std::endl;
            }
            intensityFile.close();
          }
          }
          frameCount++;
        }
        else if(Var_elev_grad == 2)
        {
          if(frameCount%interval == 0)
          {
          std::string filename = savePath + "frame_" +std::to_string(frameCount) ;
          pcl::io::savePCDFileASCII(filename+".pcd", *terrainCloudGrad);
          std::ofstream intensityFile(filename+".txt");
          if (intensityFile.is_open())
          {
            for(const auto& point : terrainCloudGrad->points)
            {
              intensityFile << point.x <<" "<< point.y<<" "<<point.z<<" "<<point.curvature<<" "<<point.intensity << std::endl;
            }
            intensityFile.close();
          }
          }
          frameCount++;
        }


      }


      // publish points with elevation
      sensor_msgs::PointCloud2 terrainCloud2;
      if(Var_elev_grad == 0){
        pcl::toROSMsg(*terrainCloudVar, terrainCloud2); 
      }
      else if(Var_elev_grad ==1)
      {
        pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
      }
      else if(Var_elev_grad == 2)
      {
        pcl::toROSMsg(*terrainCloudGrad, terrainCloud2);
      }
      
      terrainCloud2.header.stamp = ros::Time().fromSec(laserCloudTime);
      terrainCloud2.header.frame_id = "map";
      pubLaserCloud.publish(terrainCloud2);
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
