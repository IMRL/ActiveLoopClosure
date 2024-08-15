#include "osmplanner/grid.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "osmplanner/DBSCAN/DBSCAN_kdtree.h"
// #include <pcl-1.13/pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/ximgproc.hpp>

namespace osmplanner_ns
{
    OccupancyGrid::OccupancyGrid(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  initialize();
}

OccupancyGrid::~OccupancyGrid()
{
}

bool OccupancyGrid::readParameters()
{
  nh_private_.getParam("/grid/world_frame_id", world_frame_id_);
  nh_private_.getParam("/grid/odomSubTopic", sub_odom_topic_);
  nh_private_.getParam("/grid/terrainCloudSubTopic", sub_terrain_point_cloud_topic_);
  nh_private_.getParam("/grid/pubGridPointsTopic", pub_grid_points_topic_);
  nh_private_.getParam("/grid/pubRayCastPointsTopic",pub_rayCast_points_topic);
  nh_private_.getParam("/grid/rayCastLineTopic",pub_rayCast_line_topic);
  nh_private_.getParam("/grid/kMapWidth", kMapWidth);
  nh_private_.getParam("/grid/kGridSize", kGridSize);
  nh_private_.getParam("/grid/kDownsampleSize", kDownsampleSize);
  nh_private_.getParam("/grid/kObstacleHeightThre", kObstacleHeightThre);
  nh_private_.getParam("/grid/kFlyingObstacleHeightThre", kFlyingObstacleHeightThre);
  nh_private_.getParam("/rm/kBoundX", kCollisionCheckX);
  nh_private_.getParam("/rm/kBoundY", kCollisionCheckY);

  nh_private_.getParam("/grid/raycastFilterType",filterType);
  nh_private_.getParam("/grid/sorFilter/Meank",Meank);
  nh_private_.getParam("/grid/sorFilter/stddev",stddev);
  nh_private_.getParam("/grid/rorFilter/radiusSearch",radiusSearch);
  nh_private_.getParam("/grid/rorFilter/Min_neighbors",Min_neighbors);

  nh_private_.getParam("/grid/pubRoadLineTopic",pub_road_line_topic);
  // nh_private_.getParam("/grid/roadSplinePubTopic",pub_road_curve_topic);
  nh_private_.getParam("/grid/pub_Cut_RayCast_topic",pub_Cut_RayCast_topic);
  nh_private_.getParam("/grid/pub_contours_topic",pub_contours_topic);
  nh_private_.getParam("/grid/rayCast_Cutdegree",rayCast_Cutdegree);
  nh_private_.getParam("/grid/rayCast_near_range",rayCast_near_range);

  nh_private_.getParam("/grid/cluster_topic",pub_cluster_topic);
  nh_private_.getParam("/grid/k_max_radius",k_max_radius);

  //DBSCAN相关参数：
  nh_private_.getParam("/DBSCAN/CorePointMinPts",CorePointMinPts);
  nh_private_.getParam("/DBSCAN/ClusterTolerance",ClusterTolerance);
  nh_private_.getParam("/DBSCAN/MinClusterSize",MinClusterSize);
  nh_private_.getParam("/DBSCAN/MaxClusterSize",MaxClusterSize);

  //保存PCD相关：
  nh_private_.getParam("/PCD_SaveOrNot",PCD_SaveOrNot);
  // nh_private_.getParam("/PCD_save_Path_1",PCD_save_Path_1);
  // nh_private_.getParam("/PCD_save_Path_2",PCD_save_Path_2);

  //保存bev相关
  // nh_private_.getParam("/IMG_save_Path",IMG_save_Path);
  nh_private_.getParam("/img_saveorNot",img_saveorNot);


  return true;
}

bool OccupancyGrid::initialize()
{
  if (!readParameters())
    return false;
  odom_sub_.subscribe(nh_, sub_odom_topic_, 1);
  terrain_point_cloud_sub_.subscribe(nh_, sub_terrain_point_cloud_topic_, 1);
  sync_.reset(new Sync(syncPolicy(100), odom_sub_, terrain_point_cloud_sub_ ));
  sync_->registerCallback(boost::bind(&OccupancyGrid::terrainCloudAndOdomCallback, this, _1, _2));

  grid_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_grid_points_topic_, 1);
  pub_contours_cloud = nh_.advertise<sensor_msgs::PointCloud2>(pub_contours_topic,1);
  int frame_counter = 0;
  map_half_width_grid_num_ = int(kMapWidth / 2 / kGridSize);
  map_width_grid_num_ = map_half_width_grid_num_ * 2 + 1;

  clearGrid();

  ROS_INFO("Successfully launched OccupancyGrid node");
  return true;
  
}

void OccupancyGrid::terrainCloudAndOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                                const sensor_msgs::PointCloud2::ConstPtr& terrain_msg)
{
  terrain_time_ = terrain_msg->header.stamp;
  robot_position_[0] = odom_msg->pose.pose.position.x;
  robot_position_[1] = odom_msg->pose.pose.position.y;
  robot_position_[2] = odom_msg->pose.pose.position.z;
  StateVec rayCast_orin(robot_position_[0], robot_position_[1], robot_position_[2]);
  terrain_cloud_->clear();
  terrain_cloud_ds->clear();
  terrain_cloud_traversable_->clear();
  terrain_cloud_obstacle_->clear();
  pcl::fromROSMsg(*terrain_msg, *terrain_cloud_);

  pcl::VoxelGrid<pcl::PointXYZI> point_ds;
  point_ds.setLeafSize(kDownsampleSize, kDownsampleSize, kDownsampleSize);
  point_ds.setInputCloud(terrain_cloud_);
  point_ds.filter(*terrain_cloud_ds);

  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  pcl::PointXYZI point;
  int terrainCloudSize = terrain_cloud_ds->points.size();
  for (int i = 0; i < terrainCloudSize; i++)
  {
    point.x = terrain_cloud_ds->points[i].x;
    point.y = terrain_cloud_ds->points[i].y;
    point.z = terrain_cloud_ds->points[i].z;
    point.intensity = terrain_cloud_ds->points[i].intensity;
    if (point.intensity == 0 || point.intensity == 1)
    {
      terrain_cloud_obstacle_->push_back(point);
    }
    else if (point.intensity == 2)
    {
      terrain_cloud_traversable_->push_back(point);
    }
  }

  clearGrid();
  updateGrid();
  processGridMap();
  // if(PCD_SaveOrNot)
  // {
  //   std::string filename1 = PCD_save_Path_1 + "/frame_" + std::to_string(frame_counter) + ".pcd";
  //   savePCD_TxT(terrain_cloud_ds,filename1);
    
  // }
  // if(img_saveorNot){
  //   convertGridToImage();
  // }
  frame_counter++;
  publishGridMap();
  // if(PCD_SaveOrNot)
  // {
  //   std::string filename2 = PCD_save_Path_2 + "/frame_" + std::to_string(frame_counter) + ".pcd";
  //   grid_cloud_->width = grid_cloud_->points.size();
  //   grid_cloud_->height = 1;
  //   savePCD_TxT(grid_cloud_,filename2);
  //   ROS_INFO("saving gird map");
  // }
}

geometry_msgs::Point OccupancyGrid::getPoint(GridIndex p)
{
  int indX = p[0];
  int indY = p[1];
  double x = kGridSize * (indX - map_half_width_grid_num_) + robot_position_[0];
  double y = kGridSize * (indY - map_half_width_grid_num_) + robot_position_[1];
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = robot_position_[2];
  return point;
}

GridIndex OccupancyGrid::getIndex(StateVec point)
{
  int indX = int((point.x() - robot_position_[0] + kGridSize / 2) / kGridSize) + map_half_width_grid_num_;
  int indY = int((point.y() - robot_position_[1] + kGridSize / 2) / kGridSize) + map_half_width_grid_num_;
  if (point.x() - robot_position_[0] + kGridSize / 2 < 0)
    indX--;
  if (point.y() - robot_position_[1] + kGridSize / 2 < 0)
    indY--;
  if (indX < 0)
    indX = 0;
  if (indY < 0)
    indY = 0;
  if (indX > map_width_grid_num_ - 1)
    indX = map_width_grid_num_ - 1;
  if (indY > map_width_grid_num_ - 1)
    indY = map_width_grid_num_ - 1;
  GridIndex grid_index(indX, indY);
  return grid_index;
}

void OccupancyGrid::clearGrid()
{
  gridState_.clear();
  std::vector<int> y_vector;
  for (int i = 0; i < map_width_grid_num_; i++)
  {
    y_vector.clear();
    for (int j = 0; j < map_width_grid_num_; j++)
    {
      gridStatus grid_state = unknown;
      y_vector.push_back(grid_state);
    }
    gridState_.push_back(y_vector);
  }
}

void OccupancyGrid::updateGrid()
{
  pcl::PointXYZI point;
  for (int i = 0; i < terrain_cloud_obstacle_->points.size(); i++)
  {
    point = terrain_cloud_obstacle_->points[i];
    int indX = int((point.x - robot_position_[0] + kGridSize / 2) / kGridSize) + map_half_width_grid_num_;
    int indY = int((point.y - robot_position_[1] + kGridSize / 2) / kGridSize) + map_half_width_grid_num_;
    if (point.x - robot_position_[0] + kGridSize / 2 < 0)
      indX--;
    if (point.y - robot_position_[1] + kGridSize / 2 < 0)
      indY--;
    if (indX < 0)
      indX = 0;
    if (indY < 0)
      indY = 0;
    if (indX > map_width_grid_num_ - 1)
      indX = map_width_grid_num_ - 1;
    if (indY > map_width_grid_num_ - 1)
      indY = map_width_grid_num_ - 1;

    if (indX >= 0 && indX < map_width_grid_num_ && indY >= 0 && indY < map_width_grid_num_)
    {
      gridStatus grid_state = occupied;
      gridState_[indX][indY] = grid_state;
    }
  }
  for (int i = 0; i < terrain_cloud_traversable_->points.size(); i++)
  {
    point = terrain_cloud_traversable_->points[i];
    int indX = int((point.x - robot_position_[0] + kGridSize / 2) / kGridSize) + map_half_width_grid_num_;
    int indY = int((point.y - robot_position_[1] + kGridSize / 2) / kGridSize) + map_half_width_grid_num_;
    if (point.x - robot_position_[0] + kGridSize / 2 < 0)
      indX--;
    if (point.y - robot_position_[1] + kGridSize / 2 < 0)
      indY--;
    if (indX < 0)
      indX = 0;
    if (indY < 0)
      indY = 0;
    if (indX > map_width_grid_num_ - 1)
      indX = map_width_grid_num_ - 1;
    if (indY > map_width_grid_num_ - 1)
      indY = map_width_grid_num_ - 1;
    if (indX >= 0 && indX < map_width_grid_num_ && indY >= 0 && indY < map_width_grid_num_)
    {
      if (gridState_[indX][indY] == 2)
      {
        continue;
      }
      if (updateFreeGridWithSurroundingGrids(indX, indY) == false)
      {
        gridStatus grid_state = free;
        gridState_[indX][indY] = grid_state;
      }
      else
      {
        gridStatus grid_state = near_occupied;
        gridState_[indX][indY] = grid_state;
      }
    }
  }
}

void OccupancyGrid::publishGridMap()
{
  grid_cloud_->clear();
  pcl::PointXYZI p1;
  geometry_msgs::Point p2;
  GridIndex p3;
  for (int i = 0; i < map_width_grid_num_; i++)
  {
    for (int j = 0; j < map_width_grid_num_; j++)
    {
      p3[0] = i;
      p3[1] = j;
      p2 = getPoint(p3);
      p1.x = p2.x;
      p1.y = p2.y;
      p1.z = p2.z;
      p1.intensity = gridState_[i][j];
      grid_cloud_->points.push_back(p1);
    }
  }
  
  sensor_msgs::PointCloud2 gridCloud2;
  pcl::toROSMsg(*grid_cloud_, gridCloud2);
  gridCloud2.header.stamp = terrain_time_;
  gridCloud2.header.frame_id = world_frame_id_;
  grid_cloud_pub_.publish(gridCloud2);
}

bool OccupancyGrid::updateFreeGridWithSurroundingGrids(int indx, int indy)
{
  int count_x = ceil(0.5 * kCollisionCheckX / kGridSize);
  int count_y = ceil(0.5 * kCollisionCheckY / kGridSize);
  int indX;
  int indY;
  for (int i = -count_x; i <= count_x; i++)
  {
    for (int j = -count_y; j <= count_y; j++)
    {
      indX = indx + i;
      indY = indy + j;
      if (indX >= 0 && indX < map_width_grid_num_ && indY >= 0 && indY < map_width_grid_num_)
      {
        if (gridState_[indX][indY] == 2)
        {
          return true;
        }
      }
    }
  }
  return false;
}

bool OccupancyGrid::InRange(const GridIndex sub, const GridIndex max_sub, const GridIndex min_sub)
{
  return sub.x() >= min_sub.x() && sub.x() <= max_sub.x() && sub.y() >= min_sub.y() && sub.y() <= max_sub.y();
}

int OccupancyGrid::signum(int x)
{
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

double OccupancyGrid::mod(double value, double modulus)
{
  return fmod(fmod(value, modulus) + modulus, modulus);
}


void OccupancyGrid::visualizeLines(std::vector<pcl::ModelCoefficients::Ptr> lines)
{
  visualization_msgs::MarkerArray road_line;
  visualization_msgs::Marker line_list;
    line_list.header.frame_id = world_frame_id_;
    line_list.header.stamp = terrain_time_;
    line_list.ns = "Road_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = 0.5;

    line_list.color.b = 1.0;
    line_list.color.a = 1.0;

    for (auto &coefficients : lines) {
        geometry_msgs::Point p1, p2;
        p1.x = coefficients->values[0]-10 * coefficients->values[3];
        p1.y = coefficients->values[1]-10 * coefficients->values[4];
        p1.z = coefficients->values[2]-10 * coefficients->values[5];

        p2.x = coefficients->values[0] + 10*coefficients->values[3];
        p2.y = coefficients->values[1] + 10*coefficients->values[4];
        p2.z = coefficients->values[2] + 10*coefficients->values[5];

        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        ROS_INFO("RoadLine Visualization!!!!! p1: %f , p2: %f",p1.x,p2.x);
        road_line.markers.push_back(line_list);
    }
    roadLinePub.publish(road_line);
       
}

double OccupancyGrid::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr OccupancyGrid::cutPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud,
                                                   double robot_x, double robot_y, double robot_yaw)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr cut_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    double angle_range = rayCast_Cutdegree * M_PI / 180.0;
    double min_angle = normalizeAngle(robot_yaw - angle_range / 2.0);
    double max_angle = normalizeAngle(robot_yaw + angle_range / 2.0);
    for (const auto& point : full_cloud->points)
    {
        double dx = point.x - robot_x;
        double dy = point.y - robot_y;
        double point_angle = normalizeAngle(atan2(dy, dx));
        if (min_angle > max_angle) 
        {
            if (!(point_angle >= max_angle && point_angle <= min_angle))
                cut_cloud->points.push_back(point);
        } else {
            if (point_angle >= min_angle && point_angle <= max_angle)
                cut_cloud->points.push_back(point);
        }
    }
    cut_cloud->width = cut_cloud->points.size();
    cut_cloud->height = 1;
    return cut_cloud;
}

void OccupancyGrid::savePCD_TxT(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,std::string filename)
{

    if (!input_cloud->empty())
    {
      if (frame_counter % 10 == 0)
      {
          pcl::io::savePCDFileASCII (filename, *input_cloud);
      }
    }
    else
    {
        ROS_WARN("The point cloud is empty. Nothing was saved.");
    }
}

void OccupancyGrid::convertGridToImage()
{
    if(frame_counter % 10 == 0 )
    {
      cv::Mat image(map_width_grid_num_, map_width_grid_num_, CV_8UC1);

      for (int i = 0; i < map_width_grid_num_; i++)
      {
        for (int j = 0; j < map_width_grid_num_; j++)
        {
            image.at<uchar>(i, j) = gridState_[i][j]*100;
        }
      }
      std::string filename = IMG_save_Path + std::to_string(frame_counter) + ".png";
      cv::imwrite(filename, image);
    }
    
}
void OccupancyGrid::processGridMap( )
{
  cv::Mat image(map_width_grid_num_, map_width_grid_num_, CV_8UC1);
  for (int i = 0; i < map_width_grid_num_; i++)
  {
    for (int j = 0; j < map_width_grid_num_; j++)
    {
        if (gridState_[i][j] == free) {
            image.at<uchar>(i, j) = 255;
        } else {
            image.at<uchar>(i, j) = 0;
        }
    }
  }
  cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
  cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(image, image, cv::MORPH_CLOSE, element1);
  cv::morphologyEx(image, image, cv::MORPH_OPEN, element2);
  cv::Mat thinned_image;
  cv::ximgproc::thinning(image, thinned_image);
  for (int i = 0; i < map_width_grid_num_; i++)
  {
    for (int j = 0; j < map_width_grid_num_; j++)
    {
        if (thinned_image.at<uchar>(i, j) == 255) {
            gridState_[i][j] = free;
         } 
        if (image.at<uchar>(i, j) == 255) {
            gridState_[i][j] = free;
         } 
    }
  }
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(thinned_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  double max_area = 0;
  std::vector<cv::Point> max_hull;
  for (const auto& contour : contours)
  {
    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    double area = cv::contourArea(hull);
    if (area > max_area)
    {
      max_area = area;
      max_hull = hull;
    }
  }
  contours_cloud->clear();

  GridIndex p3;
  geometry_msgs::Point p2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr contours_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (const cv::Point& pt : max_hull)
  {
      pcl::PointXYZI point;
      p3[1] = pt.x;
      p3[0] = pt.y;
      p2 =getPoint(p3);
      point.x = p2.x;
      point.y = p2.y;
      point.z = robot_position_[2]; 
      point.intensity = 0;
      contours_cloud->points.push_back(point);
  }
  sensor_msgs::PointCloud2 contours_msg;
  pcl::toROSMsg(*contours_cloud, contours_msg);
  contours_msg.header.stamp = terrain_time_;
  contours_msg.header.frame_id = world_frame_id_;
  pub_contours_cloud.publish(contours_msg);
}

}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"OccupancyGrid");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    // osmplanner_ns::osmplanner planner(nh,nh_private);
    osmplanner_ns::OccupancyGrid gridmap(nh,nh_private);

    ros::spin();
    return 0;
}