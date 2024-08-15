// #gridmap 用于提供基本占用信息

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>


#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/surface/on_nurbs/fitting_curve_2d.h>
// #include <pcl/surface/on_nurbs/triangulation.h>
// #include <pcl-1.13/pcl/surface/on_nurbs/fitting_curve_2d.h>
// #include <pcl-1.13/pcl/surface/on_nurbs/triangulation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/median_filter.h>
// #include <pcl/filters/fast_bilateral.h>


#include <pcl/point_cloud.h>

#include <visualization_msgs/MarkerArray.h>

#include <opencv4/opencv2/opencv.hpp>


using namespace Eigen;
namespace osmplanner_ns
{
typedef Vector3d StateVec;
typedef Vector2i GridIndex;
class OccupancyGrid
{
  typedef std::shared_ptr<OccupancyGrid> Ptr;

public:
  OccupancyGrid(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~OccupancyGrid();

  bool readParameters();
  bool initialize();

public:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS subscribers
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> terrain_point_cloud_sub_;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;



  // ROS publishers
  ros::Publisher roadLinePub;
  ros::Publisher grid_cloud_pub_;
  ros::Publisher rayCast_pub_;
  ros::Publisher rayCastLinePub;
  ros::Publisher clusterPub;
  // ros::Publisher roadSplinePub;
  ros::Publisher PubCutRayCast;
  ros::Publisher pub_contours_cloud;  

  // String constants
  std::string world_frame_id_;
  std::string sub_odom_topic_;
  std::string sub_terrain_point_cloud_topic_;
  std::string pub_grid_points_topic_;
  std::string pub_rayCast_points_topic;
  std::string pub_rayCast_line_topic;
  std::string pub_road_line_topic;
  // std::string pub_road_curve_topic;
  std::string pub_cluster_topic;
  std::string pub_Cut_RayCast_topic;
  std::string pub_contours_topic;


  std::string PCD_save_Path_1;
  std::string PCD_save_Path_2;
  std::string IMG_save_Path;
  bool img_saveorNot;
  bool PCD_SaveOrNot;
  int frame_counter = 0;


  int filterType; //剔除raycasting中的离群点，0表示不滤除，1表示使用SOR,2表示使用ROR
  double Meank;
  double stddev;
  double radiusSearch;
  double Min_neighbors;


  // Constants
  double kMapWidth;
  double kGridSize;
  double kDownsampleSize;
  double kObstacleHeightThre;
  double kFlyingObstacleHeightThre;
  double kCollisionCheckX;
  double kCollisionCheckY;
  double rayCast_Cutdegree;


  double rayCast_near_range;
  

  double k_max_radius;

  //DBSCAN
  double CorePointMinPts;
  double ClusterTolerance;
  double MinClusterSize;
  double MaxClusterSize;

  // Variables
  enum gridStatus
  {
    unknown = 0,
    free = 1,
    occupied = 2,
    near_occupied = 3
  };
  std::vector<std::vector<int>> gridState_;
  int map_width_grid_num_;
  int map_half_width_grid_num_;

  ros::Time terrain_time_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_ds =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_traversable_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_obstacle_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr grid_cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  // pcl::PointCloud<pcl::PointXYZI>::Ptr grid_cloud_filter =
  //     pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::PointCloud<pcl::PointXYZI>::Ptr rayCast_cloud = 
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::PointCloud<pcl::PointXYZI>::Ptr filter_rayCast_cloud = 
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::PointCloud<pcl::PointXYZI>::Ptr cut_cloud  = 
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::PointCloud<pcl::PointXYZI>::Ptr contours_cloud  = 
    pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  StateVec robot_position_;

  // Functions
  geometry_msgs::Point getPoint(GridIndex p);
  GridIndex getIndex(StateVec point);
  void clearGrid();
  void updateGrid();
  void publishGridMap();
  bool collisionCheckByTerrainWithVector(StateVec origin_point, StateVec goal_point);
  bool collisionCheckByTerrain(geometry_msgs::Point origin, geometry_msgs::Point goal);
  bool InRange(const GridIndex sub, const GridIndex max_sub, const GridIndex min_sub);
  bool updateFreeGridWithSurroundingGrids(int indx, int indy);
  int signum(int x);
  double intbound(double s, double ds);
  double mod(double value, double modulus);
  std::vector<GridIndex> rayCast(GridIndex origin, GridIndex goal, GridIndex max_grid, GridIndex min_grid);
  std::vector<GridIndex> rayCast(GridIndex origin, GridIndex max_grid, GridIndex min_grid);
  std::vector<GridIndex> rayCastFree(GridIndex origin, GridIndex max_grid, GridIndex min_grid);
  void extractRayCastPoint(StateVec origin_point);
  std::vector<pcl::ModelCoefficients::Ptr> Road_ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr rayCast_loud);
  void visualizeLines(std::vector<pcl::ModelCoefficients::Ptr> lines);
  // void visualizeCurvs(std::vector<ON_NurbsCurve> lines);
  // std::vector<ON_NurbsCurve> Road_bsplines(pcl::PointCloud<pcl::PointXYZI>::Ptr rayCast_loud);

  // Callback Functions
  void terrainCloudAndOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                   const sensor_msgs::PointCloud2::ConstPtr& terrain_msg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cutPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud,
                                                   double robot_x, double robot_y, double robot_yaw);
  double normalizeAngle(double angle);
  void visualizeRayCastPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);   
  // cv::Mat pointCloudToImage(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void convertGridToImage();
  void processGridMap();
  double calculateAngle(cv::Point pt1, cv::Point pt2, cv::Point pt3);
  std::vector<cv::Point> filterHull(const std::vector<cv::Point>& hull);
  // pcl::PointCloud<pcl::PointXYZI>::Ptr applyBilateralFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr medianFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  // void savePCD_TxT(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,double robot_x, double robot_y);       
  void savePCD_TxT(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, std::string filename);        
};
}

