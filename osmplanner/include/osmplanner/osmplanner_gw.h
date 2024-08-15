// 实现waypoint计算
#include <ros/ros.h>

// #include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <osmplanner/grid.h>
// #include "octomap_world/octomap_manager.h"

#include <vector>
#include <utility>
#include <eigen3/Eigen/Dense>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// using namespace Eigen;
namespace osmplanner_ns
{
// struct Params
// {
//     ros::Publisher octomap_pub;
//     ros::Publisher OccupancyGrid_pub;
//     ros::Publisher osmplanner_waypoint;
//     ros::Publisher boundaryPub_;
//     double kGainFree;
//     double kGainOccupied;
//     double kGainUnknown;

// };
struct Peak {
    double angle;
    double distance;
    size_t index;
};//道路峰值
struct Waypoint {
    // ros::Time time;
    double time;
    double x;
    double y;
    double z;
};
struct Waypoint_score{
    double time;
    double x;
    double y;
    double z;
    double score;
};
typedef Eigen::Matrix<double, 6, 1> StateVec_osmp;
class OsmPlanner
{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // ros::Subscriber odom_sub;
    ros::Subscriber cutcontours_sub;
    ros::Subscriber waypoint_sub;

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pclSub_;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
    typedef message_filters::Synchronizer<syncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    ros::Publisher Pub_modyfied_waypoint;
    ros::Publisher Pub_road_info;
    ros::Publisher waypoint_scores_publisher;
    ros::Time terrain_time_;
    // Params params;

    OccupancyGrid* grid_;
    // OsmPlanner* osmplanner_;

    StateVec_osmp robot_state;
    Waypoint last_pub_roadwaypoint;
    // std::vector<Peak> peaks;
    // std::vector<std::pair<double, double>> top_centers; //
    std::vector<Waypoint> OSM_waypoint; //osm的waypoint点   
    std::vector<Waypoint> Modify_waypoint; //最后储存发布的waypoint
    std::vector<Waypoint> Road_waypoint; //根据道路分割得到的waypoint点
    std::vector<Waypoint> OSM_waypoint_mv;

    std::vector<Waypoint_score> top_scores_waypoints;
    
    // waypoint平滑发布
    static const int QUEUE_SIZE = 5;  // 滤波器的大小
    std::deque<double> queue_x_, queue_y_, queue_z_;  // 保存历史数据的队列

    pcl::PointCloud<pcl::PointXYZI>::Ptr contours_cloud = 
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    // volumetric_mapping::OctomapManager* manager;
    std::vector<std::pair<double, double>> point_data; // 存储每个点的角度和距离

    bool init_osmp();
    bool setParams();
    // bool initialize();
    // void odomCallback(const nav_msgs::Odometry& pose);
    // void roadCloud_cb(const sensor_msgs::PointCloud2& road_point);
    // void pclCallback(const nav_msgs::Odometry& pose);
    void waypointCallback(const geometry_msgs::PointStamped& waypoint);
    void getWaypointFromRoad(const std::vector<std::pair<double, double>>& centers);
     void ContoursAndOdom(const nav_msgs::Odometry::ConstPtr& odom_msg,
                      const sensor_msgs::PointCloud2::ConstPtr& contours_msg);
    // void setRootWithOdom(const nav_msgs::Odometry& pose);
    double calculateVariance(const std::vector<std::pair<double, double>>& data, size_t start, size_t end);
    std::vector<std::pair<double, double>> findAndFilterPeaks(const std::vector<std::pair<double, double>>& point_data);
    double calculateAverageDistance(const std::vector<std::pair<double, double>>& data, size_t start_index, size_t end_index);
    // void getWaypointScore(std::vector<Waypoint> input_waypoint);
    std::vector<Waypoint> extractRecentWaypoints(const std::vector<Waypoint>& input_waypoints, double duration);
    std::vector<Waypoint_score> calculateWaypointScores(const std::vector<Waypoint>& waypoints);
    double calculateDistanceScore(const Waypoint& waypoint); 
    double calculateAngleScore(const Waypoint& waypoint, const std::vector<Waypoint>& osm_waypoints);
    double calculateOsmDistanceScore(const Waypoint& waypoint, const std::vector<Waypoint>& waypoints);
    double calculateChangeScore(const Waypoint& waypoint, const std::vector<Waypoint>& osm_waypoints);
    void publishRoadCenters(const std::vector<std::pair<double, double>>& centers);
    void visualize_waypoint_scores(const std::vector<Waypoint_score>& top_scores_waypoints);
    void publishWaypoint(const Waypoint& waypoint);
    geometry_msgs::PointStamped transformPoint(const geometry_msgs::PointStamped& point_in, const std::string& target_frame);
    void pubModifyWaypoint(const std::vector<Waypoint_score>& top_scores_waypoints, const std::vector<Waypoint>& osm_waypoint);
    OsmPlanner(const ros::NodeHandle& nh,const ros::NodeHandle& nh_private);
    ~OsmPlanner();


    std::string odomSubTopic;
    std::string pclSubTopic;
    std::string sub_contours_topic;
    std::string sub_waypoint_topic;
    std::string pub_waypoint_topic;
    std::string pub_road_info;
    std::string pub_waypoint_scores_topic;
    int road_peaks_N;
    std::string global_frame_id;

    // 道路峰值提取相关:
    double kgain_flatness;
    double kgain_interval;
    double kgain_distance;
    double threshold_ratio;
    double threshold_distance;
    double valley_threshold;

    //waypoint score得分
    double K_density_scores;
    double K_dis_OSM_scores;
    double K_distance_scores;
    double K_angle_scores;
    double K_change_scores;
    double osm_score_MAX_DISTANCE;

    tf2_ros::Buffer tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
// private:

};
}