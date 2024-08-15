#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>
// #include <osmplanner/osmplanner_gw.h>
#include </home/gw/exploration/nav/nav_test_ws2/src/osmplanner/include/osmplanner/osmplanner_gw.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>

// TODO 1.waypoint发布频率
// 2.调节参数//
// 3.更好的策略mod-waypoint//
// 4.峰值为1的情况如何处理
namespace osmplanner_ns
{
  OsmPlanner::OsmPlanner(const ros::NodeHandle& nh,const ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private)
  {
    // grid_ = new OccupancyGrid(nh_, nh_private_);
    init_osmp();
  }
  OsmPlanner::~OsmPlanner()
  {
    // if (grid_)
    // {
    //   delete grid_;
    //   // grid_ = nullptr;
    // }
  }

  bool OsmPlanner::init_osmp()
  {
    if(!setParams()){
      return false;
    }
    waypoint_sub = nh_.subscribe(sub_waypoint_topic,10,&OsmPlanner::waypointCallback,this);

    odom_sub_.subscribe(nh_,odomSubTopic,1);
    pclSub_.subscribe(nh_,pclSubTopic,1);

    // sync_.reset(new Sync(syncPolicy(100), odom_sub_, terrain_point_cloud_sub_));
    // sync_->registerCallback(boost::bind(&OccupancyGrid::terrainCloudAndOdomCallback, this, _1, _2));  
    sync_.reset(new Sync(syncPolicy(10), odom_sub_, pclSub_));
    sync_->registerCallback(boost::bind(&OsmPlanner::ContoursAndOdom, this, _1, _2));  

    Pub_modyfied_waypoint = nh_.advertise<geometry_msgs::PointStamped>(pub_waypoint_topic,1);
    Pub_road_info = nh_.advertise<geometry_msgs::PoseArray>(pub_road_info,1);
    waypoint_scores_publisher = nh_.advertise<geometry_msgs::PoseArray>(pub_waypoint_scores_topic,1);
    tfListener = std::make_unique<tf2_ros::TransformListener>(tfBuffer);
    ROS_INFO("OSMplanner successfully launched");

    return true;
  }

  bool OsmPlanner::setParams()
  {
    nh_private_.getParam("/osmplanner/odomSubTopic",odomSubTopic);
    nh_private_.getParam("/osmplanner/pclSubTopic",pclSubTopic);
    // // nh_private_.getParam("/osmplanner/sub_contours_topic",sub_contours_topic);
    nh_private_.getParam("/osmplanner/sub_waypoint_topic",sub_waypoint_topic);
    nh_private_.getParam("/osmplanner/pub_waypoint_topic",pub_waypoint_topic);
    nh_private_.getParam("/osmplanner/pub_road_info",pub_road_info);
    nh_private_.getParam("/osmplanner/pub_waypoint_scores_topic",pub_waypoint_scores_topic);
    nh_private_.getParam("/osmplanner/road_peaks_N",road_peaks_N);
    nh_private_.getParam("/osmplanner/global_frame_id",global_frame_id);
    nh_private_.getParam("/osmplanner/kgain_interval",kgain_interval);
    nh_private_.getParam("/osmplanner/kgain_flatness",kgain_flatness);
    nh_private_.getParam("/osmplanner/kgain_distance",kgain_distance);
    nh_private_.getParam("/osmplanner/road_peak/threshold_ratio",threshold_ratio);
    nh_private_.getParam("/osmplanner/road_peak/threshold_distance",threshold_distance);
    nh_private_.getParam("/osmplanner/road_peak/valley_threshold",valley_threshold);
    nh_private_.getParam("/osmplanner/waypoint_scores/K_dis_OSM_scores",K_dis_OSM_scores);
    nh_private_.getParam("/osmplanner/waypoint_scores/K_distance_scores",K_distance_scores);
    nh_private_.getParam("/osmplanner/waypoint_scores/K_angle_scores",K_angle_scores);
    nh_private_.getParam("/osmplanner/modify_waypoint/osm_score_MAX_DISTANCE",osm_score_MAX_DISTANCE);
    nh_private_.getParam("/osmplanner/waypoint_scores/K_change_scores",K_change_scores);
    // nh_private.getParam("/osmplanner/");
    // nh_private.getParam("/osmplanner/");
    return true;
  }

  void OsmPlanner::ContoursAndOdom(const nav_msgs::Odometry::ConstPtr& odom_msg,
                      const sensor_msgs::PointCloud2::ConstPtr& contours_msg)
  {
    terrain_time_ = contours_msg->header.stamp;
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    robot_state[0] = odom_msg->pose.pose.position.x;
    robot_state[1] = odom_msg->pose.pose.position.y;
    robot_state[2] = odom_msg->pose.pose.position.z;
    robot_state[3] = roll;
    robot_state[4] = pitch;
    robot_state[5] = yaw;

    contours_cloud->clear();
    pcl::fromROSMsg(*contours_msg, *contours_cloud);
    point_data.clear();
    top_scores_waypoints.clear();
    for (const auto& point : contours_cloud->points)
    {
      double dx = point.x - robot_state[0];
      double dy = point.y - robot_state[1];
      double point_angle = atan2(dy, dx);
      double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
      point_data.push_back(std::make_pair(point_angle, distance));
    }
    publishRoadCenters(point_data);
    getWaypointFromRoad(point_data);
    std::vector<Waypoint> recent_waypoints = extractRecentWaypoints(Road_waypoint, 3);
    std::vector<Waypoint_score> waypoint_scores = calculateWaypointScores(recent_waypoints);
    size_t num_elements_to_copy = std::min(waypoint_scores.size(), size_t(1));
    for (size_t i = 0; i < num_elements_to_copy; ++i) {
        top_scores_waypoints.push_back(waypoint_scores[i]);
    }
    ROS_INFO("top_scores_waypoints have %zu points",top_scores_waypoints.size());
    if (OSM_waypoint.empty() || top_scores_waypoints.empty()) {
        ROS_WARN("Waypoints are empty");
        return;
    }
    else{
      visualize_waypoint_scores(top_scores_waypoints);
      pubModifyWaypoint(top_scores_waypoints,OSM_waypoint);
    }

  }

  void OsmPlanner::waypointCallback(const geometry_msgs::PointStamped& waypoint_msg)
  {
    geometry_msgs::PointStamped waypoint_in_map = transformPoint(waypoint_msg, global_frame_id);
    Waypoint waypoint_;
    waypoint_.time = waypoint_in_map.header.stamp.toSec();
    waypoint_.x = waypoint_in_map.point.x;
    waypoint_.y = waypoint_in_map.point.y;
    waypoint_.z = waypoint_in_map.point.z;

    OSM_waypoint.push_back(waypoint_);
    double distance_threshold = 1.0; 

    if (OSM_waypoint_mv.empty() ||
        std::hypot(waypoint_.x - OSM_waypoint_mv.back().x, waypoint_.y - OSM_waypoint_mv.back().y) > distance_threshold)
    {
      OSM_waypoint_mv.push_back(waypoint_);
    }

    ROS_INFO("osm_waypoint receving!!!! and have size: %zu", OSM_waypoint.size());
    ROS_INFO("osm_waypoint_mv receving!!!! and have size: %zu", OSM_waypoint_mv.size()); 
  }

  void OsmPlanner::getWaypointFromRoad(const std::vector<std::pair<double, double>>& centers)
  {
    for (const auto& center : centers) {
        Waypoint road_waypoint;
        road_waypoint.time = terrain_time_.toSec();
        double angle = center.first; 
        double distance = center.second;  
        road_waypoint.x = distance * cos(angle)+robot_state[0];
        road_waypoint.y = distance * sin(angle)+robot_state[1];
        road_waypoint.z = angle;
        Road_waypoint.push_back(road_waypoint);
    }
  }

  std::vector<Waypoint> OsmPlanner::extractRecentWaypoints(const std::vector<Waypoint>& input_waypoints, double duration) 
  {
    std::vector<Waypoint> recent_waypoints;
    for (const auto& waypoint : input_waypoints) {
        double time_diff = terrain_time_.toSec() - waypoint.time;
        if (time_diff < duration) {
            recent_waypoints.push_back(waypoint);
        }
    }
    ROS_INFO("recent_waypoints have %zu points",recent_waypoints.size());
    return recent_waypoints;
  }

  std::vector<Waypoint_score> OsmPlanner::calculateWaypointScores(const std::vector<Waypoint>& waypoints) 
  {
    std::vector<double> dis_OSM_scores;
    std::vector<double> distance_scores;
    std::vector<double> angle_scores;
    std::vector<double> change_scores;
    for (const auto& waypoint : waypoints) {
        double dis_OSM_score = calculateOsmDistanceScore(waypoint, OSM_waypoint);
        double distance_score = calculateDistanceScore(waypoint);
        double angle_score = calculateAngleScore(waypoint, OSM_waypoint);
        double change_score = calculateChangeScore(waypoint, OSM_waypoint_mv);

        ROS_INFO("Change Score: %f",change_score);

        dis_OSM_scores.push_back(dis_OSM_score);
        distance_scores.push_back(distance_score);
        angle_scores.push_back(angle_score);
        change_scores.push_back(change_score);
    }
    auto normalize = [](std::vector<double>& scores) {
        double min_score = *std::min_element(scores.begin(), scores.end());
        double max_score = *std::max_element(scores.begin(), scores.end());
        for (double& score : scores) {
            score = (score - min_score) / (max_score - min_score);
        }
    };
    normalize(dis_OSM_scores);
    normalize(distance_scores);
    normalize(angle_scores);
    normalize(change_scores);

    // 计算总得分并存储
    std::vector<Waypoint_score> waypoint_scores;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        double total_score = K_dis_OSM_scores * dis_OSM_scores[i] + K_distance_scores * distance_scores[i] + K_angle_scores * angle_scores[i]
         + K_change_scores*change_scores[i];
        Waypoint_score waypoint_score;
        waypoint_score.time = waypoints[i].time;
        waypoint_score.x = waypoints[i].x;
        waypoint_score.y = waypoints[i].y;
        waypoint_score.z = waypoints[i].z;
        waypoint_score.score = total_score;
        waypoint_scores.push_back(waypoint_score);
    }

    // 按得分降序排序
    std::sort(waypoint_scores.begin(), waypoint_scores.end(), [](const Waypoint_score& a, const Waypoint_score& b) {
        return a.score > b.score;
    });

    return waypoint_scores;
  }

  double OsmPlanner::calculateDistanceScore(const Waypoint& waypoint) 
  {
    double distance = std::sqrt(std::pow(waypoint.x-robot_state[0], 2) + std::pow(waypoint.y-robot_state[1], 2));
    return distance; 
  }

  double OsmPlanner::calculateOsmDistanceScore(const Waypoint& waypoint, const std::vector<Waypoint>& osm_waypoints) 
  {
      if (osm_waypoints.empty()) 
      {
          return 0.0;
      }
      const Waypoint& osm_waypoint = osm_waypoints.back();
      double distance = std::sqrt(std::pow(waypoint.x-osm_waypoint.x, 2) + std::pow(waypoint.y-osm_waypoint.y, 2));
      return 1.0 / (1.0 + distance);
  }


  double OsmPlanner::calculateAngleScore(const Waypoint& waypoint, const std::vector<Waypoint>& osm_waypoints) 
  {
      if (osm_waypoints.empty()) 
      {
          return 0.0;
      }
      const Waypoint& osm_waypoint = osm_waypoints.back();
      double dot_product = (waypoint.x - robot_state[0]) * (osm_waypoint.x - robot_state[0]) 
                      + (waypoint.y - robot_state[1]) * (osm_waypoint.y - robot_state[1]);
      double length1 = std::sqrt(std::pow(waypoint.x - robot_state[0], 2) + std::pow(waypoint.y - robot_state[1], 2));
      double length2 = std::sqrt(std::pow(osm_waypoint.x - robot_state[0], 2) + std::pow(osm_waypoint.y - robot_state[1], 2));
      double cos_angle = dot_product / (length1 * length2);

      double angle = std::acos(cos_angle);
      return 1.0 / (1.0 + angle);
  }

  double OsmPlanner::calculateChangeScore(const Waypoint& waypoint, const std::vector<Waypoint>& osm_waypoints)
  {
      if (osm_waypoints.size() < 2) 
      {
          return 0.0;
      }
      double dx1 = waypoint.x -last_pub_roadwaypoint.x;
      double dy1 = waypoint.y -last_pub_roadwaypoint.y;

      double dx2 = osm_waypoints[osm_waypoints.size() - 1].x - osm_waypoints[osm_waypoints.size() - 2].x;
      double dy2 = osm_waypoints[osm_waypoints.size() - 1].y - osm_waypoints[osm_waypoints.size() - 2].y;

      ROS_INFO("dx1 dy1 dx2 dy2 : %f,%f,%f,%f",dx1,dy1,dx2,dy2);

      double dot_product = dx1 * dx2 + dy1 * dy2;
      double length1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
      double length2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
      double cos_angle = dot_product / (length1 * length2);
      double angle = std::acos(cos_angle);

      double angle_in_degrees = angle * 180.0 / M_PI;

      return 1.0 / (1.0 + angle);
  }

  void OsmPlanner::pubModifyWaypoint(const std::vector<Waypoint_score>& top_scores_waypoints, const std::vector<Waypoint>& osm_waypoint) 
  {
    const Waypoint& latest_osm_waypoint = osm_waypoint.back();
    if (top_scores_waypoints.size()<1) {
        ROS_WARN("Score Waypoints are empty, publishing OSM waypoint instead.");
        publishWaypoint(latest_osm_waypoint);
        return;
    }

    const Waypoint_score& highest_score_waypoint = *std::max_element(top_scores_waypoints.begin(), top_scores_waypoints.end(), 
    [](const Waypoint_score& a, const Waypoint_score& b) {
        return a.score < b.score;
    });

    double dist = std::sqrt(std::pow(latest_osm_waypoint.x - highest_score_waypoint.x, 2) + std::pow(latest_osm_waypoint.y - highest_score_waypoint.y, 2));
    if (dist > osm_score_MAX_DISTANCE) {
        ROS_WARN("Distance between OSM waypoint and Score waypoint too large, publishing OSM waypoint instead.");
        publishWaypoint(latest_osm_waypoint);
        return;
    }
    Waypoint midpoint;
    midpoint.x =  highest_score_waypoint.x;
    midpoint.y =  highest_score_waypoint.y;
    midpoint.z = robot_state[2];
    publishWaypoint(midpoint);
    last_pub_roadwaypoint = midpoint;

    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!publish road waypoint: %f,%f,%f",midpoint.x,midpoint.y,midpoint.z);
  }

  void OsmPlanner::publishWaypoint(const Waypoint& waypoint) 
  {
    queue_x_.push_back(waypoint.x);
    queue_y_.push_back(waypoint.y);
    queue_z_.push_back(waypoint.z);
    if (queue_x_.size() > QUEUE_SIZE) 
    {
        queue_x_.pop_front();
        queue_y_.pop_front();
        queue_z_.pop_front();
    }

    double avg_x = std::accumulate(queue_x_.begin(), queue_x_.end(), 0.0) / queue_x_.size();
    double avg_y = std::accumulate(queue_y_.begin(), queue_y_.end(), 0.0) / queue_y_.size();
    double avg_z = robot_state[2];

    geometry_msgs::PointStamped waypoint_msg;
    waypoint_msg.header.stamp = terrain_time_;
    waypoint_msg.header.frame_id = global_frame_id; 
    waypoint_msg.point.x = avg_x;
    waypoint_msg.point.y = avg_y;
    waypoint_msg.point.z = robot_state[2];
    Pub_modyfied_waypoint.publish(waypoint_msg);
  }

  void OsmPlanner::publishRoadCenters(const std::vector<std::pair<double, double>>& centers) 
  {
      
      geometry_msgs::PoseArray pose_array_msg;

      pose_array_msg.header.stamp = terrain_time_;
      pose_array_msg.header.frame_id = global_frame_id; 

      for (const auto& center : centers) 
      {
          geometry_msgs::Pose pose_msg;
          double angle = center.first; 
          double distance = center.second;
          pose_msg.position.x = distance * cos(angle)+robot_state[0];
          pose_msg.position.y = distance * sin(angle)+robot_state[1];
          pose_msg.position.z = 0;

          tf::Quaternion q = tf::createQuaternionFromYaw(angle);
          pose_msg.orientation.x = q.x();
          pose_msg.orientation.y = q.y();
          pose_msg.orientation.z = q.z();
          pose_msg.orientation.w = q.w();
          pose_array_msg.poses.push_back(pose_msg);
      }

      Pub_road_info.publish(pose_array_msg);
  }

  void OsmPlanner::visualize_waypoint_scores(const std::vector<Waypoint_score>& top_scores_waypoints) 
  {
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = terrain_time_;
    pose_array.header.frame_id = global_frame_id;  
    for (const auto& waypoint_score : top_scores_waypoints) {
        geometry_msgs::Pose pose;
        pose.position.x = waypoint_score.x;
        pose.position.y = waypoint_score.y;
        pose.position.z = 0;
        tf::Quaternion q = tf::createQuaternionFromYaw(waypoint_score.z);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose_array.poses.push_back(pose);
    }
    waypoint_scores_publisher.publish(pose_array);
  }

  geometry_msgs::PointStamped OsmPlanner::transformPoint(const geometry_msgs::PointStamped& point_in, const std::string& target_frame)
  {
    geometry_msgs::PointStamped point_out;
    try
    {
      geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(target_frame, point_in.header.frame_id, ros::Time(0), ros::Duration(1.0));
      tf2::doTransform(point_in, point_out, transform);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
    }

    return point_out;
  }

}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"osmplanner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    osmplanner_ns::OsmPlanner planner(nh,nh_private);
    ROS_INFO("success!!!");
    ros::spin();
    return 0;
}



