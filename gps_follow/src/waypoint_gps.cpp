// sys
#include <queue>
#include <deque>
#include <mutex>
#include <fstream>

// ros
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <boost/bind.hpp>
#include <visualization_msgs/MarkerArray.h>

// eigen
#include <eigen3/Eigen/Eigen>
#include <functional>
#include <deque>

#include "gpsTools.hpp"
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>

#include "gps_follow/gpsTomap.h"

typedef struct {
    double timeSec;
    Eigen::Vector2d pos;
} OdomData;

typedef struct {
    double timeSec;
    Eigen::Vector2d pos;
    Eigen::Vector2d cov;
} GpsData;

typedef struct {
    double timeSec;
    OdomData odom;
    GpsData gps;
} FrameData;

using namespace std;

deque<ros::Time> time_buffer;

class GNSSOdom {
    public:
        ros::NodeHandle nh;
        ros::ServiceServer gps_map_convert_server;
        GpsTools gpsTools;

        ros::Publisher gpsOdomPub, gpsPathPub, waypointPub, osmPathPub,osmPathGraphPub,lioPub,refLioPathPub;
        ros::Subscriber gpsSub, lidarOdomSub , active_loop_path_sub;
        std::deque<sensor_msgs::NavSatFixConstPtr> gpsBuf;
        std::vector<Eigen::Vector3d> osmQueue;
        std::vector<Eigen::Vector3d> osmGraph; // 用于存储插值前的osm对应构建laplacian
        std::vector<Eigen::Vector3d> ecefQueue_graph;
        std::vector<Eigen::Vector3d> enuQueue_graph;

        std::vector<Eigen::Vector3d> ecefQueue;
        std::vector<Eigen::Vector3d> enuQueue;
        std::vector<Eigen::Vector3d> enuQueue_tran; //坐标转化之后的


        int queueIndex = 0;
        int currentIndex = 0;
        Eigen::Vector3d transENU;
        Eigen::Vector3d nowENU;

        bool initXYZ_ = false;
        bool initLidarXYZ_=false;
        bool initHeading_ = false;
        bool tf_calculated_ = false;

        //初始化相关
        bool inittf = false;
        bool initOSM = false;
        bool tfinfosend =false;
        double lidarStartTime=0,gpsStartTime=0;
        Eigen::Vector3d FirstLidarP;
        Eigen::Vector3d FirstGpsP;

        Eigen::Vector3d currentPose;

        std::vector<OdomData> odom_data;
        std::vector<GpsData> gps_data;
        std::vector<FrameData> frame_data;
        Eigen::Matrix3d twoDenu_oodom;

        Eigen::Isometry2d  T_enu_odom;
        Eigen::Vector2d translation_offset;
        double rotation_offset_angle; 
        double x_offset,y_offset;


        string gpsDirectory;         // gps
        string odomDirectory;         // odom from fastlio
        string saveDirectory;
        geometry_msgs::TransformStamped transformStamped;
        double slope_gps,slope_lio,intercept_gps,intercept_lio;
        double curYaw = 0, preYaw = 0;
        geometry_msgs::Quaternion yawQuat;
        nav_msgs::Path fusedPath,lioPath;
        nav_msgs::Path refPath, osmPathGraph,reflioPath;
        visualization_msgs::MarkerArray osmMarker;

        bool initlioPath =false;

        bool DealnewPath = false; //表示是否在处理主动回环新路径

        double max_osm_dis;
        double current_index_incl;
        bool save_tum;
        std::string tum_save_path;
        std::string osm_traj_path;
        std::string osm_graph_path;
        double initial_time;

        GNSSOdom() : nh("~") 
        {

            nh.getParam("max_osm_dis",max_osm_dis);
            nh.getParam("current_index_incl",current_index_incl);
            nh.getParam("save_tum",save_tum);
            nh.getParam("tum_save_path",tum_save_path);
            nh.getParam("osm_traj_path",osm_traj_path);
            nh.getParam("osm_graph_path",osm_graph_path);
            nh.getParam("initial_time",initial_time);

            nh.getParam("x_offset", x_offset);
            nh.getParam("y_offset", y_offset);
            nh.getParam("rotation_offset_angle", rotation_offset_angle);
            // nh.getParam();
            // nh.getParam();
            // nh.getParam();
            
            gpsSub = nh.subscribe("/gnss", 100, &GNSSOdom::GNSSCB, this, ros::TransportHints().tcpNoDelay());
            lidarOdomSub = nh.subscribe("/fusedOdom", 100, &GNSSOdom::lidarOdomCallback, this, ros::TransportHints().tcpNoDelay());
            active_loop_path_sub =nh.subscribe("/active_loop_path", 10, &GNSSOdom::newPathCallback, this, ros::TransportHints().tcpNoDelay());

            lioPub = nh.advertise<nav_msgs::Path>("/fast_lio_path_waypoint",100);
            gpsOdomPub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 100, false);
            gpsPathPub = nh.advertise<nav_msgs::Path>("/gps_path", 100);
            waypointPub = nh.advertise<geometry_msgs::PointStamped>("/way_point_osm", 10);
            osmPathPub = nh.advertise<nav_msgs::Path>("/ref_osm_path", 100);  //插值后用来循迹的osmPath
            osmPathGraphPub =  nh.advertise<nav_msgs::Path>("/osm_Path_Graph", 100); 
            refLioPathPub = nh.advertise<nav_msgs::Path>("/ref_lio_path", 100);

            gps_map_convert_server =  nh.advertiseService("gps_map_convert_server",&GNSSOdom::gps_map_convert,this);
            std::cout<<"max_osm_dis:"<<max_osm_dis <<std::endl;
            std::ifstream OSM_gpsframe(osm_traj_path);
            std::ifstream OSM_Graph(osm_graph_path);
            tf_calculated_ = false;
            T_enu_odom = Eigen::Isometry2d::Identity();
            // 初始化平移偏差向量
            translation_offset = Eigen::Vector2d(x_offset, y_offset);
            Eigen::Vector3d osm;
            Eigen::Vector3d fastlio;
            if(OSM_gpsframe){
                ROS_INFO("OSM_gpsframe reading");
            }
            while(!OSM_gpsframe.eof()) 
            {
                OSM_gpsframe >> osm[0] >> osm[1] >> osm[2];
                osmQueue.push_back(osm);
            }
            while(!OSM_Graph.eof())  
            {
                OSM_Graph >> osm[0] >> osm[1] >> osm[2];
                osmGraph.push_back(osm);
            }
            std::cout << "osmGraph size :" << osmGraph.size() <<std::endl;
        }
        void newPathCallback(const visualization_msgs::Marker::ConstPtr& marker_msg) 
        {
            DealnewPath = true;
            // 提取 Marker 的最后一个点作为目标节点坐标
            const geometry_msgs::Point& target_point_msg = marker_msg->points.back();
            Eigen::Vector3d target_node_point(target_point_msg.x, target_point_msg.y, target_point_msg.z);

            std::vector<Eigen::Vector3d> path_points;
            path_points.reserve(marker_msg->points.size() - 1);

            // 除最后一个点外，将geometry_msgs::Point类型的点转换为Eigen::Vector3d类型
            for (auto it = marker_msg->points.begin(); it != marker_msg->points.end() - 1; ++it) {
                path_points.emplace_back(it->x, it->y, it->z);
            }
            double LC_threshold = 20;  // 设置一个合适的距离阈值
            double distance = sqrt(pow(target_node_point.x() - currentPose.x(), 2) + pow(target_node_point.y() - currentPose.y(), 2));    

            if(distance < LC_threshold)
            {
                ROS_INFO("The start of the new path is close to the current pose. No need for updates.");
                DealnewPath = false;
            }
            else
            {
                
                ROS_INFO("================Follow New Path======================");
                std::vector<Eigen::Vector3d> point_in_mapframe;
                std::vector<Eigen::Vector3d> transformed_points; 
                std::cout<< "before Size of enuQueue" << enuQueue.size() <<std::endl;
                enuQueue.clear();
                
                // for (const auto& p : path_points) {
                //         point_in_mapframe.push_back(Eigen::Vector3d(p.x, p.y, p.z));
                //     }
                for (const auto& p : path_points) {
                    point_in_mapframe.push_back(Eigen::Vector3d(p[0], p[1], p[2]));
                }
                                
                // // 打印出接收到的点
                // for (const auto& point : point_in_mapframe) {
                //     ROS_INFO("Point: x=%f, y=%f, z=%f", point.x(), point.y(), point.z());
                // }
                
                // Eigen::Matrix3d twoDenu_oodom;
                // Eigen::Matrix3d mat_enu_odom = T_enu_odom.matrix().block<3, 3>(0, 0);
                // twoDenu_oodom << mat_enu_odom(0, 0), mat_enu_odom(0, 1), 0,
                //                 mat_enu_odom(1, 0), mat_enu_odom(1, 1), 0,
                //                 0, 0, 1;

                // for (const auto& point : point_in_mapframe) {
                //     Eigen::Vector3d map_in_gps_frame = twoDenu_oodom * point;
                //     Eigen::Vector3d map_trans_enu = Eigen::Vector3d(map_in_gps_frame[0], map_in_gps_frame[1], map_in_gps_frame[2]);
                //     transformed_points.push_back(map_trans_enu);
                // }

                for (const auto& p : path_points) {
                    Eigen::Vector3d point_in_homogeneous(p[0], p[1], 1.0);
                    Eigen::Vector3d transformed_point = T_enu_odom * point_in_homogeneous;
                    transformed_points.push_back(Eigen::Vector3d(transformed_point[0], transformed_point[1], p[2]));
                }

                // Interpolate points
                std::vector<Eigen::Vector3d> interpolated_points;
                const size_t numPointsToInsert = 50; 

                for (size_t i = 0; i < transformed_points.size() - 1; ++i) {
                    interpolated_points.push_back(transformed_points[i]);
                    Eigen::Vector3d step = (transformed_points[i + 1] - transformed_points[i]) / (numPointsToInsert + 1);
                    for (size_t j = 1; j <= numPointsToInsert; ++j) {
                        Eigen::Vector3d interpolated_point = transformed_points[i] + step * static_cast<double>(j);
                        interpolated_points.push_back(interpolated_point);
                    }
                }
                interpolated_points.push_back(transformed_points.back());
                enuQueue = interpolated_points;
        
                std::cout << "Size of enuQueue after interpolation: " << enuQueue.size() << std::endl;

                currentIndex=0;

                Eigen::Vector2d slam_pose_in_enu = T_enu_odom * Eigen::Vector2d(currentPose[0], currentPose[1]);
                std::cout<< "slam_pose_in_enu" << slam_pose_in_enu[0] <<"," << slam_pose_in_enu[1] <<std::endl;
                double min = 999;
                    
                for(int i = currentIndex ; i <=200 ; i++) 
                {
                    double dis = sqrt(pow(slam_pose_in_enu(0) - enuQueue[i](0), 2) + pow(slam_pose_in_enu(1) - enuQueue[i](1), 2));
                    if (min > dis) 
                    {
                        min = dis;
                        currentIndex = i; 
                    }
                }

                std::cout<< "current INdex of enuQueue" << currentIndex <<std::endl;
                DealnewPath = false;
                }
        }

        bool gps_map_convert(gps_follow::gpsTomap::Request &req, gps_follow::gpsTomap::Response &res) 
        {
            // if(initOSM)
            // {
                if (req.conversion_direction == "gps_to_map") {
                    Eigen::Vector3d lla = gpsTools.GpsMsg2Eigen(req.input_gps_point);
                    Eigen::Vector3d ecef = gpsTools.LLA2ECEF(lla);
                    Eigen::Vector3d enu = gpsTools.ECEF2ENU(ecef);
                    Eigen::Vector2d gps_in_map = T_enu_odom.inverse() * Eigen::Vector2d(enu(0), enu(1));
                    res.output_map_point.header.frame_id = "map"; // 转换后的地图坐标
                    res.output_map_point.header.stamp = req.input_gps_point.header.stamp;
                    res.output_map_point.point.x = gps_in_map[0];
                    res.output_map_point.point.y = gps_in_map[1];
                    res.output_map_point.point.z = enu[2];
                    res.success = true;
                    res.message = "GPS to Map conversion successful";
                } else if (req.conversion_direction == "map_to_gps") {
                    Eigen::Matrix3d mat_enu_odom =  T_enu_odom.matrix();
                    twoDenu_oodom << mat_enu_odom(0,0),mat_enu_odom(0,1),0,
                        mat_enu_odom(1,0),mat_enu_odom(1,1),0,
                        0,0,1;

                    Eigen::Vector3d mapPoint;
                    Eigen::Vector3d map_inGPSframe;
                    mapPoint[0] = req.input_map_point.point.x;
                    mapPoint[1] = req.input_map_point.point.y;
                    mapPoint[2] = req.input_map_point.point.z;
                    map_inGPSframe = twoDenu_oodom*mapPoint;

                    Eigen::Vector3d map_trannsENU = Eigen::Vector3d(-map_inGPSframe[0],-map_inGPSframe[1],-map_inGPSframe[2]);

                    Eigen::Vector3d ecef1 = gpsTools.ENU2ECEF(map_trannsENU);
                    Eigen::Vector3d lla1 = gpsTools.ECEF2LLA(ecef1);

                    res.output_gps_point.header.frame_id = "LLA"; // 转换后的GPS坐标
                    res.output_gps_point.header.stamp = req.input_map_point.header.stamp;
                    res.output_gps_point.latitude = lla1[0];
                    res.output_gps_point.longitude = lla1[1];
                    res.output_gps_point.altitude = lla1[2];

                    res.success = true;
                    res.message = "Map to GPS conversion successful";
                } else {
                    res.success = false;
                    res.message = "Unknown conversion direction requested";
                }

                return true;
            // }
            // else{
            //     ROS_WARN("osm not init,Please waiting");

            //     return false;
            // }

        }

        void GNSSCB(const sensor_msgs::NavSatFixConstPtr &msg) 
        {
            if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
                ROS_ERROR("POS LLA NAN...");
                return;
            }

            if(!inittf)
            {
                if(!initXYZ_){
                ROS_INFO("Init Origin GPS LLA %f, %f, %f", msg->latitude, msg->longitude, msg->altitude);
                gpsTools.lla_origin_ << msg->latitude, msg->longitude, msg->altitude;
                gpsStartTime = msg->header.stamp.toSec();
                initXYZ_ = true;
                ROS_ERROR("waiting init origin axis,first GPS frame");

                }
                
                Eigen::Vector3d lla = gpsTools.GpsMsg2Eigen(*msg);
                Eigen::Vector3d ecef = gpsTools.LLA2ECEF(lla);
                Eigen::Vector3d enu = gpsTools.ECEF2ENU(ecef);
                ROS_INFO("\033[1;32m----> waiting tf.\033[0m");

                GpsData gps_point;
                gps_point.timeSec = msg->header.stamp.toSec();
                gps_point.pos.x() = enu(0);
                gps_point.pos.y() = enu(1);
                gps_point.cov.x() = msg->position_covariance[0];
                gps_point.cov.y() = msg->position_covariance[4];
                gps_data.push_back(gps_point);

                ROS_INFO("gps_trajectory XYZ: %f,%f,%f",enu(0),enu(1),enu(2));
                if(gps_point.timeSec - gpsStartTime >initial_time)
                {
                    ROS_INFO("\033[1;32m----> GPS init success.\033[0m");
                    inittf =true;
                }
                return;

            }
            if (!tfinfosend) return;
            if (!initOSM ) 
            {
                Eigen::Vector3d FirstTranENU;
                FirstTranENU(0)=-FirstGpsP(0);
                FirstTranENU(1)=-FirstGpsP(1);
                FirstTranENU(2)=-FirstGpsP(2); 
                
                Eigen::Vector3d ecef1 = gpsTools.ENU2ECEF(FirstTranENU);
                Eigen::Vector3d lla1 = gpsTools.ECEF2LLA(ecef1);
                gpsTools.lla_origin_ << lla1(0),lla1(1),lla1(2);
                ROS_INFO("first gps point x,y:%f,%f",FirstGpsP(0),FirstGpsP(1));

                for(int i = 0; i < osmQueue.size(); i++) 
                {
                    Eigen::Vector3d lla(osmQueue[i][0], osmQueue[i][1], osmQueue[i][2]);
                    Eigen::Vector3d ecef = gpsTools.LLA2ECEF(lla);
                    Eigen::Vector3d enu = gpsTools.ECEF2ENU(ecef);
                    ecefQueue.push_back(ecef);
                    enuQueue.push_back(enu);

                    
                }
                refPath.header.frame_id = "gps";
     
                refPath.header.stamp = msg->header.stamp;
                for (int i = 0; i < enuQueue.size(); i++) 
                {
                    geometry_msgs::PoseStamped pose;
                    pose.header = refPath.header;
                    pose.pose.position.x = enuQueue[i](0);
                    pose.pose.position.y = enuQueue[i](1);
                    // pose.pose.position.z = enuQueue[i](2);
                    pose.pose.position.z = 1;
                    pose.pose.orientation.x = 0;
                    pose.pose.orientation.y = 0;
                    pose.pose.orientation.z = 0;
                    pose.pose.orientation.w = 1;
                    refPath.poses.push_back(pose);
                    if(save_tum)
                    {   
                        Eigen::Vector2d osm_in_map = T_enu_odom.inverse() * Eigen::Vector2d(enuQueue[i](0), enuQueue[i](1));
                        // record gps odom for EVO
                        std::ofstream OSM_gpsframe(tum_save_path+"osmXYZ_gpsframe.tum", std::ios::app);
                        OSM_gpsframe.setf(std::ios::fixed, std::ios::floatfield);
                        OSM_gpsframe.precision(5);
                        OSM_gpsframe << msg->header.stamp << " "
                                << enuQueue[i](0) << " "
                                << enuQueue[i](1) << " "
                                << enuQueue[i](2) << " "
                                << 0 << " "
                                << 0 << " "
                                << 0 << " "
                                << 0 << std::endl;
                        OSM_gpsframe.close();
                        std::ofstream OSM_mapframe(tum_save_path+"osmXYZ_mapframe.tum", std::ios::app);
                        OSM_mapframe.setf(std::ios::fixed, std::ios::floatfield);
                        OSM_mapframe.precision(5);
                        OSM_mapframe << msg->header.stamp << " "
                                << osm_in_map(0) << " "
                                << osm_in_map(1) << " "
                                << enuQueue[i](2) << " "
                                << 0 << " "
                                << 0 << " "
                                << 0 << " "
                                << 0 << std::endl;
                        OSM_mapframe.close();
                    }
                }
                osmPathPub.publish(refPath);

                //发布没插值的
                for(int i = 0; i < osmGraph.size(); ++i) 
                {
                    Eigen::Vector3d lla(osmGraph[i][0], osmGraph[i][1], osmGraph[i][2]);
                    Eigen::Vector3d ecef = gpsTools.LLA2ECEF(lla);
                    Eigen::Vector3d enu = gpsTools.ECEF2ENU(ecef);
                    ecefQueue_graph.push_back(ecef);
                    enuQueue_graph.push_back(enu);
                }
                osmPathGraph.header.frame_id = "gps";
     
                osmPathGraph.header.stamp = msg->header.stamp;
                for (int i = 0; i < enuQueue_graph.size(); ++i) 
                {
                    geometry_msgs::PoseStamped pose;
                    pose.header = osmPathGraph.header;
                    pose.pose.position.x = enuQueue_graph[i](0);
                    pose.pose.position.y = enuQueue_graph[i](1);
                    // pose.pose.position.z = enuQueue_graph[i](2);
                    pose.pose.position.z = 0.2;
                    pose.pose.orientation.x = 0;
                    pose.pose.orientation.y = 0;
                    pose.pose.orientation.z = 0;
                    pose.pose.orientation.w = 1;
                    osmPathGraph.poses.push_back(pose);
                }
                osmPathGraphPub.publish(osmPathGraph);

                
                double min = 999;
                for(int i = 0; i < 200; i++) 
                {
                    double dis = pow(msg->latitude - osmQueue[i](0), 2) + pow(msg->longitude - osmQueue[i](1), 2);
                    if (min > dis) 
                    {
                        min = dis;
                        queueIndex = i; 
                    }
                }
                ROS_INFO("start osm queueIndex :%i",queueIndex);

                initOSM=true;
                return;
            }

            refPath.header.frame_id = "gps";
            refPath.header.stamp = msg->header.stamp;
            osmPathPub.publish(refPath);
            osmPathGraph.header.stamp = msg->header.stamp;
            osmPathGraphPub.publish(osmPathGraph);
            Eigen::Vector3d lla = gpsTools.GpsMsg2Eigen(*msg);
            Eigen::Vector3d ecef = gpsTools.LLA2ECEF(lla);
            Eigen::Vector3d enu = gpsTools.ECEF2ENU(ecef);
            ROS_INFO("GPS ENU XYZ : %f, %f, %f", enu(0), enu(1), enu(2));

            nav_msgs::Odometry odomMsg;
            odomMsg.header.stamp = msg->header.stamp;
            odomMsg.header.frame_id = "gps";
            odomMsg.child_frame_id = "map";
            odomMsg.pose.pose.position.x = enu(0);
            odomMsg.pose.pose.position.y = enu(1);
            odomMsg.pose.pose.position.z = enu(2);
            odomMsg.pose.covariance[0] = msg->position_covariance[0];
            odomMsg.pose.covariance[7] = msg->position_covariance[4];
            odomMsg.pose.covariance[14] = msg->position_covariance[8];
            // record for waypoint
            odomMsg.pose.covariance[1] = lla[0];  
            odomMsg.pose.covariance[2] = lla[1];
            odomMsg.pose.covariance[3] = lla[2];
            odomMsg.pose.covariance[4] = gpsTools.lla_origin_.x();  
            odomMsg.pose.covariance[5] = gpsTools.lla_origin_.y();
            odomMsg.pose.covariance[6] = gpsTools.lla_origin_.z();
            odomMsg.pose.covariance[8] = ecef[0];  
            odomMsg.pose.covariance[9] = ecef[1];
            odomMsg.pose.covariance[10] = ecef[2];
            odomMsg.pose.covariance[11] = enu[0];  
            odomMsg.pose.covariance[12] = enu[1];
            odomMsg.pose.covariance[13] = enu[2];
            if (initHeading_)
                odomMsg.pose.pose.orientation = yawQuat;
            else 

                odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(NAN);
            gpsOdomPub.publish(odomMsg);

            if(save_tum)
            {
                // record gps odom for EVO
                std::ofstream groundtruthFile(tum_save_path+"groundtruth_gpsframe.tum", std::ios::app);
                groundtruthFile.setf(std::ios::fixed, std::ios::floatfield);
                groundtruthFile.precision(5);
                groundtruthFile << odomMsg.header.stamp << " "
                        << odomMsg.pose.pose.position.x << " "
                        << odomMsg.pose.pose.position.y << " "
                        << odomMsg.pose.pose.position.z << " "
                        << 0 << " "
                        << 0 << " "
                        << 0 << " "
                        << 0 << std::endl;
                groundtruthFile.close();
            }
            

            nav_msgs::Odometry odomMsg_map;
            odomMsg_map.header.stamp = msg->header.stamp;
            odomMsg_map.header.frame_id = "map";
            Eigen::Vector2d gps_in_map = T_enu_odom.inverse() * Eigen::Vector2d(enu(0), enu(1));
            odomMsg_map.pose.pose.position.x = gps_in_map(0);
            odomMsg_map.pose.pose.position.y = gps_in_map(1);
            odomMsg_map.pose.pose.position.z = enu(2);
            odomMsg_map.pose.covariance[0] = msg->position_covariance[0];
            odomMsg_map.pose.covariance[7] = msg->position_covariance[4];
            odomMsg_map.pose.covariance[14] = msg->position_covariance[8];
            // record for waypoint
            odomMsg_map.pose.covariance[1] = lla[0];  
            odomMsg_map.pose.covariance[2] = lla[1];
            odomMsg_map.pose.covariance[3] = lla[2];
            odomMsg_map.pose.covariance[4] = gpsTools.lla_origin_.x();  
            odomMsg_map.pose.covariance[5] = gpsTools.lla_origin_.y();
            odomMsg_map.pose.covariance[6] = gpsTools.lla_origin_.z();
            odomMsg_map.pose.covariance[8] = ecef[0];  
            odomMsg_map.pose.covariance[9] = ecef[1];
            odomMsg_map.pose.covariance[10] = ecef[2];
            odomMsg_map.pose.covariance[11] = enu[0];  
            odomMsg_map.pose.covariance[12] = enu[1];
            odomMsg_map.pose.covariance[13] = enu[2];

            if(save_tum)
            {
                std::ofstream groundtruthFile_mapframe(tum_save_path+"groundtruth_mapframe.tum", std::ios::app);
                groundtruthFile_mapframe.setf(std::ios::fixed, std::ios::floatfield);
                groundtruthFile_mapframe.precision(5);
                groundtruthFile_mapframe << odomMsg_map.header.stamp << " "
                        << odomMsg_map.pose.pose.position.x << " "
                        << odomMsg_map.pose.pose.position.y << " "
                        << odomMsg_map.pose.pose.position.z << " "
                        << odomMsg_map.pose.pose.orientation.x << " "
                        << odomMsg_map.pose.pose.orientation.y << " "
                        << odomMsg_map.pose.pose.orientation.z << " "
                        << odomMsg_map.pose.pose.orientation.w << std::endl;
                groundtruthFile_mapframe.close();
            }
            

            // publish path
            fusedPath.header.frame_id = "gps";
            fusedPath.header.stamp = msg->header.stamp;
            geometry_msgs::PoseStamped pose;
            pose.header = fusedPath.header;
            pose.pose.position.x = enu(0);
            pose.pose.position.y = enu(1);
            pose.pose.position.z = enu(2);
            pose.pose.orientation.x = yawQuat.x;
            pose.pose.orientation.y = yawQuat.y;
            pose.pose.orientation.z = yawQuat.z;
            pose.pose.orientation.w = yawQuat.w;
            fusedPath.poses.push_back(pose);
            gpsPathPub.publish(fusedPath);

        }


        // TODO: add oriantation
        void lidarOdomCallback(const nav_msgs::OdometryConstPtr &msg) {

            currentPose[0] =  msg->pose.pose.position.x;
            currentPose[1] =  msg->pose.pose.position.y;
            currentPose[2] = msg->pose.pose.position.z;
            
            if(save_tum)
            {
                std::ofstream lio_file(tum_save_path+"lio.tum", std::ios::app);
                lio_file.setf(std::ios::fixed, std::ios::floatfield);
                lio_file.precision(5);
                lio_file << msg->header.stamp << " "
                        << msg->pose.pose.position.x << " "
                        << msg->pose.pose.position.y << " "
                        << msg->pose.pose.position.z << " "
                        << msg->pose.pose.orientation.x << " "
                        << msg->pose.pose.orientation.y << " "
                        << msg->pose.pose.orientation.z << " "
                        << msg->pose.pose.orientation.w << std::endl;
                lio_file.close();
            }
            if(!inittf)
            {
                if(!initLidarXYZ_){
                lidarStartTime = msg->header.stamp.toSec();
                FirstLidarP(0) = msg->pose.pose.position.x;
                FirstLidarP(1) = msg->pose.pose.position.y;
                FirstLidarP(2) = msg->pose.pose.position.z;
                ROS_ERROR("waiting init origin axis,first Lidar frame:%f,%f,%f",FirstLidarP(0),FirstLidarP(1),FirstLidarP(1));
                initLidarXYZ_ = true;
                 }
                OdomData lioOdom;

                lioOdom.timeSec =msg->header.stamp.toSec();
                lioOdom.pos.x() = msg->pose.pose.position.x - FirstLidarP(0);
                lioOdom.pos.y() = msg->pose.pose.position.y - FirstLidarP(1);
                // lioOdom.pos.z = msg->pose.pose.position.z;
                // ROS_INFO("fastlio XYZ: %f, %f, %f",lioque.pose.position.x,lioque.pose.position.y,lioque.pose.position.z);
                odom_data.push_back(lioOdom);
                // ROS_ERROR("lidar init time,start time:%f,%f",lioOdom.timeSec , lidarStartTime);

                // if(odom_data.size() > 900)
                if(lioOdom.timeSec - lidarStartTime > initial_time )
                {
                    ROS_INFO("\033[1;32m----> Lidar init success.Use time: %f\033[0m",lioOdom.timeSec - lidarStartTime);
                    inittf =true;
                }
                return;
            }

            if(initOSM&&!DealnewPath)
            {   
                Eigen::Vector2d slam_pose_in_enu = T_enu_odom * Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
                double min = 999;
                std::cout<< "=================slam_pose_in_enu" << slam_pose_in_enu[0] <<"," << slam_pose_in_enu[1] <<std::endl;
                // for(int i = currentIndex ; i <= (currentIndex + 200); i++) 
                for(int i = currentIndex ; i <= (currentIndex + current_index_incl); i++) 
                {
                    double dis = sqrt(pow(slam_pose_in_enu(0) - enuQueue[i](0), 2) + pow(slam_pose_in_enu(1) - enuQueue[i](1), 2));
                    if (min > dis) 
                    {
                        min = dis;
                        currentIndex = i; 
                    }
                }
                ROS_INFO("current_index: %i",currentIndex);
                std::cout<<"Size of enuQueue" << enuQueue.size() <<std::endl;

                double accumulated_distance = 0.0;

                for (int i = currentIndex+1 ; i < enuQueue.size(); ++i)
                {
                    Eigen::Vector3d prev_point = enuQueue[i - 1];
                    Eigen::Vector3d current_point = enuQueue[i];

                    double segment_length = sqrt(pow(current_point(0) - prev_point(0), 2) + pow(current_point(1) - prev_point(1), 2));

                    accumulated_distance += segment_length;

                    if (accumulated_distance > max_osm_dis)
                    {
                        queueIndex = i;
                        break;
                    }
                }

                ROS_INFO("current_index: %i, queueindex: %i",currentIndex,queueIndex);
                // double dis =0;
                
                // while(queueIndex < enuQueue.size()) 
                // {
                //     // dis = sqrt(pow(msg->pose.pose.position.x- enuQueue[queueIndex](0), 2) + pow(msg->pose.pose.position.y - enuQueue[queueIndex](1), 2));
                //     // ROS_INFO("begin dis is: %f , slam_pose : %f,%f , gps_pose: %f,%f",dis,msg->pose.pose.position.x,msg->pose.pose.position.y,enuQueue[queueIndex](0),enuQueue[queueIndex](1));
                //     // if(dis > 20) break;
                //     // // if(dis > 45) break;
                //     // queueIndex++;
                //     dis = sqrt(pow(slam_pose_in_enu(0) - enuQueue[queueIndex](0), 2) + pow(slam_pose_in_enu(1) - enuQueue[queueIndex](1), 2));
                //     ROS_INFO("begin dis is: %f , slam_pose : %f,%f , gps_pose: %f,%f",dis, slam_pose_in_enu(0), slam_pose_in_enu(1), enuQueue[queueIndex](0), enuQueue[queueIndex](1));
                //     if(dis > max_osm_dis) break;
                //     queueIndex++;
                // }
                // transENU = enuQueue[queueIndex];
                // while(queueIndex < enuQueue_tran.size()) 
                // {
                //     dis = sqrt(pow(msg->pose.pose.position.x- enuQueue_tran[queueIndex](0), 2) + pow(msg->pose.pose.position.y - enuQueue_tran[queueIndex](1), 2));
                //     ROS_INFO("begin dis is: %f , slam_pose : %f,%f , gps_pose: %f,%f",dis,msg->pose.pose.position.x,msg->pose.pose.position.y,enuQueue_tran[queueIndex](0),enuQueue_tran[queueIndex](1));
                    
                //     if(dis > 30) break;
                //     queueIndex++;
                //     // if(dis > 25) break;
                // }
                transENU = enuQueue[queueIndex];
                geometry_msgs::PointStamped waypoint;
                waypoint.header.stamp = msg->header.stamp;
                waypoint.header.frame_id = "gps";
                waypoint.point.x = transENU(0);
                waypoint.point.y = transENU(1);
                // waypoint.point.z = transENU(2);
                waypoint.point.z = 1;         
                waypointPub.publish(waypoint);
            }

            geometry_msgs::PoseStamped lio;
            lioPath.header.stamp = msg->header.stamp;
            lioPath.header.frame_id = "map";
            lio.header = lioPath.header;
            lio.pose.position.x = msg->pose.pose.position.x;
            lio.pose.position.y = msg->pose.pose.position.y;
            lio.pose.position.z = msg->pose.pose.position.z;
            lio.pose.orientation.x = msg->pose.pose.orientation.x;
            lio.pose.orientation.y = msg->pose.pose.orientation.y;
            lio.pose.orientation.z = msg->pose.pose.orientation.z;
            lio.pose.orientation.w = msg->pose.pose.orientation.w;
            if(save_tum)
            {
                std::ofstream lio_file(tum_save_path+"lio_inigps.tum", std::ios::app);
                lio_file.setf(std::ios::fixed, std::ios::floatfield);
                lio_file.precision(5);
                lio_file << lio.header.stamp << " "
                        << lio.pose.position.x << " "
                        << lio.pose.position.y << " "
                        << lio.pose.position.z << " "
                        << lio.pose.orientation.x << " "
                        << lio.pose.orientation.y << " "
                        << lio.pose.orientation.z << " "
                        << lio.pose.orientation.w << std::endl;
                lio_file.close();
            }
            
            lioPath.poses.push_back(lio);
            lioPub.publish(lioPath);
            time_buffer.push_back(msg->header.stamp);
        }

        void syncFrameData(const std::vector<OdomData> &odomData, const std::vector<GpsData> &gpsData,
                   std::vector<FrameData> &frameData) 
        {
            int gpsStartIdx = 0;
            for (auto &odom: odomData) {
                for (int i = gpsStartIdx; i < gpsData.size() - 1; i++) {
                    const GpsData &g0 = gpsData[i];
                    const GpsData &g1 = gpsData[i + 1];
                    if (g1.timeSec == g0.timeSec) {
                        continue;
                    }
                    if (g1.timeSec < odom.timeSec) {
                        gpsStartIdx = i;
                        continue;
                    }
                    if (g0.timeSec > odom.timeSec) {
                        // 找不到最近时间戳的gps数据
                        break;
                    }
                    if (odom.timeSec - g0.timeSec > 0.5 || g1.timeSec - odom.timeSec > 0.5){
                        // 数据太远，不利于插值
                        continue;
                    }
                    // 找到g0 < odom < g1
                    FrameData frame;
                    frame.timeSec = odom.timeSec;
                    frame.odom = odom;
                    frame.gps = g0;
                    frame.gps.pos += ((g1.pos - g0.pos) * ((odom.timeSec - g0.timeSec) / (g1.timeSec - g0.timeSec)));
                    frameData.emplace_back(frame);
                    break;
                }
            }
            ROS_INFO("Synced odom : %d ",frameData.size()) ;
        }
        void pose_estimation_2d2d(const std::vector<Eigen::Vector2d> &pts1, const std::vector<Eigen::Vector2d> &pts2,
                                    const std::vector<Eigen::Vector2d> &cov, Eigen::Matrix2d &R, Eigen::Vector2d &T, bool use_cov){
            // pts1 = R * pts2 + T;
            Eigen::Vector2d p1(0, 0), p2(0, 0); // center of mass
            int N = pts1.size();
            for (int i = 0; i < N; i++){
                p1 += pts1[i];
                p2 += pts2[i];
            }
            p1 = p1 / N;
            p2 = p2 / N;

            std::vector<Eigen::Vector2d> q1, q2; // remove the center
            for (int i = 0; i < N; i++){
                Eigen::Vector2d q1_temp, q2_temp;
                q1_temp= pts1[i] - p1;
                q2_temp = pts2[i] - p2;
                q1.push_back(q1_temp);
                q2.push_back(q2_temp);
            }

            // compute q1*q2^T
            Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
            for(int i = 0; i < N; i++){
                if(use_cov){
                    double weight = 2.0 / (cov[i].x() + cov[i].y());
                    W += weight * q2[i] * q1[i].transpose();
                }
                else{
                    W += q2[i] * q1[i].transpose();
                }
            }
            // SVD on W
            Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix2d U = svd.matrixU();
            Eigen::Matrix2d V = svd.matrixV();

            Eigen::Matrix2d E;
            E << 1,0,0,(V * (U.transpose())).determinant();
            // pts1 = R * pts2 + T;
            // ENU = R * Lidar + T;
            R = V * E * (U.transpose());
            T = p1 - R * p2;
        }

        Eigen::Isometry2d estimate_exec(std::vector<FrameData> &frameData)
        {
            std::vector<Eigen::Vector2d>  vec_enu_ ;
            std::vector<Eigen::Vector2d>  vec_odom_ ;
            std::vector<Eigen::Vector2d>  cov_enu_ ;
            for(int i = 0; i < frame_data.size(); i++){
                vec_odom_.push_back(frameData[i].odom.pos);
                vec_enu_.push_back(frameData[i].gps.pos);
                cov_enu_.push_back(frameData[i].gps.cov);
            }
            Eigen::Matrix2d R_odom_enu; Eigen::Vector2d t_odom_enu  ;
            pose_estimation_2d2d(vec_odom_, vec_enu_, cov_enu_, R_odom_enu, t_odom_enu, true);
            Eigen::Isometry2d T_odom_enu(R_odom_enu) ;
            T_odom_enu.pretranslate(t_odom_enu) ;
            Eigen::Isometry2d T_enu_odom = T_odom_enu.inverse() ;           // odom to enu
            return  T_enu_odom ;
        }
        void sendTF(const ros::Time& time, tf::TransformBroadcaster& broadcaster)
        {
            if (!tf_calculated_) 
            {
                syncFrameData(odom_data,gps_data, frame_data);
                T_enu_odom = estimate_exec(frame_data);     // get odom2enu
                T_enu_odom.pretranslate(translation_offset);
                Eigen::Rotation2Dd rotation_offset(rotation_offset_angle);
                T_enu_odom.prerotate(rotation_offset);
                tf_calculated_ = true;
            }

            // syncFrameData(odom_data,gps_data, frame_data);
            // Eigen::Isometry2d  T_enu_odom =  estimate_exec(frame_data);     // get odom2enu
            Eigen::Matrix3d mat_enu_odom =  T_enu_odom.matrix();     // get odom2enu
            // Eigen::Matrix3d twoDenu_oodom ;
            twoDenu_oodom << mat_enu_odom(0,0),mat_enu_odom(0,1),0,
                            mat_enu_odom(1,0),mat_enu_odom(1,1),0,
                            0,0,1;
            
            
            tf::Transform transform;
            // transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            transform.setOrigin(tf::Vector3(T_enu_odom.translation().x(), T_enu_odom.translation().y(), 0.0));
            Eigen::Quaterniond quat(twoDenu_oodom);
            tf::Quaternion q(quat.x(),quat.y(),quat.z(),quat.w());   
            // q.setRPY(0, 0, 1.75);
            transform.setRotation(q);

            FirstGpsP = twoDenu_oodom*FirstLidarP;
            
            broadcaster.sendTransform(tf::StampedTransform(transform, time, "gps", "map"));
            if(!tfinfosend)
            {
                Eigen::Vector3d euler_angles = twoDenu_oodom.eulerAngles(0, 1, 2); 
                ROS_INFO("Euler Angles (Roll, Pitch, Yaw): %.3f, %.3f, %.3f", euler_angles(0), euler_angles(1), euler_angles(2));
                tfinfosend =true;
            }
        }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "lio_sam_6axis");
  static tf::TransformBroadcaster broadcaster;
  GNSSOdom gpsOdom;
  ros::Rate rate(10); 
  while(ros::ok())
  {
    if(gpsOdom.inittf && !time_buffer.empty())
    {
        ros::Time time = time_buffer.front();
        time_buffer.pop_front();
        gpsOdom.sendTF(time, broadcaster);

    }
    ros::spinOnce();
    rate.sleep();
  } 
  ROS_INFO("\033[1;32m----> Simple GPS Odmetry Started.\033[0m");
  return 0;
}