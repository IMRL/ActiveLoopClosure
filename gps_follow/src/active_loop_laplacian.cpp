// sys
#include <queue>
#include <deque>
#include <mutex>
#include <fstream>
#include <unordered_set>
#include <utility>

// ros
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <boost/bind.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <visualization_msgs/MarkerArray.h> 

//graph
#include "laplacian_graph.hpp"
#include "rppSolver.hpp"

// eigen
#include <eigen3/Eigen/Eigen>
#include <functional>
#include <deque>

#include "gpsTools.hpp"
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>

#include "gps_follow/gpsTomap.h"
#include "gps_follow/NavSatFixArray.h"



struct Pose6D_Uncertain {
    struct Pose {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    } pose;
    std::array<double, 6> uncertain;
};

struct Pose6D_Uncertain_time {
    struct Pose {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    } pose;
    std::array<double, 6> uncertain;
    // ros::Time time; 
    double time; 
    double d_opt;
};

struct ClosestPoseInfo {
    int index; 
    float distance; 
    bool isWithinThreshold;
};
struct FloatCompare {
    bool operator()(const std::pair<double, double>& a, const std::pair<double, double>& b) const {
        double tolerance = 1e-6;
        return std::fabs(a.first - b.first) < tolerance && std::fabs(a.second - b.second) < tolerance;
    }
};

struct FloatHash {
    std::size_t operator()(const std::pair<double, double>& pair) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, std::round(pair.first * 1e6));
        boost::hash_combine(seed, std::round(pair.second * 1e6));
        return seed;
    }
};
using namespace std;


class ActiveLoop {
    public:
        ros::NodeHandle nh;
        // GpsTools gpsTools;
        Graph G_osm;
        Graph G_cppTraj;
        RuralPostmanProblemSolver solver;
        double loop_weight;
        double osm_pose_dis;

        double node_gap;

        ros::ServiceClient gps_map_convert_service;
        ros::Publisher pose_fusion_uncertain , localPlanPub ,active_loop_pub, loop_point_pub ,target_pose_pub,num_marker_pub,graph_pub,full_path_pub; 
        ros::Publisher path_marker_pub1,path_marker_pub2,path_marker_pub3;
        ros::Subscriber osmSub ,fastlioOdomSub ,PgoOdomSub , LoopPairMarkerSub , PoseUncertainMarkerSub;

        Pose6D_Uncertain LioPoseUncertain;
        Pose6D_Uncertain_time PgoPoseUncertain;

        std::vector<Pose6D_Uncertain> Pgo_uncertain_buffer;
        std::vector<Pose6D_Uncertain> fusion_uncertain_buffer;

        std::vector<int> CurrentPath;
        std::vector<geometry_msgs::Pose> fusedOdom_poses;
        std::vector<geometry_msgs::Pose> gpsOdom_poses;

        std::vector<Pose6D_Uncertain_time> global_uncertain_buffer;
        std::vector<std::pair<Pose6D_Uncertain_time, Node>> visited_poses_and_nodes;
        std::vector<double> uncertainty_sum = std::vector<double>(6,0.0);
        std::vector<double> baseline_uncertainty = std::vector<double>(6,0.0);

        std::unordered_set<std::string> processed_loops; 
        std::vector<geometry_msgs::PoseStamped> osm_in_map;

        tf::TransformListener tf_listener;
        tf::StampedTransform gps2map;

        message_filters::Subscriber<nav_msgs::Odometry> GpsOdomSub;
        message_filters::Subscriber<nav_msgs::Odometry> FusedOdomSub;

        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> syncPolicy;
        typedef message_filters::Synchronizer<syncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;

        double gnss_erro;

        double pose_erro;

        double Yaw_uncertain;
        double arive_node_dis1,arive_node_dis2;

        double cpp_pathLen;
        std::vector<int> cppPath;
        double D_opt_osm;

        double visitedCoefficient;

        double active_dis,active_time,active_uncertain; 
        // bool GetTF=false;
        bool OSMini = false;
        bool saveTXT=false;


        bool need_loop_closure = true;
        Node max_j1_node;
        Node prev_max_j1_node;
                
        ros::Time last_loop_marker_time;


        ActiveLoop() : nh("~") 
        {
            osmSub =  nh.subscribe("/osm_Path_Graph", 100, &ActiveLoop::osmHandle, this, ros::TransportHints().tcpNoDelay());
            // fastlioOdomSub = nh.subscribe("/Odometry", 100, &ActiveLoop::LioOdomHandle, this, ros::TransportHints().tcpNoDelay());
            PgoOdomSub = nh.subscribe("/aft_pgo_odom", 100, &ActiveLoop::PgoOdomHandle, this, ros::TransportHints().tcpNoDelay());

            LoopPairMarkerSub = nh.subscribe("/loop_constraint_edge", 100, &ActiveLoop::LoopPairHandle, this, ros::TransportHints().tcpNoDelay());
            PoseUncertainMarkerSub = nh.subscribe("/pose_with_uncertainty", 100, &ActiveLoop::GlobalUncerainHandle, this, ros::TransportHints().tcpNoDelay());

            GpsOdomSub.subscribe(nh,"/gps_odom",1);

            gps_map_convert_service =  nh.serviceClient<gps_follow::gpsTomap>("/ini_gps/gps_map_convert_server");

            pose_fusion_uncertain = nh.advertise<nav_msgs::Odometry>("/pose_fusion_uncertain",100);
            // localPlanPub = nh.advertise<nav_msgs::Path>("/active_loop_path",100);
            // active_loop_pub = nh.advertise<visualization_msgs::Marker>("/active_loop_pose",100);
            loop_point_pub = nh.advertise<gps_follow::NavSatFixArray>("/loop_gps_point", 10);

            target_pose_pub = nh.advertise<visualization_msgs::Marker>("/target_pose", 10);

            path_marker_pub1 = nh.advertise<nav_msgs::Path>("/active_loop_path_global_1", 10);
            path_marker_pub2 = nh.advertise<nav_msgs::Path>("/active_loop_path_global_2", 10);
            path_marker_pub3 = nh.advertise<nav_msgs::Path>("/active_loop_path_global_3", 10);

            num_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/active_loop_path_global_num", 1);

            // full_path_pub = nh.advertise<nav_msgs::Path>("/active_loop_path", 1);
            full_path_pub = nh.advertise<visualization_msgs::Marker>("/active_loop_path", 1);
            // graph_pub = nh.advertise<nav_msgs::Path>("/graph_osm", 1);
            graph_pub = nh.advertise<visualization_msgs::MarkerArray>("/graph_osm", 1);

            max_j1_node.id = 0;
            prev_max_j1_node.id = 0;

            nh.getParam("gnss_erro",gnss_erro);
            nh.getParam("pose_erro",pose_erro);
            nh.getParam("saveTXT",saveTXT);
            nh.getParam("Yaw_uncertain",Yaw_uncertain);
            nh.getParam("active_dis",active_dis);
            nh.getParam("active_time",active_time);
            nh.getParam("active_uncertain",active_uncertain);
            nh.getParam("loop_weight",loop_weight);
            nh.getParam("osm_pose_dis", osm_pose_dis);
            nh.getParam("visitedCoefficient",visitedCoefficient);
            nh.getParam("node_gap",node_gap);
            nh.getParam("arive_node_dis1",arive_node_dis1);
            nh.getParam("arive_node_dis2",arive_node_dis2);

            ROS_INFO("try to  get tf!!!!!!!");

            while (!tf_init() && ros::ok()) 
            {
                ROS_INFO("waiting!!!!!");
                ros::Duration(1.0).sleep();
            }
            ROS_INFO("suceesfully get tf!!!!!!!");

        }

        bool tf_init() 
        {
            try
            {
                tf_listener.waitForTransform("map", "gps", ros::Time(0), ros::Duration(10));  
                tf_listener.lookupTransform("map", "gps", ros::Time(0), gps2map);
                return true; 
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN("%s. Retrying...", ex.what());
                return false;
            }
        }
        /**
         * 接收osm数据，构建图以及laplacian
        */
        void osmHandle(const nav_msgs::PathConstPtr &msg)
        {
            if(OSMini)
            {
                return;
            }
            osm_in_map.clear();

            for (const auto &pose_stamped : msg->poses)
            {
                geometry_msgs::PoseStamped transformed_pose_stamped;
                tf::Stamped<tf::Pose> stamped_tf_pose;
                tf::poseStampedMsgToTF(pose_stamped, stamped_tf_pose);
                stamped_tf_pose.setData(gps2map * stamped_tf_pose);
                tf::poseStampedTFToMsg(stamped_tf_pose, transformed_pose_stamped);
                osm_in_map.push_back(transformed_pose_stamped);
            }
            cpp_pathLen = calculateTotalDistance(osm_in_map);
            std::cout << "osm size" << osm_in_map.size() << " and paht len : " << cpp_pathLen <<std::endl;
            
            std::vector<geometry_msgs::PoseStamped> Interpolated_path = InterpolatedPath(osm_in_map);
            PriorTrajGraph(G_cppTraj,Interpolated_path);
            std::cout << "G_cppTraj node size :" << G_cppTraj.nodes.size() << "G_cppTraj edge size :" << G_cppTraj.edges.size() <<std::endl;
            for (const auto& edge : G_cppTraj.edges) {
                std::cout << "G_cppTraj  Edge from node " << edge.from << " to node " << edge.to << " edge weight" << edge.weight << std::endl;
            }
            TrajEdgeGraph(G_osm,Interpolated_path);
            TrajUpdateGraph(G_osm,cppPath);
            G_osm.calculateEdgeWeights();
            std::cout<<"| G_osm Graph has been build And have " << G_osm.nodes.size() << " nodes  and  " << G_osm.edges.size() << "edges |" <<std::endl;
            for (const auto& edge : G_osm.edges) 
            {
                std::cout << "| G_osm  Edge from node " << edge.from << " to node " << edge.to << " edge count "<< edge.count<<" edge weight " << edge.weight << " edge dis_weight " << edge.dis_weight<<   " |" << std::endl;
            }
            visualizeGraph(G_osm,graph_pub);
            MatrixXd L_osm_prior = G_osm.computeLaplacian();
            D_opt_osm = G_osm.calculateDOptimality(L_osm_prior);
            std::cout << "Laplacian matrix dimensions: " << L_osm_prior.rows() << "x" << L_osm_prior.cols() << std::endl;
            ROS_INFO("\033[1;32m----> D-optimality of the Trajectory_good  Laplacian matrix %f\033[0m", D_opt_osm );
            OSMini = true;
        }

        double calculateTotalDistance(const std::vector<geometry_msgs::PoseStamped>& path) {
            double total_distance = 0.0;

            for (size_t i = 1; i < path.size(); ++i) {
                const auto& prev_pose = path[i - 1].pose.position;
                const auto& current_pose = path[i].pose.position;

                double dx = current_pose.x - prev_pose.x;
                double dy = current_pose.y - prev_pose.y;
                double dz = current_pose.z - prev_pose.z;
                total_distance += std::sqrt(dx * dx + dy * dy + dz * dz);
            }

            return total_distance;
        }

        std::vector<geometry_msgs::PoseStamped> InterpolatedPath(const std::vector<geometry_msgs::PoseStamped>& path)
        {
            // 插值
            std::vector<geometry_msgs::PoseStamped> interpolated_path;
            for (size_t i = 0; i < path.size() - 1; ++i) {
                const auto& start = path[i].pose.position;
                const auto& end = path[i + 1].pose.position;

                double dx = end.x - start.x;
                double dy = end.y - start.y;
                double distance = euclidean_distance(path[i], path[i + 1]);
                int num_segments = std::max(static_cast<int>(std::ceil(distance / node_gap)), 0);

                for (int j = 0; j < num_segments; ++j) {
                    geometry_msgs::PoseStamped new_point = path[i];
                    new_point.pose.position.x = start.x + j * dx / num_segments;
                    new_point.pose.position.y = start.y + j * dy / num_segments;
                    interpolated_path.push_back(new_point);
                }
            }
            interpolated_path.push_back(path.back());

            return interpolated_path;
        }

        /**
         * 根据轨迹构建轨迹图
        */
        void PriorTrajGraph(Graph& G_Traj,  std::vector<geometry_msgs::PoseStamped> osm_in_map)
        {
            for ( int i = 0; i < osm_in_map.size(); ++i)
            {
                G_Traj.addNode(i, osm_in_map[i].pose.position.x, osm_in_map[i].pose.position.y);
            }

            for (int i = 0; i < osm_in_map.size() - 1; ++i)
            {
                double dx = osm_in_map[i+1].pose.position.x - osm_in_map[i].pose.position.x;
                double dy = osm_in_map[i+1].pose.position.y - osm_in_map[i].pose.position.y;
                double distance = std::hypot(dx, dy); 

                if (distance == 0.0) {
                    std::cerr << "Warning: Zero distance between nodes " << i << " and " << i + 1 << std::endl;
                    continue; 
                }
                // double weight = 1.0 / (0.01+distance);
                double weight = 200/ (2+distance);
                G_Traj.addEdge(i, i + 1, weight,distance);
            }
            for (size_t i = 0; i < osm_in_map.size(); ++i) 
            {
                for (size_t j = i + 1; j < osm_in_map.size(); ++j) 
                {
                    if (osm_in_map[i].pose.position == osm_in_map[j].pose.position) {
                        G_Traj.addEdge(i, j, loop_weight,0.1);
                        // std::cout << "Added loop edge: " << i << " -> " << j << " with weight " << loop_weight << std::endl;
                    }
                }
            }

        }

       void TrajEdgeGraph(Graph& G_edge, const std::vector<geometry_msgs::PoseStamped>& path) {
            if (path.empty()) return;

            std::unordered_map<std::pair<double, double>, int, FloatHash, FloatCompare> coord_to_nodeid;
            std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int, int>>> unique_edges;
            // std::vector<int> path_graph;
            int nodeId = 0;

            for (const auto& point : path) {
                double x = point.pose.position.x;
                double y = point.pose.position.y;
                auto coord_pair = std::make_pair(x, y);

                auto it = coord_to_nodeid.find(coord_pair);
                if (it == coord_to_nodeid.end()) {
                    G_edge.addNode(nodeId, x, y);
                    coord_to_nodeid[coord_pair] = nodeId;
                    cppPath.push_back(nodeId);
                    nodeId++;
                } else {
                    cppPath.push_back(it->second);
                }
            }

            for (size_t i = 1; i < cppPath.size(); ++i) {
                int from_id = cppPath[i - 1];
                int to_id = cppPath[i];

                auto edge = std::make_pair(from_id, to_id);
                auto reverse_edge = std::make_pair(to_id, from_id);

                if (from_id != to_id && unique_edges.find(edge) == unique_edges.end()) {
                    double dis_weight = euclidean_distance(path[i - 1], path[i]);
                    G_edge.addEdge(from_id, to_id, 0,dis_weight);
                    unique_edges.insert(edge);
                    unique_edges.insert(reverse_edge); // Insert reverse edge for undirected graph
                }
            }
            for (size_t i = 0; i < cppPath.size(); ++i) {
                std::cout << cppPath[i];
                if (i < cppPath.size() - 1) {
                    std::cout << " -> ";
                }
            }
            std::cout << std::endl;
        }
        /**
         * 主体函数，用于判断去哪里回环
        */
        void GlobalUncerainHandle(const visualization_msgs::MarkerArray::ConstPtr& msg)
        {
             global_uncertain_buffer.clear();
            for (int i = 0; i < msg->markers.size(); i += 4) 
            {
                Pose6D_Uncertain_time global_pose;

                global_pose.pose.x = msg->markers[i].pose.position.x;
                global_pose.pose.y = msg->markers[i].pose.position.y;
                global_pose.pose.z = msg->markers[i].pose.position.z;
                tf::Quaternion q(msg->markers[i].pose.orientation.x,
                                msg->markers[i].pose.orientation.y,
                                msg->markers[i].pose.orientation.z,
                                msg->markers[i].pose.orientation.w);
                tf::Matrix3x3(q).getRPY(global_pose.pose.roll, global_pose.pose.pitch, global_pose.pose.yaw);
                global_pose.uncertain[0] = msg->markers[i+1].scale.x;
                global_pose.uncertain[1] = msg->markers[i+1].scale.y;
                global_pose.uncertain[2] = msg->markers[i+1].scale.z;
                global_pose.uncertain[3] = msg->markers[i+2].scale.x;
                global_pose.uncertain[4] = msg->markers[i+2].scale.y;
                global_pose.uncertain[5] = msg->markers[i+2].scale.z;
                global_pose.time = msg->markers[i].header.stamp.toSec();
                global_pose.d_opt = msg->markers[i+3].scale.x;

                global_uncertain_buffer.push_back(global_pose);
            }

            visited_poses_and_nodes.clear();
            ROS_INFO("Begin to attach Weight");
            UpdatedSpaceVisited(G_osm, CurrentPath);
            std::vector<int> path_1;
                path_1 = CurrentPath;

                double max_j1 = -std::numeric_limits<double>::infinity(); 
                // Pose6D_Uncertain_time max_j1_pose;
                // Node max_j1_node;
                std::vector<int> max_path_1, max_path_2, max_path_3;


                if (uncertainty_sum[5] > Yaw_uncertain && (prev_max_j1_node.id == max_j1_node.id)) 
                {   
                
                    // std::vector<int> path_1 = PoseBufferToPath(G_osm,global_uncertain_buffer);
                    int maxNodeId = G_osm.findMaxNodeid();
                    std::cout<< " MaxNodeId of G_osm : " <<maxNodeId<<std::endl;
                    int endNode = G_osm.findMinNodeId();
                    // double graphlen = G_osm.pathLenSum();

                    
                    // Graph G_temp_0; //当前去某个点回环的全局轨迹暂时构成的图
                    
                    // G_temp_0 = insertCurrentPose(G_osm,CurrentPath,PgoPoseUncertain);

                    int CurrentNodeid = getCurrentNodeid(G_osm,CurrentPath,PgoPoseUncertain);

                    // std::cout<<"| G_temp Graph has been build And have " << G_temp.nodes.size() << " nodes  and  " << G_temp.edges.size() << "edges |" <<std::endl;
                    // for (const auto& edge : G_temp.edges) 
                    // {
                    //     std::cout << "| G_temp  Edge from node " << edge.from << " to node " << edge.to << " edge weight " << edge.weight << " edge dis_weight " << edge.dis_weight<<   " |" << std::endl;
                    // }
                    
                    for (const auto& node : G_osm.nodes)
                    {
                        
                        if(node.second.id ==endNode || node.second.id == maxNodeId)
                            continue;
                        if(node.second.actual_visited)
                        {   
                            std::cout << "+-----------------------------------------------------------------------------+"<<std::endl;
                            // std::cout<<"| G_temp Graph has been build And have " << G_temp.nodes.size() << " nodes  and  " << G_temp.edges.size() << "edges |" <<std::endl;
                            // for (const auto& edge : G_temp.edges) 
                            // {
                            //     std::cout << "| G_temp  Edge from node " << edge.from << " to node " << edge.to << " edge weight " << edge.weight << " edge dis_weight " << edge.dis_weight<<   " |" << std::endl;
                            // }
                            Graph G_temp = G_osm;
                            // int CurrentNodeid = maxNodeId+1;
                            int TargetNodeid = node.second.id; //dijskra的终点，rpp的起点
                            std::unordered_set<int> neighbors = G_temp.getNeighbors(CurrentNodeid);
                            std::cout << " CurrentNodeid : " << CurrentNodeid << " have neighbors size : " << neighbors.size()  <<  " Target Node id : " << TargetNodeid  <<" endNode :" <<endNode<<std::endl;
                            std::vector<int> path_2 = G_temp.findBestPath(CurrentNodeid,TargetNodeid,visitedCoefficient);

                            for(int nodeid : path_2)
                            {
                                G_temp.nodes.at(nodeid).actual_visited =true;
                            }
                            std::vector<int> path_2_ = path_2;
                            path_2.pop_back();
                            G_temp.updateEdgeRequirements();
                            bool allNodesVisited = true;
                            for(const auto& node : G_temp.nodes) {
                                if(!node.second.actual_visited) {
                                    allNodesVisited = false;
                                    break;
                                }
                            }
                            std::vector<int> path_3;
                            if(allNodesVisited) {
                                std::cout << "All nodes have been visited. No need to calculate path_3." << std::endl;
                                path_3 = G_temp.findShortestPath(TargetNodeid,0);

                            } else {
                                std::pair<std::list<Edge>, Graph> result = solver.solveRPP(G_temp, TargetNodeid, 0);
                                path_3 = covertToPath(result.first);
                            }
                            
                            // std::pair<std::list<Edge>, Graph> result = solver.solveRPP(G_temp,TargetNodeid,0);
                            // std::vector<int> path_3 = covertToPath(result.first);
                            std::vector<int> full_path;
                            full_path = path_1;
                            full_path.insert(full_path.end(), path_2.begin(), path_2.end());
                            full_path.insert(full_path.end(), path_3.begin(), path_3.end());
                            std::cout<<"Path 1: ";
                            for(int node_id : path_1) 
                            {
                                std::cout << node_id << "-> ";
                            }
                            std::cout << std::endl;
                            std::cout<<"Path 2: ";
                            for(int node_id : path_2) 
                            {
                                std::cout << node_id << "-> ";
                            }
                            std::cout << std::endl;
                            std::cout<<"Path 3: ";
                            for(int node_id : path_3) 
                            {
                                std::cout << node_id << "-> ";
                            }
                            std::cout << std::endl;

                            TrajUpdateGraph(G_temp,full_path);
                            G_temp.calculateEdgeWeights();
                            // std::cout<<"| G_temp Graph has been build And have " << G_temp.nodes.size() << " nodes  and  " << G_temp.edges.size() << "edges |" <<std::endl;
                            // for (const auto& edge : G_temp.edges) 
                            // {
                            //     std::cout << "| G_temp  Edge from node " << edge.from << " to node " << edge.to << " edge count "<< edge.count<<" edge weight " << edge.weight << " edge dis_weight " << edge.dis_weight<<   " |" << std::endl;
                            // }
                            MatrixXd L = G_temp.computeLaplacian();
                            std::cout << "Laplacian matrix dimensions: " << L.rows() << "x" << L.cols() << std::endl;
                            double d_opt = G_temp.calculateDOptimality(L);
                            double path_len = getPathLength(G_temp,full_path);
                            // double current_j1 = (d_opt - D_opt_osm) / (path_len-cpp_pathLen);
                            double current_j1 = d_opt  / path_len;
                            std::cout<< " Path len :" <<path_len << "path_len-cpp_pathLen" << path_len-cpp_pathLen <<" and D-opt of Path: " << d_opt << " D-opt of cpp : " <<D_opt_osm<< "current j1 =" << current_j1 << std::endl;
                            if (current_j1 > max_j1) 
                            {
                                max_j1 = current_j1;
                                max_j1_node = node.second;
                                max_path_1 = path_1; 
                                max_path_2 = path_2_;
                                max_path_3 = path_3;
                            }
                            std::cout << "+-----------------------------------------------------------------------------+"<<std::endl;
                        }
                    }
                    ROS_INFO_STREAM("\033[1;32m" << " Current pose: (" << PgoPoseUncertain.pose.x << ", " <<PgoPoseUncertain.pose.y << ", " <<PgoPoseUncertain.pose.z<<")"
                            << "Target Node: ("
                            << max_j1_node.id<< ")" << "max_j1 = " << max_j1 << "\033[0m");
                    // geometry_msgs::PoseStamped target_pose_msg;
                    // target_pose_msg.header.stamp = ros::Time::now(); // 或者使用max_j1_pose.time
                    // target_pose_msg.header.frame_id = "camera_init";
                    // target_pose_msg.pose.position.x = max_j1_node.x_coord;
                    // target_pose_msg.pose.position.y = max_j1_node.y_coord;
                    // target_pose_msg.pose.position.z = 0;
                    // target_pose_pub.publish(target_pose_msg);
                    publishTargetPoseAsArrow(max_j1_node,target_pose_pub);
                    publishPath(max_path_1, G_osm, path_marker_pub1, 2);
                    publishPath(max_path_2, G_osm, path_marker_pub2, 4);
                    publishPath(max_path_3, G_osm, path_marker_pub3, 3);
                    std::vector<int> max_full_path;
                    std::vector<int> new_path;

                    max_full_path = max_path_1;
                    max_full_path.insert(max_full_path.end(), max_path_2.begin(), max_path_2.end());
                    max_full_path.insert(max_full_path.end(), max_path_3.begin(), max_path_3.end());

                    new_path.insert(new_path.begin(), max_path_1.back());
                    new_path.insert(new_path.end(), max_path_2.begin(), max_path_2.end());
                    new_path.insert(new_path.end(), max_path_3.begin(), max_path_3.end());
                    publishArrowPathMarkers(max_full_path, G_osm, num_marker_pub, 5.0);
                    if(!(arive_target_node(PgoPoseUncertain,max_j1_node,arive_node_dis1)))
                    {
                        pubFullPath(new_path, G_osm,full_path_pub,max_j1_node);
                        //回环点要做特殊标记
                    }
                    //可视化max_j1_node对应的路线
            }
            // int maxNodeId = G_osm.findMaxNodeid();
            // std::cout<< " MaxNodeId of G_osm : " <<maxNodeId<<std::endl;
            // int endNode = G_osm.findMinNodeId();
            // if(!(max_j1_node.id ==endNode || max_j1_node.id ==maxNodeId))
            std::cout<<"!!!!!!!!!!!!!!!! max_j1_node.id  !!!!!!!!!!!!!!!" << max_j1_node.id <<std::endl;
            std::cout<<"!!!!!!!!!!!!!!!! prev_max_j1_node.id  !!!!!!!!!!!!!!!" << prev_max_j1_node.id <<std::endl;
            if(prev_max_j1_node.id != max_j1_node.id)
            {
                std::cout<<"++++++++++++=============== check loop closure node=============="<<std::endl;
                if(arive_target_node(PgoPoseUncertain,max_j1_node,arive_node_dis2))
                {
                    std::cout<<"++++++++++++=============== arvie loop closure node=============="<<std::endl;
                    prev_max_j1_node.id = max_j1_node.id;
                }
                // else
                // {
                //     need_loop_closure = false;
                // }
            }
             std::cout<<"++++++++++++============================="<<std::endl;
            // }
        }

        bool arive_target_node(Pose6D_Uncertain_time current_pose , Node target_node,double dis_thresold)
        {
            double dx = current_pose.pose.x - target_node.x_coord;
            double dy = current_pose.pose.y - target_node.y_coord;
            double distance = std::sqrt(dx * dx + dy * dy);
            return distance <= dis_thresold;
        }

        void pubFullPath(const std::vector<int>& full_path, const Graph& G_osm, ros::Publisher& marker_pub, Node target_node) 
        {
            std::vector<Eigen::Vector3d> path_xyz;
            for (int node_id : full_path) {
                const Node& node = G_osm.nodes.at(node_id);
                path_xyz.push_back(Eigen::Vector3d(node.x_coord, node.y_coord, 0));
            }
            visualization_msgs::Marker path_marker;
            path_marker.header.frame_id = "camera_init";
            path_marker.header.stamp = ros::Time::now();
            path_marker.ns = "full path";
            path_marker.id = 0;
            path_marker.type = visualization_msgs::Marker::LINE_STRIP; // LINE_STRIP/LINE_LIST/POINTS
            path_marker.action = visualization_msgs::Marker::ADD;
            path_marker.scale.x = 0.1;
            path_marker.color.a = 1.0;
            path_marker.color.r = 1.0;
            path_marker.color.g = 0.0;
            path_marker.color.b = 0.0;
            for (const Eigen::Vector3d& coord : path_xyz) {
                geometry_msgs::Point p;
                p.x = coord.x();
                p.y = coord.y();
                p.z = coord.z();
                path_marker.points.push_back(p);
            }
            geometry_msgs::Point target_point;
            target_point.x = target_node.x_coord;
            target_point.y = target_node.y_coord;
            target_point.z = 0;
            path_marker.points.push_back(target_point);
            marker_pub.publish(path_marker);
        }

        void PgoOdomHandle(const nav_msgs::OdometryConstPtr &msg)
        {
            PgoPoseUncertain.time = msg->header.stamp.toSec();
            // Pose6D_Uncertain LioPoseUncertain;
            PgoPoseUncertain.pose.x = msg->pose.pose.position.x;
            PgoPoseUncertain.pose.y = msg->pose.pose.position.y;
            PgoPoseUncertain.pose.z = msg->pose.pose.position.z;

            tf::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            PgoPoseUncertain.pose.roll = roll;
            PgoPoseUncertain.pose.pitch = pitch;
            PgoPoseUncertain.pose.yaw = yaw;

            for (int i = 0; i < 6; i++) 
            {
                PgoPoseUncertain.uncertain[i] = msg->pose.covariance[i*7];
                double current_uncertainty = msg->pose.covariance[i*7];
                uncertainty_sum[i] = current_uncertainty - baseline_uncertainty[i];
            }
            int closestNode = -1;
            double minDistance = std::numeric_limits<double>::max();
            for (size_t i = 0; i < G_osm.nodes.size(); ++i) {
                double distance = std::hypot(PgoPoseUncertain.pose.x - G_osm.nodes[i].x_coord,
                                            PgoPoseUncertain.pose.y - G_osm.nodes[i].y_coord);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestNode = i;
                }
            }
            if (closestNode != -1 && !CurrentPath.empty() && closestNode != CurrentPath.back()) {
                if (G_osm.hasEdge(CurrentPath.back(), closestNode)) {
                    CurrentPath.push_back(closestNode);
                } else {
                    vector<int> shortestPath = G_osm.findShortestPath(CurrentPath.back(), closestNode);
                    CurrentPath.insert(CurrentPath.end(), shortestPath.begin()+1, shortestPath.end());
                }
            } else if (closestNode != -1 && CurrentPath.empty()) {
                CurrentPath.push_back(closestNode);
            }
            std::cout << "Current Path: ";
            for (int node : CurrentPath) {
                std::cout << node << "------> ";
            }
            std::cout << std::endl;
        }

        /**
         * 检测是否发生回环，如果回环了就重新计算累加
        */
        void LoopPairHandle(const visualization_msgs::MarkerArray::ConstPtr& msg)
        {
            for (const auto& marker : msg->markers) 
            {
                if (marker.points.size() > 0)
                {       
                    // if (marker.color.r == 0.9 && marker.color.g == 0.9 && marker.color.b == 0)
                    if(marker.id == 1)
                    {
                        ROS_INFO("New loop pair detected");
                        for (int i = 0; i < marker.points.size(); i += 2)
                        {
                            auto& point1 = marker.points[i];   
                            auto& point2 = marker.points[i + 1];
                            std::string loop_key = std::to_string(point1.x) + "_" + std::to_string(point1.y) + "_" + std::to_string(point1.z) + "_"
                                                + std::to_string(point2.x) + "_" + std::to_string(point2.y) + "_" + std::to_string(point2.z);
                            if (processed_loops.find(loop_key) == processed_loops.end())
                            {
                                ROS_INFO("New loop pair detected: x1 = %f, y1 = %f, z1 = %f, x2 = %f, y2 = %f, z2 = %f", point1.x, point1.y, point1.z, point2.x, point2.y, point2.z);
                                std::fill(uncertainty_sum.begin(), uncertainty_sum.end(), 0.0);
                                ROS_INFO("\033[1;32m----> LoopClosure Detect Uncertain calculate from Now\033[0m");
                                for (int j = 0; j < 6; j++) {
                                    baseline_uncertainty[j] = PgoPoseUncertain.uncertain[j];
                                }
                                processed_loops.insert(loop_key);
                            }
                        }
                    }
                }
            }
        }

        void TrajUpdateGraph(Graph& G, const std::vector<int>& full_path) 
        {
            if (full_path.empty()) return;

            std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int, int>>> unique_edges;
            for (size_t i = 1; i < full_path.size(); ++i) {
                int from_id = full_path[i - 1];
                int to_id = full_path[i];
                std::pair<int, int> edge_pair = std::make_pair(from_id, to_id);

                if (unique_edges.insert(edge_pair).second) {
                    G.increaseEdgeCount(from_id, to_id);
                }
            }

            std::unordered_set<int> visited_nodes;
            for (size_t i = 0; i < full_path.size(); ++i) {
                if (visited_nodes.find(full_path[i]) != visited_nodes.end()) 
                {
                    size_t loop_start = i;
                    while (full_path[loop_start - 1] != full_path[i]) {
                        --loop_start;
                    }

                    std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int, int>>> loop_edges;
                    for (size_t j = loop_start; j < i; ++j) {
                        int from_id = full_path[j];
                        int to_id = full_path[j + 1];
                        std::pair<int, int> edge_pair = std::make_pair(from_id, to_id);

                        if (loop_edges.insert(edge_pair).second) {
                            G.increaseEdgeCount(from_id, to_id);
                        }
                    }
                    // Ensure loop closure: increase the weight of the edge from the last node of the loop to the starting node
                    int loop_end_id = full_path[i];
                    int loop_start_id = full_path[loop_start];
                    std::pair<int, int> loop_closure_edge = std::make_pair(loop_end_id, loop_start_id);

                    if (loop_edges.insert(loop_closure_edge).second) {
                        G.increaseEdgeCount(loop_end_id, loop_start_id);
                    }
                    } else {
                    visited_nodes.insert(full_path[i]);
                }
            }
        }

        Pose6D_Uncertain_time findClosestPoseInBuffer(const Node& node, const std::vector<Pose6D_Uncertain_time>& buffer) 
        {
            Pose6D_Uncertain_time closest_pose;
            double min_distance = std::numeric_limits<double>::max();
            for (const auto& pose : buffer) 
            {
                double distance = std::hypot(pose.pose.x - node.x_coord, pose.pose.y - node.y_coord);
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_pose = pose;
                }
            }
            return closest_pose;
        }

        std::vector<int> covertToPath(const std::list<Edge>& edgeList) 
        {
            std::vector<int> path;
            if (edgeList.empty()) return path;
            const Edge& firstEdge = edgeList.front();
            path.push_back(firstEdge.from);
            for (const Edge& edge : edgeList) {
                path.push_back(edge.to);
            }
            return path;
        }

        int getCurrentNodeid(Graph& G_osm, const std::vector<int>& CurrentPath, const Pose6D_Uncertain_time& PgoPoseUncertain) 
        {
            if (CurrentPath.size() < 3) 
            {
                // 如果路径中不足3个节点，则返回-1或者考虑所有节点
                return -1;
            }
            int lastVisitedNodeId = CurrentPath.back();
            int excludeLastNodeId = *(CurrentPath.end() - 2);
            int excludeSecondLastNodeId = *(CurrentPath.end() - 3);
            auto neighbors = G_osm.getNeighbors(lastVisitedNodeId);
            int closestNeighborId = -1;
            double closestDistance = std::numeric_limits<double>::max();
            for (const auto& neighborId : neighbors) 
            {
                if (neighborId == excludeLastNodeId || neighborId == excludeSecondLastNodeId) {
                    continue;
                }
                double distance = std::hypot(G_osm.nodes.at(neighborId).x_coord - PgoPoseUncertain.pose.x, G_osm.nodes.at(neighborId).y_coord - PgoPoseUncertain.pose.y);
                if (distance < closestDistance) {
                    closestNeighborId = neighborId;
                    closestDistance = distance;
                }
            }
            return closestNeighborId;
        }

        double getPathLength(const Graph& graph, const std::vector<int>& path) 
        {
            double path_length = 0.0;
            for (size_t i = 0; i < path.size() - 1; ++i) 
            {
                Node from = graph.nodes.at(path[i]);
                Node to = graph.nodes.at(path[i + 1]);
                path_length += std::sqrt(std::pow(from.x_coord - to.x_coord, 2) + std::pow(from.y_coord - to.y_coord, 2));
            }
            return path_length;
        }

        double euclidean_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) 
        {
            return sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) + pow(p1.pose.position.y - p2.pose.position.y, 2) + pow(p1.pose.position.z - p2.pose.position.z, 2));
        }

        std::vector<int> PoseBufferToPath(Graph& Graph_edge, vector<Pose6D_Uncertain_time>& global_uncertain_buffer )
        {
            vector<int> closestPoseIndices(Graph_edge.nodes.size(), -1);
            for (size_t i = 0; i < Graph_edge.nodes.size(); ++i) 
            {
                auto& node = Graph_edge.nodes[i];
                float minDistance = std::numeric_limits<float>::max();

                for (size_t j = 0; j < global_uncertain_buffer.size(); ++j) 
                {
                    float distance = std::hypot(global_uncertain_buffer[j].pose.x - node.x_coord, 
                                                global_uncertain_buffer[j].pose.y - node.y_coord);
                    if (distance < minDistance) 
                    {
                        minDistance = distance;
                        closestPoseIndices[i] = j;
                    }
                }
                if (minDistance > osm_pose_dis) 
                {
                    closestPoseIndices[i] = -1;
                }
            }
            //step 2 对于每个找到的最近位姿点，取其在global_uncertain_buffer中的一定距离里的点，此时图中的一个节点可能对应多个pose点
            vector<vector<int>> nodeToPosesMap(Graph_edge.nodes.size());
            for (size_t i = 0; i < closestPoseIndices.size(); ++i) {
                int poseIndex = closestPoseIndices[i];
                if (poseIndex != -1) {
                    float search_radius = 5;
                    for (size_t j = 0; j < global_uncertain_buffer.size(); ++j) {
                        float distance = std::hypot(global_uncertain_buffer[j].pose.x - global_uncertain_buffer[poseIndex].pose.x,
                                                    global_uncertain_buffer[j].pose.y - global_uncertain_buffer[poseIndex].pose.y);
                        if (distance <= search_radius) {
                            nodeToPosesMap[i].push_back(j);
                        }
                    }
                }
            }
            //step 3 按照pose的时间顺序，对对应的节点进行排序，
            std::vector<std::pair<int, double>> nodeTimePairs;
            for (size_t i = 0; i < nodeToPosesMap.size(); ++i) 
            {
                for (int poseIndex : nodeToPosesMap[i]) 
                {
                    nodeTimePairs.emplace_back(i, global_uncertain_buffer[poseIndex].time);
                }
            }
            std::sort(nodeTimePairs.begin(), nodeTimePairs.end(), [&](const std::pair<int, double>& a, const std::pair<int, double>& b) 
            {
                return a.second < b.second;
            });
            //step 4 得到节点的序列后，对于序列中连续出现的重复节点只取一个，对于序列中两个相邻节点在图中不存在边的情况，补充为这两点的最短dijskra路径
            std::vector<int> path;
            if (!nodeTimePairs.empty()) {
                path.push_back(nodeTimePairs.front().first);
            }
            for (size_t i = 1; i < nodeTimePairs.size(); ++i) {
                if (nodeTimePairs[i].first != nodeTimePairs[i - 1].first) {
                    path.push_back(nodeTimePairs[i].first);
                }
            }
            std::vector<int> finalPath;
            for (size_t i = 0; i < path.size() - 1; ++i) 
            {
                int startNode = path[i];
                int endNode = path[i + 1];
                if (!Graph_edge.hasEdge(startNode, endNode)) {
                    vector<int> shortestPath = Graph_edge.findShortestPath(startNode, endNode);
                    finalPath.insert(finalPath.end(), shortestPath.begin(), shortestPath.end());
                } else {
                    finalPath.push_back(startNode);
                }
            }
            finalPath.push_back(path.back());
            return finalPath;
        }

        void attachWeight(Graph& osmGraph, vector<Pose6D_Uncertain_time>& global_uncertain_buffer , vector<std::pair<Pose6D_Uncertain_time, Node>>& visited_poses_and_nodes) 
        {
            vector<ClosestPoseInfo> closestPoses(osmGraph.nodes.size());
            for (size_t i = 0; i < osmGraph.nodes.size(); ++i) 
            {
                auto& node = osmGraph.nodes[i];

                closestPoses[i].distance = std::numeric_limits<float>::max();
                closestPoses[i].index = -1;
                closestPoses[i].isWithinThreshold = false;
                for (size_t j = 0; j < global_uncertain_buffer.size(); ++j) 
                {
                    float distance = std::hypot(global_uncertain_buffer[j].pose.x - node.x_coord, 
                                        global_uncertain_buffer[j].pose.y - node.y_coord);
                    if (distance < closestPoses[i].distance)
                    {
                        closestPoses[i].distance = distance;
                        closestPoses[i].index = j;
                    }
                }
                if (closestPoses[i].distance <= osm_pose_dis) {
                    closestPoses[i].isWithinThreshold = true;
                }
            }
            const float additional_distance = 5;
            std::vector<std::vector<int>> posesIndicesWithinRangeForEachNode(osmGraph.nodes.size());
            for (size_t i = 0; i < closestPoses.size(); ++i) {
                if (closestPoses[i].isWithinThreshold) {
                    float search_radius = closestPoses[i].distance + additional_distance; 
                    auto& node = osmGraph.nodes[i];
                    std::vector<int> posesWithinRange;

                    for (size_t j = 0; j < global_uncertain_buffer.size(); ++j) {
                        float distance = std::hypot(global_uncertain_buffer[j].pose.x - node.x_coord, 
                                            global_uncertain_buffer[j].pose.y - node.y_coord);
                        if (distance <= search_radius) {
                            posesWithinRange.push_back(j);
                        }
                    }
                    posesIndicesWithinRangeForEachNode[i] = posesWithinRange;

                }
            }
            std::vector<int> optimalPosesIndices(osmGraph.nodes.size(), -1);
            if (!closestPoses.empty() && closestPoses[0].isWithinThreshold) {
                optimalPosesIndices[0] = closestPoses[0].index;
            }

            for (size_t i = 1; i < osmGraph.nodes.size(); ++i) 
            {
                if (optimalPosesIndices[i - 1] == -1) continue;

                auto& previousOptimalPose = global_uncertain_buffer[optimalPosesIndices[i - 1]];
                double bestTimeDiff = std::numeric_limits<double>::max();
                int bestIndex = -1;

                for (int poseIndex : posesIndicesWithinRangeForEachNode[i]) 
                {
                    auto& currentPose = global_uncertain_buffer[poseIndex];
                    double timeDiff = currentPose.time - previousOptimalPose.time;

                    if (timeDiff < bestTimeDiff && timeDiff >= 0) {
                        bestTimeDiff = timeDiff;
                        bestIndex = poseIndex;
                    }
                }
                if (bestIndex != -1) {
                    int bestPoseIndexWithin10 = bestIndex;
                    float bestDistance = std::hypot(global_uncertain_buffer[bestIndex].pose.x - osmGraph.nodes[i].x_coord , global_uncertain_buffer[bestIndex].pose.y - osmGraph.nodes[i].y_coord);
                    int upperBoundIndex = std::min(bestIndex + 10, static_cast<int>(global_uncertain_buffer.size()) - 1);
                    for (int j = bestIndex + 1; j <= upperBoundIndex; ++j) {
                        auto& currentPose = global_uncertain_buffer[j];
                        float currentDistance = std::hypot(currentPose.pose.x - osmGraph.nodes[i].x_coord, currentPose.pose.y - osmGraph.nodes[i].y_coord);

                        if (currentDistance < bestDistance) {
                            bestDistance = currentDistance;
                            bestPoseIndexWithin10 = j;
                        }
                    }
                    optimalPosesIndices[i] = bestPoseIndexWithin10;
                }
            }

            for (size_t i = 0; i < osmGraph.nodes.size(); ++i) 
            {
                if (optimalPosesIndices[i] != -1) 
                {
                    osmGraph.nodes[i].visited = true;
                    osmGraph.nodes[i].d_opt = global_uncertain_buffer[optimalPosesIndices[i]].d_opt;
                    visited_poses_and_nodes.push_back(std::make_pair(global_uncertain_buffer[optimalPosesIndices[i]], osmGraph.nodes[i]));
                    std::cout << "Adding node " << i << " to visited_poses_and_nodes" << std::endl;
                }
            }
        }

        void UpdatedSpaceVisited(Graph& osmGraph, const std::vector<int>& CurrentPath)
        {
            std::set<int> nodesInPath(CurrentPath.begin(), CurrentPath.end());
            for (auto& node : osmGraph.nodes)
            {
                if (nodesInPath.find(node.second.id) != nodesInPath.end())
                {

                    node.second.actual_visited = true;
                }
            }
        }

        void publishPath(const std::vector<int> &path, const Graph &graph, const ros::Publisher &path_pub, float z) 
        {
            nav_msgs::Path path_msg;
            path_msg.header.frame_id = "camera_init"; 
            path_msg.header.stamp = ros::Time::now();
            for (int node_id : path) {
                geometry_msgs::PoseStamped pose_stamped;
                Node node = graph.nodes.at(node_id);
                pose_stamped.header = path_msg.header;
                pose_stamped.pose.position.x = node.x_coord;
                pose_stamped.pose.position.y = node.y_coord;
                pose_stamped.pose.position.z = z;
                pose_stamped.pose.orientation.x = 0.0;
                pose_stamped.pose.orientation.y = 0.0;
                pose_stamped.pose.orientation.z = 0.0;
                pose_stamped.pose.orientation.w = 1.0;

                path_msg.poses.push_back(pose_stamped);
            }
            path_pub.publish(path_msg);
        }

        void visualizeGraph ( const Graph &graph, ros::Publisher& graph_pub)
        {
            visualization_msgs::MarkerArray marker_array;
            int marker_id = 0;
            for (const auto& edge : graph.edges) {
                const Node& node_start = graph.nodes.at(edge.from);
                const Node& node_end = graph.nodes.at(edge.to);

                visualization_msgs::Marker line_marker;
                line_marker.header.frame_id = "camera_init"; 
                line_marker.header.stamp = ros::Time::now();
                line_marker.ns = "graph_edges";
                line_marker.id = marker_id++;
                line_marker.type = visualization_msgs::Marker::LINE_STRIP;
                line_marker.action = visualization_msgs::Marker::ADD;
                line_marker.scale.x = 0.5;
                line_marker.color.r = 1.0;
                line_marker.color.g = 1.0;
                line_marker.color.b = 1.0;
                line_marker.color.a = 1.0;

                geometry_msgs::Point p_start;
                p_start.x = node_start.x_coord;
                p_start.y = node_start.y_coord;
                p_start.z = 1;

                geometry_msgs::Point p_end;
                p_end.x = node_end.x_coord;
                p_end.y = node_end.y_coord;
                p_end.z = 1;

                line_marker.points.push_back(p_start);
                line_marker.points.push_back(p_end);

                marker_array.markers.push_back(line_marker);
            }

            graph_pub.publish(marker_array);
        }

        void publishArrowPathMarkers(const std::vector<int> &path, const Graph &graph, const ros::Publisher &arrow_marker_pub, float z) 
        {
            visualization_msgs::MarkerArray marker_array;
            std::map<std::pair<float, float>, float> z_map;
            const float arrow_length = 5.0;
            const float z_increment = 1.0;

            for (size_t i = 0; i < path.size() - 1; ++i)
            {
                visualization_msgs::Marker arrow_marker;
                arrow_marker.header.frame_id = "camera_init";
                arrow_marker.header.stamp = ros::Time::now();
                arrow_marker.ns = "path_arrows";
                arrow_marker.id = i;
                arrow_marker.type = visualization_msgs::Marker::ARROW;
                arrow_marker.action = visualization_msgs::Marker::ADD;

                arrow_marker.scale.x = 5.0;
                arrow_marker.scale.y = 10.0; 
                arrow_marker.scale.z = 5; 

                arrow_marker.color.r = 1.0; 
                arrow_marker.color.g = 1.0; 
                arrow_marker.color.b = 0.0; 
                arrow_marker.color.a = 1.0;


                Node current_node = graph.nodes.at(path[i]);
                Node next_node = graph.nodes.at(path[i + 1]);
                std::pair<float, float> mid_key = std::make_pair(
                    (current_node.x_coord + next_node.x_coord) / 2.0,
                    (current_node.y_coord + next_node.y_coord) / 2.0
                );
                float current_z = z_map.count(mid_key) ? z_map[mid_key] + z_increment : z;
                z_map[mid_key] = current_z;
                float dir_x = next_node.x_coord - current_node.x_coord;
                float dir_y = next_node.y_coord - current_node.y_coord;
                float dir_z = 0;
                float norm = sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
                dir_x /= norm;
                dir_y /= norm;
                dir_z /= norm;
                dir_x *= arrow_length;
                dir_y *= arrow_length;
                dir_z *= arrow_length; 

                geometry_msgs::Point start_point, end_point;
                start_point.x = (current_node.x_coord + next_node.x_coord) / 2.0 - dir_x / 2.0;
                start_point.y = (current_node.y_coord + next_node.y_coord) / 2.0 - dir_y / 2.0;
                start_point.z = current_z;
                end_point.x = start_point.x + dir_x;
                end_point.y = start_point.y + dir_y;
                end_point.z = current_z;

                arrow_marker.points.clear();
                arrow_marker.points.push_back(start_point);
                arrow_marker.points.push_back(end_point);
                marker_array.markers.push_back(arrow_marker);
            }

            arrow_marker_pub.publish(marker_array);
        }
        void publishTargetPoseAsArrow(const Node& max_j1_node, ros::Publisher& marker_pub) 
        {
            visualization_msgs::Marker arrow_marker;
            
            arrow_marker.header.frame_id = "camera_init";
            arrow_marker.header.stamp = ros::Time::now();

            arrow_marker.ns = "target_pose";
            arrow_marker.id = 0;

            arrow_marker.type = visualization_msgs::Marker::ARROW;

            arrow_marker.action = visualization_msgs::Marker::ADD;

            geometry_msgs::Point start_point;
            start_point.x = max_j1_node.x_coord;
            start_point.y = max_j1_node.y_coord;
            start_point.z = 10.0; 

            geometry_msgs::Point end_point = start_point;
            end_point.z = 8.0; 

            arrow_marker.points.push_back(start_point);
            arrow_marker.points.push_back(end_point);
            arrow_marker.scale.x = 0.4; 
            arrow_marker.scale.y = 0.8; 
            arrow_marker.scale.z = 0.8; 
            arrow_marker.color.r = 1.0; 
            arrow_marker.color.g = 1.0; 
            arrow_marker.color.b = 1.0; 
            arrow_marker.color.a = 1.0; 

            marker_pub.publish(arrow_marker);
        }

        void loadCSV(const string& filePath, vector<vector<string>>& data) 
        {
            ifstream file(filePath);
            string line;

            while (getline(file, line)) {
                stringstream lineStream(line);
                string cell;
                vector<string> parsedRow;
                while (getline(lineStream, cell, ',')) {
                    parsedRow.push_back(cell);
                }
                data.push_back(parsedRow);
            }
        }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "active_loop");
    ActiveLoop active_loop;
    ros::spin();
    return 0;
}
