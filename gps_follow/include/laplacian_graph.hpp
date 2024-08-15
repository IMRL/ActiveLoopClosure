#ifndef LAPLACIAN_GRAPH_HPP
#define LAPLACIAN_GRAPH_HPP

//读取OSM原图，不包含轨迹顺序，连接关系对应原来的先验图
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

struct Node {
    int id;
    double x_coord, y_coord;
    double d_opt;
    bool visited = false;//轨迹是否visted过，也就是重边的时候会按时间更新，而不是全部更新
    double distance = numeric_limits<double>::infinity(); // 最短路径估计初始化为无穷大
    bool actual_visited = false; //空间上没有到达过的
};

struct Edge {
    int from;
    int to;
    double count = 0;
    double weight;
    double dis_weight;
    bool isRequired = true; // RPP问题求解时是否为必走边

    Edge(int f, int t, double w, double dw) : from(f), to(t), weight(w), dis_weight(dw) {}

    bool operator==(const Edge& other) const {
        return (from == other.from && to == other.to) || (from == other.to && to == other.from);
    }
};


class Graph {
public:

    unordered_map<int, Node> nodes;
    // vector<Node> nodes;
    vector<Edge> edges;

    //添加节点相关：
    void addNode(int id, double x_coord, double y_coord) 
    {
        nodes[id] = {id, x_coord, y_coord};
    }

    void addNode(Node node)
    {
        nodes[node.id] = {node.id, node.x_coord, node.y_coord, node.d_opt, node.visited};
    }

    void addEdge(int from, int to, double weight, double dis_weight = 0, bool repeat = false)
    {
        if (!repeat) 
        {
            auto it = std::find_if(edges.begin(), edges.end(), [from, to](const Edge& edge) {
                return edge.from == from && edge.to == to;
            });

            if (it == edges.end()) {
                edges.emplace_back(from, to, weight, dis_weight);
            } else {

                it->weight = weight;
                it->dis_weight = dis_weight;
            }
        } 
        else 
        {

            edges.emplace_back(from, to, weight, dis_weight);
        }
    }
    int findMaxNodeid() const {
        int maxNodeId = -1;
        for (const auto& nodePair : nodes) {
            if (nodePair.second.id > maxNodeId) {
                maxNodeId = nodePair.second.id;
            }
        }
        return maxNodeId;
    }

    int findMinNodeId() {
        int minNodeId = std::numeric_limits<int>::max();
        for (const auto& nodePair : nodes) {
            if (nodePair.second.id < minNodeId) {
                minNodeId = nodePair.second.id;
            }
        }
        return minNodeId;
    }

    int findNodeIdByCoordinates(float x, float y) {
        for(const auto& node : nodes)
        {
            if (areCoordinatesEqual(node.second.x_coord, x) && areCoordinatesEqual(node.second.y_coord, y)) {
                return node.second.id;
            }
        }
        return -1; 
    }
    unordered_map<int, list<Edge>> getEdgeMap() const 
    {
        unordered_map<int, list<Edge>> edgeMap;
        for (const Edge& edge : edges) {
            edgeMap[edge.from].push_back(edge);
            Edge reverseEdge = edge;
            std::swap(reverseEdge.from, reverseEdge.to);
            edgeMap[edge.to].push_back(reverseEdge);
        }

        return edgeMap;
    }
    std::unordered_set<int> getNeighbors(int nodeId) {
        std::unordered_set<int> neighbors;
        for (const Edge& edge : edges) {
            if (edge.from == nodeId) {
                neighbors.insert(edge.to);
            } else if (edge.to == nodeId) {
                neighbors.insert(edge.from);
            }
        }
        return neighbors;
    }
    void increaseEdgeCount(int from, int to) {
    auto it = std::find_if(edges.begin(), edges.end(), [from, to](const Edge& edge) {
        return (edge.from == from && edge.to == to) || (edge.from == to && edge.to == from);
    });
    
    if (it != edges.end()) {
        it->count += 1;
    } 
    }
    void calculateEdgeWeights() {
        for (Edge& edge : edges) {
            if (edge.count > 0) {
                edge.weight = sqrt(log(edge.count+1)) * edge.dis_weight;
            } else {
                edge.weight = 0; 
            }
        }
    }

    bool hasNode(int id) const {
    return nodes.find(id) != nodes.end();
    }

    bool hasEdge(int from, int to) const {
        return std::any_of(edges.begin(), edges.end(), [from, to](const Edge& edge) {
            return (edge.from == from && edge.to == to) || (edge.from == to && edge.to == from);
        });
    }

    void addNodeIfNotExists(int id, double x_coord = 0.0, double y_coord = 0.0, double d_opt = 0.0) {
        if (!hasNode(id)) {
            addNode(id, x_coord, y_coord);
        }
    }

    void addEdgeIfNotExists(int from, int to, double weight, double dis_weight = 0.0, bool isRequired = true) {
        if (!hasEdge(from, to)) {
            addEdge(from, to, weight, dis_weight, isRequired);
        }
    }

    // Edge getEdge(int from_id, int to_id) {
    //     auto it = std::find_if(edges.begin(), edges.end(), [from_id, to_id](const Edge& edge) {
    //         return edge == Edge(from_id, to_id, 0,0);
    //     });

    //     if(it != edges.end()) {
    //         return *it;
    //     } else {
    //         // 如果没有找到边，抛出异常
    //         throw std::runtime_error("Edge not found");
    //     }
    // }
    Edge getEdge(int from_id, int to_id) 
    {
        auto it = std::find_if(edges.begin(), edges.end(), [from_id, to_id](const Edge& edge) {
            return (edge.from == from_id && edge.to == to_id) || (edge.from == to_id && edge.to == from_id);
        });

        if (it != edges.end()) {
            return *it;
        } else {
            throw std::runtime_error("Edge not found");
        }
    }
    void updateEdgeRequirements() 
    {
        for (auto& edge : edges) {
            if (nodes[edge.from].actual_visited && nodes[edge.to].actual_visited) {
                edge.isRequired = false;
            }
        }
    }
    void removeEdge(int from, int to) {
        auto it = edges.begin();
        while (it != edges.end()) {
            if (it->from == from && it->to == to) {
                it = edges.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    void clear() 
    {
        nodes.clear(); 
        edges.clear(); 
    }
    std::vector<int> getNodes() const {
        std::vector<int> node_ids;
        for (const auto& node_pair : nodes) {
            node_ids.push_back(node_pair.first);
        }
        return node_ids;
    }

    int getDegree(int node_id) const {
        if (nodes.find(node_id) == nodes.end()) {
            return 0;
        }
        int degree = 0;
        for (const auto& edge : edges) {
            if (edge.from == node_id) {
                degree++;
                if (edge.from == edge.to) {
                    degree++;
                }
            } else if (edge.to == node_id) {
                degree++;
            }
        }
        return degree;
    }

    void updateEdgeWeight(int from, int to, double newWeight) 
    {
        bool edgeUpdated = false;
        for (Edge& edge : edges) {
            if (edge.from == from && edge.to == to) {
                edge.weight = newWeight;
                edgeUpdated = true;
                break;
            }
        }
        if (!edgeUpdated) {
            cerr << "Edge not found from " << from << " to " << to << ". Adding new edge." << endl;
        }
    }
    double pathLenSum() const {
        double totalLength = 0.0;

        for (const Edge& edge : edges) {
            const Node& nodeFrom = nodes.at(edge.from);
            const Node& nodeTo = nodes.at(edge.to);

            double dx = nodeFrom.x_coord - nodeTo.x_coord;
            double dy = nodeFrom.y_coord - nodeTo.y_coord;
            double distance = sqrt(dx * dx + dy * dy);

            totalLength += distance;
        }
        return totalLength;
    }
    vector<int> findShortestPath(int startId, int endId) 
    {
        unordered_map<int, double> distances;
        unordered_map<int, int> predecessors;
        auto compare = [](const pair<int, double>& p1, const pair<int, double>& p2) {
            return p1.second > p2.second;
        };
        priority_queue<pair<int, double>, vector<pair<int, double>>, decltype(compare)> pq(compare);
        for (auto& node : nodes) {
            distances[node.first] = numeric_limits<double>::infinity();
        }
        distances[startId] = 0.0;
        pq.push({startId, 0.0});
        while (!pq.empty()) 
        {
            int currentId = pq.top().first;
            pq.pop();

            if (currentId == endId) break;
            for (const Edge& edge : edges) 
            {        
                if (edge.from == currentId || edge.to == currentId) {
                    int neighborId = (edge.from == currentId) ? edge.to : edge.from;
                    double newDist = distances[currentId] + edge.dis_weight;
                    if (newDist < distances[neighborId]) {
                        distances[neighborId] = newDist;
                        pq.push({neighborId, newDist});
                        predecessors[neighborId] = currentId;
                    }
                }
            }
        }
        return reconstructPath(predecessors, startId, endId);
    }
    vector<int> findBestPath(int startId, int endId, double visitedCoefficient) {
        unordered_map<int, double> distances;
        unordered_map<int, int> predecessors;
        
        auto compare = [](const pair<int, double>& p1, const pair<int, double>& p2) {
            return p1.second > p2.second;
        };
        
        priority_queue<pair<int, double>, vector<pair<int, double>>, decltype(compare)> pq(compare);
        for (const auto& node : nodes) {
            distances[node.first] = numeric_limits<double>::infinity();
        }
        distances[startId] = 0.0;
        pq.push({startId, 0.0});

        while (!pq.empty()) {
            int currentId = pq.top().first;
            pq.pop();
            if (currentId == endId) break;
            for (const Edge& edge : edges) {
                if (edge.from == currentId || edge.to == currentId) {
                    int neighborId = (edge.from == currentId) ? edge.to : edge.from;
                    double weight = edge.dis_weight;
                    if (nodes[edge.from].actual_visited && nodes[edge.to].actual_visited) {
                        weight *= visitedCoefficient;
                    }
                    
                    double newDist = distances[currentId] + weight;
                    if (newDist < distances[neighborId]) {
                        distances[neighborId] = newDist;
                        predecessors[neighborId] = currentId;
                        pq.push({neighborId, newDist});
                    }
                }
            }
            nodes[currentId].visited = true;
        }

        return reconstructPath(predecessors, startId, endId);
    }

    MatrixXd computeLaplacian() const 
    {
        MatrixXd L = MatrixXd::Zero(nodes.size(), nodes.size());
        for (const Edge& e : edges) 
        {
            try 
            {
                int fromIndex =e.from;
                int toIndex = e.to;

                L(fromIndex, toIndex) = -e.weight;
                L(toIndex, fromIndex) = -e.weight;
                L(fromIndex, fromIndex) += e.weight;
                L(toIndex, toIndex) += e.weight;
            } catch (const std::out_of_range& ex) 
            {
                cerr << "Out of Range error: " << ex.what() << '\n';
                cerr << "An edge is referencing a non-existent node ID." << endl;
            }
        }
        return L;
    }

    double calculateDOptimality(const MatrixXd& L) 
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(L);
        if (eigensolver.info() != Eigen::Success) {
            cerr << "Eigenvalue computation failed!" << endl;
            return -1.0;
        }

        Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();
        double product_nonzero_eigenvalues = 1.0;
        for (int i = 1; i < eigenvalues.size(); ++i) {
            if (eigenvalues(i) > numeric_limits<double>::epsilon()) {
                product_nonzero_eigenvalues *= eigenvalues(i);
            }
        }

        if (product_nonzero_eigenvalues <= 0) {
            cerr << "Non-positive product of eigenvalues encountered." << endl;
            return -1.0;
        }

        return pow(product_nonzero_eigenvalues, 1.0 / (eigenvalues.size() - 1));
    }

private:

    vector<int> reconstructPath(unordered_map<int, int>& predecessors, int startId, int endId) 
    {
        list<int> path;
        int currentId = endId;

        path.push_front(currentId);

        while (currentId != startId) {
            if (predecessors.find(currentId) == predecessors.end()) {
                return vector<int>();
            }
            currentId = predecessors[currentId];
            path.push_front(currentId);
        }

        return vector<int>(path.begin(), path.end());
    }

    bool areCoordinatesEqual(float a, float b) {
        return std::fabs(a - b) < std::numeric_limits<float>::epsilon();
    }
};



#endif // LAPLACIAN_GRAPH_HPP