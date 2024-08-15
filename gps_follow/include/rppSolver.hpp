#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <queue>
#include <stack>
#include <limits>
#include <algorithm>
#include "laplacian_graph.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/maximum_weighted_matching.hpp>
#include <utility> // for std::pair
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/connected_components.hpp>
#include <stdexcept>


class RuralPostmanProblemSolver {
public:
    // RuralPostmanProblemSolver(const std::string& filename);
    // std::pair<std::list<Edge>, Graph> solveRPP(int startNode = -1);
    std::pair<std::list<Edge>, Graph> solveRPP(Graph graph_full , int startNode, int endNode);
private:
    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator() (const std::pair<T1, T2>& pair) const {
            auto hash1 = std::hash<T1>{}(pair.first);
            auto hash2 = std::hash<T2>{}(pair.second);
            return hash1 ^ hash2;
        }
    };

    // void readGraphFromFile(const std::string& filename);
    Graph createRequiredGraph(Graph& graph);
    bool isConnected(const Graph& graph);
    std::vector<int> getOddDegreeNodes(const Graph& graph); 
    std::vector<std::pair<int, int>> getAllNodePairs(const std::vector<int>& nodes); 
    std::unordered_map<std::pair<int, int>, double, pair_hash> computeShortestPaths(Graph& graph, const std::vector<std::pair<int, int>>& nodePairs); //使用dijkstra获取最短路径（或者floyd-warshall)
    Graph createCompleteGraph( Graph& graph,const std::unordered_map<std::pair<int, int>, double, pair_hash>& shortestPaths); //结合最短路径构建完整图
    std::unordered_set<std::pair<int, int>, pair_hash> findMinimumWeightMatching(const Graph& completeGraph);//寻找最小权重完美匹配（blosssom）
    Graph addAugmentingPathsToGraph(Graph graph_req, Graph graph_full ,const std::unordered_set<std::pair<int, int>, pair_hash>& matching);//添加增广路径

    std::list<Edge> createEulerianCircuit(Graph& graph_aug,int startNode);

    std::list<Edge> createEulerianPath(Graph& graph_aug,int startNode,int endnode);
    bool NodeInGraph(const Graph& graph, int nodeId)
    {
        return graph.nodes.find(nodeId) != graph.nodes.end();
    }
    void connectNodeToGraph(Graph& requiredGraph, Graph& graph_full, int startNode);
    // Graph ConnectIsoNodeToGraph(Graph )
    std::vector<std::unordered_set<int>> identifyDisconnectedSubgraphs(Graph& graph);
    double getPathLength(const Graph& graph, const std::vector<int>& path);
    void connectSubgraphs(Graph& requiredGraph, Graph& graph_full);
};
