#include "rppSolver.hpp"
#include <iomanip> 
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                            boost::no_property, boost::property<boost::edge_weight_t, int>> BoostGraph;
typedef boost::graph_traits<BoostGraph>::vertex_descriptor Vertex;
typedef boost::graph_traits<BoostGraph>::edge_descriptor Edge_BGL;
typedef boost::property_map<BoostGraph, boost::edge_weight_t>::type WeightMap;
typedef std::map<int, int> VertexMap;

bool printmsg = true; 
bool debug = true;

Graph RuralPostmanProblemSolver::createRequiredGraph(Graph& graph) {
    Graph requiredGraph;
    for (const auto& edge : graph.edges) { 
        if (edge.isRequired && edge.weight!=100) { 
            requiredGraph.addEdge(edge.from, edge.to, edge.weight, edge.dis_weight);
            requiredGraph.addNode(graph.nodes[edge.from]);
            requiredGraph.addNode(graph.nodes[edge.to]);
        }
    }

    return requiredGraph;

}

bool RuralPostmanProblemSolver::isConnected(const Graph& graph)
{
    if (graph.nodes.empty()) {
        return true; 
    }

    std::unordered_map<int, bool> visited;
    std::queue<int> to_visit;

    auto start_node = graph.nodes.begin()->first;
    to_visit.push(start_node);
    visited[start_node] = true;

    while (!to_visit.empty()) {
        int current = to_visit.front();
        to_visit.pop();

        for (const auto& edge : graph.edges) {
            if (edge.from == current && !visited[edge.to]) {
                visited[edge.to] = true;
                to_visit.push(edge.to);
            } else if (edge.to == current && !visited[edge.from]) {
                visited[edge.from] = true;
                to_visit.push(edge.from);
            }
        }
    }

    for (const auto& node : graph.nodes) {
        if (visited.find(node.first) == visited.end()) {
            return false;
        }
    }
    if(printmsg)
    {
        std::cout << "Graph is connected !!!!" << std::endl;
    }

    return true;
}

std::vector<int> RuralPostmanProblemSolver::getOddDegreeNodes(const Graph& graph) {
    std::vector<int> oddDegreeNodes;
    for (const auto& nodePair : graph.nodes) 
    { 
        int nodeID = nodePair.first;
        int degree = graph.getDegree(nodeID);
        if (degree % 2 == 1) {
            oddDegreeNodes.push_back(nodeID);
            if(debug)
            {
                std::cout << "Odd degree node ID: " << nodeID << " with degree " << degree << std::endl;
            }
            
        }
    }
    if(debug)
    {
    std::cout << "Odd degree nodes count: " << oddDegreeNodes.size() << std::endl;
    }
    return oddDegreeNodes;
}

double euclideanDistance(const Node& a, const Node& b) 
{
    return std::sqrt(std::pow(a.x_coord - b.x_coord, 2) + std::pow(a.y_coord - b.y_coord, 2));
}

std::vector<std::pair<int, int>> RuralPostmanProblemSolver::getAllNodePairs(const std::vector<int>& nodes) 
{
    std::vector<std::pair<int, int>> nodePairs;

    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = i + 1; j < nodes.size(); ++j) {
            nodePairs.emplace_back(nodes[i], nodes[j]);
        }
    }
    //Debug---
    if(debug)
    {
        std::cout << "Node Pairs:" << std::endl;
        for (const auto& pair : nodePairs) {
            std::cout << "(" << pair.first << ", " << pair.second << ")" << std::endl;
        }
    }


    return nodePairs;
}

// 使用 Dijkstra 或 Floyd-Warshall 算法计算最短路径
std::unordered_map<std::pair<int, int>, double, RuralPostmanProblemSolver::pair_hash> RuralPostmanProblemSolver::computeShortestPaths(Graph& graph, const std::vector<std::pair<int, int>>& nodePairs) 
{
    std::unordered_map<std::pair<int, int>, double, pair_hash> shortestPaths;
    for (const auto& pair : nodePairs) {
        vector<int> path = graph.findShortestPath(pair.first, pair.second);
        if (path.empty()) {
            shortestPaths[pair] = std::numeric_limits<double>::infinity();
        } else {
            double path_length = 0.0;
            for (size_t i = 0; i < path.size() - 1; ++i) {
                Node from = graph.nodes.at(path[i]);
                Node to = graph.nodes.at(path[i + 1]);
                path_length += euclideanDistance(from, to);
            }
            shortestPaths[pair] = path_length;
        }
    }
    //Debug:_____
    if(debug)
    {
    std::cout << "Shortest Paths Lengths:" << std::endl;
    for (const auto& pair_path : shortestPaths) {
        std::cout << "(" << pair_path.first.first << ", " << pair_path.first.second << "): "
                  << pair_path.second << std::endl;
    }
    }
    return shortestPaths;
}


Graph RuralPostmanProblemSolver::createCompleteGraph( Graph& graph, const std::unordered_map<std::pair<int, int>, double, pair_hash>& shortestPaths)
{
    Graph completeGraph;

    for (const auto& entry : shortestPaths) {
        const auto& nodePair = entry.first;
        double dis_weight = entry.second;
        if (completeGraph.nodes.find(nodePair.first) == completeGraph.nodes.end()) {
            Node fromNode = graph.nodes[nodePair.first];
            completeGraph.addNode(fromNode);
        }
        if (completeGraph.nodes.find(nodePair.second) == completeGraph.nodes.end()) {
            Node toNode = graph.nodes[nodePair.second];
            completeGraph.addNode(toNode);
        }
        
        completeGraph.addEdge(nodePair.first, nodePair.second, 0, dis_weight);
    }

    return completeGraph;
}


std::unordered_set<std::pair<int, int>, RuralPostmanProblemSolver::pair_hash> RuralPostmanProblemSolver::findMinimumWeightMatching(const Graph& completeGraph)
{

    BoostGraph boostGraph(completeGraph.nodes.size());
    for (const auto& edge : completeGraph.edges) {
        Edge_BGL e;
        bool inserted;
        boost::tie(e, inserted) = boost::add_edge(edge.from, edge.to, boostGraph);
        if (inserted) {
            boost::put(boost::get(boost::edge_weight, boostGraph), e, 500-edge.dis_weight);
        }
    }
    std::vector<Vertex> mate(boost::num_vertices(boostGraph));
    boost::maximum_weighted_matching(boostGraph, &mate[0]);
    std::unordered_set<std::pair<int, int>, RuralPostmanProblemSolver::pair_hash> matching;
    for (std::size_t i = 0; i < mate.size(); ++i) {
        if (mate[i] != BoostGraph::null_vertex() && i < mate[i]) {
            matching.insert({static_cast<int>(i), static_cast<int>(mate[i])});
        }
    }
    if(printmsg)
    {
        std::cout << "| The Augmenting side that needs to be traveled repeatedly Matching pairs: |" << std::endl;
        for (const auto& pair : matching) {
            std::cout << "(" << pair.first << "-> " << pair.second << ")" << "   ";
        }
        std::cout << std::endl;
    }

    return matching;
}

    
Graph RuralPostmanProblemSolver::addAugmentingPathsToGraph(Graph graph_req, Graph graph_full, const std::unordered_set<std::pair<int, int>, pair_hash>& matching) 
{
    if(printmsg)
    {
        std::cout << "| ------------ Add Augmenting Paths To Graph ------------ |" << std::endl;
    }
    for (const auto& match : matching) {
        auto node_ids_path = graph_full.findShortestPath(match.first, match.second);

        for (size_t i = 0; i < node_ids_path.size(); ++i) {
            int node_id = node_ids_path[i];
            if(!graph_req.hasNode(node_id))
            {
                graph_req.addNode(graph_full.nodes.at(node_id)); 
            }
            
        }

        for (size_t i = 0; i < node_ids_path.size() - 1; ++i) {
            int from_id = node_ids_path[i];
            int to_id = node_ids_path[i + 1];
            auto edge = graph_full.getEdge(from_id, to_id);
            graph_req.addEdge(edge.from, edge.to, edge.weight, edge.dis_weight,true);
        }
    }

    Graph graph_aug = graph_req;
    return graph_aug;
}

void removeEdge(std::unordered_map<int, std::list<Edge>>& remainingEdges, const Edge& edgeToRemove) {
    remainingEdges[edgeToRemove.from].remove_if([&edgeToRemove](const Edge& e) {
        return (e.from == edgeToRemove.from && e.to == edgeToRemove.to) || 
               (e.from == edgeToRemove.to && e.to == edgeToRemove.from);
    });
    remainingEdges[edgeToRemove.to].remove_if([&edgeToRemove](const Edge& e) {
        return (e.from == edgeToRemove.from && e.to == edgeToRemove.to) || 
               (e.from == edgeToRemove.to && e.to == edgeToRemove.from);
    });
}

std::vector<std::unordered_set<int>> RuralPostmanProblemSolver::identifyDisconnectedSubgraphs(Graph& graph) {
    std::vector<std::unordered_set<int>> disconnectedSubgraphs;
    std::unordered_set<int> visitedNodes;
    std::queue<int> bfsQueue;
    for (const auto& node : graph.nodes) {
        if (visitedNodes.find(node.second.id) == visitedNodes.end()) {
            std::unordered_set<int> subgraph;
            bfsQueue.push(node.second.id);
            visitedNodes.insert(node.second.id);
            while (!bfsQueue.empty()) {
                int currentNode = bfsQueue.front();
                bfsQueue.pop();
                subgraph.insert(currentNode);
                auto neighbors = graph.getNeighbors(currentNode);
                for (const auto& neighbor : neighbors) {
                    if (visitedNodes.find(neighbor) == visitedNodes.end()) {
                        bfsQueue.push(neighbor);
                        visitedNodes.insert(neighbor);
                    }
                }
            }
            disconnectedSubgraphs.push_back(subgraph);
        }
    }

    return disconnectedSubgraphs;
}

double RuralPostmanProblemSolver::getPathLength(const Graph& graph, const std::vector<int>& path) {
    double path_length = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        Node from = graph.nodes.at(path[i]);
        Node to = graph.nodes.at(path[i + 1]);
        path_length += euclideanDistance(from, to);
    }
    return path_length;
}

void RuralPostmanProblemSolver::connectSubgraphs(Graph& requiredGraph, Graph& graph_full) {
    auto disconnectedSubgraphs = identifyDisconnectedSubgraphs(requiredGraph);
    if(debug)
    {
        std::cout << "Disconnected Subgraphs:" << std::endl;
        for (size_t i = 0; i < disconnectedSubgraphs.size(); ++i) {
            std::cout << "Subgraph " << i + 1 << ": ";
            for (int node : disconnectedSubgraphs[i]) {
                std::cout << node << " ";
            }
            std::cout << std::endl;
        }
    }

    while (disconnectedSubgraphs.size() > 1) {
        double minDistance = std::numeric_limits<double>::infinity();
        std::vector<int> shortestPath;
        int bestFromSubgraphIndex = -1;
        int bestToSubgraphIndex = -1;
        for (size_t i = 0; i < disconnectedSubgraphs.size(); i++) {
            for (size_t j = i + 1; j < disconnectedSubgraphs.size(); j++) {
                for (int fromNode : disconnectedSubgraphs[i]) {
                    for (int toNode : disconnectedSubgraphs[j]) {
                        std::vector<int> currentPath = graph_full.findShortestPath(fromNode, toNode);
                        double currentDistance = getPathLength(graph_full, currentPath);

                        if (currentDistance < minDistance) {
                            minDistance = currentDistance;
                            shortestPath = currentPath;
                            bestFromSubgraphIndex = i;
                            bestToSubgraphIndex = j;
                        }
                    }
                }
            }
        }
        for (size_t i = 0; i < shortestPath.size(); ++i) {
            int node_id = shortestPath[i];
            if(!requiredGraph.hasNode(node_id))
            {
                requiredGraph.addNode(graph_full.nodes.at(node_id)); 
            }
            
        }
        for (size_t i = 0; i < shortestPath.size() - 1; ++i) {
            int from = shortestPath[i];
            int to = shortestPath[i + 1];
            Edge edge = graph_full.getEdge(from, to);
            double weight = edge.weight;
            double dis_weight = edge.dis_weight;
            if(debug)
            {
            std::cout << "Edge from " << from << " to " << to << " with weight " << weight 
                      << " and dis_weight " << dis_weight << std::endl;
            }
            requiredGraph.addEdge(from, to, weight, dis_weight);
            
        }
        disconnectedSubgraphs[bestFromSubgraphIndex].insert(
            disconnectedSubgraphs[bestToSubgraphIndex].begin(),
            disconnectedSubgraphs[bestToSubgraphIndex].end());
        disconnectedSubgraphs.erase(disconnectedSubgraphs.begin() + bestToSubgraphIndex);
    }
}

void RuralPostmanProblemSolver:: connectNodeToGraph(Graph& requiredGraph, Graph& graph_full, int startNode) {
    double minDistance = std::numeric_limits<double>::infinity();
    std::vector<int> shortestPath;
    for (const auto& node : requiredGraph.nodes) {
        std::vector<int> currentPath = graph_full.findShortestPath(startNode, node.second.id);
        double currentDistance = 0.0;
        for (size_t i = 0; i < currentPath.size() - 1; ++i) {
            Node from = graph_full.nodes.at(currentPath[i]);
            Node to = graph_full.nodes.at(currentPath[i + 1]);
            currentDistance += euclideanDistance(from, to);
        }
        if (currentDistance < minDistance) {
            minDistance = currentDistance;
            shortestPath = currentPath;
        }
    }
    if (minDistance == std::numeric_limits<double>::infinity()) {
        throw std::runtime_error("No path found from start node to required graph.");
    }

    for (size_t i = 0; i < shortestPath.size(); ++i) 
    {
        int node_id = shortestPath[i];
        if(!requiredGraph.hasNode(node_id))
        {
            requiredGraph.addNode(graph_full.nodes.at(node_id)); 
        }
        
    }
    for (size_t i = 0; i < shortestPath.size() - 1; ++i) {
        int from = shortestPath[i];
        int to = shortestPath[i + 1];
        Edge edge = graph_full.getEdge(from, to);
        double weight = edge.weight;
        double dis_weight = edge.dis_weight;
        requiredGraph.addEdge(from, to, weight, dis_weight);
    }
}

void find_and_print_mismatched_vertices(const BoostGraph& g_copy, const Graph& graph_aug) {
    vector<int> mismatched_vertices;
    for (auto vp = vertices(g_copy); vp.first != vp.second; ++vp.first) {
        int vertex_id = *vp.first;
        if (graph_aug.nodes.find(vertex_id) == graph_aug.nodes.end()) {
            mismatched_vertices.push_back(vertex_id);
        }
    }
    for (const auto& node_pair : graph_aug.nodes) {
        int node_id = node_pair.first;
        auto v_descriptor = boost::vertex(node_id, g_copy);
        if (v_descriptor == boost::graph_traits<BoostGraph>::null_vertex()) {
            mismatched_vertices.push_back(node_id);
        }
    }
    for (int vid : mismatched_vertices) {
        std::cout << "不匹配的顶点 ID: " << vid << std::endl;
    }
}
void print_zero_degree_vertices_in_graph_aug(const BoostGraph& g_copy, const Graph& graph_aug) {
    for (auto vp = vertices(g_copy); vp.first != vp.second; ++vp.first) {
        auto vertex_degree = out_degree(*vp.first, g_copy);
        if (vertex_degree == 0) {
            for (const auto& node_pair : graph_aug.nodes) {
                if (node_pair.second.id == *vp.first) 
                { 
                    std::cout << "度数为0的节点 ID: " << node_pair.first << " 在 graph_aug 中" << std::endl;
                    break;
                }
            }
        }
    }
}
std::list<Edge> RuralPostmanProblemSolver::createEulerianCircuit(Graph& graph_aug, int startnode) 
{
    BoostGraph g(graph_aug.nodes.size());
    for (const auto& edge : graph_aug.edges) {
        Edge_BGL e;
        bool inserted;
        boost::tie(e, inserted) = boost::add_edge(edge.from, edge.to, g);
        if (inserted) {
            boost::put(boost::get(boost::edge_weight, g), e, edge.dis_weight);
        }
    }
    std::list<Edge_BGL> circuit;
    BoostGraph g_copy = g;
    if (startnode < 0 || static_cast<std::size_t>(startnode) >= num_vertices(g)) {
        throw std::invalid_argument("Invalid start node index");
    }
    Vertex start_vertex = vertex(startnode, g);
    print_zero_degree_vertices_in_graph_aug(g_copy,graph_aug);
    while (num_edges(g_copy) > 0) {
        Vertex u = start_vertex;
        if (out_degree(u, g_copy) == 0) 
        {
            for (auto vp = vertices(g_copy); vp.first != vp.second; ++vp.first) 
            {
                if (out_degree(*vp.first, g_copy) > 0) {
                    u = *vp.first;
                    break;
                }
            }
        }
        std::list<Edge_BGL> temp_circuit;
        Vertex current = u;
        do {
            auto ep = out_edges(current, g_copy);
            if (ep.first == ep.second) {
                 std::cout<<"test 22222222222222222222 " <<std::endl;
                  break; 

            } else {
                Edge_BGL e = *ep.first;
            temp_circuit.push_back(e);
            remove_edge(e, g_copy);
            current = target(e, g_copy) == current ? source(e, g_copy) : target(e, g_copy);
            }
        } while (current != u);
        if (circuit.empty()) {
            circuit.swap(temp_circuit);
        } else {
            auto it = std::find_if(circuit.begin(), circuit.end(), [&](const Edge_BGL& e) {
                return source(e, g) == u || target(e, g) == u;
            });
            circuit.splice(it, temp_circuit);
        }
    }
    std::list<Edge> custom_circuit;
    for (const Edge_BGL& e_bgl : circuit) {
        int s = source(e_bgl, g);
        int t = target(e_bgl, g);
        int weight_bgl = get(boost::edge_weight, g, e_bgl);
        double weight = 0;
        double dis_weight = weight_bgl; 
        Edge custom_edge(s, t, weight, dis_weight);

        custom_circuit.push_back(custom_edge);
    }
    return custom_circuit;
}

std::list<Edge> RuralPostmanProblemSolver::createEulerianPath(Graph& graph_aug, int startnode, int endnode) {

        graph_aug.addEdge(startnode, endnode, 0, 0);



        std::list<Edge> eulerian_circuit = RuralPostmanProblemSolver::createEulerianCircuit(graph_aug, startnode);

        auto it = std::find_if(eulerian_circuit.begin(), eulerian_circuit.end(),
            [startnode, endnode](const Edge& edge) {
                return (edge.from == startnode && edge.to == endnode) || (edge.from == endnode && edge.to == startnode);
            });

        if (it != eulerian_circuit.end()) {
            std::rotate(eulerian_circuit.begin(), it, eulerian_circuit.end());
        }

        if (!eulerian_circuit.empty() && 
            ((eulerian_circuit.front().from == startnode && eulerian_circuit.front().to == endnode) || 
             (eulerian_circuit.front().from == endnode && eulerian_circuit.front().to == startnode))) {
            eulerian_circuit.pop_front();
        }

        return eulerian_circuit;
    }

void printEdgeRow(const Edge& edge) {
    std::cout << "| " << std::setw(10) << edge.from << " | "
              << std::setw(10) << edge.to << " | "
              << std::setw(15) << edge.weight << " |\n";
}

void printTableHeader() {
    std::cout << "+------------+------------+-----------------+\n";
    std::cout << "| Start Node | End Node   | Edge Weight     |\n";
    std::cout << "+------------+------------+-----------------+\n";
}

void printTableFooter(size_t pathSize) {
    std::cout << "+------------+------------+-----------------+\n";
    std::cout << "| Path Size: " << std::setw(24) << pathSize << " |\n";
    std::cout << "+------------+------------+-----------------+\n";
}

std::pair<std::list<Edge>, Graph> RuralPostmanProblemSolver::solveRPP(Graph graph_full,int startNode,  int endNode) 
{
    Graph requiredGraph;
    requiredGraph =  createRequiredGraph(graph_full);
    //两个corner case
    //1.起始点不在 required graph 中
    //2.requiredgraph是断开的
    if(printmsg)
    {
        std::cout << "+-----------------------------------------------------------------------------+"<<std::endl;
        std::cout << "| ------------------- Begin To Solve Rural Postman Problem -------------------|" <<std::endl;

        std::cout << "| Start Node: " <<  startNode <<"End Node: " << endNode << " |"<< std::endl;

        std::cout<<"| Requied Graph has been build And have " << requiredGraph.nodes.size() << " nodes  and  " << requiredGraph.edges.size() << "edges |" <<std::endl;
        for (const auto& edge : requiredGraph.edges) 
        {
        std::cout << "| requiredGraph  Edge from node " << edge.from << " to node " << edge.to << " edge weight " << edge.weight << " edge dis_weight " << edge.dis_weight<<   " |" << std::endl;
        }
    }
    if (!isConnected(requiredGraph)) 
    {
        connectSubgraphs(requiredGraph,graph_full);
    }

    if(!requiredGraph.hasNode(startNode))
    {
        if(printmsg)
        {
            std::cout << " Start Node is not in required graph" <<std::endl;
        }
        connectNodeToGraph(requiredGraph,graph_full,startNode);
    }
    if(!requiredGraph.hasNode(endNode))
    {
        if(printmsg)
        {
            std::cout << " EndNode is not in required graph" <<std::endl;
        }
        connectNodeToGraph(requiredGraph,graph_full,endNode);
    }
    if (isConnected(requiredGraph)&&requiredGraph.hasNode(startNode)) 
    {
        if(printmsg)
        {
            std::cout<< "Required Graph is connected and contain startNode" <<std::endl;
        }
    }

    requiredGraph.addEdge(startNode,endNode,0,0);
    //Debug---
    if(debug)
    {
    for (const auto& edge : requiredGraph.edges) 
    {
        std::cout << "requiredGraph  Edge from node " << edge.from << " to node " << edge.to << " edge weight" << edge.weight << "edge dis_weight" << edge.dis_weight<< std::endl;
    }
    }
    auto oddDegreeNodes =  getOddDegreeNodes(requiredGraph);
    auto nodePairs = getAllNodePairs(oddDegreeNodes);
    auto shortestPaths = computeShortestPaths(graph_full, nodePairs);
    auto completeGraph = createCompleteGraph(graph_full, shortestPaths);
    auto matching = findMinimumWeightMatching(completeGraph);
    if(debug)
    {
    std::cout<<"Before add aug path graph have node size : " << requiredGraph.nodes.size() << " edge size : " << requiredGraph.edges.size() <<std::endl; 
    }
    Graph graph_aug;
    graph_aug = addAugmentingPathsToGraph(requiredGraph,graph_full, matching);
    if(debug)
    {
    std::cout<<" After Aug Requied Graph has been build And have " << graph_aug.nodes.size() << " nodes  and  " << graph_aug.edges.size() << "edges" <<std::endl;
    for (const auto& edge : graph_aug.edges) {
    std::cout << "requiredGraph  Edge from node " << edge.from << " to node " << edge.to << " edge weight" << edge.weight << "edge dis_weight" << edge.dis_weight<< std::endl;
    }
    }

    graph_aug.removeEdge(startNode, endNode);
    if (!isConnected(graph_aug)) {
        throw std::runtime_error("The graph must be connected to find a Rural Postman path.");
    }
    auto oddDegreeNodes_aug =  getOddDegreeNodes(graph_aug);
    if(oddDegreeNodes_aug.empty())
    {
        std::cout<< "graph_aug is euler" <<std::endl;
    }
    else
    {
        std::cout<< "graph_aug is not euler" <<std::endl;
    }
    // auto oddDegreeNodes_full =  getOddDegreeNodes(graph_full);
    // if(oddDegreeNodes_full.empty())
    // {
    //     std::cout<< "graph_full is euler" <<std::endl;
    // }
    // else
    // {
    //     std::cout<< "graph_full is not euler" <<std::endl;
    // }

    if(startNode==endNode)
    {
        std::list<Edge> eulerianCircuit = createEulerianCircuit(graph_aug  , startNode);
        if(printmsg)
        {
            printTableHeader();
            for (const Edge& edge : eulerianCircuit) {
            printEdgeRow(edge);
            }
            printTableFooter(eulerianCircuit.size());

            std::cout<< "| ----------------- Successfully Solve Rural Postman Problem -----------------|" <<std::endl;
            std::cout << "+-----------------------------------------------------------------------------+"<<std::endl;
        }

        return {eulerianCircuit, requiredGraph};
    }
    else{

        std::list<Edge> eulerPath = createEulerianPath(graph_aug,startNode,endNode);
        if(printmsg)
        {
            printTableHeader();
            for (const Edge& edge : eulerPath) {
                printEdgeRow(edge);
            }
            printTableFooter(eulerPath.size());

            std::cout<< "| ----------------- Successfully Solve Rural Postman Problem -----------------|" <<std::endl;
            std::cout << "+-----------------------------------------------------------------------------+"<<std::endl;
        }

        return {eulerPath, requiredGraph};
    }
        
    }

