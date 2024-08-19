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
};

struct Edge {
    int from;
    int to;
    double weight;
};

class Graph {
public:
    unordered_map<int, Node> nodes;
    vector<Edge> edges;
    unordered_map<int, int> nodeIdToIndex;
    unordered_map<int, unordered_map<int, double>> edgeWeights;

    void addNode(int id, double x_coord, double y_coord) 
    {
        int index = nodes.size();
        nodes[id] = {id, x_coord, y_coord};
        nodeIdToIndex[id] = index;
    }

    void addEdge(int from, int to, double weight) 
    {
        edges.push_back({from, to, weight});
        edgeWeights[from][to] = weight;
    }

    MatrixXd computeLaplacian() const 
    {
        MatrixXd L = MatrixXd::Zero(nodes.size(), nodes.size());
        for (const Edge& e : edges) 
        {
            try 
            {
                int fromIndex = nodeIdToIndex.at(e.from);
                int toIndex = nodeIdToIndex.at(e.to);

                L(fromIndex, toIndex) = -e.weight;
                L(toIndex, fromIndex) = -e.weight;
                L(fromIndex, fromIndex) += e.weight;
                L(toIndex, toIndex) += e.weight;
            } catch (const std::out_of_range& ex) {
                cerr << "Out of Range error: " << ex.what() << '\n';
                cerr << "An edge is referencing a non-existent node ID." << endl;
            }
        }
        return L;
    }
};

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

void loadPath(const string& filePath, vector<int>& trajectory, Graph& G) {
    ifstream file(filePath);
    string line;
    while (getline(file, line)) 
    {
        stringstream lineStream(line);
        double y_coord, x_coord, weight;
        if (lineStream >> y_coord >> x_coord >> weight) 
        { 
            for (const auto& node : G.nodes) 
            {
                if (node.second.x_coord == x_coord && node.second.y_coord == y_coord)
                {
                    trajectory.push_back(node.first);
                    break;
                }
            }

        }
    }
}

double findEdgeWeight(int fromNodeId, int toNodeId ,vector<vector<string>> edgeData) 
{
    bool isFirstLine = true;
    for (const auto& edge : edgeData)
    {
        if (isFirstLine)
        {
            isFirstLine = false;
            continue;
        }
        int startNodeId = stoi(edge[1]);
        int endNodeId = stoi(edge[2]);
        if ((startNodeId == fromNodeId && endNodeId == toNodeId) ||(startNodeId == toNodeId && endNodeId == fromNodeId)) 
        {
            return stod(edge[3])*111139;
        }
    }
    return 0.0;
}

void writeLaplacianToFile(const MatrixXd& L, const string& filename) 
{
    ofstream outFile(filename);
    if (outFile.is_open()) 
    {
        for (int i = 0; i < L.rows(); ++i) 
        {
            for (int j = 0; j < L.cols(); ++j) 
            {
                outFile << L(i, j);
                if (j < L.cols() - 1) outFile << "\t";
            }
            outFile << "\n";
        }
        outFile.close();
        cout << "Laplacian matrix has been written to '" << filename << "'." << endl;
    }
    else 
    {
        cerr << "Error opening file for writing." << endl;
    }
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
    for (int i = 1; i < eigenvalues.size(); ++i) { // Skip the first eigenvalue (0)
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

int main() {
    string nodeFilePath = "/home/gw/exploration/nav/OSM_global/OSM_global_planning/csvfile/s4/node.csv";
    string edgeFilePath = "/home/gw/exploration/nav/OSM_global/OSM_global_planning/csvfile/s4/link.csv";
    string trajectoryFilePath_good = "/home/gw/exploration/nav/OSM_global/OSM_global_planning/s4_osm_gps.txt";
    string trajectoryFilePath_bad = "/home/gw/exploration/nav/nav_test_ws2/src/gps_follow/src/s4_bad_path.txt";

    vector<vector<string>> nodeData;
    vector<vector<string>> edgeData;
    vector<int> trajectory_good;
    vector<int> trajectory_bad;

    loadCSV(nodeFilePath, nodeData);
    loadCSV(edgeFilePath, edgeData);

    Graph G;
    for (size_t i = 1; i < nodeData.size(); ++i) 
    {
        if(stoi(nodeData[i][4])!=0)
        {
            int id = stoi(nodeData[i][0]);
            double x_coord = stod(nodeData[i][2]);
            double y_coord = stod(nodeData[i][3]);
            G.addNode(id, x_coord, y_coord);
        }
    }

    for (size_t i = 1; i < edgeData.size(); ++i) 
    { 
        int from = stoi(edgeData[i][1]);
        int to = stoi(edgeData[i][2]);
        double weight = stod(edgeData[i][3])*111139;
        G.addEdge(from, to, weight);
    }

    loadPath(trajectoryFilePath_good, trajectory_good, G);
    cout << "Path size: " << trajectory_good.size() << endl;

    Graph trajectoryGraph_good;
    for (size_t i = 0; i < trajectory_good.size(); ++i) 
    {
        trajectoryGraph_good.addNode(i, G.nodes[trajectory_good[i]].x_coord, G.nodes[trajectory_good[i]].y_coord);
    }

    for (size_t i = 0; i < trajectory_good.size() - 1; ++i) 
    {
        trajectoryGraph_good.addEdge(i, i + 1, findEdgeWeight(trajectory_good[i], trajectory_good[i + 1], edgeData));
    }
    double loop_weight = 1;

    for (size_t i = 0; i < trajectory_good.size(); ++i) 
    {
        for (size_t j = i + 1; j < trajectory_good.size(); ++j) 
        {
            if (trajectory_good[i] == trajectory_good[j]) 
            {
                trajectoryGraph_good.addEdge(i, j, loop_weight);
                std::cout << "Added loop edge: " << i << " -> " << j << " with weight " << loop_weight << std::endl;
            }
        }
    }

    std::cout<< " The number of node in trajectoryGraph_good: " << trajectoryGraph_good.nodes.size()<<std::endl;
    std::cout<< " The number of edges in trajectoryGraph_good: " << trajectoryGraph_good.edges.size()<<std::endl;

    MatrixXd L_trajectory_good = trajectoryGraph_good.computeLaplacian();
    double determinant_good = L_trajectory_good.determinant();
    cout << "Determinant of the Trajectory Laplacian matrix: " << determinant_good << endl;

    double d_opt_good = calculateDOptimality(L_trajectory_good);
    cout << "D-optimality of the Trajectory_good  Laplacian matrix: " << d_opt_good << endl;
    writeLaplacianToFile(L_trajectory_good, "/home/gw/exploration/nav/nav_test_ws2/src/gps_follow/src/trajectory_laplacian_good.txt");
    loadPath(trajectoryFilePath_bad, trajectory_bad, G);
    cout << "Path size: " << trajectory_bad.size() << endl;
    Graph trajectoryGraph_bad;
    for (size_t i = 0; i < trajectory_bad.size(); ++i) {
        trajectoryGraph_bad.addNode(i, G.nodes[trajectory_bad[i]].x_coord, G.nodes[trajectory_bad[i]].y_coord);
    }

    for (size_t i = 0; i < trajectory_bad.size() - 1; ++i) {
        trajectoryGraph_bad.addEdge(i, i + 1, findEdgeWeight(trajectory_bad[i], trajectory_bad[i + 1], edgeData));
    }


    for (size_t i = 0; i < trajectory_bad.size(); ++i) {
        for (size_t j = i + 1; j < trajectory_bad.size(); ++j) {
            if (trajectory_bad[i] == trajectory_bad[j]) {
                trajectoryGraph_bad.addEdge(i, j, loop_weight);
                std::cout << "Added loop edge: " << i << " -> " << j << " with weight " << loop_weight << std::endl;
            }
        }
    }

    std::cout<< " The number of node in trajectoryGraph_bad: " << trajectoryGraph_bad.nodes.size()<<std::endl;
    std::cout<< " The number of edges in trajectoryGraph_bad: " << trajectoryGraph_bad.edges.size()<<std::endl;
    MatrixXd L_trajectory_bad = trajectoryGraph_bad.computeLaplacian();
    double determinant_bad = L_trajectory_bad.determinant();
    cout << "Determinant of the Trajectory_bad Laplacian matrix: " << determinant_bad << endl;
    double d_opt_bad = calculateDOptimality(L_trajectory_bad);
    cout << "D-optimality of the Trajectory_bad Laplacian matrix: " << d_opt_bad << endl;
    writeLaplacianToFile(L_trajectory_bad, "/home/gw/exploration/nav/nav_test_ws2/src/gps_follow/src/trajectory_laplacian.txt");
    std::cout<< " The number of node in OSMGraph: " << G.nodes.size()<<std::endl;
    std::cout<< " The number of edges in OSMGraph: " << G.edges.size()<<std::endl;
    MatrixXd L = G.computeLaplacian();
    double determinant_osm = L.determinant();
    cout << "Determinant of the OSM Laplacian matrix: " << determinant_osm << endl;
    double d_opt_osm = calculateDOptimality(L);
    cout << "D-optimality of the osm Laplacian matrix: " << d_opt_osm << endl;
    writeLaplacianToFile(L, "/home/gw/exploration/nav/nav_test_ws2/src/gps_follow/src/laplacian.txt");

    return 0;


}
