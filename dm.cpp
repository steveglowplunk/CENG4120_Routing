#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <cmath>
#include <string>
#include <algorithm>

struct Node {
    int id;
    std::string type;
    double length;
    int begin_x, begin_y, end_x, end_y;
    std::string name;
};

struct Net {
    int id;
    std::string name;
    int source;
    std::vector<int> sinks;
};

struct State {
    int node;
    double f, g;
    bool operator<(const State& other) const {
        return f > other.f; // Min-heap
    }
};

class FPGARouter {
private:
    std::map<int, Node> nodes;
    std::map<int, std::vector<int>> adj_list;
    std::vector<Net> nets;
    std::set<int> globally_used_nodes;
    const double INF = 1e9;

    double heuristic(int n, int t) {
        const Node& node = nodes[n];
        const Node& target = nodes[t];
        return std::abs(node.begin_x - target.begin_x) + std::abs(node.begin_y - target.begin_y);
    }

    std::vector<int> a_star(int start, int target) {
        std::map<int, double> g_score;
        std::map<int, int> came_from;
        std::priority_queue<State> open;
        std::set<int> closed;

        g_score[start] = nodes[start].length;
        open.push({ start, g_score[start] + heuristic(start, target), g_score[start] });

        while (!open.empty()) {
            State current = open.top();
            open.pop();
            int curr_node = current.node;

            if (curr_node == target) {
                std::vector<int> path;
                int node = target;
                while (node != start) {
                    path.push_back(node);
                    node = came_from[node];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }

            if (closed.find(curr_node) != closed.end()) continue;
            closed.insert(curr_node);

            for (int neighbor : adj_list[curr_node]) {
                if (globally_used_nodes.find(neighbor) != globally_used_nodes.end()) continue;

                double tentative_g = g_score[curr_node] + nodes[neighbor].length;
                if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                    g_score[neighbor] = tentative_g;
                    double f = tentative_g + heuristic(neighbor, target);
                    open.push({ neighbor, f, tentative_g });
                    came_from[neighbor] = curr_node;
                }
            }
        }
        return {}; // No path found
    }

public:
    bool read_device_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;

        int n;
        file >> n;
        file.ignore();

        for (int i = 0; i < n; ++i) {
            Node node;
            std::string line;
            std::getline(file, line);
            std::istringstream iss(line);
            iss >> node.id >> node.type >> node.length >> node.begin_x >> node.begin_y >> node.end_x >> node.end_y;
            std::getline(iss, node.name);
            nodes[node.id] = node;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            int parent;
            iss >> parent;
            std::vector<int> children;
            int child;
            while (iss >> child) {
                children.push_back(child);
            }
            adj_list[parent] = children;
        }
        file.close();
        return true;
    }

    bool read_netlist_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;

        int m;
        file >> m;
        file.ignore();

        for (int i = 0; i < m; ++i) {
            Net net;
            std::string line;
            std::getline(file, line);
            std::istringstream iss(line);
            iss >> net.id;
            std::string name;
            iss >> name;
            net.name = name;
            int source;
            iss >> source;
            net.source = source;
            int sink;
            while (iss >> sink) {
                net.sinks.push_back(sink);
            }
            nets.push_back(net);
        }
        file.close();
        return true;
    }

    void route(const std::string& output_filename) {
        std::ofstream out(output_filename);
        if (!out.is_open()) {
            std::cerr << "can not open the output file" << std::endl;
            return;
        }

        for (const Net& net : nets) {
            std::set<std::pair<int, int>> tree_edges;
            std::set<int> tree_nodes;

            for (int sink : net.sinks) {
                std::vector<int> path = a_star(net.source, sink);
                if (path.empty()) {
                    std::cerr << "can not find the path for the sink node " << sink << " of the net " << net.id << std::endl;
                    continue;
                }

                for (size_t i = 0; i < path.size() - 1; ++i) {
                    tree_edges.insert({ path[i], path[i + 1] });
                    tree_nodes.insert(path[i]);
                    tree_nodes.insert(path[i + 1]);
                }
            }

            out << net.id << " " << net.name << "\n";
            for (const auto& edge : tree_edges) {
                out << edge.first << " " << edge.second << "\n";
            }
            out << "\n";

            globally_used_nodes.insert(tree_nodes.begin(), tree_nodes.end());
        }
        out.close();
    }
};

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "usage: " << argv[0] << " <device_file> <netlist_file> <output_file>" << std::endl;
        return 1;
    }

    FPGARouter router;
    if (!router.read_device_file(argv[1])) {
        std::cerr << "can not read the device file" << std::endl;
        return 1;
    }
    if (!router.read_netlist_file(argv[2])) {
        std::cerr << "can not read the netlist file" << std::endl;
        return 1;
    }

    router.route(argv[3]);
    return 0;
}