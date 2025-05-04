#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <sstream>
#include <limits>
#include <queue>
#include <functional>
#include <cmath>
#include <algorithm>
#include <thread>
#include <atomic>
#include <chrono>
#include "node.cpp"
#include "net.cpp"
#include <mutex>

// Heuristic: Manhattan distance using Node begin coordinates.
int heuristic(int nodeId, int goalId, const std::unordered_map<int, Node *> &nodeMap)
{
    int x1 = nodeMap.at(nodeId)->begin_x;
    int y1 = nodeMap.at(nodeId)->begin_y;
    int x2 = nodeMap.at(goalId)->begin_x;
    int y2 = nodeMap.at(goalId)->begin_y;
    return abs(x1 - x2) + abs(y1 - y2);
}

// A* search from start to goal on the given graph.
// 'used' contains nodes that are already occupied by previous nets.
std::vector<int> aStar(int start, int goal,
                       const std::vector<std::vector<int>> &adjacency,
                       const std::unordered_map<int, Node *> &nodeMap,
                       const std::unordered_set<int> &used)
{
    std::unordered_map<int, int> costSoFar;
    std::unordered_map<int, int> cameFrom;
    auto cmp = [&](int left, int right)
    {
        int fLeft = costSoFar[left] + heuristic(left, goal, nodeMap);
        int fRight = costSoFar[right] + heuristic(right, goal, nodeMap);
        return fLeft > fRight;
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> frontier(cmp);
    frontier.push(start);
    costSoFar[start] = 0;

    bool found = false;

    int iterations = 0;

    while (!frontier.empty())
    {
        iterations++;
        if (iterations % 10000 == 0)
        {
            std::cout << "[A*] Iteration: " << iterations
                      << ", frontier size: " << frontier.size() << std::endl;
        }

        int current = frontier.top();
        frontier.pop();

        if (iterations % 50000 == 0)
        {
            std::cout << "[A*] Expanding node: " << current << std::endl;
        }

        if (current == goal)
        {
            found = true;
            break;
        }

        // Iterate neighbors from the adjacency list; if neighbor is used, skip it.
        for (int next : adjacency[current])
        {
            if (used.find(next) != used.end())
            {
                continue;
            }
            int newCost = costSoFar[current] + 1; // all edges have cost 1
            if (costSoFar.find(next) == costSoFar.end() || newCost < costSoFar[next])
            {
                costSoFar[next] = newCost;
                cameFrom[next] = current;
                frontier.push(next);
            }
        }
    }

    // Reconstruct the path if found.
    std::vector<int> path;
    if (found)
    {
        int current = goal;
        while (current != start)
        {
            path.push_back(current);
            current = cameFrom[current];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
    }
    return path;
}

// Parses the netlist file and returns a vector of nets.
std::vector<Net> parseNetlist(const std::string &netlistFile)
{
    std::vector<Net> nets;
    std::ifstream netFile(netlistFile);
    if (!netFile.is_open())
    {
        std::cerr << "Error opening netlist file: " << netlistFile << std::endl;
        return nets;
    }
    int numNets;
    netFile >> numNets;
    std::string dummy;
    std::getline(netFile, dummy); // consume rest of line

    for (int i = 0; i < numNets; ++i)
    {
        std::string line;
        std::getline(netFile, line);
        if (line.empty())
            continue;
        std::istringstream iss(line);
        int netId;
        std::string netName;
        iss >> netId >> netName;
        int source;
        iss >> source;
        Net net(netId, netName, source);
        int sink;
        while (iss >> sink)
        {
            net.sinks.push_back(sink);
        }
        nets.push_back(net);
    }
    netFile.close();
    return nets;
}

int main(int argc, char *argv[])
{
    std::ios_base::sync_with_stdio(false); // Disable sync for faster I/O
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <device_file> <netlist_file>" << std::endl;
        return 1;
    }

    // Check if netlist file can be opened before parsing nodes.
    std::ifstream netTest(argv[2]);
    if (!netTest.is_open())
    {
        std::cerr << "Error opening netlist file: " << argv[2] << std::endl;
        return 1;
    }
    netTest.close();

    // Load routing resource graph from the device file.
    std::ifstream infile(argv[1]);
    if (!infile.is_open())
    {
        std::cerr << "Error opening device file: " << argv[1] << std::endl;
        return 1;
    }

    size_t num_nodes;
    infile >> num_nodes;
    infile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // Read all node lines into memory.
    std::vector<std::string> nodeLines(num_nodes);
    for (int i = 0; i < num_nodes; ++i)
    {
        std::getline(infile, nodeLines[i]);
    }

    // Multi-threaded parsing.
    unsigned int num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0)
        num_threads = 8;                      // default if not detected
    std::vector<Node> parsedNodes(num_nodes); // Preallocate vector for parsed nodes.
    std::vector<std::thread> threads;

    // Atomic counter for progress display.
    std::atomic<int> processedNodes{0};

    auto worker = [&](int start, int end)
    {
        for (int i = start; i < end; ++i)
        {
            std::istringstream iss(nodeLines[i]);
            int id, length, begin_x, begin_y, end_x, end_y;
            std::string dummyType;
            // Read only required values; skip the rest.
            iss >> id >> dummyType >> length >> begin_x >> begin_y >> end_x >> end_y;
            parsedNodes[i] = Node(id, length, begin_x, begin_y, end_x, end_y);
            ++processedNodes;
        }
    };

    // Spawn a progress monitor thread.
    std::thread progressThread([&]()
                               {
    while (processedNodes.load() < num_nodes)
    {
        std::cout << "Parsing nodes: " << processedNodes.load() << " / " << num_nodes << "\r" << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "Parsing nodes: " << num_nodes << " / " << num_nodes << std::endl; });

    int chunk = num_nodes / num_threads;
    int remainder = num_nodes % num_threads;
    int start = 0;
    for (unsigned int t = 0; t < num_threads; ++t)
    {
        int end = start + chunk + (t < remainder ? 1 : 0);
        threads.emplace_back(worker, start, end);
        start = end;
    }

    for (auto &t : threads)
        t.join();
    progressThread.join();

    std::cout << "Parsed " << parsedNodes.size() << " nodes." << std::endl;

    // Read remaining lines representing the adjacency list into memory.
    // Since the number of adjacency lines equals num_nodes, preallocate accordingly.
    std::vector<std::string> adjLines(num_nodes);
    for (int i = 0; i < num_nodes; ++i)
    {
        std::getline(infile, adjLines[i]);
    }
    size_t nAdj = adjLines.size();
    std::cout << "Found " << nAdj << " adjacency lines." << std::endl;

    // Preallocate vector to hold parsed adjacency info.
    std::vector<std::vector<int>> adjacency(num_nodes);
    std::vector<std::thread> adjThreads;
    std::atomic<size_t> processedAdj{0};

    auto adjWorker = [&](size_t start, size_t end)
    {
        for (size_t i = start; i < end; ++i)
        {
            std::istringstream iss(adjLines[i]);
            int parent;
            iss >> parent;
            int child;
            while (iss >> child)
            {
                adjacency[parent].push_back(child);
            }
            ++processedAdj;
        }
    };

    // Set up multithreading for adjacency parsing.
    unsigned int num_adj_threads = std::thread::hardware_concurrency();
    if (num_adj_threads == 0)
        num_adj_threads = 8; // default if not detected

    size_t chunk_adj = nAdj / num_adj_threads;
    size_t remainder_adj = nAdj % num_adj_threads;
    size_t start_index = 0;
    for (unsigned int t = 0; t < num_adj_threads; ++t)
    {
        size_t end_index = start_index + chunk_adj + (t < remainder_adj ? 1 : 0);
        adjThreads.emplace_back(adjWorker, start_index, end_index);
        start_index = end_index;
    }

    // Spawn a progress monitor thread for adjacency parsing.
    std::thread adjProgressThread([&]()
                                  {
    while (processedAdj.load() < nAdj)
    {
        std::cout << "Parsing adjacency: " << processedAdj.load() << " / " << nAdj << "\r" << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "Parsing adjacency: " << nAdj << " / " << nAdj << std::endl; });

    for (auto &t : adjThreads)
        t.join();
    adjProgressThread.join();

    // Load netlist file.
    std::vector<Net> nets = parseNetlist(argv[2]);
    std::cout << "Parsed " << nets.size() << " nets from netlist." << std::endl;

    // Create a mapping from node ID to Node* for use in A* (assumes IDs are unique)
    std::unordered_map<int, Node *> nodeMap;
    for (auto &node : parsedNodes)
    {
        nodeMap[node.id] = &node;
    }

    // Global used nodes to prevent routing congestion.
    std::unordered_set<int> usedNodes;

    // For each net, route from source to each sink using A*
    for (auto &net : nets)
    {
        for (int sink : net.sinks)
        {
            // If source equals sink skip routing.
            if (net.source == sink)
                continue;

            std::cout << "[Routing] Starting route for net " << net.id << " (" << net.name
                      << ") from " << net.source << " to " << sink << std::endl;
            // Run A* search; the usedNodes set restricts the search to unoccupied nodes.
            std::vector<int> path = aStar(net.source, sink, adjacency, nodeMap, usedNodes);
            if (path.empty())
            {
                std::cerr << "[Routing ERROR] Failed to find path for net " << net.id
                          << " (" << net.name << ") from " << net.source << " to " << sink << std::endl;
                continue;
            }
            else
            {
                std::cout << "[Routing] Found path for net " << net.id << " (" << net.name
                          << "), path length: " << path.size() << std::endl;
            }
            // Convert the path into a list of edges (parent, child) and update usedNodes.
            for (size_t i = 0; i < path.size() - 1; ++i)
            {
                net.route.push_back({path[i], path[i + 1]});
                usedNodes.insert(path[i]);
                usedNodes.insert(path[i + 1]);
            }
            std::cout << "[Routing] Routed net " << net.id << " (" << net.name
                      << ") from " << net.source << " to " << sink
                      << ". First nodes in path: ";
            for (size_t i = 0; i < std::min(path.size(), size_t(5)); ++i)
                std::cout << path[i] << " ";
            std::cout << std::endl;
        }
    }

    // Output routing results to a file.
    // Use the netlist file name and replace the extension with .route
    std::string netlistFile = argv[2];
    std::string outputFile = netlistFile.substr(0, netlistFile.find_last_of('.')) + ".route";
    std::ofstream routeFile(outputFile);
    if (!routeFile.is_open())
    {
        std::cerr << "Error opening output file: " << outputFile << std::endl;
        return 1;
    }

    // Write each net's routing tree.
    for (const auto &net : nets)
    {
        routeFile << net.id << " " << net.name << "\n";
        for (const auto &edge : net.route)
        {
            routeFile << edge.first << " " << edge.second << "\n";
        }
        routeFile << "\n";
    }
    routeFile.close();

    std::cout << "Routing complete. Results written to: " << outputFile << std::endl;

    return 0;
}