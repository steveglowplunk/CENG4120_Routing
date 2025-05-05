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
#include "graph.cpp"
#include "net.cpp"
#include <mutex>
#include <map>
#include <iomanip>
#include <random>
#include <memory>
#include <set>
#include <cstdlib>      // For getenv
#include <cstdint>      // For fixed-size integers (uint32_t, uint64_t)
#include <system_error> // For checking file errors
#include <cstdio>       // For std::remove
#include <cstring>      // For strerror

// PathFinder Parameters
const int MAX_ITERATIONS = 10; // Max number of rip-up and re-route iterations
const float INITIAL_HISTORY_COST = 1.0f;
const float HISTORY_PENALTY_FACTOR = 3.0f;            // Increased from 2.0f
const float PRESENT_CONGESTION_PENALTY_FACTOR = 3.0f; // Increased from 2.0f

// Binary Cache Constants
const uint32_t BINARY_MAGIC = 0xC0DE4120; // Custom magic number
const uint32_t BINARY_VERSION = 1;
const std::string DEFAULT_BINARY_CACHE_FILENAME = "graph_cache.bin";

// Function to save the graph data to a binary file
bool saveGraphToBinary(const std::string &filename,
                       const std::vector<std::unique_ptr<Node>> &nodes,
                       const std::unordered_map<int, std::vector<int>> &adjacency)
{
    std::ofstream outfile(filename, std::ios::binary | std::ios::trunc);
    if (!outfile)
    {
        std::cerr << "Error: Cannot open file for writing binary cache: " << filename << " (" << strerror(errno) << ")" << std::endl;
        return false;
    }

    try
    {
        // 1. Write Header
        outfile.write(reinterpret_cast<const char *>(&BINARY_MAGIC), sizeof(BINARY_MAGIC));
        outfile.write(reinterpret_cast<const char *>(&BINARY_VERSION), sizeof(BINARY_VERSION));

        // 2. Write Node Count
        uint64_t node_count = nodes.size();
        outfile.write(reinterpret_cast<const char *>(&node_count), sizeof(node_count));

        // 3. Write Node Data (only essential fields)
        for (const auto &nodePtr : nodes)
        {
            if (!nodePtr)
            {
                std::cerr << "Warning: Skipping null node pointer during serialization." << std::endl;
                // Write placeholder data? Need to handle this on load if nulls are possible
                int dummy_id = -1;
                float dummy_float = 0.0f;
                int dummy_int = 0;
                outfile.write(reinterpret_cast<const char *>(&dummy_id), sizeof(dummy_id));
                outfile.write(reinterpret_cast<const char *>(&dummy_int), sizeof(dummy_int));     // length
                outfile.write(reinterpret_cast<const char *>(&dummy_int), sizeof(dummy_int));     // begin_x
                outfile.write(reinterpret_cast<const char *>(&dummy_int), sizeof(dummy_int));     // begin_y
                outfile.write(reinterpret_cast<const char *>(&dummy_float), sizeof(dummy_float)); // baseCost
                continue;
            }
            int id = nodePtr->getId();
            int length = nodePtr->getLength();
            int begin_x = nodePtr->getBeginX();
            int begin_y = nodePtr->getBeginY();
            float baseCost = nodePtr->getBaseCost(); // Save baseCost

            outfile.write(reinterpret_cast<const char *>(&id), sizeof(id));
            outfile.write(reinterpret_cast<const char *>(&length), sizeof(length));
            outfile.write(reinterpret_cast<const char *>(&begin_x), sizeof(begin_x));
            outfile.write(reinterpret_cast<const char *>(&begin_y), sizeof(begin_y));
            outfile.write(reinterpret_cast<const char *>(&baseCost), sizeof(baseCost));
        }

        // 4. Write Adjacency List Count (number of parents)
        uint64_t adj_parent_count = adjacency.size();
        outfile.write(reinterpret_cast<const char *>(&adj_parent_count), sizeof(adj_parent_count));

        // 5. Write Adjacency Data
        for (const auto &pair : adjacency)
        {
            int parent_id = pair.first;
            const auto &children = pair.second;
            uint32_t child_count = children.size();

            outfile.write(reinterpret_cast<const char *>(&parent_id), sizeof(parent_id));
            outfile.write(reinterpret_cast<const char *>(&child_count), sizeof(child_count));
            if (child_count > 0)
            {
                outfile.write(reinterpret_cast<const char *>(children.data()), child_count * sizeof(int));
            }
        }

        outfile.close();
        if (!outfile.good())
        {
            std::cerr << "Error: Failed writing binary cache file after closing: " << filename << std::endl;
            return false;
        }
        std::cout << "Successfully saved graph data to binary cache: " << filename << std::endl;
        return true;
    }
    catch (const std::ios_base::failure &e)
    {
        std::cerr << "Filesystem exception during binary serialization: " << e.what() << " (Code: " << e.code() << ")" << std::endl;
        outfile.close();
        std::remove(filename.c_str());
        return false;
    }
    catch (const std::exception &e)
    {
        std::cerr << "General exception during binary serialization: " << e.what() << std::endl;
        outfile.close();
        std::remove(filename.c_str());
        return false;
    }
}

// Function to load the graph data from a binary file
bool loadGraphFromBinary(const std::string &filename,
                         std::vector<std::unique_ptr<Node>> &nodes, // Output
                         std::unordered_map<int, Node *> &nodeMap,  // Output
                         Graph &graph)
{ // Output (adjacency)
    std::ifstream infile(filename, std::ios::binary);
    if (!infile)
    {
        // This is expected if the cache doesn't exist yet, not necessarily an error
        // std::cout << "Binary cache file not found: " << filename << std::endl;
        return false;
    }
    // Ensure maps/vectors are clear before loading
    nodes.clear();
    nodeMap.clear();
    graph.adjacency.clear();

    try
    {
        // 1. Read and Verify Header
        uint32_t magic = 0, version = 0;
        infile.read(reinterpret_cast<char *>(&magic), sizeof(magic));
        infile.read(reinterpret_cast<char *>(&version), sizeof(version));

        if (infile.fail() || magic != BINARY_MAGIC || version != BINARY_VERSION)
        {
            std::cerr << "Error: Invalid or incompatible binary cache file header: " << filename << ". Will regenerate." << std::endl;
            infile.close();
            std::remove(filename.c_str()); // Delete invalid cache
            return false;
        }

        // 2. Read Node Count
        uint64_t node_count = 0;
        infile.read(reinterpret_cast<char *>(&node_count), sizeof(node_count));
        if (infile.fail() || node_count == 0 || node_count > 50000000)
        { // Basic sanity check on count
            std::cerr << "Error: Invalid node count in binary cache: " << node_count << ". Will regenerate." << std::endl;
            infile.close();
            std::remove(filename.c_str());
            return false;
        }
        nodes.resize(node_count);    // Pre-allocate space for unique_ptrs
        nodeMap.reserve(node_count); // Reserve space for map

        // 3. Read Node Data
        for (uint64_t i = 0; i < node_count; ++i)
        {
            int id, length, begin_x, begin_y;
            float baseCost;
            infile.read(reinterpret_cast<char *>(&id), sizeof(id));
            infile.read(reinterpret_cast<char *>(&length), sizeof(length));
            infile.read(reinterpret_cast<char *>(&begin_x), sizeof(begin_x));
            infile.read(reinterpret_cast<char *>(&begin_y), sizeof(begin_y));
            infile.read(reinterpret_cast<char *>(&baseCost), sizeof(baseCost));

            if (infile.fail())
            {
                std::cerr << "Error: Failed reading node data at index " << i << " from binary cache. Will regenerate." << std::endl;
                infile.close();
                std::remove(filename.c_str());
                return false;
            }
            // Handle potential placeholder for nullptrs if needed
            if (id == -1)
            { // Assuming -1 was placeholder used in save
                nodes[i] = nullptr;
                continue;
            }

            nodes[i] = std::make_unique<Node>(id, length, begin_x, begin_y, 0, 0); // Use dummy end coords
            if (!nodes[i])
            { // Check make_unique result
                std::cerr << "Error: Failed to allocate memory for node " << id << std::endl;
                infile.close();
                std::remove(filename.c_str());
                return false;
            }
            nodes[i]->setBaseCost(baseCost); // Set the loaded base cost
            nodeMap[id] = nodes[i].get();
        }

        // 4. Read Adjacency List Count
        uint64_t adj_parent_count = 0;
        infile.read(reinterpret_cast<char *>(&adj_parent_count), sizeof(adj_parent_count));
        if (infile.fail() || adj_parent_count > 50000000)
        { // Basic sanity check
            std::cerr << "Error: Failed reading or invalid adjacency parent count from binary cache: " << adj_parent_count << ". Will regenerate." << std::endl;
            infile.close();
            std::remove(filename.c_str());
            return false;
        }
        graph.adjacency.reserve(adj_parent_count); // Reserve space

        // 5. Read Adjacency Data
        for (uint64_t i = 0; i < adj_parent_count; ++i)
        {
            int parent_id = 0;
            uint32_t child_count = 0;
            infile.read(reinterpret_cast<char *>(&parent_id), sizeof(parent_id));
            infile.read(reinterpret_cast<char *>(&child_count), sizeof(child_count));

            if (infile.fail())
            {
                std::cerr << "Error: Failed reading adjacency parent/count at index " << i << " from binary cache. Will regenerate." << std::endl;
                infile.close();
                std::remove(filename.c_str());
                return false;
            }

            std::vector<int> children(child_count); // Create vector of correct size
            if (child_count > 0)
            {
                infile.read(reinterpret_cast<char *>(children.data()), child_count * sizeof(int));
                if (infile.fail())
                {
                    std::cerr << "Error: Failed reading children for parent " << parent_id << " from binary cache. Will regenerate." << std::endl;
                    infile.close();
                    std::remove(filename.c_str());
                    return false;
                }
            }
            // Only add if parent node actually exists from node loading phase
            if (nodeMap.count(parent_id))
            {
                graph.adjacency[parent_id] = std::move(children);
            }
            else
            {
                // This would indicate an inconsistency in the cache file
                std::cerr << "Warning: Parent ID " << parent_id << " from adjacency cache not found in loaded nodes. Skipping entry." << std::endl;
            }
        }

        // Check if we reached the end of the file cleanly (optional)
        infile.peek();
        if (!infile.eof())
        {
            std::cerr << "Warning: Extra data found at the end of binary cache file: " << filename << ". Cache might be corrupt. Regenerating next time." << std::endl;
            infile.close();
            std::remove(filename.c_str()); // Delete potentially corrupt file
            return false;
        }

        infile.close();
        std::cout << "Successfully loaded graph data from binary cache: " << filename << std::endl;
        return true;
    }
    catch (const std::ios_base::failure &e)
    {
        std::cerr << "Filesystem exception during binary deserialization: " << e.what() << " (Code: " << e.code() << "). Will attempt regeneration." << std::endl;
        infile.close();
        std::remove(filename.c_str());
        return false;
    }
    catch (const std::exception &e)
    {
        std::cerr << "General exception during binary deserialization: " << e.what() << ". Will attempt regeneration." << std::endl;
        infile.close();
        std::remove(filename.c_str());
        return false;
    }
}

// optimized heuristic function
inline int heuristic(int nodeId, int goalId, const std::unordered_map<int, Node *> &nodeMap)
{
    const Node *node1 = nodeMap.at(nodeId);
    const Node *node2 = nodeMap.at(goalId);
    return abs(node1->getBeginX() - node2->getBeginX()) + abs(node1->getBeginY() - node2->getBeginY());
}

// optimized A* search algorithm, now using PathFinder cost model
std::vector<int> aStar(int start, int goal, const Graph &graph,
                       const std::unordered_map<int, Node *> &nodeMap,
                       float lengthPriority = 1.0f) // lengthPriority might be less relevant now or integrated differently
{
    if (start == goal)
    {
        return {start};
    }

    std::unordered_map<int, float> costSoFar;
    std::unordered_map<int, int> cameFrom;

    auto cmp = [&](int left, int right)
    {
        // Use the simplified heuristic (pure Manhattan distance)
        float fLeft = costSoFar[left] + heuristic(left, goal, nodeMap);
        float fRight = costSoFar[right] + heuristic(right, goal, nodeMap);
        // Minor tie-breaking using node ID if costs are equal
        if (std::abs(fLeft - fRight) < 1e-6)
        {
            return left > right;
        }
        return fLeft > fRight;
    };

    std::priority_queue<int, std::vector<int>, decltype(cmp)> frontier(cmp);
    frontier.push(start);
    costSoFar[start] = 0;

    // Consider adjusting or removing the expansion limit based on PathFinder needs
    const int NODE_EXPANSION_LIMIT = 700000; // Reduced limit from 1.5M
    int expandedNodes = 0;

    std::unordered_set<int> visitedInSearch; // Track visited nodes within this specific A* search

    while (!frontier.empty() && expandedNodes < NODE_EXPANSION_LIMIT)
    {
        int current = frontier.top();
        frontier.pop();

        // Avoid re-expanding nodes in the same search path if found with a higher cost
        if (visitedInSearch.count(current))
        {
            continue;
        }
        visitedInSearch.insert(current);

        expandedNodes++;

        // Optional: Keep debug output
        // if (expandedNodes % 10000 == 0) {
        //     std::cout << "Expanded " << expandedNodes << " nodes, frontier size: "
        //               << frontier.size() << "\r" << std::flush;
        // }

        if (current == goal)
        {
            std::vector<int> path;
            while (current != start)
            {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Fetch adjacency list for current node once
        auto adj_it = graph.adjacency.find(current);
        if (adj_it != graph.adjacency.end()) // Check if current node has neighbors
        {
            const auto &neighbors = adj_it->second; // Get reference to vector of neighbors
            for (int next : neighbors)
            {
                // Find 'next' node once and store iterator
                auto map_it = nodeMap.find(next);
                if (map_it == nodeMap.end())
                {
                    // std::cerr << "Warning: Neighbor node " << next << " not found in nodeMap." << std::endl;
                    continue; // Skip if node doesn't exist
                }
                Node *nextNode = map_it->second; // Get pointer from iterator

                // PathFinder Cost Calculation for the edge (current -> next)
                float baseCost = nextNode->getBaseCost();
                float historyCost = nextNode->getHistoryCost();
                int occupancy = nextNode->getOccupancy();

                // Present Congestion Multiplier
                float presentCongestionMultiplier = 1.0f + std::max(0, occupancy) * PRESENT_CONGESTION_PENALTY_FACTOR;

                // Total cost calculation (including length factor)
                float lengthCost = nextNode->getLength() * 0.05f;
                float edgeCost = (baseCost + lengthCost + historyCost) * presentCongestionMultiplier;

                float newCost = costSoFar[current] + edgeCost;

                // Check cost using find first to avoid potential default construction with operator[]
                auto cost_it = costSoFar.find(next);
                if (cost_it == costSoFar.end() || newCost < cost_it->second)
                {
                    costSoFar[next] = newCost; // Update cost using operator[] only if needed
                    cameFrom[next] = current;
                    frontier.push(next);
                }
            }
        }
        // Early termination check removed for now, might interfere with finding paths when costs are high
    }

    // No path found
    return {};
}

// parse netlist file, return netlist list
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
    std::getline(netFile, dummy); // eliminate the first line

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

// calculate net wirelength
int calculateWirelength(const Net &net, const std::unordered_map<int, Node *> &nodeMap)
{
    int wirelength = 0;
    std::unordered_set<int> usedNodes;

    for (const auto &edge : net.route)
    {
        usedNodes.insert(edge.first);
        usedNodes.insert(edge.second);
    }

    for (int nodeId : usedNodes)
    {
        wirelength += nodeMap.at(nodeId)->getLength();
    }

    return wirelength;
}

// calculate Manhattan distance between two nodes
int calculateManhattanDistance(int nodeId1, int nodeId2, const std::unordered_map<int, Node *> &nodeMap)
{
    const Node *node1 = nodeMap.at(nodeId1);
    const Node *node2 = nodeMap.at(nodeId2);
    return abs(node1->getBeginX() - node2->getBeginX()) + abs(node1->getBeginY() - node2->getBeginY());
}

int main(int argc, char *argv[])
{
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr); // Untie cin/cout for potential minor speedup

    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <device_file> <netlist_file> [<output_file>]" << std::endl;
        return 1;
    }

    std::string deviceFile = argv[1];
    std::string netlistFile = argv[2];
    std::string outputFile;

    if (argc > 3)
    {
        outputFile = argv[3];
    }
    else
    {
        outputFile = netlistFile;
        size_t dotPos = outputFile.find_last_of('.');
        if (dotPos != std::string::npos)
        {
            outputFile.replace(dotPos, std::string::npos, ".route");
        }
        else
        {
            outputFile += ".route";
        }
    }

    // --- Start Timing ---
    auto startTime = std::chrono::high_resolution_clock::now();

    // --- Load Graph Data (Try Binary Cache First) ---
    std::vector<std::unique_ptr<Node>> allNodes;
    std::unordered_map<int, Node *> nodeMap;
    Graph graph;
    bool loadedFromCache = false;
    std::string cacheFilename = DEFAULT_BINARY_CACHE_FILENAME;

    // Check if device file exists before attempting cache load
    std::ifstream deviceTest(deviceFile);
    if (!deviceTest)
    {
        std::cerr << "Error: Cannot open device file: " << deviceFile << std::endl;
        return 1;
    }
    deviceTest.close();

    std::cout << "Attempting to load graph data from binary cache: " << cacheFilename << std::endl;
    loadedFromCache = loadGraphFromBinary(cacheFilename, allNodes, nodeMap, graph);

    bool parseSuccess = loadedFromCache;                                 // Assume success if loaded from cache
    auto loadOrParseEndTime = std::chrono::high_resolution_clock::now(); // Time after loading/parsing attempt
    unsigned int num_threads = 0;                                        // Declare num_threads in outer scope

    if (!loadedFromCache)
    {
        std::cout << "Binary cache not found or invalid. Parsing text device file: " << deviceFile << std::endl;
        auto textParseStartTime = std::chrono::high_resolution_clock::now();

        // --- Original Text Parsing Logic ---
        // (Includes parallel node and adjacency parsing)
        // --- Start: Parallel Node Parsing ---
        std::ifstream node_infile(deviceFile);
        if (!node_infile.is_open())
        {
            std::cerr << "Error opening device file for nodes: " << deviceFile << std::endl;
            return 1;
        }
        int num_nodes;
        node_infile >> num_nodes;
        if (node_infile.fail() || num_nodes <= 0)
        {
            std::cerr << "Error reading number of nodes or invalid number: " << num_nodes << std::endl;
            return 1;
        }
        node_infile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        std::cout << "Reading " << num_nodes << " node definition lines..." << std::endl;
        std::vector<std::string> nodeLines;
        nodeLines.reserve(num_nodes);
        std::string line;
        std::streampos adjStartPosition = node_infile.tellg();
        for (int i = 0; i < num_nodes; ++i)
        {
            if (!std::getline(node_infile, line))
            {
                std::cerr << "Error reading node line " << i + 1 << std::endl;
                node_infile.close();
                return 1;
            }
            nodeLines.push_back(line);
            adjStartPosition = node_infile.tellg();
        }
        node_infile.close();
        std::cout << "Finished reading node lines." << std::endl;

        std::cout << "Parsing " << num_nodes << " nodes using parallel threads..." << std::endl;
        allNodes.resize(num_nodes); // Ensure vector has space
        nodeMap.reserve(num_nodes); // Reserve map space

        // Determine thread count (using SLURM check) - Assign to outer scope num_threads
        // Re-initialize num_threads here just in case it was set differently before
        num_threads = 0;
        const char *slurm_cpus_var = std::getenv("SLURM_CPUS_PER_TASK");
        if (slurm_cpus_var != nullptr)
        {
            try
            {
                int c = std::stoi(slurm_cpus_var);
                if (c > 0)
                    num_threads = c;
            }
            catch (...)
            {
            }
        }
        if (num_threads == 0)
        {
            num_threads = std::thread::hardware_concurrency();
        }
        if (num_threads == 0)
            num_threads = 4;
        const unsigned int MAX_ALLOWED_THREADS = 8;
        num_threads = std::min(num_threads, MAX_ALLOWED_THREADS);
        std::cout << "Using " << num_threads << " threads for parallel parsing (capped at " << MAX_ALLOWED_THREADS << ")." << std::endl;

        std::vector<std::thread> node_threads;
        std::atomic<int> nodes_parsed_count(0);
        int node_chunk_size = (num_nodes + num_threads - 1) / num_threads;
        auto node_worker = [&](int start_idx, int end_idx)
        {
            for (int i = start_idx; i < end_idx; ++i)
            {
                if (i >= nodeLines.size())
                    break;
                std::istringstream iss(nodeLines[i]);
                int id, length, bx, by, ex, ey;
                std::string type, name;
                float baseCost = 1.0f; // Default base cost if not saved/parsed
                if (!(iss >> id >> type >> length >> bx >> by >> ex >> ey >> name))
                {
                    continue;
                }
                // Note: Text parsing doesn't provide baseCost directly, assume 1.0
                // Create Node object - ensure constructor matches (or set baseCost after)
                allNodes[i] = std::make_unique<Node>(id, length, bx, by, ex, ey);
                if (allNodes[i])
                {
                    allNodes[i]->setBaseCost(baseCost); // Set default base cost
                }
                nodes_parsed_count++;
            }
        };
        for (unsigned int i = 0; i < num_threads; ++i)
        {
            int start = i * node_chunk_size;
            int end = std::min(start + node_chunk_size, num_nodes);
            if (start < end)
            {
                node_threads.emplace_back(node_worker, start, end);
            }
        }
        while (nodes_parsed_count.load() < num_nodes)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        for (auto &t : node_threads)
        {
            t.join();
        }
        std::cout << "Populating node map..." << std::endl;
        for (const auto &nodePtr : allNodes)
        {
            if (nodePtr)
            {
                nodeMap[nodePtr->getId()] = nodePtr.get();
            }
        }
        std::cout << "Node parsing and map population complete." << std::endl;
        bool textParseNodesOk = (nodeMap.size() > 0); // Simple check
        // --- End: Parallel Node Parsing ---

        // --- Start: Parallel Adjacency Parsing ---
        std::cout << "Reading adjacency list lines..." << std::endl;
        std::vector<std::string> adjLines;
        std::ifstream adjInfile(deviceFile);
        if (!adjInfile.is_open())
        {
            std::cerr << "Error re-opening device file for adjacency." << std::endl;
            return 1;
        }
        adjInfile.seekg(adjStartPosition);
        if (!adjInfile)
        {
            std::cerr << "Error seeking to adjacency list position." << std::endl;
            return 1;
        }
        while (std::getline(adjInfile, line))
        {
            adjLines.push_back(line);
        }
        adjInfile.close();
        std::cout << "Finished reading " << adjLines.size() << " adjacency lines." << std::endl;

        std::cout << "Parsing adjacency list using " << num_threads << " parallel threads..." << std::endl;
        std::vector<std::unordered_map<int, std::vector<int>>> local_adjs(num_threads);
        std::vector<std::thread> adj_threads;
        std::atomic<int> adj_parsed_count(0);
        int adj_chunk_size = (adjLines.size() + num_threads - 1) / num_threads;
        auto adj_worker = [&](int start_idx, int end_idx, std::unordered_map<int, std::vector<int>> &my_local_adj)
        {
            for (int i = start_idx; i < end_idx; ++i)
            {
                if (i >= adjLines.size())
                    break;
                adj_parsed_count++;
                std::istringstream iss(adjLines[i]);
                int parentId;
                if (!(iss >> parentId))
                {
                    continue;
                }
                std::vector<int> children;
                int childId;
                while (iss >> childId)
                {
                    children.push_back(childId);
                }
                bool parentExists = (nodeMap.count(parentId) > 0);
                if (parentExists)
                {
                    my_local_adj[parentId] = std::move(children);
                }
            }
        };
        for (unsigned int i = 0; i < num_threads; ++i)
        {
            int start = i * adj_chunk_size;
            int end = std::min(start + adj_chunk_size, (int)adjLines.size());
            if (start < end)
            {
                adj_threads.emplace_back(adj_worker, start, end, std::ref(local_adjs[i]));
            }
        }
        while (adj_parsed_count.load() < adjLines.size())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        for (auto &t : adj_threads)
        {
            t.join();
        }
        std::cout << "Merging thread-local adjacency maps..." << std::endl;
        graph.adjacency.reserve(adj_parsed_count); // Reserve based on parsed count
        for (const auto &local_map : local_adjs)
        {
            for (auto const &[parentId, childrenVec] : local_map)
            { // Use different name
                graph.adjacency[parentId] = childrenVec;
            }
        }
        std::cout << "Finished merging. Final graph adjacency size: " << graph.adjacency.size() << std::endl;
        bool textParseAdjOk = (graph.adjacency.size() > 0); // Simple check
        // --- End: Parallel Adjacency Parsing ---

        parseSuccess = textParseNodesOk && textParseAdjOk;
        // --- End of Original Text Parsing Logic ---

        loadOrParseEndTime = std::chrono::high_resolution_clock::now(); // Update time after text parse
        if (parseSuccess)
        {
            auto parseDuration = std::chrono::duration_cast<std::chrono::milliseconds>(loadOrParseEndTime - textParseStartTime);
            std::cout << "Text parsing completed in " << parseDuration.count() / 1000.0 << " seconds." << std::endl;

            // Save to cache IF parsing was successful
            std::cout << "Saving parsed data to binary cache: " << cacheFilename << std::endl;
            if (!saveGraphToBinary(cacheFilename, allNodes, graph.adjacency))
            {
                std::cerr << "Warning: Failed to save data to binary cache." << std::endl;
            }
        }
        else
        {
            std::cerr << "Error: Failed to parse text device file." << std::endl;
            return 1; // Exit if text parsing fails
        }
    }
    else
    { // Loaded from cache
        auto loadDuration = std::chrono::duration_cast<std::chrono::milliseconds>(loadOrParseEndTime - startTime);
        std::cout << "Graph loading from cache completed in " << loadDuration.count() / 1000.0 << " seconds." << std::endl;
        // If loaded from cache, we still need num_threads for routing. Set it here.
        // It's reasonable to use the same logic as parsing.
        num_threads = 0;
        const char *slurm_cpus_var = std::getenv("SLURM_CPUS_PER_TASK");
        if (slurm_cpus_var != nullptr)
        {
            try
            {
                int c = std::stoi(slurm_cpus_var);
                if (c > 0)
                    num_threads = c;
            }
            catch (...)
            {
            }
        }
        if (num_threads == 0)
        {
            num_threads = std::thread::hardware_concurrency();
        }
        if (num_threads == 0)
            num_threads = 4;
        const unsigned int MAX_ALLOWED_THREADS = 8;
        num_threads = std::min(num_threads, MAX_ALLOWED_THREADS);
        std::cout << "Using " << num_threads << " threads for routing (capped at " << MAX_ALLOWED_THREADS << ")." << std::endl;
    }

    // --- Graph data is now loaded ---
    if (!parseSuccess)
    { // Double check just in case
        std::cerr << "Error: Graph data could not be loaded." << std::endl;
        return 1;
    }

    // Check if graph is reasonably populated
    if (nodeMap.empty() || graph.adjacency.empty())
    {
        std::cerr << "Error: Graph data is empty after loading/parsing." << std::endl;
        return 1;
    }
    std::cout << "Graph loaded. Nodes: " << nodeMap.size() << ", Adjacency Parents: " << graph.adjacency.size() << std::endl;

    // --- Load Netlist ---
    auto netlistStartTime = std::chrono::high_resolution_clock::now();
    std::cout << "Loading netlist: " << netlistFile << std::endl;
    std::vector<Net> nets = parseNetlist(netlistFile);
    if (nets.empty() && netlistFile.find(".netlist") != std::string::npos)
    {
        std::cerr << "Error: Netlist file seems empty or failed to parse." << std::endl;
        return 1;
    }
    std::cout << "Netlist loading complete. Nets to route: " << nets.size() << std::endl;
    auto netlistEndTime = std::chrono::high_resolution_clock::now();
    auto netlistDuration = std::chrono::duration_cast<std::chrono::milliseconds>(netlistEndTime - netlistStartTime);
    std::cout << "Netlist loading time: " << netlistDuration.count() / 1000.0 << " seconds." << std::endl;

    // --- PathFinder Routing ---
    auto routeStartTime = std::chrono::high_resolution_clock::now();
    std::cout << "Starting PathFinder routing..." << std::endl;
    // (Pathfinder initialization: reset occupancy, setup netsToRoute)
    std::cout << "Initializing node occupancy..." << std::endl;
    for (auto const &[id, nodePtr] : nodeMap)
    {
        if (nodePtr)
            nodePtr->resetOccupancy();
    }
    std::set<int> netsToRoute;
    for (int i = 0; i < nets.size(); ++i)
        netsToRoute.insert(i);
    int iteration;
    // Use the num_threads determined earlier
    unsigned int num_routing_threads = num_threads;

    for (iteration = 0; iteration < MAX_ITERATIONS; ++iteration)
    {
        std::cout << "--- Iteration " << iteration + 1 << "/" << MAX_ITERATIONS << " ---" << std::endl;

        auto iterStartTime = std::chrono::high_resolution_clock::now();

        if (netsToRoute.empty())
        {
            std::cout << "Convergence reached. No nets to reroute." << std::endl;
            break;
        }
        std::cout << "Nets to reroute this iteration: " << netsToRoute.size() << std::endl;

        // 1. Rip-up: Clear routes and decrement occupancy for nets being rerouted.
        std::cout << "Ripping up " << netsToRoute.size() << " nets..." << std::endl;
        for (int netIdx : netsToRoute)
        {
            if (netIdx >= nets.size())
                continue;
            Net &net = nets[netIdx];
            if (!net.route.empty())
            { // Only decrement if it had a route
                std::unordered_set<int> nodesInOldPath;
                for (const auto &edge : net.route)
                {
                    if (nodeMap.count(edge.first))
                        nodesInOldPath.insert(edge.first);
                    if (nodeMap.count(edge.second))
                        nodesInOldPath.insert(edge.second);
                }
                for (int nodeId : nodesInOldPath)
                {
                    if (nodeMap.count(nodeId))
                    {
                        nodeMap.at(nodeId)->decrementOccupancy();
                        // Ensure occupancy doesn't go below zero due to potential double-counting or errors
                        if (nodeMap.at(nodeId)->getOccupancy() < 0)
                        {
                            // This case should ideally not happen with correct logic
                            // std::cerr << "Warning: Node " << nodeId << " occupancy became negative during rip-up! Resetting to 0." << std::endl;
                            nodeMap.at(nodeId)->resetOccupancy();
                        }
                    }
                }
                net.route.clear(); // Clear the old route data
            }
            net.routedSinks = 0; // Reset routed sink count for this net
        }

        // 2. Reset Occupancy for ALL nodes (simpler than selective recalculation)
        // This avoids issues with stale occupancy from nets that failed previously but weren't in netsToRoute this time.
        // REMOVED - We will now rely on incremental updates
        // for (auto const& [id, nodePtr] : nodeMap) {
        //     nodePtr->resetOccupancy();
        // }

        // 3. Recalculate Occupancy based on *all* currently successful routes (nets NOT in netsToRoute)
        // REMOVED - Incremental updates handle this now.
        /*
         std::cout << "Calculating occupancy based on existing routes..." << std::endl;
         int existingEdges = 0;
         for (int i = 0; i < nets.size(); ++i) {
             if (netsToRoute.find(i) == netsToRoute.end() && !nets[i].route.empty()) { // If net is NOT being rerouted AND has a route
                 Net& net = nets[i];
                 std::unordered_set<int> nodesInPath;
                 for (const auto& edge : net.route) {
                    existingEdges++;
                    if (nodeMap.count(edge.first)) nodesInPath.insert(edge.first);
                    if (nodeMap.count(edge.second)) nodesInPath.insert(edge.second);
                 }
                 for(int nodeId : nodesInPath) {
                      if(nodeMap.count(nodeId)) {
                           nodeMap.at(nodeId)->incrementOccupancy();
                      } else {
                          // This shouldn't happen if parsing was correct
                          // std::cerr << "Warning: Node " << nodeId << " from existing route of net " << net.id << " not in nodeMap!" << std::endl;
                      }
                 }
             }
         }
          std::cout << "Occupancy updated based on " << existingEdges << " edges from non-rerouted nets." << std::endl;
        */

        // 4. Reroute nets in netsToRoute (Parallelized)
        // This step now implicitly handles occupancy increments for successful routes.
        std::cout << "Rerouting nets using " << num_routing_threads << " threads..." << std::endl;
        std::atomic<int> reroutedCount = 0;
        std::atomic<int> failedCount = 0;
        std::atomic<int> processedNetCount = 0;

        // Get the indices of nets to reroute
        std::vector<int> netIndicesToProcess(netsToRoute.begin(), netsToRoute.end());
        size_t totalNetsToReroute = netIndicesToProcess.size();

        // Create threads for rerouting
        std::vector<std::thread> reroute_threads;
        int reroute_chunk_size = (totalNetsToReroute + num_routing_threads - 1) / num_routing_threads;

        // Rerouting worker lambda
        auto reroute_worker = [&](int start_chunk_idx, int end_chunk_idx)
        {
            for (int i = start_chunk_idx; i < end_chunk_idx; ++i)
            {
                if (i >= totalNetsToReroute)
                    break;
                int netIdx = netIndicesToProcess[i];

                processedNetCount++; // Atomic increment for progress
                // Optional: Print progress less frequently inside the loop
                // if (processedNetCount % 50 == 0) {
                //     std::cout << "  Processed " << processedNetCount << "/" << totalNetsToReroute << " nets...\r" << std::flush;
                // }

                if (netIdx >= nets.size())
                    continue;            // Safety check
                Net &net = nets[netIdx]; // Get reference to the net
                bool netSuccess = true;
                std::vector<std::pair<int, int>> currentNetRouteSegments;
                std::unordered_set<int> currentNetNodes;

                // --- Start of single-net routing logic (same as before) ---
                if (!nodeMap.count(net.source))
                {
                    netSuccess = false;
                }
                else
                {
                    currentNetNodes.insert(net.source);
                }

                if (netSuccess)
                {
                    int currentRoutedSinks = 0; // Local count for this attempt
                    for (int sink : net.sinks)
                    {
                        if (!nodeMap.count(sink))
                        {
                            netSuccess = false;
                            continue;
                        }
                        if (net.source == sink)
                        {
                            currentRoutedSinks++;
                            continue;
                        }
                        std::vector<int> path = aStar(net.source, sink, graph, nodeMap);
                        if (!path.empty())
                        {
                            for (size_t p = 0; p < path.size() - 1; ++p)
                            {
                                currentNetRouteSegments.push_back({path[p], path[p + 1]});
                                if (nodeMap.count(path[p]))
                                    currentNetNodes.insert(path[p]);
                                if (nodeMap.count(path[p + 1]))
                                    currentNetNodes.insert(path[p + 1]);
                            }
                            currentRoutedSinks++;
                        }
                        else
                        {
                            netSuccess = false;
                            // break; // Optional: stop if one sink fails?
                        }
                    } // End sink loop
                    // Update net.routedSinks *after* checking all sinks for this attempt
                    // This needs protection if multiple threads could modify the same Net object, but they shouldn't here.
                    net.routedSinks = currentRoutedSinks;
                } // End if(netSuccess) source check
                // --- End of single-net routing logic ---

                // --- Post-routing update (inside worker thread) ---
                if (netSuccess && net.routedSinks == net.sinks.size())
                {
                    reroutedCount++;
                    net.route = std::move(currentNetRouteSegments);
                    // Increment occupancy atomically
                    for (int nodeId : currentNetNodes)
                    {
                        if (nodeMap.count(nodeId))
                        {
                            nodeMap.at(nodeId)->incrementOccupancy();
                        }
                    }
                }
                else
                {
                    failedCount++;
                    net.route.clear();
                    net.routedSinks = 0;
                    // No occupancy changes for failed nets
                }
            } // End loop over assigned net indices
        };

        // Launch rerouting threads
        for (unsigned int t = 0; t < num_routing_threads; ++t)
        {
            int start_idx = t * reroute_chunk_size;
            int end_idx = std::min(start_idx + reroute_chunk_size, (int)totalNetsToReroute);
            if (start_idx < end_idx)
            {
                reroute_threads.emplace_back(reroute_worker, start_idx, end_idx);
            }
        }

        // Optional: Progress indicator for the main thread
        while (processedNetCount.load() < totalNetsToReroute)
        {
            std::cout << "  Processed " << processedNetCount.load() << "/" << totalNetsToReroute << " nets for rerouting...\r" << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        std::cout << "  Processed " << processedNetCount.load() << "/" << totalNetsToReroute << " nets for rerouting...     " << std::endl;

        // Join rerouting threads
        for (auto &t : reroute_threads)
        {
            t.join();
        }

        // Removed the old sequential loop
        // for (int netIdx : netsToRoute) { ... }

        std::cout << "\nNets rerouted this iteration. Success: " << reroutedCount << ", Failed: " << failedCount << std::endl;

        // 5. Identify Congestion & Update History Costs
        std::cout << "Checking congestion and updating history costs..." << std::endl;
        std::set<int> congestedNodesThisIter;
        long long totalCongestionScore = 0; // Sum of (occupancy - 1) for congested nodes
        for (auto const &[id, nodePtr] : nodeMap)
        {
            int occ = nodePtr->getOccupancy();
            if (occ > 1)
            {
                congestedNodesThisIter.insert(id);
                nodePtr->setHistoryCost(nodePtr->getHistoryCost() * HISTORY_PENALTY_FACTOR);
                // Cap history cost to prevent extreme values? e.g., std::min(10000.0f, ...)
                totalCongestionScore += (occ - 1);
            }
            // Optional: Decay history cost slightly for non-congested nodes?
            else if (occ <= 1 && nodePtr->getHistoryCost() > INITIAL_HISTORY_COST)
            {
                // nodePtr->setHistoryCost(std::max(INITIAL_HISTORY_COST, nodePtr->getHistoryCost() * 0.95f)); // Decay slowly
            }
        }
        std::cout << "Congested nodes found: " << congestedNodesThisIter.size() << ". Total overuse score: " << totalCongestionScore << std::endl;

        // 6. Determine Nets for Next Iteration
        std::set<int> nextNetsToRoute;
        for (int i = 0; i < nets.size(); ++i)
        {
            Net &net = nets[i];
            bool needsReroute = false;

            // Condition 1: Net failed to route completely in this iteration
            // This check needs to be precise: was it *supposed* to be routed *and* did it fail?
            if (netsToRoute.count(i) && (net.routedSinks < net.sinks.size()))
            {
                needsReroute = true;
            }

            // Condition 2: Net *successfully* routed BUT uses a congested node
            if (!needsReroute && !net.route.empty())
            {                                             // Check only if currently routed
                std::unordered_set<int> nodesInFinalPath; // Recalculate nodes for its current route
                for (const auto &edge : net.route)
                {
                    if (nodeMap.count(edge.first))
                        nodesInFinalPath.insert(edge.first);
                    if (nodeMap.count(edge.second))
                        nodesInFinalPath.insert(edge.second);
                }
                for (int nodeId : nodesInFinalPath)
                {
                    if (congestedNodesThisIter.count(nodeId))
                    {
                        needsReroute = true;
                        break; // No need to check other nodes for this net
                    }
                }
            }

            if (needsReroute)
            {
                nextNetsToRoute.insert(i);
            }
        }
        netsToRoute = std::move(nextNetsToRoute); // Prepare for the next iteration

        auto iterEndTime = std::chrono::high_resolution_clock::now();
        auto iterDuration = std::chrono::duration_cast<std::chrono::milliseconds>(iterEndTime - iterStartTime);
        std::cout << "Iteration finished in " << iterDuration.count() / 1000.0 << " seconds." << std::endl;

    } // End PathFinder Iteration Loop

    if (iteration == MAX_ITERATIONS && !netsToRoute.empty())
    {
        std::cout << "\nWarning: PathFinder reached max iterations (" << MAX_ITERATIONS << ") with "
                  << netsToRoute.size() << " nets still marked for rerouting (likely congested)." << std::endl;
    }
    else if (iteration < MAX_ITERATIONS)
    {
        std::cout << "\nPathFinder converged after " << iteration << " iterations." << std::endl;
    }

    // --- Calculate Final Metrics ---
    std::cout << "\nCalculating final metrics..." << std::endl;
    long long finalTotalWirelength = 0;
    int finalSuccessfullyRoutedNets = 0;
    int finalCongestedNodesCount = 0;            // Count of nodes used by >1 net
    std::unordered_map<int, int> finalNodeUsage; // Track final usage count for each node

    // Populate finalNodeUsage and count successfully routed nets
    for (const auto &net : nets)
    {
        bool fullyRouted = !net.route.empty() && (net.routedSinks == net.sinks.size());
        if (fullyRouted)
        {
            finalSuccessfullyRoutedNets++;
            std::unordered_set<int> nodesInNet;
            for (const auto &edge : net.route)
            {
                if (nodeMap.count(edge.first))
                    nodesInNet.insert(edge.first);
                if (nodeMap.count(edge.second))
                    nodesInNet.insert(edge.second);
            }
            for (int nodeId : nodesInNet)
            {
                if (nodeMap.count(nodeId))
                {
                    finalTotalWirelength += nodeMap.at(nodeId)->getLength();
                    finalNodeUsage[nodeId]++;
                }
            }
        }
    }

    // Count congested nodes based on final usage
    for (auto const &[nodeId, usage] : finalNodeUsage)
    {
        if (usage > 1)
        {
            finalCongestedNodesCount++;
        }
    }

    // --- Write Output File ---
    std::cout << "Writing output file: " << outputFile << std::endl;
    std::ofstream routeFile(outputFile);
    if (!routeFile.is_open())
    {
        std::cerr << "Error opening output file: " << outputFile << std::endl;
        // Fallback: Try writing to current directory?
        outputFile = "routing_output.route";
        std::cerr << "Attempting to write to fallback file: " << outputFile << std::endl;
        routeFile.open(outputFile);
        if (!routeFile.is_open())
        {
            std::cerr << "Error opening fallback output file. Cannot write results." << std::endl;
            return 1; // Exit if cannot write results
        }
    }

    for (const auto &net : nets)
    {
        // Only output fully routed nets
        if (!net.route.empty() && (net.routedSinks == net.sinks.size()))
        {
            routeFile << net.id << " " << net.name << "\n";
            // Create a unique set of edges for this net's output
            // This avoids duplicates if multiple sinks share path segments
            std::set<std::pair<int, int>> uniqueEdges;
            for (const auto &edge : net.route)
            {
                // Ensure consistent ordering (e.g., smaller ID first) if needed, though A* usually gives directed paths
                uniqueEdges.insert({edge.first, edge.second});
            }
            for (const auto &edge : uniqueEdges)
            {
                routeFile << edge.first << " " << edge.second << "\n";
            }
            routeFile << "\n"; // Empty line separator
        }
    }
    routeFile.close();

    // --- End Timing & Print Summary ---
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "\n-------------------- FINAL SUMMARY --------------------" << std::endl;
    std::cout << "Input Device: " << deviceFile << std::endl;
    std::cout << "Input Netlist: " << netlistFile << std::endl;
    std::cout << "Output File: " << outputFile << std::endl;
    std::cout << "Total Runtime: " << duration.count() / 1000.0 << " seconds" << std::endl;
    std::cout << "PathFinder Iterations Completed: " << iteration << std::endl;
    std::cout << "Successfully Routed Nets: " << finalSuccessfullyRoutedNets << " / " << nets.size() << std::endl;
    std::cout << "Total Wirelength: " << finalTotalWirelength << std::endl;
    std::cout << "Final Congested Nodes (Usage > 1): " << finalCongestedNodesCount << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;

    return 0;
}
