#pragma once
#include <unordered_map>
#include <vector>
#include "node.cpp"

class Graph
{
public:
    // std::vector<Node> nodes; // Removed
    std::unordered_map<int, std::vector<int>> adjacency;

    // void addNode(const Node& node) { // Removed
    //     nodes.push_back(node);
    // }

    void addEdges(int parent, const std::vector<int> &children)
    {
        adjacency[parent] = children;
    }
};