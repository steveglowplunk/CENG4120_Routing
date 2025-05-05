#pragma once
#include <string>
#include <vector>
#include <utility> // For std::pair

class Net
{
public:
    int id;
    std::string name;
    int source;
    std::vector<int> sinks;
    // Holds pairs representing the routed edges (parent, child)
    std::vector<std::pair<int, int>> route;
    int routedSinks; // Tracks how many sinks are successfully reached in the current route

    Net(int id, const std::string &name, int source)
        : id(id), name(name), source(source), routedSinks(0) {}
};