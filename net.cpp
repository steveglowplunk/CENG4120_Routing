#pragma once
#include <string>
#include <vector>

class Net
{
public:
    int id;
    std::string name;
    int source;
    std::vector<int> sinks;
    // Holds pairs representing the routed edges (parent, child)
    std::vector<std::pair<int, int>> route;

    Net(int id, const std::string &name, int source)
        : id(id), name(name), source(source) {}
};