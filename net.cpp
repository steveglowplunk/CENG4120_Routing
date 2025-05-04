#pragma once
#include <string>
#include <vector>

struct Net
{
    int id;
    std::string name;
    int source;
    std::vector<int> sinks;
    std::vector<std::pair<int, int>> route; // Holds pairs representing the routed edges (parent, child)

    Net(int id, const std::string &name, int source)
        : id(id), name(name), source(source) {}
};