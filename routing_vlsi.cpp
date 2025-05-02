#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <sstream>
#include <limits>
#include "graph.cpp"

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <device_file>" << std::endl;
        system("pause"); // For Windows users to see the output before closing the console
        return 1;
    }

    std::ifstream infile(argv[1]);
    if (!infile.is_open())
    {
        std::cerr << "Error opening device file: " << argv[1] << std::endl;
        return 1;
    }

    int num_nodes;
    infile >> num_nodes;
    infile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    Graph graph;

    // Parse nodes
    for (int i = 0; i < num_nodes; ++i)
    {
        int id, length, begin_x, begin_y, end_x, end_y;
        std::string type, name;
        infile >> id >> type >> length >> begin_x >> begin_y >> end_x >> end_y;
        std::getline(infile, name);
        if (!name.empty() && name[0] == ' ')
            name = name.substr(1);
        graph.addNode(Node(id, type, length, begin_x, begin_y, end_x, end_y, name));
        std::cout << "Parsed node: " << id << " " << type << " " << length << " " << begin_x << " " << begin_y << " " << end_x << " " << end_y << " " << name << std::endl;
    }

    // Parse adjacency list
    std::string line;
    while (std::getline(infile, line))
    {
        if (line.empty())
            continue;
        std::istringstream iss(line);
        int parent;
        iss >> parent;
        std::vector<int> children;
        int child;
        while (iss >> child)
        {
            children.push_back(child);
        }
        graph.addEdges(parent, children);
        std::cout << "Parsed edges for parent: " << parent << " with children: ";
        for (const auto &child : children)
        {
            std::cout << child << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "Parsed " << graph.nodes.size() << " nodes and " << graph.adjacency.size() << " adjacency entries." << std::endl;

    // TODO: Add netlist parsing and routing logic here

    return 0;
}
