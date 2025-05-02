#pragma once
#include <string>

class Node {
	int id;
	std::string type;
	int length;
	int begin_x, begin_y, end_x, end_y;
	std::string name;

public:
	// Constructor
	Node(int id, const std::string& type, int length, int begin_x, int begin_y, int end_x, int end_y, const std::string& name)
		: id(id), type(type), length(length), begin_x(begin_x), begin_y(begin_y), end_x(end_x), end_y(end_y), name(name) {
	}
};
