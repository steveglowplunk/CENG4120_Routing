#pragma once

struct Node {
    int id;
    int length;
    int begin_x, begin_y, end_x, end_y;

    // Default constructor.
    Node() : id(0), length(0), begin_x(0), begin_y(0), end_x(0), end_y(0) {}

    // Constructor for initialization.
    Node(int id, int length, int begin_x, int begin_y, int end_x, int end_y)
        : id(id), length(length), begin_x(begin_x), begin_y(begin_y), end_x(end_x), end_y(end_y) {}
};