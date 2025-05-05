#pragma once
#include <atomic> // Include for std::atomic

class Node {
    int id;
    int length;
    int begin_x, begin_y, end_x, end_y;
    // PathFinder cost components
    float baseCost;
    std::atomic<int> occupancy; // Use atomic for thread safety
    float historyCost;


public:
    // Default constructor.
    Node() : id(0), length(0), begin_x(0), begin_y(0), end_x(0), end_y(0),
             baseCost(1.0f), occupancy(0), historyCost(1.0f) {}

    // Constructor without type and name.
    Node(int id, int length, int begin_x, int begin_y, int end_x, int end_y)
        : id(id), length(length), begin_x(begin_x), begin_y(begin_y), end_x(end_x), end_y(end_y),
          baseCost(1.0f), occupancy(0), historyCost(1.0f) { // Initialize new members
          // Optionally, baseCost could be related to length: baseCost = 1.0f + 0.1f * length;
    }
    
    // Getters for A* heuristic
    int getId() const { return id; }
    int getBeginX() const { return begin_x; }
    int getBeginY() const { return begin_y; }
    int getLength() const { return length; }

    // Getters and setters/modifiers for PathFinder costs
    float getBaseCost() const { return baseCost; }
    int getOccupancy() const { return occupancy.load(); } // Atomically load occupancy
    float getHistoryCost() const { return historyCost; }

    void incrementOccupancy() { occupancy++; } // Atomically increment
    void decrementOccupancy() { occupancy--; } // Atomically decrement
    void resetOccupancy() { occupancy.store(0); } // Atomically reset
    
    void setHistoryCost(float cost) { historyCost = cost; }
    void setBaseCost(float cost) { baseCost = cost; }

};