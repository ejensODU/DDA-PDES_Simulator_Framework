#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <list>
#include <limits>
#include <algorithm>

class Grid_VN2D_Arrive;

struct Grid_VN2D_Path {
    std::vector<size_t> nodeIndices;  // Sequence of node IDs in path
    double totalCost;                 // Sum of queue sizes along path

    Grid_VN2D_Path() : totalCost(std::numeric_limits<double>::max()) {}
    void PrintPath();
};

struct Grid_VN2D_NeighborInfo {
    std::vector<std::vector<size_t>> I;  // Hierarchical indices
    std::vector<std::shared_ptr<Grid_VN2D_Arrive>> N;  // Flat array of neighbor pointers
    std::unordered_map<size_t, size_t> globalToLocalIdx;  // Maps network node IDs to indices in N

    // Helper functions for coordinate conversion
    std::pair<size_t, size_t> getCoordinates(size_t nodeID, size_t gridSizeX) const;
    size_t getNodeID(size_t x, size_t y, size_t gridSizeX) const;

    // Neighbor finding
    std::vector<size_t> getGridNeighbors(size_t nodeID, size_t gridSizeX, size_t gridSizeY) const;
    std::vector<size_t> getValidNeighbors(size_t nodeID, size_t targetNodeID, size_t hopsFromStart,
                                         size_t gridSizeX, size_t gridSizeY) const;

    // Path finding
    Grid_VN2D_Path findShortestPath(size_t finalDestNodeID, size_t intermedTargetNodeID,
                                   size_t current_x, size_t current_y,
                                   size_t gridSizeX, size_t gridSizeY,
                                   size_t hopRadius,
                                   const std::vector<size_t>& visitedNodes,
								   size_t visitedNodeCnt) const;
};
