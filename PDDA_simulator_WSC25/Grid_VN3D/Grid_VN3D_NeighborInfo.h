#pragma once
#include <vector>
#include <memory>
#include <unordered_map>
#include <tuple>
#include <list>
#include <limits>

class Grid_VN3D_Arrive;

struct Path3D {
    std::vector<size_t> nodeIndices;
    double totalCost;
    Path3D() : totalCost(std::numeric_limits<double>::max()) {}
    void PrintPath();
};

struct Grid_VN3D_NeighborInfo {
    std::vector<std::vector<size_t>> I;  // Hierarchical indices
    std::vector<std::shared_ptr<Grid_VN3D_Arrive>> N;  // Flat array of neighbor pointers
    std::unordered_map<size_t, size_t> globalToLocalIdx;  // Maps network node IDs to indices in N

    // Coordinate conversion helpers
    std::tuple<size_t, size_t, size_t> getCoordinates(size_t nodeID, size_t gridSizeX, size_t gridSizeY) const;
    size_t getNodeID(size_t x, size_t y, size_t z, size_t gridSizeX, size_t gridSizeY) const;

    // Neighbor finding
    std::vector<size_t> getGridNeighbors(size_t nodeID, size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ) const;
    std::vector<size_t> getValidNeighbors(size_t nodeID, size_t targetNodeID, size_t hopsFromStart,
                                         size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ) const;

    // Pathfinding
    Path3D findShortestPath(size_t finalDestNodeID, size_t intermedTargetNodeID,
                         size_t current_x, size_t current_y, size_t current_z,
                         size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
                         size_t hopRadius, const std::vector<size_t>& visitedNodes,
						 size_t visitedNodeCnt) const;
};
