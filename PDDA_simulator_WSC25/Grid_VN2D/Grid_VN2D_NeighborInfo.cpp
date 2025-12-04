#include "Grid_VN2D_NeighborInfo.h"
#include "Grid_VN2D_Arrive.h"
#include <queue>
#include <algorithm>
#include <cstdio>

void Grid_VN2D_Path::PrintPath() {
    printf("%lf: ", totalCost);
    for (auto index: nodeIndices) printf("%lu,", index);
    printf("\n");
}

std::pair<size_t, size_t> Grid_VN2D_NeighborInfo::getCoordinates(size_t nodeID, size_t gridSizeX) const {
    return {nodeID % gridSizeX, nodeID / gridSizeX};
}

size_t Grid_VN2D_NeighborInfo::getNodeID(size_t x, size_t y, size_t gridSizeX) const {
    return y * gridSizeX + x;
}

std::vector<size_t> Grid_VN2D_NeighborInfo::getGridNeighbors(size_t nodeID, size_t gridSizeX, size_t gridSizeY) const {
    std::vector<size_t> neighbors;
    auto [x, y] = getCoordinates(nodeID, gridSizeX);

    // Check each direction
    if (x > 0) {  // West
        neighbors.push_back(getNodeID(x-1, y, gridSizeX));
    }
    if (x < gridSizeX - 1) {  // East
        neighbors.push_back(getNodeID(x+1, y, gridSizeX));
    }
    if (y > 0) {  // North
        neighbors.push_back(getNodeID(x, y-1, gridSizeX));
    }
    if (y < gridSizeY - 1) {  // South
        neighbors.push_back(getNodeID(x, y+1, gridSizeX));
    }

    return neighbors;
}

std::vector<size_t> Grid_VN2D_NeighborInfo::getValidNeighbors(size_t nodeID, size_t targetNodeID,
                                                         size_t hopsFromStart,
                                                         size_t gridSizeX, size_t gridSizeY) const {
    auto allNeighbors = getGridNeighbors(nodeID, gridSizeX, gridSizeY);
    std::vector<size_t> validNeighbors;
    const auto& hopLevelNeighbors = I.at(hopsFromStart);

    size_t node_x = nodeID % gridSizeX;
    size_t node_y = nodeID / gridSizeX;
    size_t target_node_x = targetNodeID % gridSizeX;
    size_t target_node_y = targetNodeID / gridSizeX;

    size_t current_distance = abs(static_cast<int>(node_x) - static_cast<int>(target_node_x)) +
                             abs(static_cast<int>(node_y) - static_cast<int>(target_node_y));

    for (size_t neighborID : allNeighbors) {
        size_t neighbor_node_x = neighborID % gridSizeX;
        size_t neighbor_node_y = neighborID / gridSizeX;
        size_t neighbor_distance = abs(static_cast<int>(neighbor_node_x) - static_cast<int>(target_node_x)) +
                                 abs(static_cast<int>(neighbor_node_y) - static_cast<int>(target_node_y));

        if (neighbor_distance >= current_distance) continue;

        if (std::find(hopLevelNeighbors.begin(), hopLevelNeighbors.end(), neighborID) != hopLevelNeighbors.end()) {
            validNeighbors.push_back(neighborID);
        }
    }
    return validNeighbors;
}

Grid_VN2D_Path Grid_VN2D_NeighborInfo::findShortestPath(size_t finalDestNodeID, size_t intermedTargetNodeID,
                                                     size_t current_x, size_t current_y,
                                                     size_t gridSizeX, size_t gridSizeY,
                                                     size_t hopRadius,
                                                     const std::vector<size_t>& visitedNodes,
													 size_t visitedNodeCnt) const {
    struct QueueEntry {
        size_t nodeID;
        std::vector<size_t> path;
        double costSoFar;
        size_t hopsFromStart;

        bool operator>(const QueueEntry& other) const {
            return costSoFar > other.costSoFar;
        }
    };

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
    std::unordered_map<size_t, double> bestCosts;
    size_t currentNodeID = current_y * gridSizeX + current_x;

    size_t FD_x = finalDestNodeID % gridSizeX;
    size_t FD_y = finalDestNodeID / gridSizeX;

    size_t current_distance_FD_node = abs(static_cast<int>(FD_x) - static_cast<int>(current_x)) +
                                    abs(static_cast<int>(FD_y) - static_cast<int>(current_y));

    queue.push({currentNodeID, {currentNodeID}, 0.0, 0});
    bestCosts[currentNodeID] = 0.0;

    while (!queue.empty()) {
        auto current = queue.top();
        queue.pop();

        if (current.nodeID == intermedTargetNodeID) {
            Grid_VN2D_Path result;
            result.nodeIndices = current.path;
            result.totalCost = current.costSoFar;
            return result;
        }

        if (current.hopsFromStart == hopRadius) {
            continue;
        }

        auto neighbors = getValidNeighbors(current.nodeID, intermedTargetNodeID,
                                         current.hopsFromStart, gridSizeX, gridSizeY);

        for (size_t neighborID : neighbors) {
            if (std::find(current.path.begin(), current.path.end(), neighborID) != current.path.end()) {
                continue;
            }
            if (std::find(visitedNodes.begin(), visitedNodes.begin() + visitedNodeCnt, neighborID) != visitedNodes.begin() + visitedNodeCnt) {
                continue;
            }

            size_t neighbor_x = neighborID % gridSizeX;
            size_t neighbor_y = neighborID / gridSizeX;

            size_t neighbor_distance_FD_node = abs(static_cast<int>(FD_x) - static_cast<int>(neighbor_x)) +
                                             abs(static_cast<int>(FD_y) - static_cast<int>(neighbor_y));

            if (neighbor_distance_FD_node >= current_distance_FD_node) {
                continue;
            }

            auto local_idx = globalToLocalIdx.at(neighborID);
            double queueSize = std::max(0.0, static_cast<double>(N[local_idx]->getPacketQueueSV().get()));
            double newCost = current.costSoFar + queueSize;

            if (bestCosts.find(neighborID) == bestCosts.end() || newCost < bestCosts[neighborID]) {
                bestCosts[neighborID] = newCost;
                std::vector<size_t> newPath = current.path;
                newPath.push_back(neighborID);
                queue.push({neighborID, newPath, newCost, current.hopsFromStart + 1});
            }
        }
    }

    return Grid_VN2D_Path();
}
