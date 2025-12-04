#include "Grid_VN3D_NeighborInfo.h"
#include "Grid_VN3D_Arrive.h"
#include <queue>
#include <algorithm>
#include <cstdio>

void Path3D::PrintPath() {
    printf("%lf: ", totalCost);
    for (auto index: nodeIndices) printf("%lu,", index);
    printf("\n");
}

std::tuple<size_t, size_t, size_t> Grid_VN3D_NeighborInfo::getCoordinates(size_t nodeID, size_t gridSizeX, size_t gridSizeY) const {
    size_t x = nodeID % gridSizeX;
    size_t y = (nodeID / gridSizeX) % gridSizeY;
    size_t z = nodeID / (gridSizeX * gridSizeY);
    return {x, y, z};
}

size_t Grid_VN3D_NeighborInfo::getNodeID(size_t x, size_t y, size_t z, size_t gridSizeX, size_t gridSizeY) const {
    return z * gridSizeX * gridSizeY + y * gridSizeX + x;
}

std::vector<size_t> Grid_VN3D_NeighborInfo::getGridNeighbors(size_t nodeID, size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ) const {
    std::vector<size_t> neighbors;
    auto [x, y, z] = getCoordinates(nodeID, gridSizeX, gridSizeY);

    // Check all six directions in 3D
    if (x > 0) neighbors.push_back(getNodeID(x-1, y, z, gridSizeX, gridSizeY));
    if (x < gridSizeX - 1) neighbors.push_back(getNodeID(x+1, y, z, gridSizeX, gridSizeY));
    if (y > 0) neighbors.push_back(getNodeID(x, y-1, z, gridSizeX, gridSizeY));
    if (y < gridSizeY - 1) neighbors.push_back(getNodeID(x, y+1, z, gridSizeX, gridSizeY));
    if (z > 0) neighbors.push_back(getNodeID(x, y, z-1, gridSizeX, gridSizeY));
    if (z < gridSizeZ - 1) neighbors.push_back(getNodeID(x, y, z+1, gridSizeX, gridSizeY));

    return neighbors;
}

std::vector<size_t> Grid_VN3D_NeighborInfo::getValidNeighbors(size_t nodeID, size_t targetNodeID, size_t hopsFromStart,
size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ) const {
auto allNeighbors = getGridNeighbors(nodeID, gridSizeX, gridSizeY, gridSizeZ);
std::vector<size_t> validNeighbors;
const auto& hopLevelNeighbors = I.at(hopsFromStart);

auto [node_x, node_y, node_z] = getCoordinates(nodeID, gridSizeX, gridSizeY);
auto [target_x, target_y, target_z] = getCoordinates(targetNodeID, gridSizeX, gridSizeY);

size_t current_distance =
abs(static_cast<int>(node_x) - static_cast<int>(target_x)) +
abs(static_cast<int>(node_y) - static_cast<int>(target_y)) +
abs(static_cast<int>(node_z) - static_cast<int>(target_z));

for (size_t neighborID : allNeighbors) {
auto [neighbor_x, neighbor_y, neighbor_z] = getCoordinates(neighborID, gridSizeX, gridSizeY);
size_t neighbor_distance =
abs(static_cast<int>(neighbor_x) - static_cast<int>(target_x)) +
abs(static_cast<int>(neighbor_y) - static_cast<int>(target_y)) +
abs(static_cast<int>(neighbor_z) - static_cast<int>(target_z));

if (neighbor_distance >= current_distance) continue;

if (std::find(hopLevelNeighbors.begin(), hopLevelNeighbors.end(), neighborID) != hopLevelNeighbors.end()) {
validNeighbors.push_back(neighborID);
}
}
return validNeighbors;
}

Path3D Grid_VN3D_NeighborInfo::findShortestPath(size_t finalDestNodeID, size_t intermedTargetNodeID, size_t current_x, size_t current_y, size_t current_z, size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ, size_t hopRadius, const std::vector<size_t>& visitedNodes, size_t visitedNodeCnt) const
{
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
    size_t currentNodeID = current_z * gridSizeX * gridSizeY + current_y * gridSizeX + current_x;

    auto [FD_x, FD_y, FD_z] = getCoordinates(finalDestNodeID, gridSizeX, gridSizeY);

    size_t current_distance_FD_node =
    abs(static_cast<int>(FD_x) - static_cast<int>(current_x)) +
    abs(static_cast<int>(FD_y) - static_cast<int>(current_y)) +
    abs(static_cast<int>(FD_z) - static_cast<int>(current_z));

    queue.push({currentNodeID, {currentNodeID}, 0.0, 0});
    bestCosts[currentNodeID] = 0.0;

    while (!queue.empty()) {
        auto current = queue.top();
        queue.pop();

        if (current.nodeID == intermedTargetNodeID) {
            Path3D result;
            result.nodeIndices = current.path;
            result.totalCost = current.costSoFar;
            return result;
        }

        if (current.hopsFromStart == hopRadius) {
            continue;
        }

        auto neighbors = getValidNeighbors(current.nodeID, intermedTargetNodeID,
                                            current.hopsFromStart, gridSizeX, gridSizeY, gridSizeZ);

        for (size_t neighborID : neighbors) {
            if (std::find(current.path.begin(), current.path.end(), neighborID) != current.path.end()) {
                continue;
            }
             if (std::find(visitedNodes.begin(), visitedNodes.begin() + visitedNodeCnt, neighborID) != visitedNodes.begin() + visitedNodeCnt) {
                continue;
            }

            auto [neighbor_x, neighbor_y, neighbor_z] = getCoordinates(neighborID, gridSizeX, gridSizeY);
            size_t neighbor_distance_FD_node =
            abs(static_cast<int>(FD_x) - static_cast<int>(neighbor_x)) +
            abs(static_cast<int>(FD_y) - static_cast<int>(neighbor_y)) +
            abs(static_cast<int>(FD_z) - static_cast<int>(neighbor_z));

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

    return Path3D();
}
