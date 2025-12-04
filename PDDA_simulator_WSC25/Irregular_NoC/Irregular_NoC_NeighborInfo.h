#pragma once
#include <vector>
#include <memory>
#include <map>
#include <tuple>
#include <array>
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>

// Include the common definitions
#include "Irregular_NoC_Common.h"

class Irregular_NoC_Arrive;

// Optimized path data structure
struct Irregular_NoC_Path {
    std::array<size_t, NOC_MAX_PATH_LENGTH> nodeIndices; // Fixed-size array
    double totalCost;
    size_t length;  // Tracks actual number of elements used in the array
    
    Irregular_NoC_Path() : totalCost(std::numeric_limits<double>::max()), length(0) {}
    void PrintPath() const;
    
    void AddNode(size_t nodeId) {
        if (length < NOC_MAX_PATH_LENGTH) {
            nodeIndices[length++] = nodeId;
        }
    }
    
    bool operator==(const Irregular_NoC_Path& other) const {
        if (length != other.length || totalCost != other.totalCost) {
            return false;
        }
        
        for (size_t i = 0; i < length; i++) {
            if (nodeIndices[i] != other.nodeIndices[i]) {
                return false;
            }
        }
        return true;
    }
};

// Neighbor info structure for irregular networks
struct Irregular_NoC_NeighborInfo {
    // Original interface members for compatibility
    std::vector<std::vector<size_t>> Indices;  // Hierarchical indices by hop level
    std::vector<std::shared_ptr<Irregular_NoC_Arrive>> Neighbors;  // Neighbor pointers
    std::map<size_t, size_t> globalToLocalIdx;  // Maps network node IDs to indices

    // Optimized internal storage for hop levels
    std::array<std::array<size_t, NOC_MAX_NEIGHBORS_PER_LEVEL>, NOC_MAX_HOP_RADIUS+1> hopLevelNodes;
    std::array<size_t, NOC_MAX_HOP_RADIUS+1> hopLevelCounts;
    std::array<size_t, NOC_MAX_TOTAL_NEIGHBORS> allNeighbors;
    size_t allNeighborCount;
    std::vector<int>& _queueSizes;  // Reference to external queue sizes
    
    // Constructor for irregular network
    Irregular_NoC_NeighborInfo(
        size_t nodeId, 
        const std::vector<std::vector<NetworkLink>>& networkTopology,
        size_t hopRadius, 
        std::vector<int>& queueSizes);

    // Path finding with queue awareness for irregular networks
    Irregular_NoC_Path FindShortestPath(
        size_t sourceNodeID, 
        size_t destNodeID,
        const std::vector<std::vector<NetworkLink>>& topology,
        size_t hopRadius, 
        const std::array<size_t, NOC_MAX_PATH_LENGTH>& visitedNodes,
        size_t visitedNodeCnt, 
        const std::vector<int>& globalQueueSizes,
        const std::vector<double>& queueTimestamps, 
        double currentTime) const;
		
	// Enhanced deterministic shortest path algorithm for breaking out of routing loops
	Irregular_NoC_Path FindDeterministicShortestPath(
    size_t sourceNodeID, 
    size_t destNodeID,
    const std::vector<std::vector<NetworkLink>>& topology,
    const std::vector<int>& visitCountArray,
    int avoidanceStrength=1) const;
    
private:
    size_t _nodeID;  // Current node ID
    mutable size_t _pathFindCalls;  // Stats counters
    
    // Utility methods
    void BuildHopLevels(
        size_t nodeId, 
        const std::vector<std::vector<NetworkLink>>& topology,
        size_t hopRadius);
    
    // Helper to estimate queue size based on history and age
    double EstimateQueueSize(
        size_t nodeId, 
        const std::vector<int>& globalQueueSizes,
        const std::vector<double>& queueTimestamps, 
        double currentTime) const;
};