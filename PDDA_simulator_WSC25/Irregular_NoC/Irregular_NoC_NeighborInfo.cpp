#include "Irregular_NoC_NeighborInfo.h"
#include "Irregular_NoC_Arrive.h"
#include "Irregular_NoC.h"  // For LinkType definition
#include <queue>
#include <algorithm>
#include <cstdio>
#include <cmath>

// Print path implementation
void Irregular_NoC_Path::PrintPath() const {
    printf("Path cost: %.2f, nodes: ", totalCost);
    for (size_t i = 0; i < length; i++) {
        printf("%lu", nodeIndices[i]);
        if (i < length - 1) printf(" → ");
    }
    printf("\n");
}

// Constructor implementation
Irregular_NoC_NeighborInfo::Irregular_NoC_NeighborInfo(
    size_t nodeId, 
    const std::vector<std::vector<NetworkLink>>& networkTopology,
    size_t hopRadius, 
    std::vector<int>& queueSizes)
    : _queueSizes(queueSizes), 
      allNeighborCount(0), 
      _nodeID(nodeId),
      _pathFindCalls(0) {
    
    // Initialize structures
    Indices.clear();
    Indices.resize(hopRadius + 1);
    
    // Build hop levels using BFS
    BuildHopLevels(nodeId, networkTopology, hopRadius);
}

void Irregular_NoC_NeighborInfo::BuildHopLevels(
    size_t nodeId, 
    const std::vector<std::vector<NetworkLink>>& topology,
    size_t hopRadius) {
    
    // Reset hop level data
    for (size_t i = 0; i <= NOC_MAX_HOP_RADIUS; i++) {
        hopLevelCounts[i] = 0;
    }
    allNeighborCount = 0;
    
    // BFS for hop level construction
    std::unordered_set<size_t> visited;
    std::vector<size_t> currentLevel, nextLevel;
    
    // Add source node itself at "level 0" (not actually used for neighbors)
    visited.insert(nodeId);
    
    // Process direct neighbors (hop level 1)
    if (nodeId < topology.size()) {
        for (const auto& link : topology[nodeId]) {
            size_t neighbor = link.targetNodeID;
            
            // Add to visited set to avoid duplicates
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                currentLevel.push_back(neighbor);
                
                // Add to hop level 1
                if (hopLevelCounts[1] < NOC_MAX_NEIGHBORS_PER_LEVEL) {
                    hopLevelNodes[1][hopLevelCounts[1]++] = neighbor;
                }
                
                // Add to flat array of all neighbors
                if (allNeighborCount < NOC_MAX_TOTAL_NEIGHBORS) {
                    allNeighbors[allNeighborCount++] = neighbor;
                }
                
                // Add to Indices for compatibility
                while (Indices.size() <= 1) Indices.push_back({});
                Indices[1].push_back(neighbor);
            }
        }
    }
    
    // Continue BFS for remaining hop levels
    for (size_t level = 2; level <= hopRadius && level <= NOC_MAX_HOP_RADIUS; level++) {
        nextLevel.clear();
        
        // For each node in current level
        for (size_t currentNode : currentLevel) {
            // Process all its neighbors
            if (currentNode < topology.size()) {
                for (const auto& link : topology[currentNode]) {
                    size_t neighbor = link.targetNodeID;
                    
                    // Only add unvisited nodes
                    if (visited.find(neighbor) == visited.end()) {
                        visited.insert(neighbor);
                        nextLevel.push_back(neighbor);
                        
                        // Add to current hop level
                        if (hopLevelCounts[level] < NOC_MAX_NEIGHBORS_PER_LEVEL) {
                            hopLevelNodes[level][hopLevelCounts[level]++] = neighbor;
                        }
                        
                        // Add to flat array of all neighbors
                        if (allNeighborCount < NOC_MAX_TOTAL_NEIGHBORS) {
                            allNeighbors[allNeighborCount++] = neighbor;
                        }
                        
                        // Add to Indices for compatibility
                        while (Indices.size() <= level) Indices.push_back({});
                        Indices[level].push_back(neighbor);
                    }
                }
            }
        }
        
        // Update for next iteration
        currentLevel = nextLevel;
        
        // Break early if no more nodes found at this level
        if (currentLevel.empty()) {
            break;
        }
    }
    
    // Deterministic sorting of indices at each level for consistent behavior
    for (auto& level : Indices) {
        std::sort(level.begin(), level.end());
    }
}

// Helper for queue size estimation with age decay
double Irregular_NoC_NeighborInfo::EstimateQueueSize(
    size_t nodeId, 
    const std::vector<int>& globalQueueSizes,
    const std::vector<double>& queueTimestamps, 
    double currentTime) const {
    
    // If we have data for this node
    if (nodeId < globalQueueSizes.size() && nodeId < queueTimestamps.size() && queueTimestamps[nodeId] > 0) {
        double age = currentTime - queueTimestamps[nodeId];
        int queueSize = globalQueueSizes[nodeId];
        
        // Fresh data (very recent)
        if (age < 0.1) return queueSize;
        
        // Apply age penalty - older data becomes less reliable
        double agePenalty = std::min(20.0, age / 5.0);
        return static_cast<double>(queueSize) + agePenalty;
    }
    
    // Default for unknown nodes - conservative estimate
    return 20.0;
}

// Replace the FindShortestPath and FindDeterministicShortestPath methods with these enhanced versions

Irregular_NoC_Path Irregular_NoC_NeighborInfo::FindShortestPath(
    size_t sourceNodeID, 
    size_t destNodeID,
    const std::vector<std::vector<NetworkLink>>& topology,
    size_t hopRadius, 
    const std::array<size_t, NOC_MAX_PATH_LENGTH>& visitedNodes,
    size_t visitedNodeCnt, 
    const std::vector<int>& globalQueueSizes,
    const std::vector<double>& queueTimestamps, 
    double currentTime) const {
    
    // Return immediately if already at destination
    if (sourceNodeID == destNodeID) {
        Irregular_NoC_Path directPath;
        directPath.AddNode(sourceNodeID);
        directPath.totalCost = 0.0;
        return directPath;
    }
    
    // Get network size for array sizing
    const size_t networkSize = topology.size();
    const size_t ARRAY_SIZE = std::min(networkSize, size_t(4096));
    
    // Use thread-local static arrays for visit count tracking
    // This is much faster than unordered_map for this use case
    static thread_local std::vector<int> visitCountArray;
    
    // Resize if needed (only happens once)
    if (visitCountArray.size() < ARRAY_SIZE) {
        visitCountArray.resize(ARRAY_SIZE, 0);
    } else {
        // Clear previous values
        std::fill(visitCountArray.begin(), visitCountArray.end(), 0);
    }
    
    // Array index mapping function
    auto idx = [ARRAY_SIZE](size_t nodeId) -> size_t { 
        return nodeId % ARRAY_SIZE; 
    };
    
    // Fill visit count array (optimize with direct array access)
    const size_t HISTORY_SIZE = std::min(visitedNodeCnt, size_t(16));
    for (size_t i = visitedNodeCnt > HISTORY_SIZE ? visitedNodeCnt - HISTORY_SIZE : 0; 
         i < visitedNodeCnt; i++) {
        const size_t nodeId = visitedNodes[i];
        visitCountArray[idx(nodeId)]++;
    }
    
    // Fast detection of looping and long paths
    bool isLooping = false;
    for (size_t i = 0; i < ARRAY_SIZE; i++) {
        if (visitCountArray[i] >= 2) {
            isLooping = true;
            break;
        }
    }
    
    // Check path length conditions
    const bool pathGettingLong = (visitedNodeCnt > NOC_MAX_PATH_LENGTH / 2);
    const size_t estimatedDiameter = static_cast<size_t>(std::sqrt(networkSize)) + 5;
    const bool pathTooLong = (visitedNodeCnt > estimatedDiameter * 2);
    
    // Use deterministic fallback for problem cases
    if (isLooping || pathTooLong || pathGettingLong) {
        int avoidanceStrength = 1; // Default
        if (isLooping) avoidanceStrength = 3; // Stronger for detected loops
        if (pathTooLong) avoidanceStrength = 4; // Strongest when path is excessively long
        
        return FindDeterministicShortestPath(sourceNodeID, destNodeID, topology, visitCountArray, avoidanceStrength);
    }
    
    // A* path finding with optimized data structures
    // Use static thread-local arrays where possible to avoid allocations
    static thread_local std::vector<double> gScores(ARRAY_SIZE, std::numeric_limits<double>::infinity());
    static thread_local std::vector<size_t> cameFrom(ARRAY_SIZE, 0);
    static thread_local std::vector<bool> inClosedSet(ARRAY_SIZE, false);
    
    // Reset arrays for this run
    for (size_t i = 0; i < ARRAY_SIZE; i++) {
        gScores[i] = std::numeric_limits<double>::infinity();
        inClosedSet[i] = false;
    }
    
    // Initialize starting node
    gScores[idx(sourceNodeID)] = 0.0;
    
    // Use priority queue with reverse order for A* open set
    // This gives us O(log n) operations with the right ordering
    std::priority_queue<std::pair<double, size_t>, 
                        std::vector<std::pair<double, size_t>>, 
                        std::greater<std::pair<double, size_t>>> openSet;
    
    openSet.push({0.0, sourceNodeID});
    
    // Maximum search iterations to prevent excessive search
    const size_t MAX_ITERATIONS = 64;
    size_t iterations = 0;
    
    // Main A* search loop
    while (!openSet.empty() && iterations++ < MAX_ITERATIONS) {
        // Get node with lowest f-score
        const auto current = openSet.top();
        const size_t currentNodeID = current.second;
        openSet.pop();
        
        // Skip if already processed (using array for faster lookups)
        if (inClosedSet[idx(currentNodeID)]) {
            continue;
        }
        
        // Mark as processed
        inClosedSet[idx(currentNodeID)] = true;
        
        // Check if reached destination
        if (currentNodeID == destNodeID) {
            // Efficiently reconstruct path
            Irregular_NoC_Path resultPath;
            
            // Use fixed-size array for path reconstruction
            std::array<size_t, NOC_MAX_PATH_LENGTH> pathArray;
            size_t pathLen = 0;
            
            // Build path in reverse order with array
            size_t node = destNodeID;
            while (node != sourceNodeID && pathLen < NOC_MAX_PATH_LENGTH) {
                pathArray[pathLen++] = node;
                node = cameFrom[idx(node)];
            }
            
            // Add source node
            if (pathLen < NOC_MAX_PATH_LENGTH) {
                pathArray[pathLen++] = sourceNodeID;
            }
            
            // Build path in correct order
            for (int i = pathLen - 1; i >= 0; i--) {
                resultPath.AddNode(pathArray[i]);
            }
            
            resultPath.totalCost = gScores[idx(destNodeID)];
            return resultPath;
        }
        
        // Process neighbors
        if (currentNodeID < topology.size()) {
            // Cache the current gScore for current node
            const double currentGScore = gScores[idx(currentNodeID)];
            
            for (const auto& link : topology[currentNodeID]) {
                const size_t neighborID = link.targetNodeID;
                
                // Skip if already processed
                if (inClosedSet[idx(neighborID)]) {
                    continue;
                }
                
                // Fast visit penalty calculation using array
                double revisitPenalty = 0.0;
                const int visitCount = visitCountArray[idx(neighborID)];
                if (visitCount > 0) {
                    // Optimized exponential calculation (2^n is just bit shifting)
                    revisitPenalty = 50.0 * (1 << visitCount);
                }
                
                // Optimized queue size estimation
                double queueSize = 1.0; // Default small cost
                
                // Directly check array bounds before accessing
                if (neighborID < globalQueueSizes.size() && 
                    neighborID < queueTimestamps.size()) {
                    
                    const double timestamp = queueTimestamps[neighborID];
                    if (timestamp > 0) {
                        const double age = currentTime - timestamp;
                        // Fast path for valid queue data
                        queueSize = globalQueueSizes[neighborID];
                        
                        // Simplified age penalty logic
                        if (age > 5.0) queueSize = std::min(queueSize, 3.0);
                        if (age > 20.0) queueSize = 1.0;
                    }
                }
                
                // Simplified cost calculation
                double linkCost;
                if (link.type == LinkType::EXPRESS_LINK) {
                    linkCost = (queueSize * 0.5 + 0.5) * link.latencyFactor;
                } else {
                    linkCost = (queueSize + 1.0) * link.latencyFactor;
                }
                
                // Calculate total cost
                const double tentativeGScore = currentGScore + linkCost + revisitPenalty;
                
                // Check if this is a better path
                if (tentativeGScore < gScores[idx(neighborID)]) {
                    // Update path info
                    cameFrom[idx(neighborID)] = currentNodeID;
                    gScores[idx(neighborID)] = tentativeGScore;
                    
                    // Simple heuristic calculation
                    const double heuristic = (link.type == LinkType::EXPRESS_LINK) ? 0.3 : 1.0;
                    
                    // Add to open set
                    openSet.push({tentativeGScore + heuristic, neighborID});
                }
            }
        }
    }
    
    // Fallback path handling - use a neighbor with minimal visit history
    Irregular_NoC_Path fallbackPath;
    fallbackPath.AddNode(sourceNodeID);
    
    // Look for unvisited neighbors, with preference for express links
    size_t bestNeighbor = 0;
    int lowestVisits = std::numeric_limits<int>::max();
    bool hasExpressLink = false;
    bool hasNeighbor = false;
    
    // Process all neighbors in one pass for efficiency
    if (sourceNodeID < topology.size()) {
        for (const auto& link : topology[sourceNodeID]) {
            const size_t neighborID = link.targetNodeID;
            const int visits = visitCountArray[idx(neighborID)];
            const bool isExpress = (link.type == LinkType::EXPRESS_LINK);
            
            // Prefer express links with low visit count, then any neighbor with low visit count
            if (!hasNeighbor || 
                (isExpress && !hasExpressLink) ||
                (isExpress == hasExpressLink && visits < lowestVisits)) {
                
                bestNeighbor = neighborID;
                lowestVisits = visits;
                hasExpressLink = isExpress;
                hasNeighbor = true;
            }
        }
        
        // Add best neighbor to path
        if (hasNeighbor) {
            fallbackPath.AddNode(bestNeighbor);
        } else if (!topology[sourceNodeID].empty()) {
            // Deterministic selection to avoid getting stuck
            size_t index = static_cast<size_t>(currentTime * 1000) % topology[sourceNodeID].size();
            fallbackPath.AddNode(topology[sourceNodeID][index].targetNodeID);
        }
    }
    
    fallbackPath.totalCost = 1.0;
    return fallbackPath;
}

// Optimized deterministic routing with variable avoidance strength
Irregular_NoC_Path Irregular_NoC_NeighborInfo::FindDeterministicShortestPath(
    size_t sourceNodeID, 
    size_t destNodeID,
    const std::vector<std::vector<NetworkLink>>& topology,
    const std::vector<int>& visitCountArray,
    int avoidanceStrength) const {
    
    // Network dimensions
    const size_t networkSize = topology.size();
    const size_t ARRAY_SIZE = std::min(networkSize, size_t(4096));
    
    // Array index mapping function
    auto idx = [ARRAY_SIZE](size_t nodeId) -> size_t { 
        return nodeId % ARRAY_SIZE; 
    };
    
    // Use static thread-local arrays for Dijkstra's algorithm
    static thread_local std::vector<double> distance(ARRAY_SIZE, std::numeric_limits<double>::infinity());
    static thread_local std::vector<size_t> predecessor(ARRAY_SIZE, 0);
    static thread_local std::vector<bool> visited(ARRAY_SIZE, false);
    
    // Reset arrays
    for (size_t i = 0; i < ARRAY_SIZE; i++) {
        distance[i] = std::numeric_limits<double>::infinity();
        visited[i] = false;
    }
    
    // Initialize source
    distance[idx(sourceNodeID)] = 0;
    
    // Priority queue for Dijkstra's algorithm - FIXED SYNTAX
    using PQEntry = std::pair<double, size_t>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
    
    pq.push({0, sourceNodeID});
    
    // Main Dijkstra loop
    while (!pq.empty()) {
        const size_t u = pq.top().second;
        const double dist_u = pq.top().first;
        pq.pop();
        
        // If we've reached destination, stop
        if (u == destNodeID) {
            break;
        }
        
        // Skip if already visited
        if (visited[idx(u)]) {
            continue;
        }
        
        visited[idx(u)] = true;
        
        // Skip if we've found a better path already
        if (dist_u > distance[idx(u)]) {
            continue;
        }
        
        // Process neighbors
        if (u < topology.size()) {
            for (const auto& link : topology[u]) {
                const size_t v = link.targetNodeID;
                
                // Calculate base weight with optimized express link handling
                double weight = (link.type == LinkType::EXPRESS_LINK) ? 0.4 : 1.0;
                
                // Apply visit history penalty
                const int visitCount = visitCountArray[idx(v)];
                if (visitCount > 0) {
                    weight *= (1.0 + 5.0 * avoidanceStrength * visitCount);
                }
                
                // Calculate new distance
                const double alt = distance[idx(u)] + weight;
                
                // Update if better
                if (alt < distance[idx(v)]) {
                    distance[idx(v)] = alt;
                    predecessor[idx(v)] = u;
                    pq.push({alt, v});
                }
            }
        }
    }
    
    // Reconstruct path
    Irregular_NoC_Path resultPath;
    
    // Check if destination was reached
    if (distance[idx(destNodeID)] != std::numeric_limits<double>::infinity()) {
        // Complete path found - use fixed array for reconstruction
        std::array<size_t, NOC_MAX_PATH_LENGTH> pathArray;
        size_t pathLen = 0;
        
        // Build path in reverse order
        size_t current = destNodeID;
        while (current != sourceNodeID && pathLen < NOC_MAX_PATH_LENGTH) {
            pathArray[pathLen++] = current;
            current = predecessor[idx(current)];
        }
        
        // Add source node
        if (pathLen < NOC_MAX_PATH_LENGTH) {
            pathArray[pathLen++] = sourceNodeID;
        }
        
        // Build path in correct order
        for (int i = pathLen - 1; i >= 0; i--) {
            resultPath.AddNode(pathArray[i]);
        }
    } else {
        // No path found - create a fallback path
        resultPath.AddNode(sourceNodeID);
        
        // Find neighbor with lowest visit count
        size_t bestNeighbor = 0;
        int lowestVisits = std::numeric_limits<int>::max();
        bool hasNeighbor = false;
        
        if (sourceNodeID < topology.size()) {
            for (const auto& link : topology[sourceNodeID]) {
                const size_t neighborID = link.targetNodeID;
                const int visits = visitCountArray[idx(neighborID)];
                
                if (!hasNeighbor || visits < lowestVisits) {
                    bestNeighbor = neighborID;
                    lowestVisits = visits;
                    hasNeighbor = true;
                }
            }
            
            if (hasNeighbor) {
                resultPath.AddNode(bestNeighbor);
            }
        }
    }
    
    resultPath.totalCost = resultPath.length - 1;
    return resultPath;
}