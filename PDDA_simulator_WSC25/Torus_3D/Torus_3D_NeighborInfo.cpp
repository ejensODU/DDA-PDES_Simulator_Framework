#include "Torus_3D_NeighborInfo.h"
#include "Torus_3D_Arrive.h"
#include <queue>
#include <algorithm>
#include <cstdio>
#include <cfloat>
//#include <set>
//#include <map>

#include <unordered_set>
#include <unordered_map>

// Updated PrintPath method
void Torus_3D_Path::PrintPath() const {
    printf("%lf: ", totalCost);
    for (size_t i = 0; i < length; i++) {
        printf("%lu,", nodeIndices[i]);
    }
    printf("\n");
}

Torus_3D_NeighborInfo::Torus_3D_NeighborInfo(
    size_t nodeId, size_t x, size_t y, size_t z,
    size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
    size_t hopRadius, std::vector<int>& queueSizes) 
    : _queueSizes(queueSizes), allNeighborCount(0), _nodeID(nodeId),
	_fullPathsCnt(0), _quickPathsCnt(0), _pathFindsCnt(0),
	_numNodes(gridSizeX*gridSizeY*gridSizeZ),
	_avgQueueSize(20), _queueSizeUpdateCounter(0){
    // Clear and reset vectors for compatibility
    Indices.clear();
    Indices.resize(hopRadius + 1);
    
    // Reset all counters
    allNeighborCount = 0;
    for (size_t i = 0; i <= T3D_MAX_HOP_RADIUS; i++) {
        hopLevelCounts[i] = 0;
    }
    
    // Initialize direct neighbor indices
    for (int i = 0; i < 6; i++) {
        directNeighborIndices[i] = -1;
    }
    
    // Create a bitmap to track visited nodes
    std::vector<bool> visited(gridSizeX * gridSizeY * gridSizeZ, false);
    
    // Mark self as visited
    visited[nodeId] = true;
    
    // Add immediate neighbors at hop level 0
    // Get the 6 direct neighbors in a fixed order
    std::array<size_t, 6> neighbors;
    
    // Calculate wrapped coordinates for all six directions
    size_t west_x = WrapCoordinate(x - 1, gridSizeX);
    size_t east_x = WrapCoordinate(x + 1, gridSizeX);
    size_t north_y = WrapCoordinate(y - 1, gridSizeY);
    size_t south_y = WrapCoordinate(y + 1, gridSizeY);
    size_t down_z = WrapCoordinate(z - 1, gridSizeZ);
    size_t up_z = WrapCoordinate(z + 1, gridSizeZ);
    
    // Add all six neighbors in a fixed order
    neighbors[0] = GetNodeID(west_x, y, z, gridSizeX, gridSizeY);  // West
    neighbors[1] = GetNodeID(east_x, y, z, gridSizeX, gridSizeY);  // East
    neighbors[2] = GetNodeID(x, north_y, z, gridSizeX, gridSizeY); // North
    neighbors[3] = GetNodeID(x, south_y, z, gridSizeX, gridSizeY); // South
    neighbors[4] = GetNodeID(x, y, down_z, gridSizeX, gridSizeY);  // Down
    neighbors[5] = GetNodeID(x, y, up_z, gridSizeX, gridSizeY);    // Up
    
    // Create vector for hop level 0 compatibility
    std::vector<size_t> level0;
    
    // Process immediate neighbors (hop level 0)
    hopLevelCounts[0] = 6;
    for (int i = 0; i < 6; i++) {
        size_t neighbor = neighbors[i];
        
        // Add to hop level 0
        hopLevelNodes[0][i] = neighbor;
        visited[neighbor] = true;
        level0.push_back(neighbor);
        
        // Add to flat array
        if (allNeighborCount < T3D_MAX_TOTAL_NEIGHBORS) {
            int idx = allNeighborCount;
            allNeighbors[idx] = neighbor;
            directNeighborIndices[i] = neighbor;
            allNeighborCount++;
        }
    }
    
    // Sort level0 deterministically (not strictly necessary as we know the order,
    // but done for consistency with other levels)
    SortNodeIndices(level0);
    
    // Add level 0 to I for compatibility
    Indices[0] = level0;
    
    // BFS for remaining hop levels
    if (hopRadius > 1) {
        std::array<size_t, 6> nextNeighbors;
        
        for (size_t level = 1; level < std::min(hopRadius, T3D_MAX_HOP_RADIUS); level++) {
            size_t prevLevelCount = hopLevelCounts[level - 1];
            std::vector<size_t> currentLevelNodes;
            
            // Process previous level nodes in a deterministic order
            std::vector<size_t> prevLevelNodesSorted;
            for (size_t i = 0; i < prevLevelCount; i++) {
                prevLevelNodesSorted.push_back(hopLevelNodes[level - 1][i]);
            }
            SortNodeIndices(prevLevelNodesSorted);
            
            for (size_t currentNode : prevLevelNodesSorted) {
                // Get immediate neighbors of this node
                auto [nx, ny, nz] = GetCoordinates(currentNode, gridSizeX, gridSizeY);
                
                // Calculate wrapped coordinates for all six directions
                west_x = WrapCoordinate(nx - 1, gridSizeX);
                east_x = WrapCoordinate(nx + 1, gridSizeX);
                north_y = WrapCoordinate(ny - 1, gridSizeY);
                south_y = WrapCoordinate(ny + 1, gridSizeY);
                down_z = WrapCoordinate(nz - 1, gridSizeZ);
                up_z = WrapCoordinate(nz + 1, gridSizeZ);
                
                // Add all six neighbors in a fixed order
                nextNeighbors[0] = GetNodeID(west_x, ny, nz, gridSizeX, gridSizeY);  // West
                nextNeighbors[1] = GetNodeID(east_x, ny, nz, gridSizeX, gridSizeY);  // East
                nextNeighbors[2] = GetNodeID(nx, north_y, nz, gridSizeX, gridSizeY); // North
                nextNeighbors[3] = GetNodeID(nx, south_y, nz, gridSizeX, gridSizeY); // South
                nextNeighbors[4] = GetNodeID(nx, ny, down_z, gridSizeX, gridSizeY);  // Down
                nextNeighbors[5] = GetNodeID(nx, ny, up_z, gridSizeX, gridSizeY);    // Up
                
                // Process each neighbor in a fixed order
                for (int j = 0; j < 6; j++) {
                    size_t neighbor = nextNeighbors[j];
                    
                    // Skip nodes we've already seen
                    if (visited[neighbor]) {
                        continue;
                    }
                    
                    // Add to current hop level
                    size_t levelCount = hopLevelCounts[level];
                    if (levelCount < T3D_MAX_NEIGHBORS_PER_LEVEL) {
                        hopLevelNodes[level][levelCount] = neighbor;
                        hopLevelCounts[level]++;
                        visited[neighbor] = true;
                        currentLevelNodes.push_back(neighbor);
                        
                        // Also add to flat array
                        if (allNeighborCount < T3D_MAX_TOTAL_NEIGHBORS) {
                            int idx = allNeighborCount;
                            allNeighbors[idx] = neighbor;
                            allNeighborCount++;
                        } else {
                            // We've hit the maximum number of neighbors we can track
                            break;
                        }
                    } else {
                        // We've hit the maximum number of neighbors at this level
                        break;
                    }
                }
                
                // Stop if we've reached the maximum total neighbors
                if (allNeighborCount >= T3D_MAX_TOTAL_NEIGHBORS) {
                    break;
                }
            }
            
            // Sort level nodes deterministically
            SortNodeIndices(currentLevelNodes);
            
            // Add this level's nodes to I for compatibility
            Indices[level] = currentLevelNodes;
        }
    }
}

// Helper to sort vectors of nodes consistently
void Torus_3D_NeighborInfo::SortNodeIndices(std::vector<size_t>& nodes) const {
    // Simple ascending order sort
    std::sort(nodes.begin(), nodes.end());
}

std::tuple<size_t, size_t, size_t> Torus_3D_NeighborInfo::GetCoordinates(
    size_t nodeID, size_t gridSizeX, size_t gridSizeY) const {
    
    size_t x = nodeID & (gridSizeX - 1);
    size_t y = (nodeID / gridSizeX) & (gridSizeY - 1);
    size_t z = nodeID / (gridSizeX * gridSizeY);
    return {x, y, z};
}

size_t Torus_3D_NeighborInfo::GetNodeID(
    size_t x, size_t y, size_t z, size_t gridSizeX, size_t gridSizeY) const {
    
    return z * gridSizeX * gridSizeY + y * gridSizeX + x;
}

std::vector<size_t> Torus_3D_NeighborInfo::GetTorusNeighbors(
    size_t nodeID, size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ) const {
	
    // Use static cache for all nodes - lives as long as the program
    static std::map<std::tuple<size_t, size_t, size_t, size_t>, std::array<size_t, 6>> neighborCache;
    
    // Create a key for the cache that includes grid dimensions
    auto cacheKey = std::make_tuple(nodeID, gridSizeX, gridSizeY, gridSizeZ);
    
    // Check if we have this in our cache
    auto it = neighborCache.find(cacheKey);
    if (it != neighborCache.end()) {
        // Convert array to vector and return
        return std::vector<size_t>(it->second.begin(), it->second.end());
    }
    
    // Calculate wrapped coordinates
    auto [x, y, z] = GetCoordinates(nodeID, gridSizeX, gridSizeY);
    
    // Calculate wrapped coordinates for all six directions
    size_t west_x = WrapCoordinate(x - 1, gridSizeX);
    size_t east_x = WrapCoordinate(x + 1, gridSizeX);
    size_t north_y = WrapCoordinate(y - 1, gridSizeY);
    size_t south_y = WrapCoordinate(y + 1, gridSizeY);
    size_t down_z = WrapCoordinate(z - 1, gridSizeZ);
    size_t up_z = WrapCoordinate(z + 1, gridSizeZ);
    
    // Create array of neighbors
    std::array<size_t, 6> cachedNeighbors = {
        GetNodeID(west_x, y, z, gridSizeX, gridSizeY),    // West
        GetNodeID(east_x, y, z, gridSizeX, gridSizeY),    // East
        GetNodeID(x, north_y, z, gridSizeX, gridSizeY),   // North
        GetNodeID(x, south_y, z, gridSizeX, gridSizeY),   // South
        GetNodeID(x, y, down_z, gridSizeX, gridSizeY),    // Down
        GetNodeID(x, y, up_z, gridSizeX, gridSizeY)       // Up
    };
    
    // Store in cache
    neighborCache[cacheKey] = cachedNeighbors;
    
    // Convert array to vector and return
    return std::vector<size_t>(cachedNeighbors.begin(), cachedNeighbors.end());
}

// Calculate distance in the torus topology
size_t Torus_3D_NeighborInfo::CalculateTorusDistance(
    size_t nodeId, size_t targetId, 
    size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ) const {
    
    auto [node_x, node_y, node_z] = GetCoordinates(nodeId, gridSizeX, gridSizeY);
    auto [target_x, target_y, target_z] = GetCoordinates(targetId, gridSizeX, gridSizeY);

    // Calculate wrapped distances
    size_t dx = GetWrappedDistance(node_x, target_x, gridSizeX);
    size_t dy = GetWrappedDistance(node_y, target_y, gridSizeY);
    size_t dz = GetWrappedDistance(node_z, target_z, gridSizeZ);
    
    return dx + dy + dz;
}

// Non-deterministic implementation with faster performance
Torus_3D_Path Torus_3D_NeighborInfo::FindShortestPath(
    size_t finalDestNodeID,
    size_t current_x, size_t current_y, size_t current_z,
    size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
    size_t hopRadius, const std::vector<int>& globalQueueSizes,
    const std::vector<double>& queueTimestamps, double currentTime) const {
    
    // Get source and target IDs
    const size_t sourceId = GetNodeID(current_x, current_y, current_z, gridSizeX, gridSizeY);
    const size_t targetId = finalDestNodeID;
    
    // Return immediately if already at destination
    if (sourceId == targetId) {
        Torus_3D_Path directPath;
        directPath.AddNode(sourceId);
        directPath.totalCost = 0.0;
        return directPath;
    }
    
    // Get target coordinates
    const size_t target_x = targetId & (gridSizeX - 1);
    const size_t target_y = (targetId / gridSizeX) & (gridSizeY - 1);
    const size_t target_z = targetId / (gridSizeX * gridSizeY);
    
    // Constants for algorithm
    const size_t MAX_EXPLORATION = 64;
    const size_t ARRAY_SIZE = 4096;  // Fixed size for arrays
    
    // Use fixed-size arrays with modulo indexing - exactly like C implementation
    // Making these static thread_local is crucial for performance
    static thread_local std::vector<double> gScores(ARRAY_SIZE, DBL_MAX);
    static thread_local std::vector<double> fScores(ARRAY_SIZE, DBL_MAX);
    static thread_local std::vector<size_t> cameFrom(ARRAY_SIZE, 0);
    static thread_local std::vector<bool> inClosedSet(ARRAY_SIZE, false);
    static thread_local std::vector<bool> inOpenSet(ARRAY_SIZE, false);
    static thread_local std::vector<size_t> openSet(ARRAY_SIZE, 0);
    
    // Reset arrays (fast memory operations)
    for (size_t i = 0; i < ARRAY_SIZE; i++) {
        gScores[i] = DBL_MAX;
        fScores[i] = DBL_MAX;
        inClosedSet[i] = false;
        inOpenSet[i] = false;
    }
    
    // Modulo index function (same as C implementation)
    auto idx = [ARRAY_SIZE](size_t nodeId) -> size_t { 
        return nodeId & (ARRAY_SIZE - 1); 
    };
    
    // Best partial path tracking
    size_t bestNodeId = sourceId;
    double bestHeuristic = DBL_MAX;
    
    // Initialize start node
    size_t src_idx = idx(sourceId);
    gScores[src_idx] = 0.0;
    
    // Calculate initial heuristic
    int dx = std::abs(static_cast<int>(current_x) - static_cast<int>(target_x));
    dx = std::min(dx, static_cast<int>(gridSizeX) - dx);
    
    int dy = std::abs(static_cast<int>(current_y) - static_cast<int>(target_y));
    dy = std::min(dy, static_cast<int>(gridSizeY) - dy);
    
    int dz = std::abs(static_cast<int>(current_z) - static_cast<int>(target_z));
    dz = std::min(dz, static_cast<int>(gridSizeZ) - dz);
    
    double heuristic = dx + dy + dz;
    fScores[src_idx] = heuristic;
    bestHeuristic = heuristic;
    
    // Simple array-based open set (exactly like C implementation)
    size_t openSetSize = 0;
    openSet[openSetSize++] = sourceId;
    inOpenSet[src_idx] = true;
    
    // Static average queue size (updated infrequently)
    //static double avgQueueSize = 20.0;
    //static size_t queueSizeUpdateCounter = 0;
    
    //if ((++_queueSizeUpdateCounter & 2) == 0 && !globalQueueSizes.empty()) {
        double sum = 0.0;
        int count = 0;
        
        // Sparse sampling for speed
		const size_t step = (_numNodes < 512) ? 1 : (_numNodes / 512);
		
        for (size_t i = 0; i < globalQueueSizes.size(); i += step) {
            if (i < queueTimestamps.size() && queueTimestamps[i] > 0) {
                sum += globalQueueSizes[i];
                count++;
            }
        }
        
        _avgQueueSize = count > 0 ? (sum / count) + 15.0 : 20.0;
		//printf("%lf\n", _avgQueueSize);
    //}
    
    // Queue estimation (inlined for speed)
    auto estimateQueue = [&](size_t nodeId) -> double {
        if (nodeId < globalQueueSizes.size() && nodeId < queueTimestamps.size()) {
            const double timestamp = queueTimestamps[nodeId];
            if (timestamp <= 0.0) return _avgQueueSize;
            
            const double age = currentTime - timestamp;
            if (age < 0.1) return globalQueueSizes[nodeId];
            
            return globalQueueSizes[nodeId] + std::min(20.0, age / 5.0);
        }
        return _avgQueueSize;
    };
    
    // Pre-calculate neighbors for source node
    std::array<size_t, 6> sourceNeighbors;
    sourceNeighbors[0] = GetNodeID((current_x + gridSizeX - 1) & (gridSizeX - 1), current_y, current_z, gridSizeX, gridSizeY);
    sourceNeighbors[1] = GetNodeID((current_x + 1) & (gridSizeX - 1), current_y, current_z, gridSizeX, gridSizeY);
    sourceNeighbors[2] = GetNodeID(current_x, (current_y + gridSizeY - 1) & (gridSizeY - 1), current_z, gridSizeX, gridSizeY);
    sourceNeighbors[3] = GetNodeID(current_x, (current_y + 1) & (gridSizeY - 1), current_z, gridSizeX, gridSizeY);
    sourceNeighbors[4] = GetNodeID(current_x, current_y, (current_z + gridSizeZ - 1) & (gridSizeZ -1), gridSizeX, gridSizeY);
    sourceNeighbors[5] = GetNodeID(current_x, current_y, (current_z + 1) & (gridSizeZ - 1), gridSizeX, gridSizeY);
    
    // Main A* loop
    for (size_t iterations = 0; openSetSize > 0 && iterations < MAX_EXPLORATION; iterations++) {
        // Find node with lowest f-score (linear search)
        size_t current_idx = 0;
        double lowest_f = DBL_MAX;
        
        // First pass: find lowest f-score
        for (size_t i = 0; i < openSetSize; i++) {
            size_t node_id = openSet[i];
            size_t node_idx = idx(node_id);
            
            if (fScores[node_idx] < lowest_f) {
                lowest_f = fScores[node_idx];
            }
        }
        
        // Second pass: deterministic tie-breaking by node ID
        size_t lowest_id = SIZE_MAX;
        for (size_t i = 0; i < openSetSize; i++) {
            size_t node_id = openSet[i];
            size_t node_idx = idx(node_id);
            
            // Use small epsilon for floating point comparison
            if (std::abs(fScores[node_idx] - lowest_f) < 0.0001 && node_id < lowest_id) {
                lowest_id = node_id;
                current_idx = i;
            }
        }
        
        // Get current node
        size_t currentNodeId = openSet[current_idx];
        size_t nodeIdx = idx(currentNodeId);
        
        // Remove from open set (fast removal by swapping with last element)
        openSet[current_idx] = openSet[--openSetSize];
        inOpenSet[nodeIdx] = false;
        
        // Add to closed set
        inClosedSet[nodeIdx] = true;
        
        // Reached destination?
        if (currentNodeId == targetId) {
            // Path found - reconstruct
            Torus_3D_Path resultPath;
            
            // Build path in reverse order first
            std::array<size_t, T3D_MAX_PATH_LENGTH> revPath;
            size_t pathLen = 0;
            
            size_t curr = targetId;
            while (curr != sourceId && pathLen < T3D_MAX_PATH_LENGTH) {
                revPath[pathLen++] = curr;
                curr = cameFrom[idx(curr)];
            }
            
            // Add source
            if (pathLen < T3D_MAX_PATH_LENGTH) {
                revPath[pathLen++] = sourceId;
            }
            
            // Create result path in correct order
            for (int i = pathLen - 1; i >= 0; i--) {
                resultPath.AddNode(revPath[i]);
            }
            
            resultPath.totalCost = gScores[idx(targetId)];
            return resultPath;
        }
        
        // Update best partial path
        const size_t cx = currentNodeId & (gridSizeX - 1);
        const size_t cy = (currentNodeId / gridSizeX) & (gridSizeY - 1);
        const size_t cz = currentNodeId / (gridSizeX * gridSizeY);
        
        int cdx = std::abs(static_cast<int>(cx) - static_cast<int>(target_x));
        cdx = std::min(cdx, static_cast<int>(gridSizeX) - cdx);
        
        int cdy = std::abs(static_cast<int>(cy) - static_cast<int>(target_y));
        cdy = std::min(cdy, static_cast<int>(gridSizeY) - cdy);
        
        int cdz = std::abs(static_cast<int>(cz) - static_cast<int>(target_z));
        cdz = std::min(cdz, static_cast<int>(gridSizeZ) - cdz);
        
        double currentHeuristic = cdx + cdy + cdz;
        
        if (currentHeuristic < bestHeuristic) {
            bestHeuristic = currentHeuristic;
            bestNodeId = currentNodeId;
        }
        
        // Calculate neighbors
        std::array<size_t, 6> neighbors;
        neighbors[0] = GetNodeID((cx + gridSizeX - 1) & (gridSizeX - 1), cy, cz, gridSizeX, gridSizeY);
        neighbors[1] = GetNodeID((cx + 1) & (gridSizeX - 1), cy, cz, gridSizeX, gridSizeY);
        neighbors[2] = GetNodeID(cx, (cy + gridSizeY - 1) & (gridSizeY - 1), cz, gridSizeX, gridSizeY);
        neighbors[3] = GetNodeID(cx, (cy + 1) & (gridSizeY - 1), cz, gridSizeX, gridSizeY);
        neighbors[4] = GetNodeID(cx, cy, (cz + gridSizeZ - 1) & (gridSizeZ - 1), gridSizeX, gridSizeY);
        neighbors[5] = GetNodeID(cx, cy, (cz + 1) & (gridSizeZ - 1), gridSizeX, gridSizeY);
        
        // Cache current g-score
        const double currentGScore = gScores[nodeIdx];
        
        // Process all neighbors
        for (int i = 0; i < 6; i++) {
            const size_t neighbor = neighbors[i];
            const size_t neighborIdx = idx(neighbor);
            
            // Skip if already in closed set
            if (inClosedSet[neighborIdx]) {
                continue;
            }
            
            // Calculate cost
            const double queueSize = estimateQueue(neighbor);
            const double moveCost = queueSize + 1.0;
            const double tentativeG = currentGScore + moveCost;
            
            // Check if better path
            if (!inOpenSet[neighborIdx] || tentativeG < gScores[neighborIdx]) {
                // Update path
                cameFrom[neighborIdx] = currentNodeId;
                gScores[neighborIdx] = tentativeG;
                
                // Calculate heuristic
                const size_t nx = neighbor & (gridSizeX - 1);
                const size_t ny = (neighbor / gridSizeX) & (gridSizeY - 1);
                const size_t nz = neighbor / (gridSizeX * gridSizeY);
                
                int ndx = std::abs(static_cast<int>(nx) - static_cast<int>(target_x));
                ndx = std::min(ndx, static_cast<int>(gridSizeX) - ndx);
                
                int ndy = std::abs(static_cast<int>(ny) - static_cast<int>(target_y));
                ndy = std::min(ndy, static_cast<int>(gridSizeY) - ndy);
                
                int ndz = std::abs(static_cast<int>(nz) - static_cast<int>(target_z));
                ndz = std::min(ndz, static_cast<int>(gridSizeZ) - ndz);
                
                // Update f-score
                fScores[neighborIdx] = tentativeG + (ndx + ndy + ndz);
                
                // Add to open set if not already there
                if (!inOpenSet[neighborIdx]) {
                    // Make sure we don't overflow
                    if (openSetSize < ARRAY_SIZE) {
                        openSet[openSetSize++] = neighbor;
                        inOpenSet[neighborIdx] = true;
                    }
                }
            }
        }
    }
    
    // If we found a partial path to best node, use it
    if (bestNodeId != sourceId) {
        Torus_3D_Path partialPath;
        
        // Reconstruct path in reverse order
        std::array<size_t, T3D_MAX_PATH_LENGTH> revPath;
        size_t pathLen = 0;
        
        size_t curr = bestNodeId;
        while (curr != sourceId && pathLen < T3D_MAX_PATH_LENGTH) {
            revPath[pathLen++] = curr;
            curr = cameFrom[idx(curr)];
        }
        
        // Add source
        if (pathLen < T3D_MAX_PATH_LENGTH) {
            revPath[pathLen++] = sourceId;
        }
        
        // Build path in correct order
        for (int i = pathLen - 1; i >= 0; i--) {
            partialPath.AddNode(revPath[i]);
        }
        
        partialPath.totalCost = gScores[idx(bestNodeId)];
        return partialPath;
    }
    
    // Fall back to greedy path if needed
    Torus_3D_Path greedyPath;
    greedyPath.AddNode(sourceId);
    
    // Deterministic direction selection
    int dx_wrapped = std::abs(static_cast<int>(current_x) - static_cast<int>(target_x));
    dx_wrapped = std::min(dx_wrapped, static_cast<int>(gridSizeX) - dx_wrapped);
    
    int dy_wrapped = std::abs(static_cast<int>(current_y) - static_cast<int>(target_y));
    dy_wrapped = std::min(dy_wrapped, static_cast<int>(gridSizeY) - dy_wrapped);
    
    int dz_wrapped = std::abs(static_cast<int>(current_z) - static_cast<int>(target_z));
    dz_wrapped = std::min(dz_wrapped, static_cast<int>(gridSizeZ) - dz_wrapped);
    
    int bestDir = -1;
    
    if (dx_wrapped >= dy_wrapped && dx_wrapped >= dz_wrapped) {
        // X dimension has biggest distance
        if ((current_x < target_x && (target_x - current_x) <= gridSizeX/2) ||
            (current_x > target_x && (current_x - target_x) > gridSizeX/2)) {
            bestDir = 1; // East
        } else {
            bestDir = 0; // West
        }
    } else if (dy_wrapped >= dx_wrapped && dy_wrapped >= dz_wrapped) {
        // Y dimension has biggest distance
        if ((current_y < target_y && (target_y - current_y) <= gridSizeY/2) ||
            (current_y > target_y && (current_y - target_y) > gridSizeY/2)) {
            bestDir = 3; // South
        } else {
            bestDir = 2; // North
        }
    } else {
        // Z dimension has biggest distance
        if ((current_z < target_z && (target_z - current_z) <= gridSizeZ/2) ||
            (current_z > target_z && (current_z - target_z) > gridSizeZ/2)) {
            bestDir = 5; // Up
        } else {
            bestDir = 4; // Down
        }
    }
    
    // Ensure valid direction
    bestDir = std::max(0, std::min(bestDir, 5));
    
    // Add to path
    greedyPath.AddNode(sourceNeighbors[bestDir]);
    greedyPath.totalCost = 1.0;
	
    return greedyPath;
}