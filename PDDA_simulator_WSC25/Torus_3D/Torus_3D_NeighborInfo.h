#pragma once
#include <vector>
#include <memory>
#include <map>
#include <tuple>
#include <array>
#include <limits>
#include <set>

class Torus_3D_Arrive;

// Constants for fixed-size arrays
/* const size_t MAX_PATH_LENGTH = 32;
const size_t MAX_HOP_RADIUS = 6;
const size_t MAX_NEIGHBORS_PER_LEVEL = 146;
const size_t MAX_TOTAL_NEIGHBORS = 377; */

const size_t T3D_MAX_PATH_LENGTH = 64;
const size_t T3D_MAX_HOP_RADIUS = 4;
const size_t T3D_MAX_NEIGHBORS_PER_LEVEL = 66;
const size_t T3D_MAX_TOTAL_NEIGHBORS = 129;

// 3D coordinates structure for caching
struct Coords3D {
    unsigned int x, y, z;
};

// Modified Torus_3D_Path struct definition
struct Torus_3D_Path {
    std::array<size_t, T3D_MAX_PATH_LENGTH> nodeIndices; // Fixed-size array instead of vector
    double totalCost;
    size_t length;  // Tracks actual number of elements used in the array
    
    Torus_3D_Path() : totalCost(std::numeric_limits<double>::max()), length(0) {}
    void PrintPath() const;
    
    // Update method to add a node to the fixed array
    void AddNode(size_t nodeId) {
        if (length < T3D_MAX_PATH_LENGTH) {
            nodeIndices[length++] = nodeId;
        }
    }
    
    // Modified equality operator for fixed-size array
    bool operator==(const Torus_3D_Path& other) const {
        if (length != other.length || totalCost != other.totalCost) {
            return false;
        }
        
        // Compare only the elements up to length
        for (size_t i = 0; i < length; i++) {
            if (nodeIndices[i] != other.nodeIndices[i]) {
                return false;
            }
        }
        return true;
    }
};

// Strict deterministic neighbor info structure
struct Torus_3D_NeighborInfo {
    // Original interface members for compatibility
    std::vector<std::vector<size_t>> Indices;  // Hierarchical indices
    std::vector<std::shared_ptr<Torus_3D_Arrive>> Neighbors;  // Flat array of neighbor pointers
    std::map<size_t, size_t> globalToLocalIdx;  // Maps network node IDs to indices in N - ordered map

    // Optimized internal storage
    std::array<std::array<size_t, T3D_MAX_NEIGHBORS_PER_LEVEL>, T3D_MAX_HOP_RADIUS+1> hopLevelNodes;
    std::array<size_t, T3D_MAX_HOP_RADIUS+1> hopLevelCounts;
    std::array<size_t, T3D_MAX_TOTAL_NEIGHBORS> allNeighbors;
    size_t allNeighborCount;
    std::vector<int>& _queueSizes;  // Reference to external queue sizes
    std::array<int, 6> directNeighborIndices;
    static std::map<std::tuple<size_t, size_t, size_t, size_t>, std::array<size_t, 6>> _neighborCache;
    // In Torus_3D_NeighborInfo class declaration
    mutable std::array<std::pair<size_t, double>, 6> _safeNeighbors; // Max 6 neighbors in 3D torus
    mutable size_t _safeNeighborCount; // Number of safe neighbors found
	
    // Constructor initializes structures
    Torus_3D_NeighborInfo(size_t nodeId, size_t x, size_t y, size_t z, 
                   size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
                   size_t hopRadius, std::vector<int>& queueSizes);

    // Modified pathfinding with global queue knowledge
    Torus_3D_Path FindShortestPath(
        size_t finalDestNodeID,
        size_t current_x, size_t current_y, size_t current_z,
        size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
        size_t hopRadius, const std::vector<int>& globalQueueSizes,
        const std::vector<double>& queueTimestamps, double currentTime) const;
    
    // Helper methods with capitalized names to match interface
    std::tuple<size_t, size_t, size_t> GetCoordinates(size_t nodeID, size_t gridSizeX, size_t gridSizeY) const;
    size_t GetNodeID(size_t x, size_t y, size_t z, size_t gridSizeX, size_t gridSizeY) const;
    std::vector<size_t> GetTorusNeighbors(size_t nodeID, size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ) const;
    
    // Calculate torus distances deterministically
    size_t CalculateTorusDistance(size_t nodeId, size_t targetId, 
                                 size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ) const;
    
private:
	// Statistics
	size_t _nodeID;
	mutable size_t _fullPathsCnt;
	mutable size_t _quickPathsCnt;
	mutable size_t _pathFindsCnt;
	
	// Queue-size estimation
	const size_t _numNodes;
	mutable double _avgQueueSize;
	mutable size_t _queueSizeUpdateCounter;

    // Helper methods for coordinate wrapping
    size_t WrapCoordinate(size_t coord, size_t size) const {
        return (coord + size) % size;
    }

    // Helper method for calculating wrapped distance
    size_t GetWrappedDistance(int coord1, int coord2, size_t size) const {
        int direct_dist = abs(coord2 - coord1);
        int wrapped_dist = size - direct_dist;
        return std::min(direct_dist, wrapped_dist);
    }
                                        
    // Sort nodes deterministically (used in initialization)
    void SortNodeIndices(std::vector<size_t>& nodes) const;
    
    struct PathEntry {
        size_t nodeID;
        std::array<size_t, T3D_MAX_PATH_LENGTH> path; // Fixed-size array instead of vector
        size_t pathLength;                        // Track actual path length
        double costSoFar;
        double estimatedTotalCost;
        size_t hopsFromStart;
        size_t insertOrder; // Tie-breaker for identical costs
        
        PathEntry() : pathLength(0), costSoFar(0), estimatedTotalCost(0), 
                      hopsFromStart(0), insertOrder(0) {}
        
        // Add a node to the path
        void AddNode(size_t nodeId) {
            if (pathLength < T3D_MAX_PATH_LENGTH) {
                path[pathLength++] = nodeId;
            }
        }
        
        // Strict deterministic comparison
        bool operator<(const PathEntry& other) const {
            // Primary sort: estimated total cost
            if (estimatedTotalCost != other.estimatedTotalCost) {
                return estimatedTotalCost < other.estimatedTotalCost;
            }
            // Secondary sort: actual cost so far
            if (costSoFar != other.costSoFar) {
                return costSoFar < other.costSoFar;
            }
            // Third sort: hop count (prefer shorter paths)
            if (hopsFromStart != other.hopsFromStart) {
                return hopsFromStart < other.hopsFromStart;
            }
            // Fourth sort: node ID 
            if (nodeID != other.nodeID) {
                return nodeID < other.nodeID;
            }
            // Final sort: insertion order
            return insertOrder < other.insertOrder;
        }
    };
};