#pragma once
#include <array>
#include "../PDDA_SimExec.h"
#include "Torus_3D_NeighborInfo.h"

// Define circular buffer size for queue history (power of 2 for efficient circular indexing)
#define QUEUE_HISTORY_SIZE 16
#define QUEUE_HISTORY_MASK (QUEUE_HISTORY_SIZE - 1) // For circular indexing with bitwise AND

// Optimized packet class using fixed arrays and minimizing memory allocations
class Torus_3D_Packet : public Entity {
public:
    Torus_3D_Packet();
    
    Torus_3D_Packet(double genTime, size_t originNetworkNodeID, size_t destNetworkNodeID,
                    size_t destX, size_t destY, size_t destZ,
                    size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ);

    // Add a new node to the packet's path with queue size information
    void AddNetworkNodeData(double arrivalTime, size_t networkNodeID, int queueSize);
    
    // Get path information - Modified to return direct array reference
	const std::array<size_t, T3D_MAX_PATH_LENGTH>& getVisitedNetworkNodes(size_t& visitedNodeCnt) const;
    const std::array<size_t, QUEUE_HISTORY_SIZE>& getRecentQueueNodes(size_t& queueHistoryIndex, size_t& queueHistoryCnt) const;
    const std::array<int, QUEUE_HISTORY_SIZE>& getRecentQueueSizes() const;
	const std::array<double, QUEUE_HISTORY_SIZE>& getRecentQueueTimes() const;
    
    // Get arrival time for a specific node in the path
    double getNodeArrivalTime(size_t index) const {
        if (index < _visitedNodeCnt) {
            return _nodeArrivalTimes[index];
        }
        return -1.0; // Invalid index
    }
    
    // Get destination information
    size_t getDestNetworkNodeID() const;
    size_t getDestX() const;
    size_t getDestY() const;
    size_t getDestZ() const;
    
    // Calculate time in network
    double GetTimeInNetwork() const;
	
	double getGenTime() const { return _genTime; }
    
    // Access the circular queue history
    int getQueueSizeAt(size_t index) const;
    double getQueueTimeAt(size_t index) const;
    size_t getQueueHistoryCount() const;
    
    // For debugging
    void PrintData() const override;

private:
    // Optimized method for coordinate wrapping
    size_t getWrappedDistance(int coord1, int coord2, size_t size) const {
        int direct_dist = abs(coord2 - coord1);
        int wrapped_dist = size - direct_dist;
        return std::min(direct_dist, wrapped_dist);
    }

    // Path tracking with fixed arrays
    size_t _visitedNodeCnt;
    std::array<double, T3D_MAX_PATH_LENGTH> _nodeArrivalTimes;
    std::array<size_t, T3D_MAX_PATH_LENGTH> _visitedNodes;
    //std::array<int, T3D_MAX_PATH_LENGTH> _queueSizes;  // Queue sizes at visited nodes
    
    // Circular buffer for recent queue size history
    std::array<int, QUEUE_HISTORY_SIZE> _recentQueueSizes;      // Recent queue sizes (circular buffer)
    std::array<size_t, QUEUE_HISTORY_SIZE> _recentQueueNodes;   // Nodes for recent queue sizes
    std::array<double, QUEUE_HISTORY_SIZE> _recentQueueTimes;   // Times for recent queue sizes
    size_t _queueHistoryCount;                                  // Number of entries in history
    size_t _queueHistoryIndex;                                  // Current index in circular buffer

    // Origin and destination information
    size_t _originNetworkNodeID;
    size_t _destNetworkNodeID;
    size_t _destX;
    size_t _destY;
    size_t _destZ;
    size_t _gridSizeX;
    size_t _gridSizeY;
    size_t _gridSizeZ;
    size_t _minWrappedDist;
};