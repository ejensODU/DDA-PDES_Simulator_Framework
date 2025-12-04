#pragma once
#include <array>
#include "../PDDA_SimExec.h"
#include "Irregular_NoC_Common.h"

// Define circular buffer size for queue history (power of 2 for efficient circular indexing)
#define QUEUE_HISTORY_SIZE 16
#define QUEUE_HISTORY_MASK (QUEUE_HISTORY_SIZE - 1) // For circular indexing with bitwise AND

// Optimized packet class for irregular NoC
class Irregular_NoC_Packet : public Entity {
public:
    // Default constructor
    Irregular_NoC_Packet();
    
    // Constructor with packet parameters
    Irregular_NoC_Packet(double genTime, 
                       size_t originNetworkNodeID, 
                       size_t destNetworkNodeID,
                       size_t numNodes);

    // Add a new node to the packet's path with queue size information
    void AddNetworkNodeData(double arrivalTime, size_t networkNodeID, int queueSize);
    
	// Get path information - Modified to return direct array reference
	const std::array<size_t, NOC_MAX_PATH_LENGTH>& getVisitedNetworkNodes(size_t& visitedNodeCnt) const;
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
    
    // Calculate time in network (exit - generation)
    double GetTimeInNetwork() const;
    
    // Access the circular queue history
    int getQueueSizeAt(size_t index) const;
    double getQueueTimeAt(size_t index) const;
    size_t getQueueHistoryCount() const;
    
    // Get hop count (number of nodes visited)
    size_t getHopCount() const { return _visitedNodeCnt; }
    
    // Check if a packet has been in a routing loop
    bool isInRoutingLoop() const;
    
    // Check if packet has exceeded maximum hop count
    bool hasExceededMaxHops() const { return _visitedNodeCnt >= NOC_MAX_PATH_LENGTH; }
    
    // Mark packet as forcibly terminated
    void markForciblyTerminated() { _forciblyTerminated = true; }
    bool wasForciblyTerminated() const { return _forciblyTerminated; }
    
    // For debugging
    void PrintData() const override;

private:
    // Path tracking with fixed arrays
    size_t _visitedNodeCnt;
    std::array<double, NOC_MAX_PATH_LENGTH> _nodeArrivalTimes;
    std::array<size_t, NOC_MAX_PATH_LENGTH> _visitedNodes;
    std::array<int, NOC_MAX_PATH_LENGTH> _queueSizes;  // Queue sizes at visited nodes
    
    // Circular buffer for recent queue size history
    std::array<int, QUEUE_HISTORY_SIZE> _recentQueueSizes;      // Recent queue sizes (circular buffer)
    std::array<size_t, QUEUE_HISTORY_SIZE> _recentQueueNodes;   // Nodes for recent queue sizes
    std::array<double, QUEUE_HISTORY_SIZE> _recentQueueTimes;   // Times for recent queue sizes
    size_t _queueHistoryCount;                                  // Number of entries in history
    size_t _queueHistoryIndex;                                  // Current index in circular buffer

    // Origin and destination information
    size_t _originNetworkNodeID;
    size_t _destNetworkNodeID;
    size_t _numNodes;            // Total nodes in the network
    
    // Stuck packet detection
    bool _forciblyTerminated = false;    // Flag to indicate if packet was forcibly terminated
};