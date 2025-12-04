#include "Irregular_NoC_Packet.h"
#include <cstdio>
#include <string>
#include <algorithm>
#include <limits>
#include <unordered_map>

// Update both constructors in Irregular_NoC_Packet.cpp by adding the _forciblyTerminated initialization

// Default constructor update
Irregular_NoC_Packet::Irregular_NoC_Packet()
: Entity(),
  _originNetworkNodeID(0),
  _destNetworkNodeID(0),
  _numNodes(0),
  _visitedNodeCnt(0),
  _queueHistoryCount(0),
  _queueHistoryIndex(0),
  _forciblyTerminated(false) {  // Add this line
    // Initialize arrays with default values
    _nodeArrivalTimes.fill(-1.0);
    _visitedNodes.fill(999999);
    _queueSizes.fill(-1);
    _recentQueueSizes.fill(-1);
    _recentQueueNodes.fill(999999);
    _recentQueueTimes.fill(-1.0);
}

// Parameter constructor update
Irregular_NoC_Packet::Irregular_NoC_Packet(
    double genTime, 
    size_t originNetworkNodeID, 
    size_t destNetworkNodeID,
    size_t numNodes)
    : Entity(genTime),
      _originNetworkNodeID(originNetworkNodeID),
      _destNetworkNodeID(destNetworkNodeID),
      _numNodes(numNodes),
      _visitedNodeCnt(0),
      _queueHistoryCount(0),
      _queueHistoryIndex(0),
      _forciblyTerminated(false) {  // Add this line
    
    // Initialize arrays with default values
    _nodeArrivalTimes.fill(-1.0);
    _visitedNodes.fill(999999);
    _queueSizes.fill(-1);
    _recentQueueSizes.fill(-1);
    _recentQueueNodes.fill(999999);
    _recentQueueTimes.fill(-1.0);
}

void Irregular_NoC_Packet::AddNetworkNodeData(double arrivalTime, size_t networkNodeID, int queueSize) {
    // Check if node already visited (using fast array search)
    bool alreadyVisited = false;
    for (size_t i = 0; i < _visitedNodeCnt; i++) {
        if (_visitedNodes[i] == networkNodeID) {
            alreadyVisited = true;
            break;
        }
    }

    // Add to circular buffer for recent history
	_recentQueueSizes[_queueHistoryIndex] = queueSize;
	_recentQueueNodes[_queueHistoryIndex] = networkNodeID;
	_recentQueueTimes[_queueHistoryIndex] = arrivalTime;
	
	// Update circular buffer index and count
	_queueHistoryIndex = (_queueHistoryIndex + 1) & QUEUE_HISTORY_MASK; // Efficient circular increment
	if (_queueHistoryCount < QUEUE_HISTORY_SIZE) {
		_queueHistoryCount++;
	}

    // Add node to path (with bounds checking)
    if (_visitedNodeCnt < NOC_MAX_PATH_LENGTH) {
        _nodeArrivalTimes[_visitedNodeCnt] = arrivalTime;
        _visitedNodes[_visitedNodeCnt] = networkNodeID;
        //_queueSizes[_visitedNodeCnt] = queueSize;
        _visitedNodeCnt++;
    }
}

// Return a reference to the internal array of visited nodes
const std::array<size_t, NOC_MAX_PATH_LENGTH>& Irregular_NoC_Packet::getVisitedNetworkNodes(size_t& visitedNodeCnt) const {
    // Set the reference parameter to the current count
    visitedNodeCnt = _visitedNodeCnt;
    // Return a direct reference to the internal array
    return _visitedNodes;
}

// Modified function to return a reference to the internal array
const std::array<size_t, QUEUE_HISTORY_SIZE>& Irregular_NoC_Packet::getRecentQueueNodes(size_t& queueHistoryIndex, size_t& queueHistoryCnt) const {
    // Set the reference parameter to the current count
    queueHistoryIndex = _queueHistoryIndex;
	queueHistoryCnt = _queueHistoryCount;
    // Return a direct reference to the internal array
    return _recentQueueNodes;
}

// Return queue sizes at visited nodes
const std::array<int, QUEUE_HISTORY_SIZE>& Irregular_NoC_Packet::getRecentQueueSizes() const {
    return _recentQueueSizes;
}

// Return queue sizes at visited nodes
const std::array<double, QUEUE_HISTORY_SIZE>& Irregular_NoC_Packet::getRecentQueueTimes() const {
    return _recentQueueTimes;
}

// Access the circular queue history
int Irregular_NoC_Packet::getQueueSizeAt(size_t index) const {
    if (index < _queueHistoryCount) {
        // Calculate actual index in circular buffer
        size_t actualIndex = (_queueHistoryIndex - 1 - index) & QUEUE_HISTORY_MASK;
        return _recentQueueSizes[actualIndex];
    }
    return -1; // Invalid index
}

double Irregular_NoC_Packet::getQueueTimeAt(size_t index) const {
    if (index < _queueHistoryCount) {
        // Calculate actual index in circular buffer
        size_t actualIndex = (_queueHistoryIndex - 1 - index) & QUEUE_HISTORY_MASK;
        return _recentQueueTimes[actualIndex];
    }
    return -1.0; // Invalid index
}

// Add this method to your Irregular_NoC_Packet.cpp file
bool Irregular_NoC_Packet::isInRoutingLoop() const {
    // If we have fewer than 6 hops, can't be a meaningful loop yet
    if (_visitedNodeCnt < 6) return false;
    
    // Look for a pattern where the same node is visited 3 or more times
    std::unordered_map<size_t, int> visitCounts;
    
    // Count the most recent 16 visited nodes
    size_t checkCount = std::min(_visitedNodeCnt, (size_t)16);
    size_t startIdx = _visitedNodeCnt - checkCount;
    
    for (size_t i = startIdx; i < _visitedNodeCnt; i++) {
        visitCounts[_visitedNodes[i]]++;
        
        // If any node has been visited 3 or more times recently, it's likely a loop
        if (visitCounts[_visitedNodes[i]] >= 3) {
            return true;
        }
    }
    
    return false;
}

size_t Irregular_NoC_Packet::getQueueHistoryCount() const {
    return _queueHistoryCount;
}

size_t Irregular_NoC_Packet::getDestNetworkNodeID() const {
    return _destNetworkNodeID;
}

double Irregular_NoC_Packet::GetTimeInNetwork() const {
    return _exitTime - _genTime;
}

// Update the PrintData method in Irregular_NoC_Packet.cpp

void Irregular_NoC_Packet::PrintData() const {
    std::string arrival_times;
    for (size_t i = 0; i < _visitedNodeCnt; i++) {
        arrival_times.append(std::to_string(_nodeArrivalTimes[i]) + ",");
    }

    std::string traversed_nodes;
    for (size_t i = 0; i < _visitedNodeCnt; i++) {
        traversed_nodes.append(std::to_string(_visitedNodes[i]) + ",");
    }
    
    std::string queue_sizes;
    for (size_t i = 0; i < _visitedNodeCnt; i++) {
        queue_sizes.append(std::to_string(_queueSizes[i]) + ",");
    }

    printf("ID: %d, gen t: %lf, o: %lu, d: %lu, arrival ts: %s, nodes: %s, queues: %s",
           _ID, _genTime, _originNetworkNodeID, _destNetworkNodeID,
           arrival_times.c_str(), traversed_nodes.c_str(), queue_sizes.c_str());
           
    // Add status information
    if (_forciblyTerminated) {
        printf(" [FORCIBLY TERMINATED]");
    }
    printf("\n");
}