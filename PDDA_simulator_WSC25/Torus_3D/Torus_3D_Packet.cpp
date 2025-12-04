#include "Torus_3D_Packet.h"
#include <cstdio>
#include <string>
#include <algorithm>
#include <limits>

Torus_3D_Packet::Torus_3D_Packet()
: Entity(),
  _originNetworkNodeID(0),
  _destNetworkNodeID(0),
  _destX(0), _destY(0), _destZ(0),
  _gridSizeX(0), _gridSizeY(0), _gridSizeZ(0),
  _visitedNodeCnt(0),
  _queueHistoryCount(0),
  _queueHistoryIndex(0),
  _minWrappedDist(std::numeric_limits<size_t>::max()) {
    _nodeArrivalTimes.fill(-1.0);
    _visitedNodes.fill(999999);
    //_queueSizes.fill(-1);
    _recentQueueSizes.fill(-1);
    _recentQueueNodes.fill(999999);
    _recentQueueTimes.fill(-1.0);
}

Torus_3D_Packet::Torus_3D_Packet(
    double genTime, size_t originNetworkNodeID, size_t destNetworkNodeID,
    size_t destX, size_t destY, size_t destZ,
    size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ)
    : Entity(genTime),
      _originNetworkNodeID(originNetworkNodeID),
      _destNetworkNodeID(destNetworkNodeID),
      _destX(destX), _destY(destY), _destZ(destZ),
      _gridSizeX(gridSizeX), _gridSizeY(gridSizeY), _gridSizeZ(gridSizeZ),
      _visitedNodeCnt(0),
      _queueHistoryCount(0),
      _queueHistoryIndex(0),
      _minWrappedDist(std::numeric_limits<size_t>::max()) {
    
    // Initialize arrays
    _nodeArrivalTimes.fill(-1.0);
    _visitedNodes.fill(999999);
    //_queueSizes.fill(-1);
    _recentQueueSizes.fill(-1);
    _recentQueueNodes.fill(999999);
    _recentQueueTimes.fill(-1.0);
}

void Torus_3D_Packet::AddNetworkNodeData(double arrivalTime, size_t networkNodeID, int queueSize) {
    // Check if node already visited (using fast array search)
    bool alreadyVisited = false;
    for (size_t i = 0; i < _visitedNodeCnt; i++) {
        if (_visitedNodes[i] == networkNodeID) {
            alreadyVisited = true;
            break;
        }
    }
    
    //if (alreadyVisited) {
    //    printf("WARNING: Packet %d revisiting node %lu\n", _ID, networkNodeID);
    //}
	
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
    if (_visitedNodeCnt < T3D_MAX_PATH_LENGTH) {
        _nodeArrivalTimes[_visitedNodeCnt] = arrivalTime;
        _visitedNodes[_visitedNodeCnt] = networkNodeID;
        //_queueSizes[_visitedNodeCnt] = queueSize;
        _visitedNodeCnt++;
    } //else {
        //printf("WARNING: Packet %d reached maximum path length, cannot add node %lu, visited node count: %lu\n", 
        //       _ID, networkNodeID, _visitedNodeCnt);
    //}
}

// Modified function to return a reference to the internal array
const std::array<size_t, T3D_MAX_PATH_LENGTH>& Torus_3D_Packet::getVisitedNetworkNodes(size_t& visitedNodeCnt) const {
    // Set the reference parameter to the current count
    visitedNodeCnt = _visitedNodeCnt;
    // Return a direct reference to the internal array
    return _visitedNodes;
}

// Modified function to return a reference to the internal array
const std::array<size_t, QUEUE_HISTORY_SIZE>& Torus_3D_Packet::getRecentQueueNodes(size_t& queueHistoryIndex, size_t& queueHistoryCnt) const {
    // Set the reference parameter to the current count
    queueHistoryIndex = _queueHistoryIndex;
	queueHistoryCnt = _queueHistoryCount;
    // Return a direct reference to the internal array
    return _recentQueueNodes;
}

// Return queue sizes at visited nodes
const std::array<int, QUEUE_HISTORY_SIZE>& Torus_3D_Packet::getRecentQueueSizes() const {
    return _recentQueueSizes;
}

// Return queue sizes at visited nodes
const std::array<double, QUEUE_HISTORY_SIZE>& Torus_3D_Packet::getRecentQueueTimes() const {
    return _recentQueueTimes;
}

// Access the circular queue history
int Torus_3D_Packet::getQueueSizeAt(size_t index) const {
    if (index < _queueHistoryCount) {
        // Calculate actual index in circular buffer
        size_t actualIndex = (_queueHistoryIndex - 1 - index) & QUEUE_HISTORY_MASK;
        return _recentQueueSizes[actualIndex];
    }
    return -1; // Invalid index
}

double Torus_3D_Packet::getQueueTimeAt(size_t index) const {
    if (index < _queueHistoryCount) {
        // Calculate actual index in circular buffer
        size_t actualIndex = (_queueHistoryIndex - 1 - index) & QUEUE_HISTORY_MASK;
        return _recentQueueTimes[actualIndex];
    }
    return -1.0; // Invalid index
}

size_t Torus_3D_Packet::getQueueHistoryCount() const {
    return _queueHistoryCount;
}

size_t Torus_3D_Packet::getDestNetworkNodeID() const {
    return _destNetworkNodeID;
}

size_t Torus_3D_Packet::getDestX() const {
    return _destX;
}

size_t Torus_3D_Packet::getDestY() const {
    return _destY;
}

size_t Torus_3D_Packet::getDestZ() const {
    return _destZ;
}

double Torus_3D_Packet::GetTimeInNetwork() const {
    return _exitTime - _genTime;
}

void Torus_3D_Packet::PrintData() const {
    std::string arrival_times;
    for (size_t i = 0; i < _visitedNodeCnt; i++) {
        arrival_times.append(std::to_string(_nodeArrivalTimes[i]) + ",");
    }

    std::string traversed_nodes;
    for (size_t i = 0; i < _visitedNodeCnt; i++) {
        traversed_nodes.append(std::to_string(_visitedNodes[i]) + ",");
    }
    
    //std::string queue_sizes;
    //for (size_t i = 0; i < _visitedNodeCnt; i++) {
    //    queue_sizes.append(std::to_string(_queueSizes[i]) + ",");
    //}

    printf("ID: %d, gen t: %lf, o: %lu, d: %lu, arrival ts: %s, nodes: %s\n",
           _ID, _genTime, _originNetworkNodeID, _destNetworkNodeID,
           arrival_times.c_str(), traversed_nodes.c_str());
}