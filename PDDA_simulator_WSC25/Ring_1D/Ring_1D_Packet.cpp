#include "Ring_1D_Packet.h"
#include <cstdio>
#include <string>
#include <algorithm>
#include <limits>

Ring_1D_Packet::Ring_1D_Packet()
: Entity(),
_originNodeID(0),
_destNodeID(0),
_ringSize(0),
_clockwise(false),
_visitedNodeCnt(0) {
    // Initialize arrays with default values
    _nodeArrivalTimes.fill(-1.0);
    _visitedNodes.fill(999999);
}

Ring_1D_Packet::Ring_1D_Packet(double genTime, size_t originNodeID, size_t destNodeID,
                               size_t ringSize, bool clockwise)
: Entity(genTime),
_originNodeID(originNodeID),
_destNodeID(destNodeID),
_ringSize(ringSize),
_clockwise(clockwise),
_visitedNodeCnt(0) {
    // Initialize arrays with default values
    _nodeArrivalTimes.fill(-1.0);
    _visitedNodes.fill(999999);
}

// Copy constructor
Ring_1D_Packet::Ring_1D_Packet(const Ring_1D_Packet& other)
: Entity(other),  // Copy construct the base class part
_originNodeID(other._originNodeID),
_destNodeID(other._destNodeID),
_ringSize(other._ringSize),
_clockwise(other._clockwise),
_visitedNodeCnt(other._visitedNodeCnt),
_nodeArrivalTimes(other._nodeArrivalTimes),
_visitedNodes(other._visitedNodes),
_minWrappedDist(other._minWrappedDist) {}


void Ring_1D_Packet::AddNodeData(double arrivalTime, size_t networkNodeID) {
    if (std::find(_visitedNodes.begin(), _visitedNodes.begin() + _visitedNodeCnt, networkNodeID) != _visitedNodes.begin() + _visitedNodeCnt) {
        printf("WARNING: Packet %d revisiting node %lu\n", _ID, networkNodeID);
    }

    // For clockwise direction: Check if we've passed our starting point
    bool crossed_zero = false;
    if (_visitedNodeCnt > 0) {
        size_t last_node = _visitedNodes[_visitedNodeCnt-1];
        if (_clockwise && last_node > networkNodeID) {
            crossed_zero = true;  // We've wrapped from high to low
        } else if (!_clockwise && last_node < networkNodeID) {
            crossed_zero = true;  // We've wrapped from low to high
        }
    }

    // Don't do distance checking if we've crossed the zero point
    if (!crossed_zero && _visitedNodeCnt > 0) {
        size_t last_node = _visitedNodes[_visitedNodeCnt-1];
        bool making_progress = false;

        if (_clockwise) {
            // For clockwise: check if we're moving forward without wrapping
            if (networkNodeID > last_node &&
                (networkNodeID <= _destNodeID || last_node >= _destNodeID)) {
                making_progress = true;
                }
        } else {
            // For counterclockwise: check if we're moving backward without wrapping
            if (networkNodeID < last_node &&
                (networkNodeID >= _destNodeID || last_node <= _destNodeID)) {
                making_progress = true;
                }
        }

        if (!making_progress) {
            printf("error: packet not making progress towards destination!\n");
            printf("\tnode ID: %lu, direction: %s\n", networkNodeID,
                   _clockwise ? "clockwise" : "counterclockwise");
            printf("\tlast node: %lu, destination: %lu\n", last_node, _destNodeID);
            PrintData();
            exit(1);
        }
    }

    _nodeArrivalTimes[_visitedNodeCnt] = arrivalTime;
    _visitedNodes[_visitedNodeCnt++] = networkNodeID;
}

const std::array<size_t, Ring_1D_Packet::MAX_VISITED_NODES>& Ring_1D_Packet::getVisitedNodes() const {
    return _visitedNodes;
}

size_t Ring_1D_Packet::getDestNodeID() const {
    return _destNodeID;
}

double Ring_1D_Packet::GetTimeInNetwork() const {
    return _exitTime - _genTime;
}

void Ring_1D_Packet::PrintData() const {
    std::string arrival_times;
    for (int i=0; i<_visitedNodeCnt; i++) {
        arrival_times.append(std::to_string(_nodeArrivalTimes[i]) + ",");
    }

    std::string visited_nodes;
    for (int i=0; i<_visitedNodeCnt; i++) {
        visited_nodes.append(std::to_string(_visitedNodes[i]) + ",");
    }

    printf("ID: %d, gen t: %lf, o: %lu, d: %lu, direction: %s, arrival ts: %s, nodes: %s\n",
           _ID, _genTime, _originNodeID, _destNodeID,
           _clockwise ? "clockwise" : "counterclockwise",
           arrival_times.c_str(), visited_nodes.c_str());
}