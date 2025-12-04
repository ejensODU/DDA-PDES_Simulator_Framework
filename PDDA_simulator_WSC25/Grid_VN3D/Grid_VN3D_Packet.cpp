#include "Grid_VN3D_Packet.h"
#include <cstdio>
#include <string>
#include <algorithm>
#include <limits>

Grid_VN3D_Packet::Grid_VN3D_Packet()
: Entity(),
_originNetworkNodeID(0),
_destNetworkNodeID(0),
_destX(0), _destY(0), _destZ(0),
_gridSizeX(0), _gridSizeY(0), _gridSizeZ(0),
_visitedNodeCnt(0) {}

Grid_VN3D_Packet::Grid_VN3D_Packet(double genTime, size_t originNetworkNodeID, size_t destNetworkNodeID,
               size_t destX, size_t destY, size_t destZ,
               size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ)
    : Entity(genTime),
      _originNetworkNodeID(originNetworkNodeID),
      _destNetworkNodeID(destNetworkNodeID),
      _destX(destX), _destY(destY), _destZ(destZ),
      _gridSizeX(gridSizeX), _gridSizeY(gridSizeY), _gridSizeZ(gridSizeZ),
      _visitedNodeCnt(0) {
    _minManhattanDist = std::numeric_limits<size_t>::max();
    // Pre-allocate vectors based on worst-case path length
    size_t maxPathLength = gridSizeX + gridSizeY + gridSizeZ;
    _nodeArrivalTimes.resize(maxPathLength, -1.0);
    _visitedNodes.resize(maxPathLength, 999999);
}

void Grid_VN3D_Packet::AddNetworkNodeData(double arrivalTime, size_t networkNodeID) {
    if (std::find(_visitedNodes.begin(), _visitedNodes.begin() + _visitedNodeCnt, networkNodeID) != _visitedNodes.begin() + _visitedNodeCnt) {
        printf("WARNING: Packet %d revisiting node %lu\n", _ID, networkNodeID);
    }

    size_t node_x = networkNodeID % _gridSizeX;
    size_t node_y = (networkNodeID / _gridSizeX) % _gridSizeY;
    size_t node_z = networkNodeID / (_gridSizeX * _gridSizeY);

    size_t dest_distance = abs(static_cast<int>(_destX) - static_cast<int>(node_x)) +
                          abs(static_cast<int>(_destY) - static_cast<int>(node_y)) +
                          abs(static_cast<int>(_destZ) - static_cast<int>(node_z));

    if (dest_distance >= _minManhattanDist) {
        printf("error: packet manhattan distance not decreased!\n");
        printf("\tnode ID: %lu, node x: %lu, node y: %lu, node z: %lu\n",
               networkNodeID, node_x, node_y, node_z);
        printf("\tmin MH dist: %lu, dest MH dist: %lu\n", _minManhattanDist, dest_distance);
        PrintData();
        exit(1);
    }
    _minManhattanDist = dest_distance;

    _nodeArrivalTimes[_visitedNodeCnt] = arrivalTime;
    _visitedNodes[_visitedNodeCnt++] = networkNodeID;
}

const std::vector<size_t>& Grid_VN3D_Packet::getVisitedNetworkNodes() const {
    return _visitedNodes;
}

size_t Grid_VN3D_Packet::getVisitedNodeCnt() const {
    return _visitedNodeCnt;
}

size_t Grid_VN3D_Packet::getDestNetworkNodeID() const {
    return _destNetworkNodeID;
}

double Grid_VN3D_Packet::GetTimeInNetwork() const {
	
	//printf("ID: %d, exit time: %lf, gen time: %lf\n", _ID, _exitTime, _genTime);
	
    return _exitTime - _genTime;
}

void Grid_VN3D_Packet::PrintData() const {
    std::string arrival_times;
    for (int i = 0; i < _visitedNodeCnt; i++) {
        arrival_times.append(std::to_string(_nodeArrivalTimes[i]) + ",");
    }

    std::string traversed_nodes;
    for (int i = 0; i < _visitedNodeCnt; i++) {
        traversed_nodes.append(std::to_string(_visitedNodes[i]) + ",");
    }

    printf("ID: %d, gen t: %lf, o: %lu, d: %lu, arrival ts: %s, nodes: %s\n",
           _ID, _genTime, _originNetworkNodeID, _destNetworkNodeID,
           arrival_times.c_str(), traversed_nodes.c_str());
}