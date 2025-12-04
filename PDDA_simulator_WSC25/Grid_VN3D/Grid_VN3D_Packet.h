#pragma once
#include <vector>
#include "../PDDA_SimExec.h"

class Grid_VN3D_Packet : public Entity {
public:
	Grid_VN3D_Packet();
    Grid_VN3D_Packet(double genTime, size_t originNetworkNodeID, size_t destNetworkNodeID,
           size_t destX, size_t destY, size_t destZ,
           size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ);

    void AddNetworkNodeData(double arrivalTime, size_t networkNodeID);
    const std::vector<size_t>& getVisitedNetworkNodes() const;
	size_t getVisitedNodeCnt() const;
    size_t getDestNetworkNodeID() const;
    double GetTimeInNetwork() const;
    void PrintData() const override;

private:
	size_t _visitedNodeCnt;
	std::vector<double> _nodeArrivalTimes;
    std::vector<size_t> _visitedNodes;
    size_t _originNetworkNodeID;
    size_t _destNetworkNodeID;
    size_t _destX;
    size_t _destY;
    size_t _destZ;
    size_t _gridSizeX;
    size_t _gridSizeY;
    size_t _gridSizeZ;
    size_t _minManhattanDist;
};
