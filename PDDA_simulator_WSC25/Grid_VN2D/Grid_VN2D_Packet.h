#pragma once
#include <vector>
#include "../PDDA_SimExec.h"

class Grid_VN2D_Packet : public Entity {
public:
	Grid_VN2D_Packet();
    Grid_VN2D_Packet(double genTime, size_t originNetworkNodeID, size_t destNetworkNodeID,
                     size_t destX, size_t destY, size_t gridSizeX, size_t gridSizeY);

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
    size_t _gridSizeX;
    size_t _gridSizeY;
    size_t _minManhattanDist;
};
