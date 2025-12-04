#pragma once

#include <array>  // Added for std::array
#include "../PDDA_SimExec.h"

class Ring_1D_Packet : public Entity {
public:
    // Maximum number of nodes a packet can visit is half the ring size
    // Worst case scenario is a packet traversing half the ring (2048)
    static constexpr size_t MAX_VISITED_NODES = 2048;

    Ring_1D_Packet();
    Ring_1D_Packet(double genTime, size_t originNodeID, size_t destNodeID,
                   size_t ringSize, bool clockwise);
    Ring_1D_Packet(const Ring_1D_Packet& other);  // Copy constructor
    void AddNodeData(double arrivalTime, size_t nodeID);
    const std::array<size_t, MAX_VISITED_NODES>& getVisitedNodes() const;
    size_t getDestNodeID() const;
    bool isClockwise() const { return _clockwise; }
    double GetTimeInNetwork() const;
    void PrintData() const override;

private:
    // Helper method for wrapped distance calculation
    size_t getWrappedDistance(size_t pos1, size_t pos2) const {
        size_t clockwise_dist = (pos2 >= pos1) ?
        pos2 - pos1 :
        _ringSize - (pos1 - pos2);
        size_t counter_dist = (pos1 >= pos2) ?
        pos1 - pos2 :
        _ringSize - (pos2 - pos1);
        return std::min(clockwise_dist, counter_dist);
    }

    size_t _visitedNodeCnt;
    std::array<double, MAX_VISITED_NODES> _nodeArrivalTimes;
    std::array<size_t, MAX_VISITED_NODES> _visitedNodes;
    size_t _originNodeID;
    size_t _destNodeID;
    size_t _ringSize;
    size_t _minWrappedDist;  // Minimum wrapped distance seen so far
    bool _clockwise;  // Direction flag
};