#pragma once

#include <memory>
#include <queue>
#include <vector>
#include "../Vertex.h"
#include "../PDDA_SV.h"
#include "Ring_1D_Packet.h"

// Forward declaration
class Ring_1D_Arrive;

class Ring_1D_Depart : public Vertex, public std::enable_shared_from_this<Ring_1D_Depart> {
public:
    Ring_1D_Depart(size_t networkNodeID,
                   size_t ringSize,
                   const std::string& vertexName,
                   size_t distSeed, std::string traceFolderName,
                   std::string traceFileHeading,
                   double minServiceTime, double modeServiceTime, double maxServiceTime,
                   double minTransitTime, double modeTransitTime, double maxTransitTime,
                   std::vector<Ring_1D_Packet>& packets,
                   PDDA_SV<int>& packetQueueSV,
                   std::queue<int>& packetQueue);

    void AddArriveVertices(std::vector<std::shared_ptr<Ring_1D_Arrive>> arriveVertices);
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) override;
    virtual void Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) override;

private:
	// Helper method for determining routing direction
    bool shouldRouteClockwise(size_t destNodeID) const {
        size_t clockwise_dist = (destNodeID >= _networkNodeID) ?
        destNodeID - _networkNodeID :
        _ringSize - (_networkNodeID - destNodeID);
        size_t counter_dist = (_networkNodeID >= destNodeID) ?
        _networkNodeID - destNodeID :
        _ringSize - (destNodeID - _networkNodeID);
        return clockwise_dist <= counter_dist;
    }

    // 8-byte aligned pointers/references/vectors first
    std::vector<std::shared_ptr<Ring_1D_Arrive>> _arriveVertices; // 24 bytes (vector)
    std::unique_ptr<class TriangularDist> _serviceDelay; // 8 bytes
    std::unique_ptr<class TriangularDist> _transitDelay; // 8 bytes
    std::vector<Ring_1D_Packet>& _packets;          // 8 bytes (reference)
    PDDA_SV<int>& _packetQueueSV;                   // 8 bytes (reference)
    std::queue<int>& _packetQueue;                  // 8 bytes (reference)
    // 8-byte size_t members
    const size_t _networkNodeID;                    // 8 bytes
    const size_t _ringSize;                         // 8 bytes
    // 1-byte bool
    bool _packetInQueue;                            // 1 byte
};
