#pragma once

#include <memory>
#include <queue>
#include <list>
#include <atomic>
#include "../Vertex.h"
#include "../PDDA_SV.h"
#include "Ring_1D_Packet.h"

class Ring_1D_Depart;

class Ring_1D_Arrive : public Vertex, public std::enable_shared_from_this<Ring_1D_Arrive> {
public:
    Ring_1D_Arrive(size_t networkNodeID,
                 size_t ringSize,
                 const std::string& vertexName, size_t distSeed,
                 std::string traceFolderName, std::string traceFileHeading,
                 size_t maxNumArriveEvents,
                 double minIntraArrivalTime, double modeIntraArrivalTime, double maxIntraArrivalTime,
                 double minServiceTime, double modeServiceTime, double maxServiceTime,
                 std::vector<Ring_1D_Packet>& packets,
                 PDDA_SV<int>& packetQueueSV,
                 std::queue<int>& packetQueue);

    size_t getNetworkNodeID() const;
    PDDA_SV<int>& getPacketQueueSV();
    void AddDepartVertex(std::shared_ptr<Ring_1D_Depart> departVertex);
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) override;
    virtual void Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) override;

private:
    // 8-byte aligned members first
    std::unique_ptr<class UniformIntDist> _randomNodeID;    // 8 bytes
    std::unique_ptr<class TriangularDist> _intraArrivalDelay; // 8 bytes
    std::unique_ptr<class TriangularDist> _serviceDelay;    // 8 bytes
    std::shared_ptr<Ring_1D_Depart> _departVertex;          // 8 bytes
    std::vector<Ring_1D_Packet>& _packets;                  // 8 bytes (reference)
    PDDA_SV<int>& _packetQueueSV;                           // 8 bytes (reference)
    std::queue<int>& _packetQueue;                          // 8 bytes (reference)
    const size_t _networkNodeID;                            // 8 bytes
    const size_t _ringSize;                                 // 8 bytes
    // 4-byte members
    int _numIntraArriveEvents;                             // 4 bytes
    const int _maxNumIntraArriveEvents;                    // 4 bytes
    // 1-byte members (will be packed together)
    bool _atDestination;                                   // 1 byte
    bool _intraArrival;                                    // 1 byte
    bool _serverAvailable;                                 // 1 byte
    // 1 byte padding likely added here to maintain alignment
};
