#pragma once

#include <memory>
#include <queue>
#include <list>
#include <atomic>
#include <unordered_set>
#include "../Vertex.h"
#include "../PDDA_SV.h"
#include "Irregular_NoC_Packet.h"

// Forward declarations
class Irregular_NoC_Depart;
class Irregular_NoC;

// External declarations of static variables for packet tracking
extern std::atomic<size_t> gTotalActivePackets;
extern std::atomic<size_t> gTotalTerminatedPackets;
extern std::atomic<size_t> gTotalCreatedPackets;
extern std::atomic<size_t> gLastTerminationCount;
extern std::atomic<double> gLastTerminationTime;

// Arrive vertex for the irregular NoC
class Irregular_NoC_Arrive : public Vertex, public std::enable_shared_from_this<Irregular_NoC_Arrive> {
public:
    Irregular_NoC_Arrive(size_t networkNodeID,
                      size_t numNodes,
                      const std::string& vertexName, size_t distSeed,
                      std::string traceFolderName, std::string traceFileHeading,
                      size_t maxNumArriveEvents,
                      double minIntraArrivalTime, double modeIntraArrivalTime, double maxIntraArrivalTime,
                      double minServiceTime, double modeServiceTime, double maxServiceTime,
                      std::vector<Irregular_NoC_Packet>& packets,
                      PDDA_SV<int>& packetQueueSV,
                      std::queue<int>& packetQueue, std::vector<int>& queueSizes, bool slowNode);

    // Get node ID
    size_t getNetworkNodeID() const;
    
    // Get packet queue SV reference
    PDDA_SV<int>& getPacketQueueSV();
    
    // Connect with depart vertex
    void AddDepartVertex(std::shared_ptr<Irregular_NoC_Depart> departVertex);
    
    // Required Vertex methods
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) override;
    virtual void Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) override;

private:
    // Check if a packet should be forcibly terminated
    bool ShouldForceTerminate(int packetId, double currentTime);
    
    // Packet and queue management
    std::vector<Irregular_NoC_Packet>& _packets;
    std::queue<int>& _packetQueue;
    std::shared_ptr<Irregular_NoC_Depart> _departVertex;
    std::vector<int>& _queueSizes;
    
    // Random number generators
    std::unique_ptr<class TriangularDist> _intraArrivalDelay;
    std::unique_ptr<class TriangularDist> _serviceDelay;
    std::unique_ptr<class UniformIntDist> _randomNodeID;
    
    // State variables
    PDDA_SV<int>& _packetQueueSV;
    const size_t _networkNodeID;
    const size_t _numNodes;
    const int _maxNumIntraArriveEvents;
    int _numIntraArriveEvents;
    
    // Condition flags
    bool _atDestination;
    bool _intraArrival;
    bool _serverAvailable;
    
    // Last known simulation time
    double _lastSimTime;
    
    // Track packets that have passed through this node for timeout detection
    std::unordered_set<int> _processedPackets;
};