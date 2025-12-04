#pragma once

#include <memory>
#include <queue>
#include <vector>
#include <unordered_map>
#include "../Vertex.h"
#include "../PDDA_SV.h"
#include "Irregular_NoC_Common.h"
#include "Irregular_NoC_Packet.h"
#include "Irregular_NoC_NeighborInfo.h"

class Irregular_NoC_Arrive;

// Optimized version of Depart class for irregular networks
class Irregular_NoC_Depart : public Vertex, public std::enable_shared_from_this<Irregular_NoC_Depart> {
public:
    // Updated constructor signature to match the call in Irregular_NoC.cpp
    Irregular_NoC_Depart(size_t networkNodeID,
                       const std::vector<std::vector<NetworkLink>>& adjacencyList,
                       size_t hopRadius,
                       const std::string& vertexName,
                       size_t distSeed, 
                       std::string traceFolderName,
                       std::string traceFileHeading,
                       double minServiceTime, 
                       double modeServiceTime, 
                       double maxServiceTime,
                       double minTransitTime, 
                       double modeTransitTime, 
                       double maxTransitTime,
                       std::vector<Irregular_NoC_Packet>& packets,
                       PDDA_SV<int>& packetQueueSV,
                       std::queue<int>& packetQueue, 
                       std::vector<int>& queueSizes, 
                       bool slowNode);

    // Interface methods
    void AddNeighborInfo(const Irregular_NoC_NeighborInfo& info);
    void AddArriveVertices(const std::unordered_map<size_t, std::shared_ptr<Irregular_NoC_Arrive>>& arriveVertexMap);
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) override;
    virtual void Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) override;
    void PrintNeighborInfo() const; // Added missing declaration
    
    // Queue size knowledge management
    void UpdateGlobalQueueSizes(const Irregular_NoC_Packet& packet);
    void UpdateLocalQueueSizes(double simTime);

private:
    // Core data structures
    Irregular_NoC_NeighborInfo _neighborInfo;
    std::vector<Irregular_NoC_Packet>& _packets;
    std::queue<int>& _packetQueue;
    
    // Network topology reference
    const std::vector<std::vector<NetworkLink>>& _adjacencyList;
    
    // Map of node IDs to arrive vertices for packet forwarding
    std::unordered_map<size_t, std::shared_ptr<Irregular_NoC_Arrive>> _nodeIdToArriveVertex;
    
    // Shared queue size data
    std::vector<int>& _queueSizes;  // Direct access to global queue sizes
    
    // Node-local queue size knowledge
    std::vector<int> _globalQueueSizes;        // Node's view of all queue sizes in network
    std::vector<double> _queueSizeTimestamps;  // When queue sizes were last updated
    std::vector<bool> _knownQueueSizes;        // Whether we have data for a node
    
    // Random number generators
    std::unique_ptr<class TriangularDist> _serviceDelay;
    std::unique_ptr<class TriangularDist> _transitDelay;
    
    // Configuration
    PDDA_SV<int>& _packetQueueSV;
    const size_t _networkNodeID;
    const size_t _hopRadius;
    const size_t _numNodes;  // Total number of nodes in network
    
    // State flag
    bool _packetInQueue;
};