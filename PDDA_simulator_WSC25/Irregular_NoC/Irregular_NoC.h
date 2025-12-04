#pragma once

#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <atomic>
#include "../PDDA_SimModel.h"
#include "Irregular_NoC_Common.h"
#include "Irregular_NoC_Arrive.h"
#include "Irregular_NoC_Depart.h"
#include "Irregular_NoC_NeighborInfo.h"

class Irregular_NoC : public PDDA_SimModel {
public:
    Irregular_NoC(size_t numNodes,
             size_t hopRadius, size_t numServersPerNetworkNode,
             size_t maxNumArriveEvents, double maxSimTime,
             size_t numThreads, size_t distSeed,
             double bucketWidth, size_t targetBinSize,
             std::string traceFolderName,
             std::string distParamsFile,
             std::string networkTopologyFile = "");

    Irregular_NoC_NeighborInfo GetHopNeighborStructures(size_t nodeID);
    void PrintMeanPacketNetworkTime() const;
    virtual void PrintSVs() const override;
    virtual void PrintNumVertexExecs() const override;
    void PrintFinishedPackets() const;
    void PrintNetworkTopology() const;
	
	// Add these methods to your public section
    void PacketGenerated();                  // Call when a new packet is generated
    void PacketTerminated();                 // Call when a packet reaches destination

private:
    // Internal methods
    void BuildModel();
    void CreateVertices();
    void AddVertices();
    void CreateEdges();
    void IO_SVs();
    void InitEvents();
    void LoadNetworkTopology(const std::string& topologyFile);
    void GenerateNetworkTopology(); // Alternative to loading from file

    // Packets and SVs
    std::vector<Irregular_NoC_Packet> _packets;
    std::vector<PDDA_SV<int>> _packetQueueSVs;
    std::vector<std::queue<int>> _packetQueues;
    std::vector<int> _queueSizes;
    
    // Vertices
    std::vector<std::shared_ptr<Irregular_NoC_Arrive>> _arriveVertices;
    std::vector<std::shared_ptr<Irregular_NoC_Depart>> _departVertices;

    // Network topology
    std::vector<std::vector<NetworkLink>> _adjacencyList; // Connection graph
    
    // Delay times
    double _minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime;
    double _minServiceTime, _modeServiceTime, _maxServiceTime;
    double _minTransitTime, _modeTransitTime, _maxTransitTime;
    double _expressTransitFactor;  // Time multiplier for express links (e.g., 0.5 means twice as fast)

    // Network parameters
    const size_t _numNodes;               // Total number of nodes
    const size_t _hopRadius;              // Visibility radius
    const size_t _numServersPerNetworkNode;
    const size_t _maxNumArriveEvents;
    const std::string _networkTopologyFile; // File defining network topology
	
	// Packet tracking for early termination
    std::atomic<size_t> _activePackets;      // Count of packets still in transit
    std::atomic<size_t> _terminatedPackets;  // Count of packets that reached destination
    size_t _totalPackets;                    // Total packets in the simulation
};