#pragma once

#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <unordered_map>
#include <atomic>
#include "../PDDA_SimModel.h"
#include "Ring_1D_Arrive.h"
#include "Ring_1D_Depart.h"

class Ring_1D : public PDDA_SimModel {
public:
    Ring_1D(size_t ringSize,
            size_t numServersPerNetworkNode,
            size_t maxNumArriveEvents, double maxSimTime,
            size_t numThreads, size_t distSeed,
            double bucketWidth, size_t targetBinSize,
            std::string traceFolderName,
            std::string distParamsFile);

    void PrintMeanPacketNetworkTime() const;
    virtual void PrintSVs() const override;
    virtual void PrintNumVertexExecs() const override;
    void PrintFinishedPackets() const;

private:
	// Internal methods
    void BuildModel();
    void CreateVertices();
    void AddVertices();
    void CreateEdges();
    void IO_SVs();
    void InitEvents();
    size_t GetVertexID(size_t pos, size_t type) const;

    // Helper method for ring wrapping
    size_t WrapPosition(size_t pos) const {
        return pos % _ringSize;
    }

	// Packets and SVs
    std::vector<Ring_1D_Packet> _packets;
    std::vector<PDDA_SV<int>> _packetQueueSVs;
    std::vector<std::queue<int>> _packetQueues;
	
	// Vertices
    std::vector<std::shared_ptr<Ring_1D_Arrive>> _arriveVertices;
    std::vector<std::shared_ptr<Ring_1D_Depart>> _departVertices;

    // Delay times
    double _minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime;
    double _minServiceTime, _modeServiceTime, _maxServiceTime;
    double _minTransitTime, _modeTransitTime, _maxTransitTime;

    // Ring parameters
    const size_t _ringSize;
    const size_t _numServersPerNetworkNode;
    const size_t _maxNumArriveEvents;
};
