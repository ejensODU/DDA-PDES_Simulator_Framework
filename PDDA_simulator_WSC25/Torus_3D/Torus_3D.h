#pragma once

#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <atomic>
#include "../PDDA_SimModel.h"
#include "Torus_3D_Arrive.h"
#include "Torus_3D_Depart.h"
#include "Torus_3D_NeighborInfo.h"

class Torus_3D : public PDDA_SimModel {
public:
    Torus_3D(size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
             size_t hopRadius, size_t numServersPerNetworkNode,
             size_t maxNumArriveEvents, double maxSimTime,
             size_t numThreads, size_t distSeed,
             double bucketWidth, size_t targetBinSize,
             std::string traceFolderName,
             std::string distParamsFile);

    Torus_3D_NeighborInfo GetHopNeighborStructures(size_t x, size_t y, size_t z);
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
    size_t GetVertexIndex(size_t x, size_t y, size_t z, size_t type) const;

    // Helper methods for torus wrapping
    size_t WrapCoordinate(size_t coord, size_t size) const;

	// Packets and SVs
	std::vector<Torus_3D_Packet> _packets;
	std::vector<PDDA_SV<int>> _packetQueueSVs;
    std::vector<std::queue<int>> _packetQueues;
	std::vector<int> _queueSizes;
	
	// Vertices
    std::vector<std::shared_ptr<Torus_3D_Arrive>> _arriveVertices;
    std::vector<std::shared_ptr<Torus_3D_Depart>> _departVertices;

    // Delay times
    double _minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime;
    double _minServiceTime, _modeServiceTime, _maxServiceTime;
    double _minTransitTime, _modeTransitTime, _maxTransitTime;

    // Grid parameters
    const size_t _gridSizeX;
    const size_t _gridSizeY;
    const size_t _gridSizeZ;
    const size_t _hopRadius;
    const size_t _numServersPerNetworkNode;
    const size_t _maxNumArriveEvents;
};
