#pragma once

#include <memory>
#include <queue>
#include <vector>
#include "../Vertex.h"
#include "../PDDA_SV.h"
#include "Torus_3D_Packet.h"
#include "Torus_3D_NeighborInfo.h"

class Torus_3D_Arrive;

// Optimized version of Torus_3D_Depart with global queue size knowledge
class Torus_3D_Depart : public Vertex, public std::enable_shared_from_this<Torus_3D_Depart> {
public:
    Torus_3D_Depart(size_t networkNodeID,
                    size_t x, size_t y, size_t z,
                    std::vector<bool> neighbors,
                    size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
                    size_t hopRadius, const std::string& vertexName,
                    size_t distSeed, std::string traceFolderName,
                    std::string traceFileHeading,
                    double minServiceTime, double modeServiceTime, double maxServiceTime,
                    double minTransitTime, double modeTransitTime, double maxTransitTime,
                    std::vector<Torus_3D_Packet>& packets,
                    PDDA_SV<int>& packetQueueSV,
                    std::queue<int>& packetQueue, std::vector<int>& queueSizes, bool slowNode);

    // Original interface methods
    void AddNeighborInfo(const Torus_3D_NeighborInfo& info);
    void AddArriveVertices(std::vector<std::shared_ptr<Torus_3D_Arrive>> arriveVertices);
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) override;
    virtual void Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) override;
    void PrintNeighborInfo() const;
    
    // Queue size knowledge management
    void UpdateGlobalQueueSizes(const Torus_3D_Packet& packet);
    void UpdateLocalQueueSizes(double simTime);

private:
    // Helper methods
    size_t WrapCoordinate(size_t coord, size_t size) const {
        return (coord + size) % size;
    }

    size_t GetWrappedDistance(int a, int b, size_t gridSize) const {
        int dist = std::abs(a - b);
        return std::min(dist, static_cast<int>(gridSize) - dist);
    }
    
    // Core data structures
    Torus_3D_NeighborInfo _neighborInfo;
    std::vector<Torus_3D_Packet>& _packets;
    std::queue<int>& _packetQueue;
    std::vector<std::shared_ptr<Torus_3D_Arrive>> _arriveVertices;
    std::vector<bool> _neighbors;
    
    // Shared queue size data (updated by all nodes)
    std::vector<int>& _queueSizes;  // Direct access to global queue sizes (for nodes within hop radius)
    
    // Node-local queue size knowledge (private to each node)
    std::vector<int> _globalQueueSizes;        // Node's view of all queue sizes in network
    std::vector<double> _queueSizeTimestamps;  // When queue sizes were last updated
    std::vector<bool> _knownQueueSizes;        // Whether we have data for a node
    
    // Random number generators
    std::unique_ptr<class TriangularDist> _serviceDelay;
    std::unique_ptr<class TriangularDist> _transitDelay;
    
    // Configuration
    PDDA_SV<int>& _packetQueueSV;
    const size_t _networkNodeID;
    const size_t _x;
    const size_t _y;
    const size_t _z;
    const size_t _hopRadius;
    const size_t _gridSizeX;
    const size_t _gridSizeY;
    const size_t _gridSizeZ;
    const size_t _totalNodes;  // Total number of nodes in network
    
    // State flag
    bool _packetInQueue;
};