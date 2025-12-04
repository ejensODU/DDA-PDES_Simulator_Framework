#pragma once

#include <memory>
#include <queue>
#include <list>
#include <atomic>
#include "../Vertex.h"
#include "../PDDA_SV.h"
#include "Torus_3D_Packet.h"

class Torus_3D_Depart;  // Forward declaration for compatibility

// Optimized version of Torus_3D_Arrive that replaces the original implementation
class Torus_3D_Arrive : public Vertex, public std::enable_shared_from_this<Torus_3D_Arrive> {
public:
    Torus_3D_Arrive(size_t networkNodeID,
                    size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
                    const std::string& vertexName, size_t distSeed,
                    std::string traceFolderName, std::string traceFileHeading,
                    size_t maxNumArriveEvents,
                    double minIntraArrivalTime, double modeIntraArrivalTime, double maxIntraArrivalTime,
                    double minServiceTime, double modeServiceTime, double maxServiceTime,
                    std::vector<Torus_3D_Packet>& packets,
                    PDDA_SV<int>& packetQueueSV,
                    std::queue<int>& packetQueue, std::vector<int>& queueSizes, bool slowNode);

    // Keep the original interface but use optimized implementations
    size_t getNetworkNodeID() const;
    PDDA_SV<int>& getPacketQueueSV();
    void AddDepartVertex(std::shared_ptr<Torus_3D_Depart> departVertex);
    
    // Required Vertex methods
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) override;
    virtual void Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) override;

private:
    // Helper method for coordinate wrapping
    size_t WrapCoordinate(size_t coord, size_t size) const {
        return (coord + size) % size;
    }

    // Packet and queue management
    std::vector<Torus_3D_Packet>& _packets;
    std::queue<int>& _packetQueue;
    std::shared_ptr<Torus_3D_Depart> _departVertex;
	std::vector<int>& _queueSizes;
    
    // Random number generators (cached for performance)
    std::unique_ptr<class TriangularDist> _intraArrivalDelay;
    std::unique_ptr<class TriangularDist> _serviceDelay;
    std::unique_ptr<class UniformIntDist> _randomNodeID;
    
    // State variables
    PDDA_SV<int>& _packetQueueSV;
    const size_t _networkNodeID;
    const size_t _gridSizeX;
    const size_t _gridSizeY;
    const size_t _gridSizeZ;
    const int _maxNumIntraArriveEvents;
    int _numIntraArriveEvents;
    
    // Condition flags (reused to avoid recreating in each call)
    bool _atDestination;
    bool _intraArrival;
    bool _serverAvailable;
};