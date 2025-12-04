#pragma once

#include <memory>
#include <queue>
#include <vector>
#include "../Vertex.h"
#include "../PDDA_SV.h"
#include "Grid_VN3D_Packet.h"
#include "Grid_VN3D_NeighborInfo.h"

class Grid_VN3D_Depart : public Vertex, public std::enable_shared_from_this<Grid_VN3D_Depart> {
public:
    Grid_VN3D_Depart(size_t networkNodeID,
                    size_t x, size_t y, size_t z,
                    std::vector<bool> neighbors,
                    size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
                    size_t hopRadius, const std::string& vertexName,
                    size_t distSeed, std::string traceFolderName,
                    std::string traceFileHeading,
                    double minServiceTime, double modeServiceTime, double maxServiceTime,
                    double minTransitTime, double modeTransitTime, double maxTransitTime,
                    std::vector<Grid_VN3D_Packet>& packets,
                    PDDA_SV<int>& packetQueueSV,
                    std::queue<int>& packetQueue);

    void AddNeighborInfo(const Grid_VN3D_NeighborInfo& info);
    void AddArriveVertices(std::vector<std::shared_ptr<Grid_VN3D_Arrive>> arriveVertices);
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) override;
    virtual void Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) override;
    void PrintNeighborInfo() const;

private:
	Grid_VN3D_NeighborInfo _neighborInfo;
	std::queue<int>& _packetQueue;
    std::vector<std::shared_ptr<Grid_VN3D_Arrive>> _arriveVertices;
	std::vector<Grid_VN3D_Packet>& _packets;
	std::vector<bool> _neighbors;  // Six neighbors in 3D: West, East, North, South, Up, Down
    std::unique_ptr<class TriangularDist> _serviceDelay;
    std::unique_ptr<class TriangularDist> _transitDelay;
	PDDA_SV<int>& _packetQueueSV;
    const size_t _networkNodeID;
    const size_t _x;
    const size_t _y;
    const size_t _z;
    const size_t _hopRadius;
    const size_t _gridSizeX;
    const size_t _gridSizeY;
    const size_t _gridSizeZ;
    bool _packetInQueue;    
};
