#pragma once

#include <memory>
#include <queue>
#include <vector>
#include "../Vertex.h"
#include "../PDDA_SV.h"
#include "Grid_VN2D_Packet.h"
#include "Grid_VN2D_NeighborInfo.h"

class Grid_VN2D_Arrive;

class Grid_VN2D_Depart : public Vertex, public std::enable_shared_from_this<Grid_VN2D_Depart> {
public:
    Grid_VN2D_Depart(size_t networkNodeID, size_t x, size_t y,
                    std::vector<bool> neighbors,
                    size_t gridSizeX, size_t gridSizeY, size_t hopRadius,
                    const std::string& vertexName,
                    size_t distSeed, std::string traceFolderName,
                    std::string traceFileHeading,
                    double minServiceTime, double modeServiceTime, double maxServiceTime,
                    double minTransitTime, double modeTransitTime, double maxTransitTime,
                    std::vector<Grid_VN2D_Packet>& packets,
					PDDA_SV<int>& packetQueueSV,
                    std::queue<int>& packetQueue);

    void AddNeighborInfo(const Grid_VN2D_NeighborInfo& info);
    void AddArriveVertices(std::vector<std::shared_ptr<Grid_VN2D_Arrive>> arriveVertices);
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) override;
    virtual void Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) override;
    void PrintNeighborInfo() const;

private:
	Grid_VN2D_NeighborInfo _neighborInfo;
	std::queue<int>& _packetQueue;
	PDDA_SV<int>& _packetQueueSV;
	std::vector<bool> _neighbors;
	std::vector<Grid_VN2D_Packet>& _packets;
	std::vector<std::shared_ptr<Grid_VN2D_Arrive>> _arriveVertices;
	std::unique_ptr<class TriangularDist> _serviceDelay;
	std::unique_ptr<class TriangularDist> _transitDelay;
	const size_t _networkNodeID;
	const size_t _x;
	const size_t _y;
	const size_t _hopRadius;
	const size_t _gridSizeX;
	const size_t _gridSizeY;
	bool _packetInQueue;
};
