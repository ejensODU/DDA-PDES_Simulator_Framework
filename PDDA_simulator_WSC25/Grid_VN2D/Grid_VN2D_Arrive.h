#pragma once

#include <memory>
#include <queue>
#include <list>
#include <atomic>
#include "../Vertex.h"
#include "../PDDA_SV.h"
#include "Grid_VN2D_Packet.h"

class Grid_VN2D_Depart;

class Grid_VN2D_Arrive : public Vertex, public std::enable_shared_from_this<Grid_VN2D_Arrive> {
public:
    Grid_VN2D_Arrive(size_t networkNodeID, size_t gridSizeX, size_t gridSizeY,
                     const std::string& vertexName, size_t distSeed,
                     std::string traceFolderName, std::string traceFileHeading,
                     size_t maxNumArriveEvents,
                     double minIntraArrivalTime, double modeIntraArrivalTime, double maxIntraArrivalTime,
                     double minServiceTime, double modeServiceTime, double maxServiceTime,
					 std::vector<Grid_VN2D_Packet>& packets,
                     PDDA_SV<int>& packetQueueSV,
                     std::queue<int>& packetQueue);

    size_t getNetworkNodeID() const;
    PDDA_SV<int>& getPacketQueueSV();
    void AddDepartVertex(std::shared_ptr<Grid_VN2D_Depart> departVertex);
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) override;
    virtual void Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) override;

private:
	std::vector<Grid_VN2D_Packet>& _packets;
	PDDA_SV<int>& _packetQueueSV;
	std::queue<int>& _packetQueue;
	std::shared_ptr<Grid_VN2D_Depart> _departVertex;
	std::unique_ptr<class TriangularDist> _intraArrivalDelay;
	std::unique_ptr<class TriangularDist> _serviceDelay;
	std::unique_ptr<class UniformIntDist> _randomNodeID;
	const size_t _networkNodeID;
	const size_t _gridSizeX;
	const size_t _gridSizeY;
	const int _maxNumIntraArriveEvents;
	int _numIntraArriveEvents;
	bool _atDestination;
	bool _intraArrival;
	bool _serverAvailable;  
};
