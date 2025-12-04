#include "Grid_VN3D_Arrive.h"
#include "Grid_VN3D_Depart.h"
#include "../Dist.h"
#include "../PDDA_SimExec.h"
#include <thread>

extern void SpinLockData(std::atomic<int>& shared_lock);
extern void UnlockData(std::atomic<int>& shared_lock);

Grid_VN3D_Arrive::Grid_VN3D_Arrive(size_t networkNodeID,
                                size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
                                const std::string& vertexName, size_t distSeed,
                                std::string traceFolderName, std::string traceFileHeading,
                                size_t maxNumArriveEvents,
                                double minIntraArrivalTime, double modeIntraArrivalTime, double maxIntraArrivalTime,
                                double minServiceTime, double modeServiceTime, double maxServiceTime,
                                std::vector<Grid_VN3D_Packet>& packets,
								PDDA_SV<int>& packetQueueSV,
								std::queue<int>& packetQueue)
    : Vertex(vertexName, 0, distSeed, traceFolderName, traceFileHeading),
      _networkNodeID(networkNodeID),
      _gridSizeX(gridSizeX), _gridSizeY(gridSizeY), _gridSizeZ(gridSizeZ),
      _numIntraArriveEvents(0), _maxNumIntraArriveEvents(maxNumArriveEvents),
      _packets(packets), _packetQueueSV(packetQueueSV), _packetQueue(packetQueue) {

    _randomNodeID = std::make_unique<UniformIntDist>(0, _gridSizeX * _gridSizeY * _gridSizeZ - 1, distSeed + networkNodeID);

    int high_activity_node = UniformIntDist(1, 6, distSeed + networkNodeID).GenRV();
    if (1 == high_activity_node) {
        minIntraArrivalTime /= 2;
        modeIntraArrivalTime /= 2;
        maxIntraArrivalTime /= 2;
    }

    _intraArrivalDelay = std::make_unique<TriangularDist>(minIntraArrivalTime,
                                                         modeIntraArrivalTime,
                                                         maxIntraArrivalTime,
                                                         distSeed + networkNodeID);
    _serviceDelay = std::make_unique<TriangularDist>(minServiceTime,
                                                    modeServiceTime,
                                                    maxServiceTime,
                                                    distSeed + networkNodeID);
}

size_t Grid_VN3D_Arrive::getNetworkNodeID() const {
    return _networkNodeID;
}

PDDA_SV<int>& Grid_VN3D_Arrive::getPacketQueueSV() {
    return _packetQueueSV;
}

void Grid_VN3D_Arrive::AddDepartVertex(std::shared_ptr<Grid_VN3D_Depart> departVertex) {
    _departVertex = departVertex;
}

void Grid_VN3D_Arrive::IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) {
    std::vector<size_t> input_SV_indices;
    input_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Is.push_back(input_SV_indices);

    std::vector<size_t> output_SV_indices;
    output_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Os.push_back(output_SV_indices);
}

void Grid_VN3D_Arrive::Run(PDDA_CalendarQueue& CQ, double simTime, int entityID)
{
    int packet_ID = entityID;

    // Evaluate Conditions
    _atDestination = false;
    if (-1 == packet_ID) {
        _intraArrival = true;
    } else {
        _intraArrival = false;
        if (_networkNodeID == _packets.at(packet_ID).getDestNetworkNodeID()) {
            _atDestination = true;
        }
    }

    _serverAvailable = (_packetQueueSV.get() < 0);

    // Update SVs
    if (_intraArrival) {
        size_t dest_network_node_ID = _randomNodeID->GenRV();
        while (_networkNodeID == dest_network_node_ID) {
            dest_network_node_ID = _randomNodeID->GenRV();
        }

        size_t dest_x = dest_network_node_ID % _gridSizeX;
        size_t dest_y = (dest_network_node_ID / _gridSizeX) % _gridSizeY;
        size_t dest_z = dest_network_node_ID / (_gridSizeX * _gridSizeY);

		// Create packet
        Grid_VN3D_Packet new_packet(simTime, _networkNodeID, dest_network_node_ID,
                                        dest_x, dest_y, dest_z,
                                        _gridSizeX, _gridSizeY, _gridSizeZ);
										
		packet_ID = new_packet.getID();
        //printf("\tnew packet ID: %d, sim time: %lf\n", packet_ID, simTime);
        _packets.at(packet_ID) = std::move(new_packet);
    }

    _packets.at(packet_ID).AddNetworkNodeData(simTime, _networkNodeID);

    if (!_atDestination) {
        _packetQueueSV.inc();
        if (!_serverAvailable) {
            _packetQueue.push(packet_ID);
        }
    } else {
        _packets.at(packet_ID).setExitTime(simTime);
    }

    // Schedule New Events
    if (!_atDestination && _serverAvailable) {
        //newEvents.push_back(new PDDA_Event(_departVertex,
        //                                 simTime + _serviceDelay->GenRV(),
        //                                 packet));
		
		CQ.AddEvent(_departVertex->getVertexID(), simTime + _serviceDelay->GenRV(), packet_ID);
    }

    if (_intraArrival && ++_numIntraArriveEvents < _maxNumIntraArriveEvents) {
        //newEvents.push_back(new PDDA_Event(shared_from_this(),
        //                                 simTime + _intraArrivalDelay->GenRV(),
        //                                 nullptr));
		
		CQ.AddEvent(_vertexID, simTime + _intraArrivalDelay->GenRV(), -1);
    }

    // Trace
	if (!_traceFolderName.empty()) {
		std::string trace_string = std::to_string(simTime) + ", " + std::to_string(_packetQueueSV.get());
		WriteToTrace(trace_string);
	}

    _numExecutions++;
}
