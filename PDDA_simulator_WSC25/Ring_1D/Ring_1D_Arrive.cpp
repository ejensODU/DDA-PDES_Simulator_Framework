#include "Ring_1D_Arrive.h"
#include "Ring_1D_Depart.h"
#include "../Dist.h"
#include "../PDDA_SimExec.h"
#include <thread>

extern void SpinLockData(std::atomic<int>& shared_lock);
extern void UnlockData(std::atomic<int>& shared_lock);

Ring_1D_Arrive::Ring_1D_Arrive(size_t networkNodeID,
                             size_t ringSize,
                             const std::string& vertexName, size_t distSeed,
                             std::string traceFolderName, std::string traceFileHeading,
                             size_t maxNumArriveEvents,
                             double minIntraArrivalTime, double modeIntraArrivalTime, double maxIntraArrivalTime,
                             double minServiceTime, double modeServiceTime, double maxServiceTime,
                             std::vector<Ring_1D_Packet>& packets,
                             PDDA_SV<int>& packetQueueSV,
                             std::queue<int>& packetQueue)
    : Vertex(vertexName, 0, distSeed, traceFolderName, traceFileHeading),
      _networkNodeID(networkNodeID),
      _ringSize(ringSize),
      _numIntraArriveEvents(0), _maxNumIntraArriveEvents(maxNumArriveEvents),
      _packets(packets), _packetQueueSV(packetQueueSV), _packetQueue(packetQueue) {

    _randomNodeID = std::make_unique<UniformIntDist>(0, _ringSize - 1, distSeed + networkNodeID);

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

size_t Ring_1D_Arrive::getNetworkNodeID() const {
    return _networkNodeID;
}

PDDA_SV<int>& Ring_1D_Arrive::getPacketQueueSV() {
    return _packetQueueSV;
}

void Ring_1D_Arrive::AddDepartVertex(std::shared_ptr<Ring_1D_Depart> departVertex) {
    _departVertex = departVertex;
}

void Ring_1D_Arrive::IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) {
    std::vector<size_t> input_SV_indices;
    input_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Is.push_back(input_SV_indices);

    std::vector<size_t> output_SV_indices;
    output_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Os.push_back(output_SV_indices);
}

void Ring_1D_Arrive::Run(PDDA_CalendarQueue& CQ, double simTime, int entityID)
{
    int packet_ID = entityID;

    //printf("\n%lu ARRIVE: time %lf, packet ID %d\n", _networkNodeID, simTime, packet_ID);

    // Evaluate Conditions
    _atDestination = false;
    if (-1 == packet_ID) {
        _intraArrival = true;
    } else {
        _intraArrival = false;
        if (_networkNodeID == _packets.at(packet_ID).getDestNodeID()) {
            _atDestination = true;
        }
    }

    _serverAvailable = (_packetQueueSV.get() < 0);

    // Update SVs
    if (_intraArrival) {
        size_t dest_node_id = _randomNodeID->GenRV();
        while (_networkNodeID == dest_node_id) {
            dest_node_id = _randomNodeID->GenRV();
        }

        // Determine initial direction using shortest path
        bool clockwise;
        size_t clockwise_dist = (dest_node_id >= _networkNodeID) ?
        dest_node_id - _networkNodeID :
        _ringSize - (_networkNodeID - dest_node_id);
        size_t counter_dist = (_networkNodeID >= dest_node_id) ?
        _networkNodeID - dest_node_id :
        _ringSize - (dest_node_id - _networkNodeID);
        clockwise = (clockwise_dist <= counter_dist);

        // Create packet with determined direction
        Ring_1D_Packet new_packet(simTime, _networkNodeID, dest_node_id, _ringSize, clockwise);
        //packet_ID = new_packet.getID();
		packet_ID = _maxNumIntraArriveEvents*_networkNodeID + _numIntraArriveEvents;
        _packets.at(packet_ID) = std::move(new_packet);
    }

    _packets.at(packet_ID).AddNodeData(simTime, _networkNodeID);

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

        //printf("arrive vertex ID: %lu, depart vertex ID: %lu\n", _vertexID, _departVertex->getVertexID());

        CQ.AddEvent(_departVertex->getVertexID(), simTime + _serviceDelay->GenRV(), packet_ID);
        //newEvents.push_back(new PDDA_Event(_departVertex,
        //                                  simTime + _serviceDelay->GenRV(),
        //                                  packet));
    }

    if (_intraArrival && ++_numIntraArriveEvents < _maxNumIntraArriveEvents) {
        CQ.AddEvent(_vertexID, simTime + _intraArrivalDelay->GenRV(), -1);
        //newEvents.push_back(new PDDA_Event(shared_from_this(),
        //                                  simTime + _intraArrivalDelay->GenRV(),
        //                                  nullptr));
    }

    // Trace
	if (!_traceFolderName.empty()) {
		std::string trace_string = std::to_string(simTime) + ", " + std::to_string(_packetQueueSV.get());
		WriteToTrace(trace_string);
	}

    _numExecutions++;
}
