#include "Ring_1D_Depart.h"
#include "Ring_1D_Arrive.h"
#include "../Dist.h"
#include "../PDDA_SimExec.h"
#include <algorithm>

Ring_1D_Depart::Ring_1D_Depart(size_t networkNodeID,
                               size_t ringSize,
                               const std::string& vertexName,
                               size_t distSeed, std::string traceFolderName,
                               std::string traceFileHeading,
                               double minServiceTime, double modeServiceTime, double maxServiceTime,
                               double minTransitTime, double modeTransitTime, double maxTransitTime,
                               std::vector<Ring_1D_Packet>& packets,
                               PDDA_SV<int>& packetQueueSV,
                               std::queue<int>& packetQueue)
: Vertex(vertexName, 0, distSeed, traceFolderName, traceFileHeading),
_networkNodeID(networkNodeID),
_ringSize(ringSize),
_packets(packets), _packetQueueSV(packetQueueSV), _packetQueue(packetQueue) {

    _serviceDelay = std::make_unique<TriangularDist>(minServiceTime, modeServiceTime, maxServiceTime, distSeed + networkNodeID);
    _transitDelay = std::make_unique<TriangularDist>(minTransitTime, modeTransitTime, maxTransitTime, distSeed + networkNodeID);
}

void Ring_1D_Depart::AddArriveVertices(std::vector<std::shared_ptr<Ring_1D_Arrive>> arriveVertices) {
    _arriveVertices = arriveVertices;
}

void Ring_1D_Depart::IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) {
    std::vector<size_t> input_SV_indices;
    input_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Is.push_back(input_SV_indices);

    std::vector<size_t> output_SV_indices;
    output_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Os.push_back(output_SV_indices);
}

void Ring_1D_Depart::Run(PDDA_CalendarQueue& CQ, double simTime, int entityID)
{
    int packet_ID = entityID;

    //printf("\n%lu DEPART: time %lf, packet ID %d\n", _networkNodeID, simTime, packet_ID);

    // Evaluate Conditions
    _packetInQueue = (_packetQueueSV.get() > 0);

    // Update SVs
    //std::shared_ptr<Ring_1D_Packet> queue_packet;
    int queue_packet_ID = -1;
    _packetQueueSV.dec();
    if (_packetInQueue) {
        queue_packet_ID = _packetQueue.front();
        _packetQueue.pop();
    }

    // Use packet's stored direction
    int dest_dir = _packets.at(packet_ID).isClockwise() ? 0 : 1;  // 0 for clockwise, 1 for counterclockwise

    // Schedule New Events
    if (_packetInQueue) {
        CQ.AddEvent(_vertexID, simTime + _serviceDelay->GenRV(), queue_packet_ID);
        //newEvents.push_back(new PDDA_Event(shared_from_this(),
        //                                  simTime + _serviceDelay->GenRV(),
        //                                  queue_packet));
    }

    CQ.AddEvent(_arriveVertices.at(dest_dir)->getVertexID(), simTime + _transitDelay->GenRV(), packet_ID);
    //newEvents.push_back(new PDDA_Event(_arriveVertices.at(dest_dir),
    //                                  simTime + _transitDelay->GenRV(),
    //                                  packet));

    // Trace
	if (!_traceFolderName.empty()) {
		std::string trace_string = std::to_string(simTime) + ", " + std::to_string(_packetQueueSV.get());
		WriteToTrace(trace_string);
	}

    _numExecutions++;
}
