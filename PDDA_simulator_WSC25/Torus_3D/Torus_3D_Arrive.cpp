#include "Torus_3D_Arrive.h"
#include "Torus_3D_Depart.h"
#include "../Dist.h"
#include "../PDDA_SimExec.h"
#include <thread>

extern void SpinLockData(std::atomic<int>& shared_lock);
extern void UnlockData(std::atomic<int>& shared_lock);

Torus_3D_Arrive::Torus_3D_Arrive(size_t networkNodeID,
                                size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
                                const std::string& vertexName, size_t distSeed,
                                std::string traceFolderName, std::string traceFileHeading,
                                size_t maxNumArriveEvents,
                                double minIntraArrivalTime, double modeIntraArrivalTime, double maxIntraArrivalTime,
                                double minServiceTime, double modeServiceTime, double maxServiceTime,
                                std::vector<Torus_3D_Packet>& packets,
                                PDDA_SV<int>& packetQueueSV,
                                std::queue<int>& packetQueue, std::vector<int>& queueSizes, bool slowNode)
    : Vertex(vertexName, 0, distSeed, traceFolderName, traceFileHeading),
      _networkNodeID(networkNodeID),
      _gridSizeX(gridSizeX), _gridSizeY(gridSizeY), _gridSizeZ(gridSizeZ),
      _numIntraArriveEvents(0), _maxNumIntraArriveEvents(maxNumArriveEvents),
      _packets(packets), _packetQueueSV(packetQueueSV), _packetQueue(packetQueue),
      _atDestination(false), _intraArrival(false), _serverAvailable(false),
      _queueSizes(queueSizes) {

    // Initialize random number generators (once at construction)
    _randomNodeID = std::make_unique<UniformIntDist>(0, _gridSizeX * _gridSizeY * _gridSizeZ - 1, distSeed + networkNodeID);

    // Adjust service times for slow nodes
    if (slowNode) {
        minServiceTime *= 10;
        modeServiceTime *= 10;
        maxServiceTime *= 10;
    }

    // Create distribution generators
    _intraArrivalDelay = std::make_unique<TriangularDist>(minIntraArrivalTime,
                                                         modeIntraArrivalTime,
                                                         maxIntraArrivalTime,
                                                         distSeed + networkNodeID);
    _serviceDelay = std::make_unique<TriangularDist>(minServiceTime,
                                                    modeServiceTime,
                                                    maxServiceTime,
                                                    distSeed + networkNodeID);
}

size_t Torus_3D_Arrive::getNetworkNodeID() const {
    return _networkNodeID;
}

PDDA_SV<int>& Torus_3D_Arrive::getPacketQueueSV() {
    return _packetQueueSV;
}

void Torus_3D_Arrive::AddDepartVertex(std::shared_ptr<Torus_3D_Depart> departVertex) {
    _departVertex = departVertex;
}

void Torus_3D_Arrive::IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) {
    // Keep this method exactly as in the original implementation
    std::vector<size_t> input_SV_indices;
    input_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Is.push_back(input_SV_indices);

    std::vector<size_t> output_SV_indices;
    output_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Os.push_back(output_SV_indices);
}

void Torus_3D_Arrive::Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) {
    // Keep the same general structure as the original Run method
    int packet_ID = entityID;

    // Evaluate Conditions
    _atDestination = false;
    if (-1 == packet_ID) {
        _intraArrival = true;
    } else {
        _intraArrival = false;
        
        // Check if packet has reached destination
        if (_networkNodeID == _packets.at(packet_ID).getDestNetworkNodeID()) {
            _atDestination = true;
        }
        
        // CORRECT LOCATION: Update global queue knowledge when packet arrives
        if (_departVertex && packet_ID >= 0) {
            // Extract queue size history from incoming packet
            _departVertex->UpdateGlobalQueueSizes(_packets.at(packet_ID));
        }
    }

    _serverAvailable = (_packetQueueSV.get() < 0);

    // Update SVs
    if (_intraArrival) {
        // Generate a new packet
        size_t dest_network_node_ID = _randomNodeID->GenRV();
        while (_networkNodeID == dest_network_node_ID) {
            dest_network_node_ID = _randomNodeID->GenRV();
        }

        // Calculate coordinates for destination
        size_t dest_x = dest_network_node_ID & (_gridSizeX - 1);
        size_t dest_y = (dest_network_node_ID / _gridSizeX) & (_gridSizeY - 1);
        size_t dest_z = dest_network_node_ID / (_gridSizeX * _gridSizeY);

        // Create packet in pre-allocated packet pool
        Torus_3D_Packet new_packet(simTime, _networkNodeID, dest_network_node_ID,
                                  dest_x, dest_y, dest_z,
                                  _gridSizeX, _gridSizeY, _gridSizeZ);
                                        
        //packet_ID = new_packet.getID();
		packet_ID = _maxNumIntraArriveEvents*_networkNodeID + _numIntraArriveEvents;
        _packets.at(packet_ID) = std::move(new_packet);
    }

    // Add this node to the packet's path with queue size information
    int currentQueueSize = _packetQueueSV.get();
    _packets.at(packet_ID).AddNetworkNodeData(simTime, _networkNodeID, currentQueueSize);
    
    if (!_atDestination) {
        // Packet needs to continue through the network
        _packetQueueSV.inc();
        // Update queue size data (for path-finding in NeighborInfo)
        _queueSizes[_networkNodeID] = _packetQueueSV.get();
        
        if (!_serverAvailable) {
            _packetQueue.push(packet_ID);
        }
    } else {
        // Packet has reached its destination
        _packets.at(packet_ID).setExitTime(simTime);
		
		//printf("Packet %d reached destination node %lu at sim time %lf, origin time: %lf\n", packet_ID, _networkNodeID, simTime, _packets.at(packet_ID).getGenTime());
    }

    // Schedule New Events
    if (!_atDestination && _serverAvailable) {
        // Schedule processing at the depart vertex
        double serviceDelay = _serviceDelay->GenRV();
        CQ.AddEvent(_departVertex->getVertexID(), simTime + serviceDelay, packet_ID);
    }

    if (_intraArrival && ++_numIntraArriveEvents < _maxNumIntraArriveEvents) {
        // Schedule next packet generation
        double arrivalDelay = _intraArrivalDelay->GenRV();
        CQ.AddEvent(_vertexID, simTime + arrivalDelay, -1);    
    }

    // Trace
    if (!_traceFolderName.empty()) {
        std::string trace_string = std::to_string(simTime) + ", " + std::to_string(_packetQueueSV.get());
        WriteToTrace(trace_string);
    }

    _numExecutions++;
}