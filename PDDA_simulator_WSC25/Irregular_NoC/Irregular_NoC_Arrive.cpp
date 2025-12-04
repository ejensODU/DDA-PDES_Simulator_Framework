#include "Irregular_NoC_Arrive.h"
#include "Irregular_NoC_Depart.h"
#include "../Dist.h"
#include "../PDDA_SimExec.h"
#include <thread>
#include <iostream>
#include <algorithm>

extern void SpinLockData(std::atomic<int>& shared_lock);
extern void UnlockData(std::atomic<int>& shared_lock);

// Define the global atomic variables
std::atomic<size_t> gTotalActivePackets(0);
std::atomic<size_t> gTotalTerminatedPackets(0);
std::atomic<size_t> gTotalCreatedPackets(0);
std::atomic<size_t> gLastTerminationCount(0);
std::atomic<double> gLastTerminationTime(0.0);

Irregular_NoC_Arrive::Irregular_NoC_Arrive(size_t networkNodeID,
                                size_t numNodes,
                                const std::string& vertexName, size_t distSeed,
                                std::string traceFolderName, std::string traceFileHeading,
                                size_t maxNumArriveEvents,
                                double minIntraArrivalTime, double modeIntraArrivalTime, double maxIntraArrivalTime,
                                double minServiceTime, double modeServiceTime, double maxServiceTime,
                                std::vector<Irregular_NoC_Packet>& packets,
                                PDDA_SV<int>& packetQueueSV,
                                std::queue<int>& packetQueue, std::vector<int>& queueSizes, bool slowNode)
    : Vertex(vertexName, 0, distSeed, traceFolderName, traceFileHeading),
      _networkNodeID(networkNodeID),
      _numNodes(numNodes),
      _numIntraArriveEvents(0), _maxNumIntraArriveEvents(maxNumArriveEvents),
      _packets(packets), _packetQueueSV(packetQueueSV), _packetQueue(packetQueue),
      _atDestination(false), _intraArrival(false), _serverAvailable(false),
      _queueSizes(queueSizes),
      _lastSimTime(0.0) {

    // Initialize random number generators
    _randomNodeID = std::make_unique<UniformIntDist>(0, numNodes - 1, distSeed + networkNodeID);

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

size_t Irregular_NoC_Arrive::getNetworkNodeID() const {
    return _networkNodeID;
}

PDDA_SV<int>& Irregular_NoC_Arrive::getPacketQueueSV() {
    return _packetQueueSV;
}

void Irregular_NoC_Arrive::AddDepartVertex(std::shared_ptr<Irregular_NoC_Depart> departVertex) {
    _departVertex = departVertex;
}

void Irregular_NoC_Arrive::IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) {
    std::vector<size_t> input_SV_indices;
    input_SV_indices.push_back(_packetQueueSV.getModelIndex());
	//printf("arrive (%lu) input SV index: %lu\n", _networkNodeID, _packetQueueSV.getModelIndex());
    Is.push_back(input_SV_indices);

    std::vector<size_t> output_SV_indices;
    output_SV_indices.push_back(_packetQueueSV.getModelIndex());
	//printf("arrive (%lu) output SV index: %lu\n", _networkNodeID, _packetQueueSV.getModelIndex());
    Os.push_back(output_SV_indices);
}

bool Irregular_NoC_Arrive::ShouldForceTerminate(int packetId, double currentTime) {
    // Simply check if the packet has exceeded the maximum allowed hops
    if (_packets.at(packetId).hasExceededMaxHops()) {
        std::cerr << "WARNING: Packet " << packetId << " forcibly terminated after exceeding " 
                  << (NOC_MAX_PATH_LENGTH - 2) << " hops." << std::endl;
        return true;
    }
    
    /* // Alternatively, check for obvious routing loops (same node visited multiple times)
    if (_packets.at(packetId).isInRoutingLoop()) {
        std::cerr << "WARNING: Packet " << packetId << " forcibly terminated after detecting routing loop." 
                  << std::endl;
        return true;
    } */
    
    return false;
}

void Irregular_NoC_Arrive::Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) {
    int packet_ID = entityID;
	
	//if (69 == packet_ID) printf("packet 69 arrive: %lf, node ID: %lu\n", simTime, _networkNodeID);
    
    // Update last known simulation time
    _lastSimTime = simTime;

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
        
        // Check if packet should be forcibly terminated
        if (!_atDestination && ShouldForceTerminate(packet_ID, simTime)) {
            _atDestination = true;
            _packets.at(packet_ID).markForciblyTerminated();
        }
        
        // Update global queue knowledge when packet arrives
        if (_departVertex && packet_ID >= 0) {
            // Extract queue size history from incoming packet
            _departVertex->UpdateGlobalQueueSizes(_packets.at(packet_ID));
        }
    }

    _serverAvailable = (_packetQueueSV.get() < 0);

    // Update SVs
    if (_intraArrival) {
        // Generate a new packet with random destination
        size_t dest_network_node_ID;
        do {
            dest_network_node_ID = _randomNodeID->GenRV();
        } while (_networkNodeID == dest_network_node_ID);

        // Create packet in pre-allocated packet pool
        Irregular_NoC_Packet new_packet(
            simTime, _networkNodeID, dest_network_node_ID,
            _numNodes);
                                        
        //packet_ID = new_packet.getID();
		packet_ID = _maxNumIntraArriveEvents*_networkNodeID + _numIntraArriveEvents;
        _packets.at(packet_ID) = std::move(new_packet);
        
        // Track new packet creation globally
        gTotalCreatedPackets++;
        gTotalActivePackets++;
		
		// Print completion info
		//std::cout << "Packet " << packet_ID << " generated, sim time: " << simTime << std::endl;
		//printf("packet %d generated, time: %lf\n", packet_ID, simTime);
        
        // Special check for first packet - initialize last termination time
        if (gTotalCreatedPackets.load() == 1) {
            gLastTerminationTime.store(simTime);
        }
    }

    // Add this node to the packet's path with queue size information
    int currentQueueSize = _packetQueueSV.get();
    _packets.at(packet_ID).AddNetworkNodeData(simTime, _networkNodeID, currentQueueSize);
    
    // Update the packet processing section in Run method to avoid double counting
	// Replace the relevant part of the Run method with this:

	if (!_atDestination) {
		// Packet needs to continue through the network
		_packetQueueSV.inc();
		// Update queue size data (for path-finding)
		_queueSizes[_networkNodeID] = _packetQueueSV.get();
		
		if (!_serverAvailable) {
			_packetQueue.push(packet_ID);
		}
	} else {
		// Packet has reached its destination (or was forcibly terminated)
		_packets.at(packet_ID).setExitTime(simTime);
		
		// Only count termination if this is the first time we're processing this termination
		// Check if the packet was already marked as terminated but not yet counted
		if (!_packets.at(packet_ID).wasForciblyTerminated()) {
			// Update global packet tracking - with underflow protection
			if (gTotalActivePackets.load() > 0) {
				gTotalActivePackets--;
			}
			gTotalTerminatedPackets++;
			
			// Update last termination time and count for progress tracking
			gLastTerminationTime.store(simTime);
			gLastTerminationCount.store(gTotalTerminatedPackets.load());
		}
		
		// Print completion info
		//std::cout << "\tPacket " << packet_ID << " reached destination, sim time: " << simTime;
		//if (_packets.at(packet_ID).wasForciblyTerminated()) {
		//	std::cout << " (forcibly terminated)";
		//}
		//std::cout << std::endl;
		
		//printf("\tpacket %d reached destination, time: %lf\n", packet_ID, simTime);
		
		// Print completion statistics at regular intervals
		//if (gTotalTerminatedPackets.load() % 100 == 0 || 
		//	(gTotalCreatedPackets.load() > 0 && 
		//	 gTotalTerminatedPackets.load() >= gTotalCreatedPackets.load() - 10)) {
		//	std::cout << "Packets completed: " << gTotalTerminatedPackets.load()
		//			  << " / " << gTotalCreatedPackets.load()
		//			  << " (Active: " << gTotalActivePackets.load() << ")" << std::endl;
		//}
		
		// Special handling for the last few packets - force terminate any remaining
		/* if (gTotalCreatedPackets.load() > 0 && 
			gTotalTerminatedPackets.load() >= gTotalCreatedPackets.load() - 5 && 
			gTotalActivePackets.load() > 0) {
			
			// Aggressively terminate remaining packets
			std::cout << "Close to completion - forcing termination of remaining packets" << std::endl;
			
			for (size_t i = 0; i < _packets.size(); i++) {
				// Skip packets we've already processed or are already terminated
				if (i == static_cast<size_t>(packet_ID) || 
					_packets[i].wasForciblyTerminated() ||
					_packets[i].GetTimeInNetwork() > 0) {
					continue;
				}
				
				// Check if this packet is active but hasn't completed
				if (_packets[i].getHopCount() > 0) { // Ensure it was actually created and used
					// Mark packet as forcibly terminated
					_packets[i].markForciblyTerminated();
					_packets[i].setExitTime(simTime);
					
					std::cout << "Forcibly terminated packet " << i << std::endl;
					
					// Update global counters - with underflow protection
					if (gTotalActivePackets.load() > 0) {
						gTotalActivePackets--;
					}
					gTotalTerminatedPackets++;
				}
			}
		} */
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