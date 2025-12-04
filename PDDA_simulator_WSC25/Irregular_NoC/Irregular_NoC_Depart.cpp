#include "Irregular_NoC_Depart.h"
#include "Irregular_NoC_Arrive.h"
#include "../Dist.h"
#include "../PDDA_SimExec.h"
#include <algorithm>
#include <cmath>
#include <iostream>

Irregular_NoC_Depart::Irregular_NoC_Depart(
    size_t networkNodeID,
    const std::vector<std::vector<NetworkLink>>& adjacencyList,
    size_t hopRadius, const std::string& vertexName,
    size_t distSeed, std::string traceFolderName,
    std::string traceFileHeading,
    double minServiceTime, double modeServiceTime, double maxServiceTime,
    double minTransitTime, double modeTransitTime, double maxTransitTime,
    std::vector<Irregular_NoC_Packet>& packets,
    PDDA_SV<int>& packetQueueSV,
    std::queue<int>& packetQueue, std::vector<int>& queueSizes, bool slowNode)
    : Vertex(vertexName, 0, distSeed, traceFolderName, traceFileHeading),
      // Initialize _neighborInfo with required parameters
      _neighborInfo(networkNodeID, adjacencyList, hopRadius, queueSizes),
      _networkNodeID(networkNodeID),
      _hopRadius(hopRadius),
      _adjacencyList(adjacencyList),
      _packets(packets), 
      _packetQueueSV(packetQueueSV), 
      _packetQueue(packetQueue),
      _packetInQueue(false), 
      _queueSizes(queueSizes),
      _numNodes(adjacencyList.size()) {

    // Initialize global queue size knowledge storage
    _globalQueueSizes.resize(_numNodes, -1);          // Queue sizes for all nodes
    _queueSizeTimestamps.resize(_numNodes, 0.0);     // When each queue size was last updated
    _knownQueueSizes.resize(_numNodes, false);       // Whether we have data for a node
    
    // Mark own node's data as known
    _knownQueueSizes[_networkNodeID] = true;
    _globalQueueSizes[_networkNodeID] = _queueSizes[_networkNodeID];
    
    // Initialize service and transit delay generators
    if (slowNode) {
        minServiceTime *= 10;
        modeServiceTime *= 10;
        maxServiceTime *= 10;
    }

    _serviceDelay = std::make_unique<TriangularDist>(minServiceTime, modeServiceTime, maxServiceTime, distSeed + networkNodeID);
    _transitDelay = std::make_unique<TriangularDist>(minTransitTime, modeTransitTime, maxTransitTime, distSeed + networkNodeID);
}

void Irregular_NoC_Depart::PrintNeighborInfo() const {
    printf("\n=== Irregular NoC Neighbor Info ===\n");
    printf("Current node: %zu\n", _networkNodeID);
    printf("Hop radius: %zu\n", _hopRadius);

    printf("\nHop level counts:\n");
    for (size_t i = 0; i <= _hopRadius && i < NOC_MAX_HOP_RADIUS; i++) {
        printf("Hop level %zu: %zu nodes\n", i, _neighborInfo.hopLevelCounts[i]);
    }

    printf("\nTotal neighbors: %zu\n", _neighborInfo.allNeighborCount);
    
    printf("\nNeighbors by ID: ");
    for (size_t i = 0; i < _neighborInfo.allNeighborCount && i < 20; i++) { // Limit to 20 for brevity
        printf("%zu ", _neighborInfo.allNeighbors[i]);
    }
    if (_neighborInfo.allNeighborCount > 20) printf("...");
    
    printf("\nKnown queue sizes: ");
    int knownCount = 0;
    for (size_t i = 0; i < _numNodes; i++) {
        if (_knownQueueSizes[i]) knownCount++;
    }
    printf("%d / %zu nodes\n", knownCount, _numNodes);
    
    printf("==============================\n\n");
}

// Update global queue size knowledge with packet data
void Irregular_NoC_Depart::UpdateGlobalQueueSizes(const Irregular_NoC_Packet& packet) {
    // Get visited nodes and queue sizes from the packet
    size_t queue_history_index;
	size_t queue_history_cnt;
    const auto& recentQueueNodes = packet.getRecentQueueNodes(queue_history_index, queue_history_cnt);
    const auto& recentQueueSizes = packet.getRecentQueueSizes();
	const auto& recentQueueTimes = packet.getRecentQueueTimes();
	int circular_index;
    
    // Update global knowledge with packet data
    for (size_t i = 0; i < queue_history_cnt; i++) {
		circular_index = (queue_history_index + i) & QUEUE_HISTORY_MASK;
        size_t nodeID = recentQueueNodes[circular_index];
        int queueSize = recentQueueSizes[circular_index];
		double queueTime = recentQueueTimes[circular_index];
        
        // Only update if the information is newer than what we have
        if (nodeID < _numNodes && (!_knownQueueSizes[nodeID] || queueTime > _queueSizeTimestamps[nodeID])) {
            _globalQueueSizes[nodeID] = queueSize;
            _queueSizeTimestamps[nodeID] = queueTime;
            _knownQueueSizes[nodeID] = true;
        }
    }
}

// Update local nodes' queue sizes from direct neighbors (within hop radius)
void Irregular_NoC_Depart::UpdateLocalQueueSizes(double simTime) {
    // Update queue sizes for nodes within hop radius (direct knowledge)
    for (size_t i = 0; i < _neighborInfo.allNeighborCount; i++) {
        size_t neighborId = _neighborInfo.allNeighbors[i];
        if (neighborId < _queueSizes.size()) {
            _globalQueueSizes[neighborId] = _queueSizes[neighborId];
            _queueSizeTimestamps[neighborId] = simTime;
            _knownQueueSizes[neighborId] = true;
        }
    }
    
    // Update own node's data
    _globalQueueSizes[_networkNodeID] = _queueSizes[_networkNodeID];
    _queueSizeTimestamps[_networkNodeID] = simTime;
}

void Irregular_NoC_Depart::AddNeighborInfo(const Irregular_NoC_NeighborInfo& info) {
    // Copy needed fields individually
    _neighborInfo.Indices = info.Indices;
    _neighborInfo.Neighbors = info.Neighbors;
    _neighborInfo.globalToLocalIdx = info.globalToLocalIdx;
    
    // Copy the arrays manually
    for (size_t i = 0; i <= NOC_MAX_HOP_RADIUS; i++) {
        _neighborInfo.hopLevelCounts[i] = info.hopLevelCounts[i];
        for (size_t j = 0; j < info.hopLevelCounts[i] && j < NOC_MAX_NEIGHBORS_PER_LEVEL; j++) {
            _neighborInfo.hopLevelNodes[i][j] = info.hopLevelNodes[i][j];
        }
    }
    
    for (size_t i = 0; i < info.allNeighborCount && i < NOC_MAX_TOTAL_NEIGHBORS; i++) {
        _neighborInfo.allNeighbors[i] = info.allNeighbors[i];
    }
    
    _neighborInfo.allNeighborCount = info.allNeighborCount;
}

void Irregular_NoC_Depart::AddArriveVertices(const std::unordered_map<size_t, std::shared_ptr<Irregular_NoC_Arrive>>& arriveVertexMap) {
    // Store the map directly for efficient lookups
    _nodeIdToArriveVertex = arriveVertexMap;
    
    // Populate Neighbors vector using the arriveVertexMap and allNeighbors array
    _neighborInfo.Neighbors.clear();
    for (size_t i = 0; i < _neighborInfo.allNeighborCount && i < NOC_MAX_TOTAL_NEIGHBORS; i++) {
        size_t neighborId = _neighborInfo.allNeighbors[i];
        auto it = arriveVertexMap.find(neighborId);
        if (it != arriveVertexMap.end()) {
            _neighborInfo.Neighbors.push_back(it->second);
        }
    }
}

void Irregular_NoC_Depart::IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) {
    // Input SVs
    std::vector<size_t> input_SV_indices;
    input_SV_indices.push_back(_packetQueueSV.getModelIndex());

    // Add all neighbor queue SVs to inputs
    for (const auto& neighbor : _neighborInfo.Neighbors) {
        if (neighbor) {
            input_SV_indices.push_back(neighbor->getPacketQueueSV().getModelIndex());
			//printf("\tdepart (%lu) input neighbor node ID: %lu, SV index: %lu\n", _networkNodeID, neighbor->getNetworkNodeID(), neighbor->getPacketQueueSV().getModelIndex());
        }
    }
    Is.push_back(input_SV_indices);

    // Output SVs
    std::vector<size_t> output_SV_indices;
    output_SV_indices.push_back(_packetQueueSV.getModelIndex());
	//printf("\tdepart (%lu) output SV index: %lu\n", _networkNodeID, _packetQueueSV.getModelIndex());
    Os.push_back(output_SV_indices);
}

void Irregular_NoC_Depart::Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) {
    int packet_ID = entityID;
	
	//if (69 == packet_ID) printf("packet 69 depart: %lf, node ID: %lu\n", simTime, _networkNodeID);

    // Evaluate Conditions
    _packetInQueue = (_packetQueueSV.get() > 0);

    // Update SVs
    int queue_packet_ID = -1;
    _packetQueueSV.dec();
    
    // Update local queue size data
    _queueSizes[_networkNodeID] = _packetQueueSV.get();
    _globalQueueSizes[_networkNodeID] = _packetQueueSV.get();
    _queueSizeTimestamps[_networkNodeID] = simTime;
    
    if (_packetInQueue) {
        queue_packet_ID = _packetQueue.front();
        _packetQueue.pop();
    }

    // 1. UPDATE LOCAL DATA: Refresh knowledge of nearby nodes before routing
    UpdateLocalQueueSizes(simTime);

    // Get destination information
    size_t dest_network_node_ID = _packets.at(packet_ID).getDestNetworkNodeID();
    
    // Get path information
    size_t visited_node_cnt;
    const std::array<size_t, NOC_MAX_PATH_LENGTH>& visited_nodes = 
        _packets.at(packet_ID).getVisitedNetworkNodes(visited_node_cnt);

    // 2. FIND SHORTEST PATH: Use global knowledge to find the complete path
    // This is the key algorithm that has been modified for irregular networks
    Irregular_NoC_Path complete_path = _neighborInfo.FindShortestPath(
        _networkNodeID,
        dest_network_node_ID,
        _adjacencyList,
        _hopRadius, 
        visited_nodes, 
        visited_node_cnt,
        _globalQueueSizes, 
        _queueSizeTimestamps, 
        simTime);
    
    // Determine next hop from the path
    size_t next_node_id = _networkNodeID; // Default to self (should never happen)
    
    // Log the complete path if tracing is enabled
    if (!_traceFolderName.empty() && complete_path.length > 1) {
        std::string path_string = "Path from " + std::to_string(_networkNodeID) + 
                                 " to " + std::to_string(dest_network_node_ID) + ": ";
        for (size_t i = 0; i < complete_path.length; i++) {
            path_string += std::to_string(complete_path.nodeIndices[i]);
            if (i < complete_path.length - 1) path_string += " -> ";
        }
        path_string += " (cost: " + std::to_string(complete_path.totalCost) + ")";
        // Could write this to a separate trace file
    }
    
    // Extract next hop node from path (if exists)
    if (complete_path.length > 1) {
        next_node_id = complete_path.nodeIndices[1];
    }
    
    // VALIDATION: Ensure next node ID is valid
    if (next_node_id >= _adjacencyList.size()) {
        std::cerr << "ERROR: Invalid next node ID " << next_node_id 
                  << " (out of range). Keeping packet at current node." << std::endl;
        next_node_id = _networkNodeID; // Stay at current node if next is invalid
    }
    
    // Identify link to target node and get its properties
    LinkType linkType = LinkType::MESH_LINK;
    double latencyFactor = 1.0;
    
    // Only look for link properties if we're actually moving
    if (next_node_id != _networkNodeID) {
        bool linkFound = false;
        for (const auto& link : _adjacencyList[_networkNodeID]) {
            if (link.targetNodeID == next_node_id) {
                linkType = link.type;
                latencyFactor = link.latencyFactor;
                linkFound = true;
                break;
            }
        }
        
        // VALIDATION: If link not found, stay at current node
        if (!linkFound) {
            std::cerr << "ERROR: No link found from node " << _networkNodeID 
                      << " to node " << next_node_id << ". Keeping packet at current node." << std::endl;
            next_node_id = _networkNodeID;
        }
    }

    // Schedule New Events
    if (_packetInQueue) {
        // Schedule processing for next packet in queue
        double service_delay = _serviceDelay->GenRV();
        CQ.AddEvent(_vertexID, simTime + service_delay, queue_packet_ID);
    }

    // 3. UPDATE PACKET: Add current queue data to the packet before sending
    _packets.at(packet_ID).AddNetworkNodeData(simTime, _networkNodeID, _packetQueueSV.get());
    
    // 4. FORWARD PACKET: Schedule arrival at next node with appropriate delay
    
    // If we're staying at the current node (due to error), treat it specially
    if (next_node_id == _networkNodeID) {
        // Re-enqueue the packet at the current node
        _packetQueueSV.inc();
        _packetQueue.push(packet_ID);
        
        // Add trace info if enabled
        if (!_traceFolderName.empty()) {
            std::string trace_string = std::to_string(simTime) + " REROUTE, " + std::to_string(_packetQueueSV.get());
            WriteToTrace(trace_string);
        }
        
        _numExecutions++;
        return;
    }
    
    // Normal case - forward to next node
    double transit_delay = _transitDelay->GenRV();
    
    // Adjust transit time based on link type
    if (linkType == LinkType::EXPRESS_LINK) {
        // Express links have faster transit time
        transit_delay *= latencyFactor;
		//printf("\texpress: ");
    }
    
    // Find the arrive vertex for the target node
    auto targetArriveVertexIt = _nodeIdToArriveVertex.find(next_node_id);
    if (targetArriveVertexIt != _nodeIdToArriveVertex.end()) {
        CQ.AddEvent(targetArriveVertexIt->second->getVertexID(), 
                   simTime + transit_delay, packet_ID);
		
		//if (69 == packet_ID) printf("packet 69 depart: %lf, node ID: %lu, arrive vertex ID: %lu, transit delay: %lf\n", simTime, _networkNodeID, targetArriveVertexIt->second->getVertexID(), transit_delay);
		
		//printf("transit delay: %lf\n", transit_delay);
    } else {
        // If we can't find the target node in our map, this is an error
        // IMPROVED ERROR HANDLING: Reroute the packet back to this node instead of printing error
        std::cerr << "ERROR: No arrive vertex found for node " << next_node_id << std::endl;
        
        // Re-enqueue the packet at the current node
        _packetQueueSV.inc();
        _packetQueue.push(packet_ID);
    }
    
    // Trace
    if (!_traceFolderName.empty()) {
        std::string trace_string = std::to_string(simTime) + ", ";
        for (const auto& neighbor : _neighborInfo.Neighbors) {
            if (neighbor) {
                trace_string.append(std::to_string(neighbor->getPacketQueueSV().get()) + ", ");
            }
        }
        trace_string.append(std::to_string(_packetQueueSV.get()));
        WriteToTrace(trace_string);
    }

    _numExecutions++;
}