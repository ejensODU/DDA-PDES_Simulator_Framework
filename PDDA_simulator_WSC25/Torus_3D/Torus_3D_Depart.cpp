#include "Torus_3D_Depart.h"
#include "Torus_3D_Arrive.h"
#include "../Dist.h"
#include "../PDDA_SimExec.h"
#include <algorithm>
#include <cmath>

Torus_3D_Depart::Torus_3D_Depart(size_t networkNodeID,
                                size_t x, size_t y, size_t z,
                                std::vector<bool> neighbors,
                                size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
                                size_t hopRadius, const std::string& vertexName,
                                size_t distSeed, std::string traceFolderName,
                                std::string traceFileHeading,
                                double minServiceTime, double modeServiceTime, double maxServiceTime,
                                double minTransitTime, double modeTransitTime, double maxTransitTime,
                                std::vector<Torus_3D_Packet>& packets,
                                PDDA_SV<int>& packetQueueSV,
                                std::queue<int>& packetQueue, std::vector<int>& queueSizes, bool slowNode)
    : Vertex(vertexName, 0, distSeed, traceFolderName, traceFileHeading),
      _networkNodeID(networkNodeID),
      _x(x), _y(y), _z(z), _hopRadius(hopRadius),
      _neighbors(neighbors),
      _gridSizeX(gridSizeX), _gridSizeY(gridSizeY), _gridSizeZ(gridSizeZ),
      _packets(packets), _packetQueueSV(packetQueueSV), _packetQueue(packetQueue),
      _packetInQueue(false), _queueSizes(queueSizes),
      _totalNodes(gridSizeX * gridSizeY * gridSizeZ),
      _neighborInfo(networkNodeID, x, y, z, gridSizeX, gridSizeY, gridSizeZ, hopRadius, queueSizes) {

    // Initialize global queue size knowledge storage
    _globalQueueSizes.resize(_totalNodes, -1);          // Queue sizes for all nodes
    _queueSizeTimestamps.resize(_totalNodes, 0.0);     // When each queue size was last updated
    _knownQueueSizes.resize(_totalNodes, false);       // Whether we have data for a node
    
    // Mark nodes within hop radius as known
    for (size_t i = 0; i < _neighborInfo.allNeighborCount; i++) {
        size_t neighborId = _neighborInfo.allNeighbors[i];
        _knownQueueSizes[neighborId] = true;
        
        // Initialize with current queue sizes of neighbors
        _globalQueueSizes[neighborId] = _queueSizes[neighborId];
        _queueSizeTimestamps[neighborId] = 0.0; // Will be updated on first run
    }
    
    // Initialize own node's data
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

void Torus_3D_Depart::AddNeighborInfo(const Torus_3D_NeighborInfo& info) {
    // Can't use assignment operator with reference members, so copy needed fields individually
    _neighborInfo.Indices = info.Indices;
    _neighborInfo.Neighbors = info.Neighbors;
    _neighborInfo.globalToLocalIdx = info.globalToLocalIdx;
    
    // Copy the arrays manually
    for (size_t i = 0; i <= T3D_MAX_HOP_RADIUS; i++) {
        _neighborInfo.hopLevelCounts[i] = info.hopLevelCounts[i];
        for (size_t j = 0; j < info.hopLevelCounts[i] && j < T3D_MAX_NEIGHBORS_PER_LEVEL; j++) {
            _neighborInfo.hopLevelNodes[i][j] = info.hopLevelNodes[i][j];
        }
    }
    
    for (size_t i = 0; i < info.allNeighborCount && i < T3D_MAX_TOTAL_NEIGHBORS; i++) {
        _neighborInfo.allNeighbors[i] = info.allNeighbors[i];
    }
    
    _neighborInfo.allNeighborCount = info.allNeighborCount;
    
    // Copy direct neighbor indices
    for (int i = 0; i < 6; i++) {
        _neighborInfo.directNeighborIndices[i] = info.directNeighborIndices[i];
    }
}

void Torus_3D_Depart::AddArriveVertices(std::vector<std::shared_ptr<Torus_3D_Arrive>> arriveVertices) {
    _arriveVertices = arriveVertices;
    
    // Update the arrival vertices in the neighbor info
    _neighborInfo.Neighbors = arriveVertices;
}

void Torus_3D_Depart::IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) {
    // Keep this method exactly as in the original implementation
    std::vector<size_t> input_SV_indices;
    input_SV_indices.push_back(_packetQueueSV.getModelIndex());

    for (const auto& neighbor : _neighborInfo.Neighbors) {
        if (neighbor) {
            input_SV_indices.push_back(neighbor->getPacketQueueSV().getModelIndex());
        }
    }

    Is.push_back(input_SV_indices);

    std::vector<size_t> output_SV_indices;
    output_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Os.push_back(output_SV_indices);
}

// Update global queue size knowledge with packet data
void Torus_3D_Depart::UpdateGlobalQueueSizes(const Torus_3D_Packet& packet) {
    // Get visited nodes and queue sizes from the packet
    size_t queue_history_index;
	size_t queue_history_cnt;
    const auto& recentQueueNodes = packet.getRecentQueueNodes(queue_history_index, queue_history_cnt);
    const auto& recentQueueSizes = packet.getRecentQueueSizes();
	const auto& recentQueueTimes = packet.getRecentQueueTimes();
	int circular_index;
	
	//printf("index: %lu, count: %lu\n", queue_history_index, queue_history_cnt);
    
    // Update global knowledge with packet data
    for (size_t i = 0; i < queue_history_cnt; i++) {
		circular_index = (queue_history_index + i) & QUEUE_HISTORY_MASK;
        size_t nodeID = recentQueueNodes[circular_index];
        int queueSize = recentQueueSizes[circular_index];
		double queueTime = recentQueueTimes[circular_index];
        
        // Only update if the information is newer than what we have
        if (nodeID < _totalNodes && (!_knownQueueSizes[nodeID] || queueTime > _queueSizeTimestamps[nodeID])) {
            _globalQueueSizes[nodeID] = queueSize;
            _queueSizeTimestamps[nodeID] = queueTime;
            _knownQueueSizes[nodeID] = true;
        }
    }
}

// Update local nodes' queue sizes from direct neighbors (within hop radius)
void Torus_3D_Depart::UpdateLocalQueueSizes(double simTime) {
    // Update queue sizes for nodes within hop radius (direct knowledge)
    for (size_t i = 0; i < _neighborInfo.allNeighborCount; i++) {
        size_t neighborId = _neighborInfo.allNeighbors[i];
        _globalQueueSizes[neighborId] = _queueSizes[neighborId];
        _queueSizeTimestamps[neighborId] = simTime;
        _knownQueueSizes[neighborId] = true;
    }
    
    // Update own node's data
    _globalQueueSizes[_networkNodeID] = _queueSizes[_networkNodeID];
    _queueSizeTimestamps[_networkNodeID] = simTime;
}

void Torus_3D_Depart::PrintNeighborInfo() const {
    printf("\n=== Optimized Neighbor Info Debug ===\n");
    printf("Current node position: x=%lu, y=%lu, z=%lu\n", _x, _y, _z);
    printf("Grid size: %lu x %lu x %lu\n", _gridSizeX, _gridSizeY, _gridSizeZ);
    printf("Hop radius: %lu\n", _hopRadius);

    printf("\nHop level counts:\n");
    for (size_t i = 0; i <= _hopRadius && i < T3D_MAX_HOP_RADIUS; i++) {
        printf("Hop level %lu: %lu nodes\n", i, _neighborInfo.hopLevelCounts[i]);
    }

    printf("\nTotal neighbors: %lu\n", _neighborInfo.allNeighborCount);
    
    printf("\nDirect neighbor indices: ");
    for (int i = 0; i < 6; i++) {
        printf("%d ", _neighborInfo.directNeighborIndices[i]);
    }
    
    printf("\nKnown queue sizes: ");
    int knownCount = 0;
    for (size_t i = 0; i < _totalNodes; i++) {
        if (_knownQueueSizes[i]) knownCount++;
    }
    printf("%d / %lu nodes\n", knownCount, _totalNodes);
    
    printf("==============================\n\n");
}

void Torus_3D_Depart::Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) {
    int packet_ID = entityID;

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
    size_t dest_x = _packets.at(packet_ID).getDestX();
    size_t dest_y = _packets.at(packet_ID).getDestY();
    size_t dest_z = _packets.at(packet_ID).getDestZ();

    // Get path information
    //size_t visited_node_cnt;
    //const std::array<size_t, T3D_MAX_PATH_LENGTH>& visited_nodes = 
    //    _packets.at(packet_ID).getVisitedNetworkNodes(visited_node_cnt);
		
	//printf("Debug before FindShortestPath: _x=%lu, _y=%lu, _z=%lu\n", _x, _y, _z);
	//printf("Debug: _neighborInfo address = %p\n", (void*)&_neighborInfo);

    // 2. FIND COMPLETE PATH: Use the global knowledge to find the complete shortest path
    Torus_3D_Path complete_path = _neighborInfo.FindShortestPath(
        dest_network_node_ID,
        _x, _y, _z, _gridSizeX, _gridSizeY, _gridSizeZ,
        _hopRadius, _globalQueueSizes, _queueSizeTimestamps, simTime);
    
    // Store the complete path in the packet for potential future use
    // (Optional enhancement - would require adding this field to the packet class)
    // _packets.at(packet_ID).setPlannedPath(complete_path);
    
    // Determine next hop direction from the path
    int dest_dir = -1;
    
    // Log the complete path if tracing is enabled
    if (!_traceFolderName.empty() && complete_path.length > 1) {
        std::string path_string = "Path from " + std::to_string(_networkNodeID) + 
                                 " to " + std::to_string(dest_network_node_ID) + ": ";
        for (size_t i = 0; i < complete_path.length; i++) {
            path_string += std::to_string(complete_path.nodeIndices[i]) + " -> ";
        }
        path_string += " (cost: " + std::to_string(complete_path.totalCost) + ")";
        // Could write this to a separate trace file
    }
    
    if (complete_path.length > 1) {
        // Get the first hop in the path (index 1, since index 0 is the current node)
        size_t next_node_id = complete_path.nodeIndices[1];
        
        // Calculate next node coordinates
        auto [next_x, next_y, next_z] = _neighborInfo.GetCoordinates(next_node_id, _gridSizeX, _gridSizeY);
        
        // Calculate wrapped differences
        int dx = next_x - _x;
        if (std::abs(dx) > static_cast<int>(_gridSizeX/2)) {
            dx = (dx > 0) ? dx - _gridSizeX : dx + _gridSizeX;
        }

        int dy = next_y - _y;
        if (std::abs(dy) > static_cast<int>(_gridSizeY/2)) {
            dy = (dy > 0) ? dy - _gridSizeY : dy + _gridSizeY;
        }

        int dz = next_z - _z;
        if (std::abs(dz) > static_cast<int>(_gridSizeZ/2)) {
            dz = (dz > 0) ? dz - _gridSizeZ : dz + _gridSizeZ;
        }

        // Determine direction based on coordinate differences
        if (dx < 0) dest_dir = 0;      // West
        else if (dx > 0) dest_dir = 1;  // East
        else if (dy < 0) dest_dir = 2;  // North
        else if (dy > 0) dest_dir = 3;  // South
        else if (dz < 0) dest_dir = 4;  // Down
        else if (dz > 0) dest_dir = 5;  // Up
    } else {
        // Fallback - simple greedy direction if no path found
        dest_dir = 0; // Default direction
        
        // Calculate wrapped distances in each dimension
        size_t dx = GetWrappedDistance(_x, dest_x, _gridSizeX);
        size_t dy = GetWrappedDistance(_y, dest_y, _gridSizeY);
        size_t dz = GetWrappedDistance(_z, dest_z, _gridSizeZ);
        
        // Find which dimension has the biggest distance
        if (dx >= dy && dx >= dz) {
            // X dimension has biggest distance - go east or west
            if ((_x < dest_x && dest_x - _x <= _gridSizeX/2) || 
                (_x > dest_x && _x - dest_x > _gridSizeX/2)) {
                // Need to go east
                dest_dir = 1;
            } else {
                // Need to go west
                dest_dir = 0;
            }
        } else if (dy >= dx && dy >= dz) {
            // Y dimension has biggest distance - go north or south
            if ((_y < dest_y && dest_y - _y <= _gridSizeY/2) || 
                (_y > dest_y && _y - dest_y > _gridSizeY/2)) {
                // Need to go south
                dest_dir = 3;
            } else {
                // Need to go north
                dest_dir = 2;
            }
        } else {
            // Z dimension has biggest distance - go up or down
            if ((_z < dest_z && dest_z - _z <= _gridSizeZ/2) || 
                (_z > dest_z && _z - dest_z > _gridSizeZ/2)) {
                // Need to go up
                dest_dir = 5;
            } else {
                // Need to go down
                dest_dir = 4;
            }
        }
    }

    // Schedule New Events
    if (_packetInQueue) {
        // Schedule processing for the next packet in queue
        double service_delay = _serviceDelay->GenRV();
        CQ.AddEvent(_vertexID, simTime + service_delay, queue_packet_ID);
    }

    // 3. UPDATE PACKET: Add current queue data to the packet before sending
    _packets.at(packet_ID).AddNetworkNodeData(simTime, _networkNodeID, _packetQueueSV.get());
    
    // 4. FORWARD PACKET: Schedule arrival at next node
	// Ensure dest_dir is valid before accessing _arriveVertices
	if (dest_dir < 0 || dest_dir >= static_cast<int>(_arriveVertices.size())) {
		printf("Warning: Invalid dest_dir %d, using fallback direction\n", dest_dir);
		// Safety fallback - use west (0) direction if invalid
		dest_dir = 0;
	}
    double transit_delay = _transitDelay->GenRV();
    CQ.AddEvent(_arriveVertices.at(dest_dir)->getVertexID(), simTime + transit_delay, packet_ID);
    
    // Trace and increment execution count
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