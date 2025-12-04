#include "Grid_VN3D_Depart.h"
#include "Grid_VN3D_Arrive.h"
#include "../Dist.h"
#include "../PDDA_SimExec.h"
#include <algorithm>

Grid_VN3D_Depart::Grid_VN3D_Depart(size_t networkNodeID,
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
								std::queue<int>& packetQueue)
    : Vertex(vertexName, 0, distSeed, traceFolderName, traceFileHeading),
      _networkNodeID(networkNodeID),
      _x(x), _y(y), _z(z), _hopRadius(hopRadius),
      _neighbors(neighbors),
      _gridSizeX(gridSizeX), _gridSizeY(gridSizeY), _gridSizeZ(gridSizeZ),
      _packets(packets), _packetQueueSV(packetQueueSV), _packetQueue(packetQueue) {

    _serviceDelay = std::make_unique<TriangularDist>(minServiceTime, modeServiceTime, maxServiceTime, distSeed + networkNodeID);
    _transitDelay = std::make_unique<TriangularDist>(minTransitTime, modeTransitTime, maxTransitTime, distSeed + networkNodeID);
}

void Grid_VN3D_Depart::AddNeighborInfo(const Grid_VN3D_NeighborInfo& info) {
    _neighborInfo = info;
}

void Grid_VN3D_Depart::AddArriveVertices(std::vector<std::shared_ptr<Grid_VN3D_Arrive>> arriveVertices) {
    _arriveVertices = arriveVertices;
}

void Grid_VN3D_Depart::IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) {
    std::vector<size_t> input_SV_indices;
    input_SV_indices.push_back(_packetQueueSV.getModelIndex());

    for (const auto& neighbor : _neighborInfo.N) {
        if (neighbor) {
            input_SV_indices.push_back(neighbor->getPacketQueueSV().getModelIndex());
        }
    }

    Is.push_back(input_SV_indices);

    std::vector<size_t> output_SV_indices;
    output_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Os.push_back(output_SV_indices);
}

void Grid_VN3D_Depart::PrintNeighborInfo() const {
    printf("\n=== Neighbor Info Debug Output ===\n");
    printf("Current node position: x=%lu, y=%lu, z=%lu\n", _x, _y, _z);
    printf("Grid size: %lu x %lu x %lu\n", _gridSizeX, _gridSizeY, _gridSizeZ);
    printf("Hop radius: %lu\n", _hopRadius);

    printf("\nI structure (size: %lu):\n", _neighborInfo.I.size());
    for (size_t i = 0; i < _neighborInfo.I.size(); i++) {
        printf("Hop level %lu (size: %lu): ", i, _neighborInfo.I[i].size());
        for (const auto& idx : _neighborInfo.I[i]) {
            printf("%lu ", idx);
        }
        printf("\n");
    }

    printf("\nN structure (size: %lu):\n", _neighborInfo.N.size());
    for (size_t i = 0; i < _neighborInfo.N.size(); i++) {
        if (_neighborInfo.N[i]) {
            printf("Index %lu: Node ID %lu (Queue size: %d)\n",
                   i, _neighborInfo.N[i]->getNetworkNodeID(),
                   _neighborInfo.N[i]->getPacketQueueSV().get());
        } else {
            printf("Index %lu: nullptr\n", i);
        }
    }
    printf("==============================\n\n");
}

void Grid_VN3D_Depart::Run(PDDA_CalendarQueue& CQ, double simTime, int entityID)
{
    int packet_ID = entityID;

    // Evaluate Conditions
    _packetInQueue = (_packetQueueSV.get() > 0);

    // Update SVs
    int queue_packet_ID = -1;
    _packetQueueSV.dec();
    if (_packetInQueue) {
        queue_packet_ID = _packetQueue.front();
        _packetQueue.pop();
    }

    // Determine routing
    size_t dest_network_node_ID = _packets.at(packet_ID).getDestNetworkNodeID();
    size_t dest_x = dest_network_node_ID % _gridSizeX;
    size_t dest_y = (dest_network_node_ID / _gridSizeX) % _gridSizeY;
    size_t dest_z = dest_network_node_ID / (_gridSizeX * _gridSizeY);

    size_t current_network_node_ID = _z * _gridSizeX * _gridSizeY + _y * _gridSizeX + _x;

    size_t dest_x_diff = abs(static_cast<int>(dest_x) - static_cast<int>(_x));
    size_t dest_y_diff = abs(static_cast<int>(dest_y) - static_cast<int>(_y));
    size_t dest_z_diff = abs(static_cast<int>(dest_z) - static_cast<int>(_z));
    size_t current_distance = dest_x_diff + dest_y_diff + dest_z_diff;

    auto visited_nodes = _packets.at(packet_ID).getVisitedNetworkNodes();
	size_t visited_node_cnt = _packets.at(packet_ID).getVisitedNodeCnt();

    // Find best path
    Path3D best_path;

    // If destination is within hop radius
    if (current_distance <= _hopRadius) {
        best_path = _neighborInfo.findShortestPath(dest_network_node_ID, dest_network_node_ID,
                                                 _x, _y, _z,
                                                 _gridSizeX, _gridSizeY, _gridSizeZ,
                                                 _hopRadius, visited_nodes, visited_node_cnt);
    }
    // Destination is beyond hop radius
    else {
        const auto& max_hop_neighbors = _neighborInfo.I.back();
        size_t min_path_manhattan_distance = std::numeric_limits<size_t>::max();
        std::vector<Path3D> max_hop_shortest_paths;

        // Find max-hop neighbors that reduce distance to destination
        for (size_t neighbor_id : max_hop_neighbors) {
            if (std::find(visited_nodes.begin(), visited_nodes.end(), neighbor_id) != visited_nodes.end()) {
                continue;
            }

            auto [neighbor_x, neighbor_y, neighbor_z] = _neighborInfo.getCoordinates(neighbor_id, _gridSizeX, _gridSizeY);

            size_t neighbor_distance =
                abs(static_cast<int>(dest_x) - static_cast<int>(neighbor_x)) +
                abs(static_cast<int>(dest_y) - static_cast<int>(neighbor_y)) +
                abs(static_cast<int>(dest_z) - static_cast<int>(neighbor_z));

            if (neighbor_distance < current_distance) {
                Path3D path = _neighborInfo.findShortestPath(dest_network_node_ID, neighbor_id,
                                                         _x, _y, _z,
                                                         _gridSizeX, _gridSizeY, _gridSizeZ,
                                                         _hopRadius, visited_nodes, visited_node_cnt);
                if (!path.nodeIndices.empty()) {
                    max_hop_shortest_paths.push_back(path);
                }
            }
        }

        if (max_hop_shortest_paths.empty()) {
            printf("Error: No valid paths to max-hop neighbors found!\n");
            _packets.at(packet_ID).PrintData();
            printf("Current: (%lu, %lu, %lu) %lu, Destination: (%lu, %lu, %lu) %lu\n",
                   _x, _y, _z, _networkNodeID, dest_x, dest_y, dest_z, dest_network_node_ID);
            exit(1);
        }

        // Sort paths by total cost
        std::sort(max_hop_shortest_paths.begin(), max_hop_shortest_paths.end(),
                 [](const Path3D& a, const Path3D& b) {
                     return a.totalCost < b.totalCost;
                 });

        // Keep only the cheaper half of paths
        size_t num_paths_to_keep = (max_hop_shortest_paths.size() + 1) / 2;
        max_hop_shortest_paths.resize(num_paths_to_keep);

        // Among remaining paths, find the one ending closest to destination
        min_path_manhattan_distance = std::numeric_limits<size_t>::max();

        for (const Path3D& path : max_hop_shortest_paths) {
            size_t max_hop_node = path.nodeIndices.back();
            auto [max_hop_x, max_hop_y, max_hop_z] = _neighborInfo.getCoordinates(max_hop_node, _gridSizeX, _gridSizeY);

            size_t distance =
                abs(static_cast<int>(dest_x) - static_cast<int>(max_hop_x)) +
                abs(static_cast<int>(dest_y) - static_cast<int>(max_hop_y)) +
                abs(static_cast<int>(dest_z) - static_cast<int>(max_hop_z));

            if (distance < min_path_manhattan_distance) {
                min_path_manhattan_distance = distance;
                best_path = path;
            }
        }
    }

    // Determine direction from best path
    int dest_dir = -1;
    if (!best_path.nodeIndices.empty() && best_path.nodeIndices.size() > 1) {
        size_t next_node_id = best_path.nodeIndices[1];  // First hop
        const auto& one_hop_neighbors = _neighborInfo.I[0];

        if (std::find(one_hop_neighbors.begin(), one_hop_neighbors.end(), next_node_id) == one_hop_neighbors.end()) {
            printf("ERROR: Next hop %lu is not a valid 1-hop neighbor!\n", next_node_id);
            exit(1);
        }

        // Convert to direction (now with 6 possible directions in 3D)
        if (next_node_id == current_network_node_ID - 1) dest_dir = 0;                    // West
        else if (next_node_id == current_network_node_ID + 1) dest_dir = 1;               // East
        else if (next_node_id == current_network_node_ID - _gridSizeX) dest_dir = 2;      // North
        else if (next_node_id == current_network_node_ID + _gridSizeX) dest_dir = 3;      // South
        else if (next_node_id == current_network_node_ID - _gridSizeX * _gridSizeY) dest_dir = 4;  // Down
        else if (next_node_id == current_network_node_ID + _gridSizeX * _gridSizeY) dest_dir = 5;  // Up
    }

    if (dest_dir == -1) {
        printf("Error: No valid direction found!\n");
        printf("1-hop neighbors in I[0]: ");
        for (size_t id : _neighborInfo.I[0]) printf("%lu ", id);
        printf("\n");
        exit(1);
    }

    // Schedule New Events
    if (_packetInQueue) {
        //newEvents.push_back(new PDDA_Event(shared_from_this(),
        //                                simTime + _serviceDelay->GenRV(),
        //                                queue_packet));
		
		CQ.AddEvent(_vertexID, simTime + _serviceDelay->GenRV(), queue_packet_ID);
    }

    //newEvents.push_back(new PDDA_Event(_arriveVertices.at(dest_dir),
    //                                 simTime + _transitDelay->GenRV(),
    //                                 packet));
	
	CQ.AddEvent(_arriveVertices.at(dest_dir)->getVertexID(), simTime + _transitDelay->GenRV(), packet_ID);

    // Trace
	if (!_traceFolderName.empty()) {
		std::string trace_string = std::to_string(simTime) + ", ";
		for (const auto& neighbor : _neighborInfo.N) {
			if (neighbor) {
				trace_string.append(std::to_string(neighbor->getPacketQueueSV().get()) + ", ");
			}
		}
		trace_string.append(std::to_string(_packetQueueSV.get()));
		WriteToTrace(trace_string);
	}

    _numExecutions++;
}
