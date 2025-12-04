#include "Grid_VN2D.h"
#include "../Dist.h"
#include <iostream>
#include <thread>
#include <filesystem>
#include <fstream>
#include <regex>

namespace fs = std::filesystem;

bool TryLockData(std::atomic<int>& shared_lock) {
    int expected = 0;
    return shared_lock.compare_exchange_strong(expected, -1);
}

void SpinLockData(std::atomic<int>& shared_lock) {
    int expected = 0;
    while (!shared_lock.compare_exchange_weak(expected, -1)) {
        expected = 0;
        std::this_thread::yield();
    }
}

void UnlockData(std::atomic<int>& shared_lock) {
    shared_lock.store(0);
}

Grid_VN2D::Grid_VN2D(size_t gridSizeX, size_t gridSizeY, size_t hopRadius,
                     size_t numServersPerNetworkNode, size_t maxNumArriveEvents,
                     double maxSimTime, size_t numThreads, size_t distSeed,
					 double bucketWidth, size_t targetBinSize,
                     std::string traceFolderName,
                     std::string distParamsFile)
    : PDDA_SimModel(maxSimTime, numThreads, distSeed, bucketWidth, targetBinSize, traceFolderName),
      _gridSizeX(gridSizeX), _gridSizeY(gridSizeY),
      _hopRadius(hopRadius), _numServersPerNetworkNode(numServersPerNetworkNode),
      _maxNumArriveEvents(maxNumArriveEvents) {

    // Read distribution parameters
    std::ifstream dist_params_file(distParamsFile);
    dist_params_file >> _minIntraArrivalTime >> _modeIntraArrivalTime >> _maxIntraArrivalTime;
    std::cout << "Intra Arrival Times - Min: " << _minIntraArrivalTime
    << ", Mode: " << _modeIntraArrivalTime
    << ", Max: " << _maxIntraArrivalTime << std::endl;

    dist_params_file >> _minServiceTime >> _modeServiceTime >> _maxServiceTime;
    std::cout << "Service Times - Min: " << _minServiceTime
    << ", Mode: " << _modeServiceTime
    << ", Max: " << _maxServiceTime << std::endl;

    dist_params_file >> _minTransitTime >> _modeTransitTime >> _maxTransitTime;
    std::cout << "Transit Times - Min: " << _minTransitTime
    << ", Mode: " << _modeTransitTime
    << ", Max: " << _maxTransitTime << std::endl;

    // Create trace folder if needed
    if (!_traceFolderName.empty() && !fs::exists(_traceFolderName)) {
        if (fs::create_directory(_traceFolderName)) {
            std::cout << "Folder '" << _traceFolderName << "' created successfully.\n";
        } else {
            std::cerr << "Error: Failed to create folder '" << _traceFolderName << "'.\n";
        }
    }

    BuildModel();

    std::string params;
    size_t configPos = distParamsFile.rfind("params_");  // Use rfind instead of find
    if (configPos != std::string::npos) {
        size_t trialPos = distParamsFile.find("_trial", configPos);
        if (trialPos != std::string::npos) {
            params = distParamsFile.substr(configPos + 7, trialPos - (configPos + 7));
        }
    }
    std::string table_filename = "ITL_table_PDDA_VN2D_grid_network_size_" +
                                std::to_string(_gridSizeX) + "_" +
                                std::to_string(_gridSizeY) + "_hops_" +
                                std::to_string(_hopRadius) + "_params_" +
                                params + ".csv";
    Init_PDDA(table_filename);
	
	_packets.resize(_gridSizeX*_gridSizeY*_maxNumArriveEvents);
}

void Grid_VN2D::BuildModel() {
    CreateVertices();
    AddVertices();
    CreateEdges();
    IO_SVs();
    InitEvents();
}

void Grid_VN2D::CreateVertices() {
    // Initialize SVs
    int init_packet_queue_val = -1 * _numServersPerNetworkNode;
    for (size_t i = 0; i < _gridSizeX * _gridSizeY; i++) {
        std::string name = "Packet Queue " + std::to_string(i);
        _packetQueueSVs.emplace_back(name, init_packet_queue_val,
                                    init_packet_queue_val - 1,
                                    std::numeric_limits<int>::max());
    }
    _packetQueues.resize(_gridSizeX * _gridSizeY);

    // Initialize vertices
    size_t network_node_ID = 0;
    for (size_t y = 0; y < _gridSizeY; y++) {
        for (size_t x = 0; x < _gridSizeX; x++) {
            // arrive trace file header
            std::string trace_file_heading_arrive = "ts, Q" + std::to_string(network_node_ID);

            // Create arrive vertex
            _arriveVertices.push_back(std::make_shared<Grid_VN2D_Arrive>(
                network_node_ID, _gridSizeX, _gridSizeY,
                "Arrive_" + std::to_string(x) + "_" + std::to_string(y),
                _distSeed, _traceFolderName, trace_file_heading_arrive,
                _maxNumArriveEvents,
                _minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime,
                _minServiceTime, _modeServiceTime, _maxServiceTime,
                _packets, _packetQueueSVs.at(network_node_ID),
				_packetQueues.at(network_node_ID)));

            // for sim exec
			AppendVertex(_arriveVertices.back());

            // Set up neighbors
            std::vector<bool> neighbors(4, 0);

            // Calculate how many nodes are at each hop distance
            std::vector<std::vector<size_t>> hop_neighbors;
            hop_neighbors.resize(_hopRadius + 1);

            // For each hop distance
            for (size_t h = 1; h <= _hopRadius; h++) {
                // For each possible combination of x, y movements that sum to h
                for (size_t x_dist = 0; x_dist <= h; x_dist++) {
                    size_t y_dist = h - x_dist;

                    // For each direction combination
                    for (int x_dir = -1; x_dir <= 1; x_dir += 2) {
                        if (x_dist == 0 && x_dir > 0) continue;
                        for (int y_dir = -1; y_dir <= 1; y_dir += 2) {
                            if (y_dist == 0 && y_dir > 0) continue;

                            // Calculate coordinates and check bounds
                            int x_coord = x + x_dir * x_dist;
                            int y_coord = y + y_dir * y_dist;

                            // Skip if outside grid bounds
                            if (x_coord < 0 || x_coord >= _gridSizeX ||
                                y_coord < 0 || y_coord >= _gridSizeY) continue;

                            // Convert to network node ID
                            size_t neighbor_id = y_coord * _gridSizeX + x_coord;
                            hop_neighbors[h].push_back(neighbor_id);
                        }
                    }
                }
            }

            // Build trace file header from hop neighbors
            std::string trace_file_heading_depart = "ts, ";
            for (size_t h = 1; h <= _hopRadius; h++) {
                for (size_t neighbor_id : hop_neighbors[h]) {
                    trace_file_heading_depart.append("Q" + std::to_string(neighbor_id) + ", ");
                }
            }
            trace_file_heading_depart.append("Q" + std::to_string(network_node_ID));

            // Create depart vertex
            _departVertices.push_back(std::make_shared<Grid_VN2D_Depart>(
                network_node_ID, x, y, neighbors,
                _gridSizeX, _gridSizeY, _hopRadius,
                "Depart_" + std::to_string(x) + "_" + std::to_string(y),
                _distSeed, _traceFolderName, trace_file_heading_depart,
                _minServiceTime, _modeServiceTime, _maxServiceTime,
                _minTransitTime, _modeTransitTime, _maxTransitTime,
                _packets, _packetQueueSVs.at(network_node_ID),
				_packetQueues.at(network_node_ID)));

            // for sim exec
			AppendVertex(_departVertices.back());

            network_node_ID++;
        }
    }

    setNumVertices(Vertex::getNumVertices());
}

Grid_VN2D_NeighborInfo Grid_VN2D::GetHopNeighborStructures(size_t x, size_t y) {
    Grid_VN2D_NeighborInfo info;
    size_t centralNodeID = y * _gridSizeX + x;
    size_t localIdx = 0;

    // Add central node to N
    info.globalToLocalIdx[centralNodeID] = localIdx++;
    info.N.push_back(_arriveVertices[centralNodeID]);

    // For each hop up to _hopRadius, collect nodes at that Manhattan distance
    for (size_t hop = 1; hop <= _hopRadius; hop++) {
        std::vector<size_t> hopNeighbors;

        // Check all nodes in grid
        for (size_t nodeID = 0; nodeID < _gridSizeX * _gridSizeY; nodeID++) {
            size_t node_x = nodeID % _gridSizeX;
            size_t node_y = nodeID / _gridSizeX;
            size_t distance = abs(static_cast<int>(node_x) - static_cast<int>(x)) +
                            abs(static_cast<int>(node_y) - static_cast<int>(y));

            if (distance == hop) {
                hopNeighbors.push_back(nodeID);
                if (info.globalToLocalIdx.find(nodeID) == info.globalToLocalIdx.end()) {
                    info.globalToLocalIdx[nodeID] = localIdx++;
                    info.N.push_back(_arriveVertices[nodeID]);
                }
            }
        }

        info.I.push_back(hopNeighbors);
    }

    return info;
}

void Grid_VN2D::AddVertices() {
    size_t network_node_ID = 0;
    for (size_t y = 0; y < _gridSizeY; y++) {
        for (size_t x = 0; x < _gridSizeX; x++) {
            // Connect arrive vertices to their corresponding depart vertices
            _arriveVertices[network_node_ID]->AddDepartVertex(_departVertices[network_node_ID]);

            // Set up immediate neighbor connections
            std::vector<std::shared_ptr<Grid_VN2D_Arrive>> arrive_vertices;
            if (x > 0)              arrive_vertices.push_back(_arriveVertices[network_node_ID - 1]); // West
            else                    arrive_vertices.push_back(nullptr);
            if (x < _gridSizeX-1)   arrive_vertices.push_back(_arriveVertices[network_node_ID + 1]); // East
            else                    arrive_vertices.push_back(nullptr);
            if (y > 0)              arrive_vertices.push_back(_arriveVertices[network_node_ID - _gridSizeX]); // North
            else                    arrive_vertices.push_back(nullptr);
            if (y < _gridSizeY-1)   arrive_vertices.push_back(_arriveVertices[network_node_ID + _gridSizeX]); // South
            else                    arrive_vertices.push_back(nullptr);

            _departVertices[network_node_ID]->AddArriveVertices(arrive_vertices);

            // Generate and add neighbor structures
            Grid_VN2D_NeighborInfo neighborInfo = GetHopNeighborStructures(x, y);
            _departVertices[network_node_ID]->AddNeighborInfo(neighborInfo);

            network_node_ID++;
        }
    }
}

void Grid_VN2D::CreateEdges() {
    _edges.resize(getNumVertices());

    for (size_t y = 0; y < _gridSizeY; y++) {
        for (size_t x = 0; x < _gridSizeX; x++) {
            size_t arriveIdx = GetVertexIndex(x, y, 0);
            size_t departIdx = GetVertexIndex(x, y, 1);

            // Arrive -> Depart
            _edges[arriveIdx].emplace_back(arriveIdx, departIdx, _minServiceTime);

            // Connect Depart to neighboring Arrive vertices
            if (x > 0)              _edges[departIdx].emplace_back(departIdx, GetVertexIndex(x-1, y, 0), _minTransitTime); // West
            if (x < _gridSizeX-1)   _edges[departIdx].emplace_back(departIdx, GetVertexIndex(x+1, y, 0), _minTransitTime); // East
            if (y > 0)              _edges[departIdx].emplace_back(departIdx, GetVertexIndex(x, y-1, 0), _minTransitTime); // North
            if (y < _gridSizeY-1)   _edges[departIdx].emplace_back(departIdx, GetVertexIndex(x, y+1, 0), _minTransitTime); // South
        }
    }
}

void Grid_VN2D::IO_SVs() {
    for (size_t i = 0; i < _gridSizeX * _gridSizeY; i++) {
        _arriveVertices[i]->IO_SVs(_Is, _Os);
        _departVertices[i]->IO_SVs(_Is, _Os);
    }
}

void Grid_VN2D::InitEvents() {
    TriangularDist init_arrive_delays(_minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime, _distSeed);

    for (size_t i = 0; i < _gridSizeX * _gridSizeY; i++) {
        AppendInitEvent(_arriveVertices[i]->getVertexID(),
                        init_arrive_delays.GenRV(),
                        -1);
    }
}

size_t Grid_VN2D::GetVertexIndex(size_t x, size_t y, size_t type) const {
    return (y * _gridSizeX + x) * 2 + type;
}

void Grid_VN2D::PrintMeanPacketNetworkTime() const {
    double total_network_time = 0;
    for (const auto& packet : _packets) {
        total_network_time += packet.GetTimeInNetwork();
    }
    printf("Mean packet network time: %lf\n", total_network_time / _packets.size());
}

void Grid_VN2D::PrintSVs() const {
    printf("\n");
    for (size_t y = 0; y < _gridSizeY; y++) {
        for (size_t x = 0; x < _gridSizeX; x++) {
            printf("%d ", _packetQueueSVs[y * _gridSizeX + x].get());
        }
        printf("\n");
    }
}

void Grid_VN2D::PrintNumVertexExecs() const {
    printf("\n");
    for (size_t y = 0; y < _gridSizeY; y++) {
        for (size_t x = 0; x < _gridSizeX; x++) {
            printf("(%lu %lu) ",
                   _arriveVertices[y * _gridSizeX + x]->getNumExecs(),
                   _departVertices[y * _gridSizeX + x]->getNumExecs());
        }
        printf("\n");
    }
}

void Grid_VN2D::PrintFinishedPackets() const {
    printf("\n");
    for (const auto& packet : _packets) {
        packet.PrintData();
    }
}
