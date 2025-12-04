#include "Torus_3D.h"
#include "../Dist.h"
#include <iostream>
#include <thread>
#include <filesystem>
#include <fstream>
#include <regex>
#include <float.h>

namespace fs = std::filesystem;

Torus_3D::Torus_3D(size_t gridSizeX, size_t gridSizeY, size_t gridSizeZ,
                   size_t hopRadius, size_t numServersPerNetworkNode,
                   size_t maxNumArriveEvents, double maxSimTime,
                   size_t numThreads, size_t distSeed,
                   double bucketWidth, size_t targetBinSize,
                   std::string traceFolderName,
                   std::string distParamsFile)
    : PDDA_SimModel(maxSimTime, numThreads, distSeed, bucketWidth, targetBinSize, traceFolderName),
      _gridSizeX(gridSizeX), _gridSizeY(gridSizeY), _gridSizeZ(gridSizeZ),
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
    std::string table_filename = "ITL_table_PDDA_Torus_3D_network_size_" +
                                std::to_string(_gridSizeX) + "_" +
                                std::to_string(_gridSizeY) + "_" +
                                std::to_string(_gridSizeZ) + "_hops_" +
                                std::to_string(_hopRadius) + "_params_" +
                                params + ".csv";
    Init_PDDA(table_filename);
	
	_packets.resize(_gridSizeX*_gridSizeY*_gridSizeZ*_maxNumArriveEvents);
}

size_t Torus_3D::WrapCoordinate(size_t coord, size_t size) const {
    return (coord + size) % size;
}

void Torus_3D::BuildModel() {
    CreateVertices();
    AddVertices();
    CreateEdges();
    IO_SVs();
    InitEvents();
}

void Torus_3D::CreateVertices() {
    // Initialize SVs
    size_t total_nodes = _gridSizeX * _gridSizeY * _gridSizeZ;
    int init_packet_queue_val = -1 * _numServersPerNetworkNode;

    for (size_t i = 0; i < total_nodes; i++) {
        std::string name = "Packet Queue " + std::to_string(i);
        _packetQueueSVs.emplace_back(name, init_packet_queue_val,
                                    init_packet_queue_val - 1,
                                    std::numeric_limits<int>::max());
    }
    _packetQueues.resize(total_nodes);
	_queueSizes.resize(total_nodes, init_packet_queue_val);

    // Create a CSV file to store node status
    std::ofstream nodeStatusFile("node_status.csv");
    nodeStatusFile << "node_id,x,y,z,is_slow_node" << std::endl;

    // Create vertices for each node in the 3D torus
    size_t network_node_ID = 0;
    for (size_t z = 0; z < _gridSizeZ; z++) {
        for (size_t y = 0; y < _gridSizeY; y++) {
            for (size_t x = 0; x < _gridSizeX; x++) {
				
				bool slow_node = false;
				int slow_node_RN = UniformIntDist(1, 6, network_node_ID).GenRV();
				if (1 == slow_node_RN) slow_node = true;
				
				//if (slow_node) printf("node %lu is a slow node (%d)\n", network_node_ID, slow_node_RN);
				
                // Write node status to CSV
                nodeStatusFile << network_node_ID << "," 
                              << x << "," 
                              << y << "," 
                              << z << "," 
                              << (slow_node ? 1 : 0) << std::endl;
                
                // arrive trace file header
                std::string trace_file_heading_arrive = "ts, Q" + std::to_string(network_node_ID);

                // Create arrive vertex
                _arriveVertices.push_back(std::make_shared<Torus_3D_Arrive>(
                    network_node_ID, _gridSizeX, _gridSizeY, _gridSizeZ,
                    "Arrive_" + std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(z),
                    _distSeed, _traceFolderName, trace_file_heading_arrive,
                    _maxNumArriveEvents,
                    _minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime,
                    _minServiceTime, _modeServiceTime, _maxServiceTime,
                    _packets, _packetQueueSVs.at(network_node_ID),
					_packetQueues.at(network_node_ID), _queueSizes, slow_node));

                // for sim exec
				AppendVertex(_arriveVertices.back());

                // In torus, all nodes have 6 neighbors (wrap around edges)
                std::vector<bool> neighbors(6, 1);

                // Calculate how many nodes are at each hop distance
                std::vector<std::vector<size_t>> hop_neighbors;  // Stores nodes at each hop distance
                hop_neighbors.resize(_hopRadius + 1);  // Include distance 0 to _hopRadius

                // For each hop distance
                for (size_t h = 1; h <= _hopRadius; h++) {
                    // For each possible combination of x, y, z movements that sum to h
                    for (size_t x_dist = 0; x_dist <= h; x_dist++) {
                        for (size_t y_dist = 0; y_dist <= h - x_dist; y_dist++) {
                            size_t z_dist = h - x_dist - y_dist;
                            if (z_dist > h) continue;  // Skip invalid combinations

                            // For each direction combination
                            for (int x_dir = -1; x_dir <= 1; x_dir += 2) {
                                if (x_dist == 0 && x_dir > 0) continue;
                                for (int y_dir = -1; y_dir <= 1; y_dir += 2) {
                                    if (y_dist == 0 && y_dir > 0) continue;
                                    for (int z_dir = -1; z_dir <= 1; z_dir += 2) {
                                        if (z_dist == 0 && z_dir > 0) continue;

                                        // Calculate wrapped coordinates
                                        size_t x_coord = (x + x_dir * x_dist + _gridSizeX) & (_gridSizeX - 1);
                                        size_t y_coord = (y + y_dir * y_dist + _gridSizeY) & (_gridSizeY - 1);
                                        size_t z_coord = (z + z_dir * z_dist + _gridSizeZ) & (_gridSizeZ - 1);

                                        // Convert to network node ID
                                        size_t neighbor_id = z_coord * _gridSizeX * _gridSizeY +
                                        y_coord * _gridSizeX + x_coord;
                                        hop_neighbors[h].push_back(neighbor_id);
                                    }
                                }
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
                _departVertices.push_back(std::make_shared<Torus_3D_Depart>(
                    network_node_ID, x, y, z, neighbors,
                    _gridSizeX, _gridSizeY, _gridSizeZ, _hopRadius,
                    "Depart_" + std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(z),
                    _distSeed, _traceFolderName, trace_file_heading_depart,
                    _minServiceTime, _modeServiceTime, _maxServiceTime,
                    _minTransitTime, _modeTransitTime, _maxTransitTime,
                    _packets, _packetQueueSVs.at(network_node_ID),
					_packetQueues.at(network_node_ID), _queueSizes, slow_node));

                // for sim exec
				AppendVertex(_departVertices.back());

                network_node_ID++;
            }
        }
    }

    // Close the node status file
    nodeStatusFile.close();

    setNumVertices(Vertex::getNumVertices());
}

Torus_3D_NeighborInfo Torus_3D::GetHopNeighborStructures(size_t x, size_t y, size_t z) {
    size_t centralNodeID = z * _gridSizeX * _gridSizeY + y * _gridSizeX + x;
    size_t localIdx = 0;
	
	//printf("GHNS, hop radius: %lu\n", _hopRadius);
	
	Torus_3D_NeighborInfo info(centralNodeID, x, y, z, _gridSizeX, _gridSizeY, _gridSizeZ, _hopRadius, _queueSizes);

    // Add central node to N
    info.globalToLocalIdx[centralNodeID] = localIdx++;
    info.Neighbors.push_back(_arriveVertices[centralNodeID]);

    // For each hop up to _hopRadius, collect nodes at that Manhattan distance
    for (size_t hop = 1; hop <= _hopRadius; hop++) {
        std::vector<size_t> hopNeighbors;

        // Check all nodes in the grid
        for (size_t nz = 0; nz < _gridSizeZ; nz++) {
            for (size_t ny = 0; ny < _gridSizeY; ny++) {
                for (size_t nx = 0; nx < _gridSizeX; nx++) {
                    // Calculate Manhattan distance with torus wrapping
                    size_t dx = std::min({
                        abs(static_cast<int>(nx) - static_cast<int>(x)),
                        abs(static_cast<int>(nx) - static_cast<int>(x + _gridSizeX)),
                        abs(static_cast<int>(nx + _gridSizeX) - static_cast<int>(x))
                    });
                    size_t dy = std::min({
                        abs(static_cast<int>(ny) - static_cast<int>(y)),
                        abs(static_cast<int>(ny) - static_cast<int>(y + _gridSizeY)),
                        abs(static_cast<int>(ny + _gridSizeY) - static_cast<int>(y))
                    });
                    size_t dz = std::min({
                        abs(static_cast<int>(nz) - static_cast<int>(z)),
                        abs(static_cast<int>(nz) - static_cast<int>(z + _gridSizeZ)),
                        abs(static_cast<int>(nz + _gridSizeZ) - static_cast<int>(z))
                    });

                    size_t distance = dx + dy + dz;

                    if (distance == hop) {
						
                        size_t nodeID = nz * _gridSizeX * _gridSizeY + ny * _gridSizeX + nx;
                        hopNeighbors.push_back(nodeID);
						
						//printf("distance: %lu, node ID: %lu, in GTL: %d\n", distance, nodeID, info.globalToLocalIdx.find(nodeID) == info.globalToLocalIdx.end());
						
						/* std::cout << "globalToLocalIdx contents:\n";
						for (const auto& pair : info.globalToLocalIdx) {
							std::cout << "Global ID: " << pair.first << " -> Local Index: " << pair.second << std::endl;
						} */
											
						//std::cout << "GTL: " << *info.globalToLocalIdx.find(nodeID) << std::endl;

                        if (info.globalToLocalIdx.find(nodeID) == info.globalToLocalIdx.end()) {
                            info.globalToLocalIdx[nodeID] = localIdx++;
                            info.Neighbors.push_back(_arriveVertices[nodeID]);
							
							//printf("adding neighbor, central node id: %lu\n", centralNodeID);
                        }
                    }
                }
            }
        }

        info.Indices.push_back(hopNeighbors);
    }

    return info;
}

void Torus_3D::AddVertices() {
    size_t network_node_ID = 0;
    for (size_t z = 0; z < _gridSizeZ; z++) {
        for (size_t y = 0; y < _gridSizeY; y++) {
            for (size_t x = 0; x < _gridSizeX; x++) {
                // Connect arrive vertices to their corresponding depart vertices
                _arriveVertices[network_node_ID]->AddDepartVertex(_departVertices[network_node_ID]);

                // Set up neighbors with torus wrapping (6 directions in 3D)
                std::vector<std::shared_ptr<Torus_3D_Arrive>> arrive_vertices(6);

                // Calculate wrapped indices
                size_t west_x = WrapCoordinate(x - 1, _gridSizeX);
                size_t east_x = WrapCoordinate(x + 1, _gridSizeX);
                size_t north_y = WrapCoordinate(y - 1, _gridSizeY);
                size_t south_y = WrapCoordinate(y + 1, _gridSizeY);
                size_t down_z = WrapCoordinate(z - 1, _gridSizeZ);
                size_t up_z = WrapCoordinate(z + 1, _gridSizeZ);

                // Connect to wrapped neighbors
                arrive_vertices[0] = _arriveVertices[z * _gridSizeX * _gridSizeY + y * _gridSizeX + west_x];    // West
                arrive_vertices[1] = _arriveVertices[z * _gridSizeX * _gridSizeY + y * _gridSizeX + east_x];    // East
                arrive_vertices[2] = _arriveVertices[z * _gridSizeX * _gridSizeY + north_y * _gridSizeX + x];   // North
                arrive_vertices[3] = _arriveVertices[z * _gridSizeX * _gridSizeY + south_y * _gridSizeX + x];   // South
                arrive_vertices[4] = _arriveVertices[down_z * _gridSizeX * _gridSizeY + y * _gridSizeX + x];    // Down
                arrive_vertices[5] = _arriveVertices[up_z * _gridSizeX * _gridSizeY + y * _gridSizeX + x];      // Up

                _departVertices[network_node_ID]->AddArriveVertices(arrive_vertices);

                // Generate and add hierarchical neighbor structures
                Torus_3D_NeighborInfo neighborInfo = GetHopNeighborStructures(x, y, z);
                _departVertices[network_node_ID]->AddNeighborInfo(neighborInfo);

                network_node_ID++;
            }
        }
    }
}

void Torus_3D::CreateEdges() {
    _edges.resize(getNumVertices());

    for (size_t z = 0; z < _gridSizeZ; z++) {
        for (size_t y = 0; y < _gridSizeY; y++) {
            for (size_t x = 0; x < _gridSizeX; x++) {
                size_t arriveIdx = GetVertexIndex(x, y, z, 0);
                size_t departIdx = GetVertexIndex(x, y, z, 1);

                // Arrive -> Depart
                _edges[arriveIdx].emplace_back(arriveIdx, departIdx, _minServiceTime);

                // Calculate wrapped indices
                size_t west_x = WrapCoordinate(x - 1, _gridSizeX);
                size_t east_x = WrapCoordinate(x + 1, _gridSizeX);
                size_t north_y = WrapCoordinate(y - 1, _gridSizeY);
                size_t south_y = WrapCoordinate(y + 1, _gridSizeY);
                size_t down_z = WrapCoordinate(z - 1, _gridSizeZ);
                size_t up_z = WrapCoordinate(z + 1, _gridSizeZ);

                // Connect Depart to neighboring Arrive vertices (with wrapping)
                _edges[departIdx].emplace_back(departIdx, GetVertexIndex(west_x, y, z, 0), _minTransitTime);     // West
                _edges[departIdx].emplace_back(departIdx, GetVertexIndex(east_x, y, z, 0), _minTransitTime);     // East
                _edges[departIdx].emplace_back(departIdx, GetVertexIndex(x, north_y, z, 0), _minTransitTime);    // North
                _edges[departIdx].emplace_back(departIdx, GetVertexIndex(x, south_y, z, 0), _minTransitTime);    // South
                _edges[departIdx].emplace_back(departIdx, GetVertexIndex(x, y, down_z, 0), _minTransitTime);     // Down
                _edges[departIdx].emplace_back(departIdx, GetVertexIndex(x, y, up_z, 0), _minTransitTime);       // Up
            }
        }
    }
}

void Torus_3D::IO_SVs() {
    size_t total_nodes = _gridSizeX * _gridSizeY * _gridSizeZ;
    for (size_t i = 0; i < total_nodes; i++) {
        _arriveVertices[i]->IO_SVs(_Is, _Os);
        _departVertices[i]->IO_SVs(_Is, _Os);
    }
}

void Torus_3D::InitEvents() {
    TriangularDist init_arrive_delays(_minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime, _distSeed);

    for (size_t i = 0; i < _gridSizeX * _gridSizeY * _gridSizeZ; i++) {
        AppendInitEvent(_arriveVertices[i]->getVertexID(),
                        init_arrive_delays.GenRV(),
                        -1);
    }
}

size_t Torus_3D::GetVertexIndex(size_t x, size_t y, size_t z, size_t type) const {
    return (z * _gridSizeX * _gridSizeY + y * _gridSizeX + x) * 2 + type;
}

void Torus_3D::PrintMeanPacketNetworkTime() const {
    double total_network_time = 0;
    for (const auto& packet : _packets) {
        total_network_time += packet.GetTimeInNetwork();
    }
    printf("Mean packet network time: %.*lf\n", DBL_DIG, total_network_time / _packets.size());
}

void Torus_3D::PrintSVs() const {
    printf("\n");
    for (size_t z = 0; z < _gridSizeZ; z++) {
        printf("Layer z=%lu:\n", z);
        for (size_t y = 0; y < _gridSizeY; y++) {
            for (size_t x = 0; x < _gridSizeX; x++) {
                size_t idx = z * _gridSizeX * _gridSizeY + y * _gridSizeX + x;
                printf("%d ", _packetQueueSVs[idx].get());
            }
            printf("\n");
        }
        printf("\n");
    }
}

void Torus_3D::PrintNumVertexExecs() const {
    printf("\n");
    for (size_t z = 0; z < _gridSizeZ; z++) {
        printf("Layer z=%lu:\n", z);
        for (size_t y = 0; y < _gridSizeY; y++) {
            for (size_t x = 0; x < _gridSizeX; x++) {
                size_t idx = z * _gridSizeX * _gridSizeY + y * _gridSizeX + x;
                printf("(%lu %lu) ", _arriveVertices[idx]->getNumExecs(),
                       _departVertices[idx]->getNumExecs());
            }
            printf("\n");
        }
        printf("\n");
    }
}

void Torus_3D::PrintFinishedPackets() const {
    printf("\n");
    for (const auto& packet : _packets) {
        packet.PrintData();
    }
}
