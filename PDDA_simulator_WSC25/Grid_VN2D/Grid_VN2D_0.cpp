#include "Grid_VN2D.h"
#include "../Dist.h"

#include <iostream>
#include <thread>
#include <filesystem>
#include <regex>

namespace fs = std::filesystem;

bool TryLockData(std::atomic<int>& shared_lock)
{
    int expected = 0;

    // Try to acquire the exclusive lock. If successful, return true.
    // Otherwise, return false.
    return shared_lock.compare_exchange_strong(expected, -1);
}

void SpinLockData(std::atomic<int>& shared_lock)
{
    int expected = 0;
    while (!shared_lock.compare_exchange_weak(expected, -1)) {
        expected = 0;
        std::this_thread::yield();
    }
}

void UnlockData(std::atomic<int>& shared_lock)
{
    shared_lock.store(0);
}

struct NodeInfo {
    size_t id;
    int cost;
    std::vector<int> path;  // stores the sequence of directions

    // Default constructor
    NodeInfo() : id(0), cost(0) {}

    // Regular constructor
    NodeInfo(size_t i, int c) : id(i), cost(c) {}
};

// Packet class
Packet::Packet(double genTime, size_t originNetworkNodeID, size_t destNetworkNodeID, size_t destX, size_t destY, size_t gridSizeX, size_t gridSizeY)
:Entity(genTime), _originNetworkNodeID(originNetworkNodeID), _destNetworkNodeID(destNetworkNodeID), _destX(destX), _destY(destY), _gridSizeX(gridSizeX), _gridSizeY(gridSizeY)
{
    _minManhattanDist = std::numeric_limits<size_t>::max();
}

void Packet::AddNetworkNodeData(double arrivalTime, size_t networkNodeID)
{
    // Check if this node has already been visited
    if (std::find(_visitedNetworkNodes.begin(), _visitedNetworkNodes.end(), networkNodeID) != _visitedNetworkNodes.end()) {
        printf("WARNING: Packet %lu revisiting node %lu\n", _ID, networkNodeID);
    }

    size_t node_x = networkNodeID % _gridSizeX;
    size_t node_y = networkNodeID / _gridSizeX;

    size_t dest_distance = abs(static_cast<int>(_destX) - static_cast<int>(node_x)) +
    abs(static_cast<int>(_destY) - static_cast<int>(node_y));

    if (dest_distance >= _minManhattanDist) {
        printf("error: packet manhattan distance not decreased!\n");
        printf("\tnode ID: %lu, node x: %lu, node y: %lu\n", networkNodeID, node_x, node_y);
        printf("\tmin MH dist: %lu, dest MH dist: %lu\n", _minManhattanDist, dest_distance);
        PrintData();
        exit(1);
    }
    _minManhattanDist = dest_distance;

    _networkNodeArrivalTimes.push_back(arrivalTime);
    _visitedNetworkNodes.push_back(networkNodeID);
}

const std::list<size_t>& Packet::getVisitedNetworkNodes() const { return _visitedNetworkNodes; }

size_t Packet::getDestNetworkNodeID() const { return _destNetworkNodeID; }

double Packet::GetTimeInNetwork() const { return _exitTime - _genTime; }

void Packet::PrintData() const
{
    std::string arrival_times;
    for (auto arrival_time : _networkNodeArrivalTimes) arrival_times.append(std::to_string(arrival_time) + ",");

    std::string traversed_nodes;
    for (auto node : _visitedNetworkNodes) traversed_nodes.append(std::to_string(node) + ",");

    printf("ID: %lu, gen t: %lf, o: %lu, d: %lu, arrival ts: %s, nodes: %s\n", _ID, _genTime, _originNetworkNodeID, _destNetworkNodeID, arrival_times.c_str(), traversed_nodes.c_str());
}


// Arrive class
Grid_VN2D_Arrive::Grid_VN2D_Arrive(size_t networkNodeID, size_t gridSizeX, size_t gridSizeY, const std::string& vertexName, size_t distSeed, std::string traceFolderName, std::string traceFileHeading, size_t maxNumIntraArriveEvents, size_t minIntraArrivalTime, size_t modeIntraArrivalTime, size_t maxIntraArrivalTime, size_t minServiceTime, size_t modeServiceTime, size_t maxServiceTime, PDDA_SV<int>& packetQueueSV, std::queue<std::shared_ptr<Packet>>& packetQueue, std::list<std::shared_ptr<Packet>>& finishedPackets, std::atomic<int>& finishedPacketListLock)
:Vertex(vertexName, 0, distSeed, traceFolderName, traceFileHeading), _networkNodeID(networkNodeID), _gridSizeX(gridSizeX), _gridSizeY(gridSizeY), _numIntraArriveEvents(0), _maxNumIntraArriveEvents(maxNumIntraArriveEvents), _packetQueueSV(packetQueueSV), _packetQueue(packetQueue), _finishedPackets(finishedPackets), _finishedPacketListLock(finishedPacketListLock)
{
    _randomNodeID = std::make_unique<UniformIntDist>(0, _gridSizeX*_gridSizeY-1, distSeed + networkNodeID);

    int high_activity_node = UniformIntDist(1,6,distSeed + networkNodeID).GenRV();
    if (1 == high_activity_node) {
        minIntraArrivalTime /= 2;
        modeIntraArrivalTime /= 2;
        maxIntraArrivalTime /= 2;
    }
    _intraArrivalDelay = std::make_unique<TriangularDist>(minIntraArrivalTime, modeIntraArrivalTime, maxIntraArrivalTime, distSeed + networkNodeID);
    _serviceDelay = std::make_unique<TriangularDist>(minServiceTime, modeServiceTime, maxServiceTime, distSeed + networkNodeID);
}

size_t Grid_VN2D_Arrive::getNetworkNodeID() const { return _networkNodeID; }

PDDA_SV<int>& Grid_VN2D_Arrive::getPacketQueueSV() { return _packetQueueSV; }

void Grid_VN2D_Arrive::AddDepartVertex(std::shared_ptr<Grid_VN2D_Depart> departVertex) { _departVertex = departVertex; }

void Grid_VN2D_Arrive::IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os)
{
    // Input SV Indices
    std::vector<size_t> input_SV_indices;
    input_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Is.push_back(input_SV_indices);

    // Output SV Indices
    std::vector<size_t> output_SV_indices;
    output_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Os.push_back(output_SV_indices);
}

void Grid_VN2D_Arrive::Run(std::list<PDDA_Event*>& newEvents, double simTime, std::shared_ptr<Entity> entity)
{
    //printf("Arrive ID: %lu,, sim time: %lf\n", _networkNodeID, simTime);

    std::shared_ptr<Packet> packet = std::dynamic_pointer_cast<Packet>(entity);

    // (1) Evaluate Conditions
    _atDestination = false;
    if (nullptr == packet) _intraArrival = true;
    else {
        _intraArrival = false;
        if (_networkNodeID == packet->getDestNetworkNodeID()) _atDestination = true;
    }
    if (_packetQueueSV.get() < 0) {  // queue SV doubles as server SV
        _serverAvailable = true;
    }
    else _serverAvailable = false;

    // (2) Update SVs
    // designate destination for new packet
    if (_intraArrival) {
        size_t dest_network_node_ID = _randomNodeID->GenRV();
        //printf("dest node ID: %lu, orig node ID: %lu\n", dest_network_node_ID, _networkNodeID);
        while (_networkNodeID == dest_network_node_ID) {
            dest_network_node_ID = _randomNodeID->GenRV();
            //printf("\tdest node ID: %lu, orig node ID: %lu\n", dest_network_node_ID, _networkNodeID);
        }
        packet = std::make_shared<Packet>(simTime, _networkNodeID, dest_network_node_ID, dest_network_node_ID % _gridSizeX, dest_network_node_ID / _gridSizeX, _gridSizeX, _gridSizeY);
    }
    // update packet data
    packet->AddNetworkNodeData(simTime, _networkNodeID);
    if (!_atDestination) {
        _packetQueueSV.inc();  // queue is incremented regardless of server availability
        if (!_serverAvailable) _packetQueue.push(packet);
    }
    else {
        packet->setExitTime(simTime);
        SpinLockData(_finishedPacketListLock);
        _finishedPackets.push_back(packet);
        UnlockData(_finishedPacketListLock);
        //printf("packet finished!\n");
        //packet->PrintData();
    }

    // (3) Schedule New Events
    if (!_atDestination && _serverAvailable) newEvents.push_back(new PDDA_Event(_departVertex, simTime + _serviceDelay->GenRV(), packet));
    if (_intraArrival && ++_numIntraArriveEvents < _maxNumIntraArriveEvents) newEvents.push_back(new PDDA_Event(shared_from_this(), simTime + _intraArrivalDelay->GenRV(), nullptr));

    _numExecutions++;
}


// // Service class
// Grid_VN2DService::Grid_VN2DService(size_t networkNodeID, const std::string& vertexName, int distSeed, std::string traceFolderName, std::string traceFileHeading, size_t minServiceTime, size_t modeServiceTime, size_t maxServiceTime)
// :Vertex(vertexName, extraWork, distSeed, traceFolderName, traceFileHeading), _networkNodeID(networkNodeID)
// {
//     _serviceDelay = std::make_unique<TriangularDist>(minServiceTime, modeServiceTime, maxServiceTime, distSeed + networkNodeID);
// }
//
// void Grid_VN2DService::AddDepartVertex(std::shared_ptr<Depart> departVertex) { _departVertex = departVertex; }
//
// void Grid_VN2DService::IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os)
// {
//     // Input SV Indices
//     std::vector<int> input_SV_indices;
//     Is.push_back(input_SV_indices);
//
//     // Output SV Indices
//     std::vector<int> output_SV_indices;
//     Os.push_back(output_SV_indices);
// }
//
// void Grid_VN2DService::Run(std::list<PDDA_Event*>& newEvents, double simTime, std::shared_ptr<Entity> entity)
// {
//     std::shared_ptr<Packet> packet = std::dynamic_pointer_cast<Packet>(entity);
//
//     // (1) Evaluate Conditions
//
//     // (2) Update SVs
//
//     // (3) Schedule New Events
//     newEvents.push_back(new PDDA_Event(_departVertex, simTime + _serviceDelay->GenRV(), packet));
//
//     _numExecutions++;
// }


// Depart class
Grid_VN2D_Depart::Grid_VN2D_Depart(size_t networkNodeID, size_t x, size_t y, std::vector<bool> neighbors, size_t gridSizeX, size_t gridSizeY, size_t hopRadius, const std::string& vertexName, size_t distSeed, std::string traceFolderName, std::string traceFileHeading, size_t minServiceTime, size_t modeServiceTime, size_t maxServiceTime, size_t minTransitTime, size_t modeTransitTime, size_t maxTransitTime, PDDA_SV<int>& packetQueueSV, std::queue<std::shared_ptr<Packet>>& packetQueue)
:Vertex(vertexName, 0, distSeed, traceFolderName, traceFileHeading), _networkNodeID(networkNodeID), _x(x), _y(y), _hopRadius(hopRadius), _neighbors(neighbors), _gridSizeX(gridSizeX), _gridSizeY(gridSizeY), _packetQueueSV(packetQueueSV), _packetQueue(packetQueue)
{
    _serviceDelay = std::make_unique<TriangularDist>(minServiceTime, modeServiceTime, maxServiceTime, distSeed + networkNodeID);
    _transitDelay = std::make_unique<TriangularDist>(minTransitTime, modeTransitTime, maxTransitTime, distSeed + networkNodeID);

    //for (int i=0; i<4; i++) std::cout << _neighbors[i] << ',';
    //std::cout << std::endl;

    //size_t x = _networkNodeID % _gridSizeX;
    //size_t y = _networkNodeID / _gridSizeX;
    // _neighbors = std::fill(_neighbors.begin(), _neighbors.begin()+4, 0);
    //
    // if (x > 0)              _neighbors[0] = 1; // West
    // if (x < _gridSizeX-1)   _neighbors[1] = 1; // East
    // if (y > 0)              _neighbors[2] = 1; // North
    // if (y < _gridSizeY-1)   _neighbors[3] = 1; // South
}

void Grid_VN2D_Depart::AddArriveVertices(std::vector<std::shared_ptr<Grid_VN2D_Arrive>> arriveVertices) { _arriveVertices = arriveVertices; }

void Grid_VN2D_Depart::AddNeighborInfo(const NeighborInfo& info) { _neighborInfo = info; }

void Grid_VN2D_Depart::IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) {
    // Input SV Indices
    std::vector<size_t> input_SV_indices;

    // Add own queue SV
    input_SV_indices.push_back(_packetQueueSV.getModelIndex());

    // Add all neighbor SVs from the NeighborInfo structure
    // N contains pointers to all neighbor nodes in order of their local indices
    for (const auto& neighbor : _neighborInfo.N) {
        if (neighbor) {
            input_SV_indices.push_back(neighbor->getPacketQueueSV().getModelIndex());
        }
    }

    Is.push_back(input_SV_indices);

    // Output SV Indices
    std::vector<size_t> output_SV_indices;
    output_SV_indices.push_back(_packetQueueSV.getModelIndex());
    Os.push_back(output_SV_indices);
}

// Path Grid_VN2D_Depart::findShortestPath(size_t targetIdx)
// {
//     std::unordered_map<size_t, double> distances;
//     std::unordered_map<size_t, size_t> previous;
//     std::set<std::pair<double, size_t>> queue;  // (distance, node)
//
//     // Initialize distances
//     distances[0] = 0;  // Start from central node
//     queue.insert({0, 0});
//
//     while (!queue.empty()) {
//         size_t current = queue.begin()->second;
//         queue.erase(queue.begin());
//
//         if (current == targetIdx) break;
//
//         // Process neighbors from I
//         if (current < _neighborInfo.I.size()) {
//             const auto& neighbors = _neighborInfo.I[current];
//             for (size_t next : neighbors) {
//                 // Get queue size from N
//                 double queueSize = _neighborInfo.N[next]->getPacketQueueSV().get();
//                 double newDist = distances[current] + queueSize;
//
//                 if (!distances.count(next) || newDist < distances[next]) {
//                     queue.erase({distances[next], next});
//                     distances[next] = newDist;
//                     previous[next] = current;
//                     queue.insert({newDist, next});
//                 }
//             }
//         }
//     }
//
//     // Reconstruct path
//     Path result;
//     if (distances.find(targetIdx) == distances.end()) {
//         return result;  // No path found
//     }
//
//     result.totalCost = distances[targetIdx];
//     size_t current = targetIdx;
//     while (previous.find(current) != previous.end()) {
//         result.nodeIndices.push_back(current);
//         current = previous[current];
//     }
//     result.nodeIndices.push_back(0);  // Add central node
//     std::reverse(result.nodeIndices.begin(), result.nodeIndices.end());
//
//     return result;
// }

void Grid_VN2D_Depart::PrintNeighborInfo() const {
    printf("\n=== Neighbor Info Debug Output ===\n");

    // Print I structure
    printf("I structure (size: %lu):\n", _neighborInfo.I.size());
    for (size_t i = 0; i < _neighborInfo.I.size(); i++) {
        printf("Hop level %lu (size: %lu): ", i, _neighborInfo.I[i].size());
        for (const auto& idx : _neighborInfo.I[i]) {
            printf("%lu ", idx);
        }
        printf("\n");
    }

    // Print N structure
    printf("\nN structure (size: %lu):\n", _neighborInfo.N.size());
    for (size_t i = 0; i < _neighborInfo.N.size(); i++) {
        if (_neighborInfo.N[i]) {
            printf("Index %lu: Node ID %lu", i, _neighborInfo.N[i]->getNetworkNodeID());
            // Also print queue size if needed
            printf(" (Queue size: %d)", _neighborInfo.N[i]->getPacketQueueSV().get());
        } else {
            printf("Index %lu: nullptr", i);
        }
        printf("\n");
    }

    // Print current node info
    printf("\nCurrent node position: x=%lu, y=%lu\n", _x, _y);
    printf("Grid size: %lu x %lu\n", _gridSizeX, _gridSizeY);
    printf("Hop radius: %lu\n", _hopRadius);
    printf("==============================\n\n");
}

void Grid_VN2D_Depart::Run(std::list<PDDA_Event*>& newEvents, double simTime, std::shared_ptr<Entity> entity) {

    //printf("Depart x,y: %lu,%lu, ID: %lu, sim time: %lf\n", _x, _y, _networkNodeID, simTime);

    std::shared_ptr<Packet> packet = std::dynamic_pointer_cast<Packet>(entity);

    //PrintNeighborInfo();

    // (1) Evaluate Conditions
    if (_packetQueueSV.get() > 0) {
        _packetInQueue = true;
    }
    else _packetInQueue = false;

    // (2) Update SVs
    std::shared_ptr<Packet> queue_packet;
    _packetQueueSV.dec();
    if (_packetInQueue) {
        queue_packet = std::move(_packetQueue.front());
        _packetQueue.pop();
    }

    // Determine routing
    size_t dest_network_node_ID = packet->getDestNetworkNodeID();
    size_t dest_x = dest_network_node_ID % _gridSizeX;
    size_t dest_y = dest_network_node_ID / _gridSizeX;
    size_t current_network_node_ID = _y * _gridSizeX + _x;

    //printf("Current: (%lu, %lu), Destination: (%lu, %lu)\n", _x, _y, dest_x, dest_y);

    size_t dest_x_diff = abs(static_cast<int>(dest_x) - static_cast<int>(_x));
    size_t dest_y_diff = abs(static_cast<int>(dest_y) - static_cast<int>(_y));
    size_t current_distance = dest_x_diff + dest_y_diff;

    auto visited_nodes = packet->getVisitedNetworkNodes();

    // Find best path
    Path best_path;

    // If destination is within hop radius
    if (current_distance <= _hopRadius) {
        //printf("Destination within hop radius\n");
        best_path = _neighborInfo.findShortestPath(dest_network_node_ID, dest_network_node_ID, _x, _y, _gridSizeX, _gridSizeY, _hopRadius, visited_nodes);
        //printf("LEQ hop radius, Current: (%lu, %lu) %lu, Destination: (%lu, %lu) %lu\n", _x, _y, _networkNodeID, dest_x, dest_y, dest_network_node_ID);
        //best_path.PrintPath();
    }
    // Destination is beyond hop radius
    else {
        //printf("Destination beyond hop radius, checking max-hop neighbors\n");
        const auto& max_hop_neighbors = _neighborInfo.I.back();  // Get max-hop neighbors

        //printf("Available max-hop neighbors: ");
        //for (size_t id : max_hop_neighbors) printf("%lu ", id);
        //printf("\n");

        //double min_path_cost = std::numeric_limits<double>::max();
        size_t min_path_manhattan_distance = std::numeric_limits<size_t>::max();
        std::vector<Path> max_hop_shortest_paths;

        // Find max-hop neighbors that reduce distance to destination
        for (size_t neighbor_id : max_hop_neighbors) {

            if (std::find(visited_nodes.begin(), visited_nodes.end(), neighbor_id) != visited_nodes.end()) continue;

            size_t neighbor_x = neighbor_id % _gridSizeX;
            size_t neighbor_y = neighbor_id / _gridSizeX;

            size_t neighbor_distance = abs(static_cast<int>(dest_x) - static_cast<int>(neighbor_x)) +
            abs(static_cast<int>(dest_y) - static_cast<int>(neighbor_y));

            if (neighbor_distance < current_distance) {
                //printf("\nChecking max-hop neighbor %lu at (%lu, %lu):\n",
                //       neighbor_id, neighbor_x, neighbor_y);
                //printf("- Distance to dest: %lu (current: %lu)\n",
                //       neighbor_distance, current_distance);

                //printf("neighbor ID %lu distance (%lu) is less than current distance (%lu)\n", neighbor_id, neighbor_distance, current_distance);

                Path path = _neighborInfo.findShortestPath(dest_network_node_ID, neighbor_id, _x, _y, _gridSizeX, _gridSizeY, _hopRadius, visited_nodes);
                if (!path.nodeIndices.empty()) max_hop_shortest_paths.push_back(path);
            }
            //else printf("neighbor ID %lu distance (%lu) is NOT less than current distance (%lu)\n", neighbor_id, neighbor_distance, current_distance);
        }
        // Given all paths in max_hop_shortest_paths, use the shortest path to the max_hop_neighbor, where:
        // (1) the manhattan distance from max_hop_neighbor to dest_network_node_ID is the smallest distance, for which
        // (2) the total cost of the path is in the bottom half of all the shortest path total costs.
        // In other words, sort the shortest paths by total costs in ascending order, size n, remove the most expensive
        // (in terms of total cost) floor(0.5*n) shortest paths, then select the remaining shortest path with the
        // smallest manhattan distance, i.e. from the max_hop_neighbor at the end of that shortest path, to dest_network_node_ID.
        // Implement this here:

        if (max_hop_shortest_paths.empty()) {
            printf("max-hop shortest paths empty!\n");
            packet->PrintData();
            printf("Current: (%lu, %lu) %lu, Destination: (%lu, %lu) %lu\n", _x, _y, _networkNodeID, dest_x, dest_y, dest_network_node_ID);
        }

        // Sort paths by total cost
        std::sort(max_hop_shortest_paths.begin(), max_hop_shortest_paths.end(),
                  [](const Path& a, const Path& b) {
                      return a.totalCost < b.totalCost;
                  });

        // Keep only the cheaper half of paths
        size_t num_paths_to_keep = (max_hop_shortest_paths.size() + 1) / 2;  // Round up
        if (num_paths_to_keep > 0) {
            max_hop_shortest_paths.resize(num_paths_to_keep);

            // Among remaining paths, find the one ending closest to destination
            min_path_manhattan_distance = std::numeric_limits<size_t>::max();

            for (const Path& path : max_hop_shortest_paths) {
                // Get the max-hop neighbor (last node in path)
                size_t max_hop_node = path.nodeIndices.back();
                size_t max_hop_x = max_hop_node % _gridSizeX;
                size_t max_hop_y = max_hop_node / _gridSizeX;

                // Calculate Manhattan distance to destination
                size_t distance = abs(static_cast<int>(dest_x) - static_cast<int>(max_hop_x)) +
                abs(static_cast<int>(dest_y) - static_cast<int>(max_hop_y));

                //printf("Path ending at %lu has distance %lu to destination\n", max_hop_node, distance);

                if (distance < min_path_manhattan_distance) {
                    min_path_manhattan_distance = distance;
                    best_path = path;
                }
            }
        }
        else {
            printf("error: no paths to max-hop neighbors!\n");
            exit(1);
        }
    }

    // Determine direction from best path
    int dest_dir = -1;
    if (!best_path.nodeIndices.empty() && best_path.nodeIndices.size() > 1) {
        size_t next_node_id = best_path.nodeIndices[1];  // First hop
        //printf("Next hop node ID: %lu\n", next_node_id);

        // Check if next_node_id is a valid 1-hop neighbor
        const auto& one_hop_neighbors = _neighborInfo.I[0];
        if (std::find(one_hop_neighbors.begin(), one_hop_neighbors.end(), next_node_id)
            == one_hop_neighbors.end()) {
            printf("ERROR: Next hop %lu is not a valid 1-hop neighbor!\n", next_node_id);
            }
            else {
                // Convert to direction
                if (next_node_id == current_network_node_ID - 1) dest_dir = 0;      // West
                else if (next_node_id == current_network_node_ID + 1) dest_dir = 1;  // East
                else if (next_node_id == current_network_node_ID - _gridSizeX) dest_dir = 2;  // North
                else if (next_node_id == current_network_node_ID + _gridSizeX) dest_dir = 3;  // South
            }
    }

    if (dest_dir == -1) {
        printf("Error: No valid path found!\n");
        printf("1-hop neighbors in I[0]: ");
        for (size_t id : _neighborInfo.I[0]) printf("%lu ", id);
        printf("\n");
        exit(1);
    }

    // (3) Schedule New Events
    if (_packetInQueue) {
        newEvents.push_back(new PDDA_Event(shared_from_this(),
                                          simTime + _serviceDelay->GenRV(),
                                          queue_packet));
    }

    newEvents.push_back(new PDDA_Event(_arriveVertices.at(dest_dir),
                                      simTime + _transitDelay->GenRV(),
                                      packet));

    _numExecutions++;
}

// Grid_VN2D class
Grid_VN2D::Grid_VN2D(size_t gridSizeX, size_t gridSizeY, size_t hopRadius, size_t numServersPerNetworkNode, size_t maxNumArriveEvents, double maxSimTime, size_t numThreads, size_t distSeed, int numSerialPDDA_Execs, std::string traceFolderName, std::string distParamsFile)
:PDDA_SimModel(maxSimTime, numThreads, distSeed, numSerialPDDA_Execs, traceFolderName), _gridSizeX(gridSizeX), _gridSizeY(gridSizeY), _hopRadius(hopRadius), _numServersPerNetworkNode(numServersPerNetworkNode), _maxNumArriveEvents(maxNumArriveEvents)
{
    std::cout << "PDDA_grid_network " << numSerialPDDA_Execs << std::endl;

    // distribution parameters
    std::ifstream dist_params_file(distParamsFile);
    dist_params_file >> _minIntraArrivalTime >> _modeIntraArrivalTime >> _maxIntraArrivalTime;
    dist_params_file >> _minServiceTime >> _modeServiceTime >> _maxServiceTime;
    dist_params_file >> _minTransitTime >> _modeTransitTime >> _maxTransitTime;

    // Check if the folder exists, and create it if it doesn't
    if (!fs::exists(_traceFolderName)) {
        if (fs::create_directory(_traceFolderName)) {
            std::cout << "Folder '" << _traceFolderName << "' created successfully.\n";
        } else {
            std::cerr << "Error: Failed to create folder '" << _traceFolderName << "'.\n";
        }
    } else {
        std::cout << "Folder '" << _traceFolderName << "' already exists.\n";
    }

    BuildModel();

    std::string table_filename = "ITL_table_PDDA_VN2D_grid_network_size_" + std::to_string(_gridSizeX) + "_" + std::to_string(_gridSizeY) + "_hop_radius_" + std::to_string(_hopRadius) + ".csv";
    Init_PDDA(table_filename);
}

void Grid_VN2D::BuildModel() {
    //printf("creating vertices\n");
    CreateVertices();
    //printf("adding vertices\n");
    AddVertices();
    //printf("creating edges\n");
    CreateEdges();
    IO_SVs();
    InitEvents();
}

void Grid_VN2D::CreateVertices() {
    // initialize SVs
    int init_packet_queue_val = -1 * _numServersPerNetworkNode;
    for (size_t i=0; i<_gridSizeX*_gridSizeY; i++) {
        std::string name = "Packet Queue " + std::to_string(i);
        _packetQueueSVs.emplace_back(name, init_packet_queue_val, init_packet_queue_val - 1, std::numeric_limits<int>::max());
    }
    _packetQueues.resize(_gridSizeX*_gridSizeY);
    //printf("created SVs\n");

    // initialize vertices
    size_t network_node_ID = 0;
    for (size_t y=0; y<_gridSizeY; y++) {
        for (size_t x=0; x<_gridSizeX; x++) {

            //std::cout << x << ',' << y << std::endl;
            //std::cout << _packetQueueSVs.at(network_node_ID).get() << std::endl;

            // arrive vertices
            std::string trace_file_heading = "ts, Q" + std::to_string(network_node_ID);
            _arriveVertices.push_back(std::make_shared<Grid_VN2D_Arrive>(network_node_ID, _gridSizeX, _gridSizeY, "Arrive_" + std::to_string(x) + "_" + std::to_string(y), _distSeed, _traceFolderName, trace_file_heading, _maxNumArriveEvents, _minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime, _minServiceTime, _modeServiceTime, _maxServiceTime, _packetQueueSVs.at(network_node_ID), _packetQueues.at(network_node_ID), _finishedPackets, _finishedPacketListLock));

            //printf("made arrive vertex\n");

            //size_t networkNodeID, size_t gridSizeX, size_t gridSizeY, const std::string& vertexName, int distSeed, std::string traceFolderName, std::string traceFileHeading, size_t maxNumArriveEvents, size_t minIntraArrivalTime, size_t modeIntraArrivalTime, size_t maxIntraArrivalTime, size_t minServiceTime, size_t modeServiceTime, size_t maxServiceTime, PDDA_SV<int>& packetQueueSV, std::queue<std::shared_ptr<Packet>>& packetQueue, std::list<std::shared_ptr<Packet>>& finishedPackets, std::atomic<int>& finishedPacketListLock

            // // service vertices
            // trace_file_heading = "ts, ";
            // _serviceVertices.emplace_back(std::make_shared<Service>(network_node_ID, "Service_" + std::to_string(x) + "_" + std::to_string(y), _distSeed, _traceFolderName, trace_file_heading, _minServiceTime, _modeServiceTime, _maxServiceTime));

            // depart vertices
            std::vector<bool> neighbors(4, 0);
            if (x > 0)              neighbors[0] = 1; // West
            if (x < _gridSizeX-1)   neighbors[1] = 1; // East
            if (y > 0)              neighbors[2] = 1; // North
            if (y < _gridSizeY-1)   neighbors[3] = 1; // South

            //for (int i=0; i<4; i++) std::cout << neighbors[i] << ',';
            //std::cout << std::endl;

            trace_file_heading = "ts, Q" + std::to_string(network_node_ID);
            _departVertices.push_back(std::make_shared<Grid_VN2D_Depart>(network_node_ID, x, y, neighbors, _gridSizeX, _gridSizeY, _hopRadius, "Depart_" + std::to_string(x) + "_" + std::to_string(y), _distSeed, _traceFolderName, trace_file_heading, _minServiceTime, _modeServiceTime, _maxServiceTime, _minTransitTime, _modeTransitTime, _maxTransitTime, _packetQueueSVs.at(network_node_ID), _packetQueues.at(network_node_ID)));

            network_node_ID++;

            //printf("made depart vertex\n");
        }
    }
    //printf("created vertices\n");

    // set number of vertices in PDDA_SimModel
    setNumVertices(Vertex::getNumVertices());
}

// // Helper function to get next-hop neighbors for a given node
// std::vector<size_t> Grid_VN2D::getNextHopNeighbors(size_t node, const std::unordered_set<size_t>& seenNodes) const
// {
//     std::vector<size_t> neighbors;
//     size_t x = node % _gridSizeX;
//     size_t y = node / _gridSizeX;
//
//     // Check all four directions (W, E, N, S)
//     if (x > 0) {
//         size_t west = node - 1;
//         if (seenNodes.find(west) == seenNodes.end())
//             neighbors.push_back(west);
//     }
//     if (x < _gridSizeX - 1) {
//         size_t east = node + 1;
//         if (seenNodes.find(east) == seenNodes.end())
//             neighbors.push_back(east);
//     }
//     if (y > 0) {
//         size_t north = node - _gridSizeX;
//         if (seenNodes.find(north) == seenNodes.end())
//             neighbors.push_back(north);
//     }
//     if (y < _gridSizeY - 1) {
//         size_t south = node + _gridSizeX;
//         if (seenNodes.find(south) == seenNodes.end())
//             neighbors.push_back(south);
//     }
//
//     return neighbors;
// }

NeighborInfo Grid_VN2D::GetHopNeighborStructures(size_t x, size_t y) {
    NeighborInfo info;
    size_t centralNodeID = y * _gridSizeX + x;
    size_t localIdx = 0;

    // Add central node index to I
    //std::vector<size_t> central_node_vec = {centralNodeID};
    //info.I.push_back(central_node_vec);

    // Add central node to N (even though it's not a neighbor)
    info.globalToLocalIdx[centralNodeID] = localIdx++;
    info.N.push_back(_arriveVertices[centralNodeID]);

    // For each hop from 1 to _hopRadius, collect nodes at that Manhattan distance
    for (size_t hop = 1; hop <= _hopRadius; hop++) {
        std::vector<size_t> hopNeighbors;

        // Iterate over all possible nodes and check their Manhattan distance
        for (size_t nodeID = 0; nodeID < _gridSizeX * _gridSizeY; nodeID++) {
            size_t node_x = nodeID % _gridSizeX;
            size_t node_y = nodeID / _gridSizeX;
            size_t distance = abs(static_cast<int>(node_x) - static_cast<int>(x)) +
            abs(static_cast<int>(node_y) - static_cast<int>(y));

            if (distance == hop) {
                // Add to this hop level
                hopNeighbors.push_back(nodeID);
                if (info.globalToLocalIdx.find(nodeID) == info.globalToLocalIdx.end()) {
                    info.globalToLocalIdx[nodeID] = localIdx++;
                    info.N.push_back(_arriveVertices[nodeID]);
                }
            }
        }

        info.I.push_back(hopNeighbors);
    }

    // Debug output
    // printf("=== GetHopNeighborStructures (Manhattan) ===\n");
    // printf("Central node: x=%lu, y=%lu, ID=%lu\n", x, y, centralNodeID);
    // for (size_t h = 0; h < info.I.size(); h++) {
    //     printf("Hop %lu neighbors: ", h + 1);
    //     for (size_t nodeID : info.I[h]) printf("%lu ", nodeID);
    //     printf("\n");
    // }
    // printf("==========================================\n");

    return info;
}

void Grid_VN2D::AddVertices() {
    size_t network_node_ID = 0;
    for (size_t y = 0; y < _gridSizeY; y++) {
        for (size_t x = 0; x < _gridSizeX; x++) {
            // Connect arrive vertices to their corresponding depart vertices
            _arriveVertices.at(network_node_ID)->AddDepartVertex(_departVertices.at(network_node_ID));

            // Set up immediate neighbor connections for basic routing
            std::vector<std::shared_ptr<Grid_VN2D_Arrive>> arrive_vertices;
            if (x > 0)              arrive_vertices.push_back(_arriveVertices.at(network_node_ID - 1)); // West
            else                    arrive_vertices.push_back(nullptr);                                 // No West
            if (x < _gridSizeX-1)   arrive_vertices.push_back(_arriveVertices.at(network_node_ID + 1)); // East
            else                    arrive_vertices.push_back(nullptr);                                 // No East
            if (y > 0)              arrive_vertices.push_back(_arriveVertices.at(network_node_ID - _gridSizeX)); // North
            else                    arrive_vertices.push_back(nullptr);                                          // No North
            if (y < _gridSizeY-1)   arrive_vertices.push_back(_arriveVertices.at(network_node_ID + _gridSizeX)); // South
            else                    arrive_vertices.push_back(nullptr);                                          // No South
            _departVertices.at(network_node_ID)->AddArriveVertices(arrive_vertices);

            // Generate hierarchical neighbor structures for this node
            NeighborInfo neighborInfo = GetHopNeighborStructures(x, y);

            // Add the neighbor structures to the depart vertex
            _departVertices.at(network_node_ID)->AddNeighborInfo(neighborInfo);

            network_node_ID++;
        }
    }
}

// for DDA
void Grid_VN2D::CreateEdges() {
    _edges.resize(getNumVertices());

    for (size_t y=0; y<_gridSizeY; y++) {
        for (size_t x=0; x<_gridSizeX; x++) {
            size_t arriveIdx = GetVertexIndex(x,y,0);
            //size_t serviceIdx = GetVertexIndex(x,y,1);
            size_t departIdx = GetVertexIndex(x,y,1);

            // Arrive -> Depart
            _edges[arriveIdx].emplace_back(arriveIdx, departIdx, _minServiceTime);
            // // Service -> Depart
            // _edges[serviceIdx].push_back(departIdx, _minServiceTime);

            // Connect Depart to neighboring Arrive vertices
            if (x > 0)              _edges[departIdx].emplace_back(departIdx, GetVertexIndex(x-1,y,0), _minTransitTime); // West
            if (x < _gridSizeX-1)   _edges[departIdx].emplace_back(departIdx, GetVertexIndex(x+1,y,0), _minTransitTime); // East
            if (y > 0)              _edges[departIdx].emplace_back(departIdx, GetVertexIndex(x,y-1,0), _minTransitTime); // North
            if (y < _gridSizeY-1)   _edges[departIdx].emplace_back(departIdx, GetVertexIndex(x,y+1,0), _minTransitTime); // South
        }
    }
}

void Grid_VN2D::IO_SVs()
{
    for (size_t i=0; i<_gridSizeX*_gridSizeY; i++) {
        _arriveVertices.at(i)->IO_SVs(_Is, _Os);
        _departVertices.at(i)->IO_SVs(_Is, _Os);
    }
}

void Grid_VN2D::InitEvents()
{
    TriangularDist init_arrive_delays(_minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime, _distSeed);

    for (size_t i=0; i<_gridSizeX*_gridSizeY; i++) {
        _initEvents.push_back(new PDDA_Event(_arriveVertices.at(i), init_arrive_delays.GenRV(), nullptr));
    }
}

size_t Grid_VN2D::GetVertexIndex(size_t x, size_t y, size_t type) const
{
    return (y * _gridSizeX + x) * 2 + type;
}

void Grid_VN2D::PrintMeanPacketNetworkTime() const
{
    double total_network_time = 0;
    for (auto packet : _finishedPackets) total_network_time += packet->GetTimeInNetwork();
    printf("Mean packet network time: %lf\n", total_network_time / _finishedPackets.size());
}

void Grid_VN2D::PrintSVs() const
{
    printf("\n");
    for (size_t y=0; y<_gridSizeY; y++) {
        for (size_t x=0; x<_gridSizeX; x++) {
            printf("%d ", _packetQueueSVs.at(y*_gridSizeX + x).get());
        }
        printf("\n");
    }
}

void Grid_VN2D::PrintNumVertexExecs() const
{
    printf("\n");
    for (size_t y=0; y<_gridSizeY; y++) {
        for (size_t x=0; x<_gridSizeX; x++) {
            printf("(%d %d) ", _arriveVertices.at(y*_gridSizeX + x)->getNumExecs(), _departVertices.at(y*_gridSizeX + x)->getNumExecs());
        }
        printf("\n");
    }
}

void Grid_VN2D::PrintFinishedPackets() const
{
    printf("\n");
    for (auto packet : _finishedPackets) packet->PrintData();
}
