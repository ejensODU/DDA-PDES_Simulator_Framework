#include "Irregular_NoC.h"
#include "../Dist.h"
#include <iostream>
#include <thread>
#include <filesystem>
#include <fstream>
#include <regex>
#include <random>
#include <cmath>
#include <float.h>

namespace fs = std::filesystem;

Irregular_NoC::Irregular_NoC(size_t numNodes, size_t hopRadius, size_t numServersPerNetworkNode,
                   size_t maxNumArriveEvents, double maxSimTime,
                   size_t numThreads, size_t distSeed,
                   double bucketWidth, size_t targetBinSize,
                   std::string traceFolderName,
                   std::string distParamsFile,
                   std::string networkTopologyFile)
    : PDDA_SimModel(maxSimTime, numThreads, distSeed, bucketWidth, targetBinSize, traceFolderName),
      _numNodes(numNodes), _hopRadius(hopRadius), _numServersPerNetworkNode(numServersPerNetworkNode),
      _maxNumArriveEvents(maxNumArriveEvents), _networkTopologyFile(networkTopologyFile),
	  _activePackets(0), _terminatedPackets(0), _totalPackets(0) {

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
    
    // Express link latency factor (can be added to params file in future)
    _expressTransitFactor = 0.4; // Express links are 2.5x faster

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
    size_t configPos = distParamsFile.rfind("params_");
    if (configPos != std::string::npos) {
        size_t trialPos = distParamsFile.find("_trial", configPos);
        if (trialPos != std::string::npos) {
            params = distParamsFile.substr(configPos + 7, trialPos - (configPos + 7));
        }
    }
    std::string table_filename = "ITL_table_PDDA_Irregular_NoC_network_size_" +
                                std::to_string(_numNodes) + "_hops_" +
                                std::to_string(_hopRadius) + "_params_" +
                                params + "_" + _networkTopologyFile + ".csv";
    Init_PDDA(table_filename);
	
	_packets.resize(_numNodes * _maxNumArriveEvents);
}

void Irregular_NoC::BuildModel() {
    // First, set up the network topology
    if (!_networkTopologyFile.empty()) {
        LoadNetworkTopology(_networkTopologyFile);
    } else {
        GenerateNetworkTopology();
    }
    
    CreateVertices();
    AddVertices();
    CreateEdges();
    IO_SVs();
    InitEvents();
}

void Irregular_NoC::CreateVertices() {
    // Initialize SVs
    int init_packet_queue_val = -1 * _numServersPerNetworkNode;

    for (size_t i = 0; i < _numNodes; i++) {
        std::string name = "Packet Queue " + std::to_string(i);
        _packetQueueSVs.emplace_back(name, init_packet_queue_val,
                                    init_packet_queue_val - 1,
                                    std::numeric_limits<int>::max());
    }
    _packetQueues.resize(_numNodes);
    _queueSizes.resize(_numNodes, init_packet_queue_val);

    // Create vertices for each node in the network
    for (size_t nodeID = 0; nodeID < _numNodes; nodeID++) {
        // Determine if this is a slow node (1/6 probability)
        bool slow_node = false;
        int slow_node_RN = UniformIntDist(1, 6, nodeID).GenRV();
        if (1 == slow_node_RN) slow_node = true;
        
        // arrive trace file header
        std::string trace_file_heading_arrive = "ts, Q" + std::to_string(nodeID);

        // Create arrive vertex with appropriate parameters
        _arriveVertices.push_back(std::make_shared<Irregular_NoC_Arrive>(
            nodeID, _numNodes, "Arrive_" + std::to_string(nodeID),
            _distSeed, _traceFolderName, trace_file_heading_arrive,
            _maxNumArriveEvents,
            _minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime,
            _minServiceTime, _modeServiceTime, _maxServiceTime,
            _packets, _packetQueueSVs.at(nodeID),
            _packetQueues.at(nodeID), _queueSizes, slow_node));

        // for sim exec
        AppendVertex(_arriveVertices.back());

        // Build trace file header that lists all neighboring nodes
        std::string trace_file_heading_depart = "ts, ";
        for (const auto& link : _adjacencyList[nodeID]) {
            trace_file_heading_depart.append("Q" + std::to_string(link.targetNodeID) + ", ");
        }
        trace_file_heading_depart.append("Q" + std::to_string(nodeID));

        // Fixed create depart vertex - removed extraneous parameters to match the constructor
        _departVertices.push_back(std::make_shared<Irregular_NoC_Depart>(
            nodeID,                          // networkNodeID 
            _adjacencyList,                  // adjacencyList (network topology)
            _hopRadius,                      // hopRadius
            "Depart_" + std::to_string(nodeID), // vertexName
            _distSeed,                       // distSeed
            _traceFolderName,                // traceFolderName
            trace_file_heading_depart,       // traceFileHeading
            _minServiceTime,                 // minServiceTime
            _modeServiceTime,                // modeServiceTime
            _maxServiceTime,                 // maxServiceTime
            _minTransitTime,                 // minTransitTime
            _modeTransitTime,                // modeTransitTime
            _maxTransitTime,                 // maxTransitTime
            _packets,                        // packets
            _packetQueueSVs.at(nodeID),      // packetQueueSV
            _packetQueues.at(nodeID),        // packetQueue
            _queueSizes,                     // queueSizes
            slow_node                        // slowNode
        ));

        // for sim exec
        AppendVertex(_departVertices.back());
    }

    setNumVertices(Vertex::getNumVertices());
}

Irregular_NoC_NeighborInfo Irregular_NoC::GetHopNeighborStructures(size_t nodeID) {
    // Create neighbor info using the adjacency list
    Irregular_NoC_NeighborInfo info(nodeID, _adjacencyList, _hopRadius, _queueSizes);
    return info;
}

void Irregular_NoC::AddVertices() {
    // Create a map of node IDs to arrive vertices for efficient lookups
    std::unordered_map<size_t, std::shared_ptr<Irregular_NoC_Arrive>> nodeIdToArriveVertex;
    
    // First, populate the map with all arrive vertices
    for (size_t i = 0; i < _numNodes; i++) {
        nodeIdToArriveVertex[i] = _arriveVertices[i];
        
        // Connect arrive vertices to their corresponding depart vertices
        _arriveVertices[i]->AddDepartVertex(_departVertices[i]);
    }
    
    // Then process each node's neighbor info and connections
    for (size_t i = 0; i < _numNodes; i++) {
        // Generate neighbor info for this node
        Irregular_NoC_NeighborInfo neighborInfo = GetHopNeighborStructures(i);
        
        // Add neighbor info to depart vertex
        _departVertices[i]->AddNeighborInfo(neighborInfo);
        
        // Add arrive vertices map to the depart vertex
        _departVertices[i]->AddArriveVertices(nodeIdToArriveVertex);
        
        // Verify that all connections in the adjacency list have valid mappings
        for (const auto& link : _adjacencyList[i]) {
            if (nodeIdToArriveVertex.find(link.targetNodeID) == nodeIdToArriveVertex.end()) {
                std::cerr << "WARNING: No arrive vertex mapping for node " << link.targetNodeID 
                          << " referenced from node " << i << std::endl;
            }
        }
    }
    
    // Print connection info
    std::cout << "Node mappings created, " << nodeIdToArriveVertex.size() << " nodes mapped." << std::endl;
}

void Irregular_NoC::CreateEdges() {
    _edges.resize(getNumVertices());

    for (size_t nodeID = 0; nodeID < _numNodes; nodeID++) {
        size_t arriveIdx = nodeID * 2;
        size_t departIdx = nodeID * 2 + 1;

        // Arrive -> Depart (internal connection)
        _edges[arriveIdx].emplace_back(arriveIdx, departIdx, _minServiceTime);

        // Connect Depart to neighboring Arrive vertices
        for (const auto& link : _adjacencyList[nodeID]) {
            size_t targetNodeID = link.targetNodeID;
            size_t targetArriveIdx = targetNodeID * 2;
            
            // Use minimum transit time based on link type
            double minTransitTime = _minTransitTime;
            if (link.type == LinkType::EXPRESS_LINK) {
                minTransitTime *= _expressTransitFactor;
				//printf("\tadded express edge: %lu-%lu (%lf)\n", nodeID, targetNodeID, minTransitTime);
            }
			//else printf("added local edge: %lu-%lu (%lf)\n", nodeID, targetNodeID, minTransitTime);
            _edges[departIdx].emplace_back(departIdx, targetArriveIdx, minTransitTime);
        }
    }
}

void Irregular_NoC::IO_SVs() {
    for (size_t i = 0; i < _numNodes; i++) {
        _arriveVertices[i]->IO_SVs(_Is, _Os);
        _departVertices[i]->IO_SVs(_Is, _Os);
    }
}

void Irregular_NoC::InitEvents() {
    TriangularDist init_arrive_delays(_minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime, _distSeed);

    for (size_t i = 0; i < _numNodes; i++) {
        AppendInitEvent(_arriveVertices[i]->getVertexID(),
                        init_arrive_delays.GenRV(),
                        -1);
    }
}

void Irregular_NoC::LoadNetworkTopology(const std::string& topologyFile) {
    std::ifstream infile(topologyFile);
    
    if (!infile.is_open()) {
        std::cerr << "Error: Could not open topology file " << topologyFile << std::endl;
        // Fall back to generating a network
        std::cerr << "Falling back to generating a network automatically." << std::endl;
        GenerateNetworkTopology();
        return;
    }
    
    // Read number of nodes
    size_t file_num_nodes;
    infile >> file_num_nodes;
    
    if (file_num_nodes != _numNodes) {
        std::cerr << "Warning: Topology file has " << file_num_nodes 
                  << " nodes, but simulator is configured for " << _numNodes 
                  << " nodes. Using topology file node count." << std::endl;
        // Adjust node count
        // (This might require additional adjustments to other data structures)
    }
    
    // Initialize adjacency list
    _adjacencyList.clear();
    _adjacencyList.resize(file_num_nodes);
    
    // Read each node's connections
    for (size_t i = 0; i < file_num_nodes; i++) {
        size_t node_id, num_links;
        infile >> node_id >> num_links;
        
        if (node_id >= file_num_nodes) {
            std::cerr << "Error: Invalid node ID " << node_id << " in topology file" << std::endl;
            continue;
        }
        
        for (size_t j = 0; j < num_links; j++) {
            size_t target;
            int link_type_int;
            double latency_factor, bandwidth_factor;
            
            infile >> target >> link_type_int >> latency_factor >> bandwidth_factor;
            
            if (target >= file_num_nodes) {
                std::cerr << "Error: Invalid target node ID " << target << " in topology file" << std::endl;
                continue;
            }
            
            // Convert link type from int to enum
            LinkType link_type = (link_type_int == 0) ? LinkType::MESH_LINK : LinkType::EXPRESS_LINK;
            
            // Add the link to adjacency list
            NetworkLink link;
            link.targetNodeID = target;
            link.type = link_type;
            link.latencyFactor = latency_factor;
            link.bandwidthFactor = bandwidth_factor;
            
            _adjacencyList[node_id].push_back(link);
        }
    }
    
    // Verify that all nodes have at least one connection
    for (size_t i = 0; i < _adjacencyList.size(); i++) {
        if (_adjacencyList[i].empty()) {
            std::cerr << "Warning: Node " << i << " has no connections in the topology file" << std::endl;
        }
    }
    
    std::cout << "Network topology loaded from " << topologyFile << std::endl;
    
    // Calculate and print statistics
    size_t total_links = 0;
    size_t express_links = 0;
    
    for (const auto& links : _adjacencyList) {
        total_links += links.size();
        for (const auto& link : links) {
            if (link.type == LinkType::EXPRESS_LINK) {
                express_links++;
            }
        }
    }
    
    std::cout << "Network Statistics:" << std::endl;
    std::cout << "  Nodes: " << _adjacencyList.size() << std::endl;
    std::cout << "  Total links: " << total_links << std::endl;
    std::cout << "  Average node degree: " << static_cast<double>(total_links) / _adjacencyList.size() << std::endl;
    std::cout << "  Express links: " << express_links << " ("
              << (express_links * 100.0 / total_links) << "%)" << std::endl;
}

void Irregular_NoC::GenerateNetworkTopology() {
    // Initialize adjacency list
    _adjacencyList.resize(_numNodes);
    
    // Create a base 2D grid structure for logical organization
    size_t gridSize = static_cast<size_t>(ceil(sqrt(_numNodes)));
    
    // Random number generator with distSeed for reproducibility
    std::mt19937 rng(_distSeed);
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::uniform_int_distribution<size_t> node_dist(0, _numNodes - 1);
    
    // Create local mesh connections (not fully connected)
    for (size_t nodeID = 0; nodeID < _numNodes; nodeID++) {
        // Calculate 2D coordinates (for placement only)
        size_t x = nodeID % gridSize;
        size_t y = nodeID / gridSize;
        
        // Add mesh connections with high probability
        const double MESH_CONNECTION_PROB = 0.85;  // 85% chance of having a connection
        
        // Connect to east neighbor if it exists and within bounds
        if (x + 1 < gridSize) {
            size_t eastID = nodeID + 1;
            // Ensure the neighbor is a valid node (within total node count)
            if (eastID < _numNodes && dist(rng) < MESH_CONNECTION_PROB) {
                NetworkLink eastLink;
                eastLink.targetNodeID = eastID;
                eastLink.type = LinkType::MESH_LINK;
                eastLink.latencyFactor = 1.0;
                eastLink.bandwidthFactor = 1.0;
                _adjacencyList[nodeID].push_back(eastLink);
                
                // Add reverse link for bidirectional connectivity
                NetworkLink westLink;
                westLink.targetNodeID = nodeID;
                westLink.type = LinkType::MESH_LINK;
                westLink.latencyFactor = 1.0;
                westLink.bandwidthFactor = 1.0;
                _adjacencyList[eastID].push_back(westLink);
            }
        }
        
        // Connect to south neighbor if it exists and within bounds
        if (y + 1 < gridSize) {
            size_t southID = nodeID + gridSize;
            // Ensure the neighbor is a valid node (within total node count)
            if (southID < _numNodes && dist(rng) < MESH_CONNECTION_PROB) {
                NetworkLink southLink;
                southLink.targetNodeID = southID;
                southLink.type = LinkType::MESH_LINK;
                southLink.latencyFactor = 1.0;
                southLink.bandwidthFactor = 1.0;
                _adjacencyList[nodeID].push_back(southLink);
                
                // Add reverse link for bidirectional connectivity
                NetworkLink northLink;
                northLink.targetNodeID = nodeID;
                northLink.type = LinkType::MESH_LINK;
                northLink.latencyFactor = 1.0;
                northLink.bandwidthFactor = 1.0;
                _adjacencyList[southID].push_back(northLink);
            }
        }
    }
    
    // Add express links (long-range connections)
    // Express links typically have lower latency (faster transit)
    const double EXPRESS_LINK_DENSITY = 0.15; // 15% of nodes get express links
    const double EXPRESS_LATENCY_FACTOR = 0.4; // Express links are 2.5x faster
    const double EXPRESS_BANDWIDTH_FACTOR = 2.0; // Express links have 2x bandwidth
    
    // Number of express links to add
    size_t numExpressLinks = static_cast<size_t>(_numNodes * EXPRESS_LINK_DENSITY);
    
    // Add express links following a deterministic pattern for reproducibility
    for (size_t i = 0; i < numExpressLinks; i++) {
        // Pick nodes for express links in a distributed pattern
        // CRITICAL FIX: Ensure source node is within valid range 
        size_t source = (i * 7) % _numNodes;
        
        // CRITICAL FIX: Ensure destination node is within valid range
        // Use a different pattern to ensure good distribution while keeping within bounds
        size_t offset = (_numNodes / 4 + i * 5) % (_numNodes - 1) + 1;
        size_t dest = (source + offset) % _numNodes;
        
        // Ensure different nodes
        if (source == dest) continue;
        
        // Calculate coordinates for distance check
        size_t source_x = source % gridSize;
        size_t source_y = source / gridSize;
        size_t dest_x = dest % gridSize;
        size_t dest_y = dest / gridSize;
        size_t manhattan_dist = std::abs(static_cast<int>(source_x) - static_cast<int>(dest_x)) + 
                              std::abs(static_cast<int>(source_y) - static_cast<int>(dest_y));
        
        // Only create express links for nodes that are far apart (at least 3 hops away)
        if (manhattan_dist >= 3) {
            // Check if link already exists
            bool linkExists = false;
            for (const auto& link : _adjacencyList[source]) {
                if (link.targetNodeID == dest) {
                    linkExists = true;
                    break;
                }
            }
            
            if (!linkExists) {
                // Create express link
                NetworkLink expressLink;
                expressLink.targetNodeID = dest;
                expressLink.type = LinkType::EXPRESS_LINK;
                expressLink.latencyFactor = EXPRESS_LATENCY_FACTOR;
                expressLink.bandwidthFactor = EXPRESS_BANDWIDTH_FACTOR;
                _adjacencyList[source].push_back(expressLink);
                
                // Create reverse express link for bidirectional connection
                NetworkLink reverseExpressLink;
                reverseExpressLink.targetNodeID = source;
                reverseExpressLink.type = LinkType::EXPRESS_LINK;
                reverseExpressLink.latencyFactor = EXPRESS_LATENCY_FACTOR;
                reverseExpressLink.bandwidthFactor = EXPRESS_BANDWIDTH_FACTOR;
                _adjacencyList[dest].push_back(reverseExpressLink);
            }
        }
    }
    
    // Ensure network is fully connected (connect any isolated nodes)
    std::vector<bool> reachable(_numNodes, false);
    std::queue<size_t> bfsQueue;
    
    // Start BFS from node 0
    bfsQueue.push(0);
    reachable[0] = true;
    
    while (!bfsQueue.empty()) {
        size_t current = bfsQueue.front();
        bfsQueue.pop();
        
        for (const auto& link : _adjacencyList[current]) {
            size_t neighbor = link.targetNodeID;
            if (!reachable[neighbor]) {
                reachable[neighbor] = true;
                bfsQueue.push(neighbor);
            }
        }
    }
    
    // Check if all nodes are reachable
    for (size_t i = 0; i < _numNodes; i++) {
        if (!reachable[i]) {
            // Connect to a random reachable node
            // CRITICAL FIX: Use a proper random generator with valid range
            size_t connectTo = 0;
            do {
                connectTo = node_dist(rng);
            } while (connectTo == i || !reachable[connectTo]);
            
            // Add mesh connection
            NetworkLink newLink;
            newLink.targetNodeID = connectTo;
            newLink.type = LinkType::MESH_LINK;
            newLink.latencyFactor = 1.0;
            newLink.bandwidthFactor = 1.0;
            _adjacencyList[i].push_back(newLink);
            
            // Add reverse link
            NetworkLink reverseLink;
            reverseLink.targetNodeID = i;
            reverseLink.type = LinkType::MESH_LINK;
            reverseLink.latencyFactor = 1.0;
            reverseLink.bandwidthFactor = 1.0;
            _adjacencyList[connectTo].push_back(reverseLink);
            
            // Update reachability
            reachable[i] = true;
        }
    }
    
    // Verify all node IDs are valid
    for (size_t i = 0; i < _numNodes; i++) {
        for (auto& link : _adjacencyList[i]) {
            if (link.targetNodeID >= _numNodes) {
                std::cerr << "WARNING: Invalid node ID " << link.targetNodeID 
                          << " in topology. Fixing it." << std::endl;
                // Fix by connecting to a valid node instead
                link.targetNodeID = node_dist(rng);
            }
        }
    }
    
    // Log network topology info
    std::cout << "Generated irregular NoC with " << _numNodes << " nodes\n";
    
    // Calculate average node degree
    size_t totalLinks = 0;
    for (const auto& links : _adjacencyList) {
        totalLinks += links.size();
    }
    std::cout << "Average node degree: " << static_cast<double>(totalLinks) / _numNodes << "\n";
    
    // Count express links
    size_t expressCount = 0;
    for (const auto& links : _adjacencyList) {
        for (const auto& link : links) {
            if (link.type == LinkType::EXPRESS_LINK) {
                expressCount++;
            }
        }
    }
    std::cout << "Number of express links: " << expressCount / 2 << " (bidirectional pairs)\n";
}

void Irregular_NoC::PrintNetworkTopology() const {
    std::cout << "Irregular NoC with " << _numNodes << " nodes\n";
    
    // Calculate average node degree
    size_t totalLinks = 0;
    for (const auto& links : _adjacencyList) {
        totalLinks += links.size();
    }
    std::cout << "Average node degree: " << static_cast<double>(totalLinks) / _numNodes << "\n";
    
    // Count express links
    size_t expressCount = 0;
    for (const auto& links : _adjacencyList) {
        for (const auto& link : links) {
            if (link.type == LinkType::EXPRESS_LINK) {
                expressCount++;
            }
        }
    }
    std::cout << "Number of express links: " << expressCount / 2 << " (bidirectional pairs)\n";
}

void Irregular_NoC::PrintMeanPacketNetworkTime() const {
    double total_network_time = 0;
    size_t completed_packets = 0;
    
    for (const auto& packet : _packets) {
        if (packet.GetTimeInNetwork() > 0) { // Only count completed packets
            total_network_time += packet.GetTimeInNetwork();
            completed_packets++;
        }
    }
    
    if (completed_packets > 0) {
        printf("Mean packet network time: %.*lf (across %zu completed packets)\n", 
               DBL_DIG, total_network_time / completed_packets, completed_packets);
    } else {
        printf("No completed packets to calculate mean network time\n");
    }
}

void Irregular_NoC::PrintSVs() const {
    printf("\nQueue sizes for all nodes:\n");
    for (size_t i = 0; i < _numNodes; i++) {
        printf("Node %zu: %d\n", i, _packetQueueSVs[i].get());
    }
    printf("\n");
}

void Irregular_NoC::PrintNumVertexExecs() const {
    printf("\nExecution counts for all nodes:\n");
    for (size_t i = 0; i < _numNodes; i++) {
        printf("Node %zu: (Arrive: %zu, Depart: %zu)\n", 
               i, _arriveVertices[i]->getNumExecs(), _departVertices[i]->getNumExecs());
    }
    printf("\n");
}

void Irregular_NoC::PrintFinishedPackets() const {
    printf("\nCompleted packets:\n");
    size_t printLimit = 20; // Limit to first 20 packets for brevity
    size_t printed = 0;
    
    for (const auto& packet : _packets) {
        if (packet.GetTimeInNetwork() > 0 && printed < printLimit) { // Only print completed packets
            packet.PrintData();
            printed++;
        }
    }
    
    if (printed == printLimit) {
        printf("... (showing only first %zu completed packets)\n", printLimit);
    }
    printf("\n");
}

// Add these implementations to your Irregular_NoC.cpp file
// Implementation of packet tracking methods
void Irregular_NoC::PacketGenerated() {
    _activePackets++;
}

void Irregular_NoC::PacketTerminated() {
    _activePackets--;
    _terminatedPackets++;
    
    // Log termination milestones
    if (_terminatedPackets % 100 == 0 || _terminatedPackets >= _totalPackets - 10) {
        std::cout << "Packets completed: " << _terminatedPackets 
                  << " / " << _totalPackets 
                  << " (Active: " << _activePackets << ")" << std::endl;
    }
}
