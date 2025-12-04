#include "Ring_1D.h"
#include "../Dist.h"
#include <iostream>
#include <thread>
#include <filesystem>
#include <fstream>
#include <regex>
#include <float.h>

namespace fs = std::filesystem;

Ring_1D::Ring_1D(size_t ringSize,
                 size_t numServersPerNetworkNode,
                 size_t maxNumArriveEvents, double maxSimTime,
                 size_t numThreads, size_t distSeed,
                 double bucketWidth, size_t targetBinSize,
                 std::string traceFolderName,
                 std::string distParamsFile)
	: PDDA_SimModel(maxSimTime, numThreads, distSeed, bucketWidth, targetBinSize, traceFolderName),
	_ringSize(ringSize),
	_numServersPerNetworkNode(numServersPerNetworkNode),
	_maxNumArriveEvents(maxNumArriveEvents) {

    // Read distribution parameters
    std::cout << "dist params file name: " << distParamsFile << std::endl;
    std::ifstream dist_params_file(distParamsFile);
    if (!dist_params_file) {
        throw std::runtime_error("Required input parameter file " + distParamsFile + " not found");
    }
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
    std::string table_filename = "ITL_table_PDDA_Ring_1D_network_size_" +
    std::to_string(_ringSize) + "_params_" + params + ".csv";
    Init_PDDA(table_filename);

    _packets.resize(_ringSize*_maxNumArriveEvents);
}

void Ring_1D::BuildModel() {
    CreateVertices();
    AddVertices();
    CreateEdges();
    IO_SVs();
    InitEvents();
}

void Ring_1D::CreateVertices() {
    // Initialize SVs
    int init_packet_queue_val = -1 * _numServersPerNetworkNode;

    for (size_t i = 0; i < _ringSize; i++) {
        std::string name = "Packet Queue " + std::to_string(i);
        _packetQueueSVs.emplace_back(name, init_packet_queue_val,
                                     init_packet_queue_val - 1,
                                     std::numeric_limits<int>::max());
    }
    _packetQueues.resize(_ringSize);

    // Create vertices for each node in the ring
    for (size_t pos = 0; pos < _ringSize; pos++) {
        std::string trace_file_heading = "ts, Q" + std::to_string(pos);

        // Create arrive vertex
        _arriveVertices.push_back(std::make_shared<Ring_1D_Arrive>(
            pos, _ringSize,
            "Arrive_" + std::to_string(pos),
            _distSeed, _traceFolderName, trace_file_heading,
            _maxNumArriveEvents,
            _minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime,
            _minServiceTime, _modeServiceTime, _maxServiceTime,
            _packets, _packetQueueSVs.at(pos),
            _packetQueues.at(pos)));

        // for sim exec
        AppendVertex(_arriveVertices.back());

        // Create depart vertex
        _departVertices.push_back(std::make_shared<Ring_1D_Depart>(
            pos,
            _ringSize,
            "Depart_" + std::to_string(pos),
            _distSeed, _traceFolderName, trace_file_heading,
            _minServiceTime, _modeServiceTime, _maxServiceTime,
            _minTransitTime, _modeTransitTime, _maxTransitTime,
            _packets, _packetQueueSVs.at(pos),
            _packetQueues.at(pos)));

        // for sim exec
        AppendVertex(_departVertices.back());
    }

    setNumVertices(Vertex::getNumVertices());
}

void Ring_1D::AddVertices() {
    for (size_t pos = 0; pos < _ringSize; pos++) {
        // Connect arrive vertices to their corresponding depart vertices
        _arriveVertices[pos]->AddDepartVertex(_departVertices[pos]);

        // Set up neighbors (clockwise and counterclockwise)
        std::vector<std::shared_ptr<Ring_1D_Arrive>> arrive_vertices(2);

        size_t clockwise_pos = WrapPosition(pos + 1);      // Clockwise neighbor
        size_t counter_pos = WrapPosition(pos + _ringSize - 1);  // Counterclockwise neighbor

        arrive_vertices[0] = _arriveVertices[clockwise_pos];   // Clockwise
        arrive_vertices[1] = _arriveVertices[counter_pos];     // Counterclockwise

        _departVertices[pos]->AddArriveVertices(arrive_vertices);
    }
}

void Ring_1D::CreateEdges() {
    _edges.resize(getNumVertices());

    for (size_t pos = 0; pos < _ringSize; pos++) {
        size_t arriveIdx = GetVertexID(pos, 0);
        size_t departIdx = GetVertexID(pos, 1);

        // Arrive -> Depart
        _edges[arriveIdx].emplace_back(arriveIdx, departIdx, _minServiceTime);

        // Connect Depart to neighboring Arrive vertices with wrapping
        size_t clockwise_pos = WrapPosition(pos + 1);      // Clockwise neighbor
        size_t counter_pos = WrapPosition(pos + _ringSize - 1);  // Counterclockwise neighbor

        _edges[departIdx].emplace_back(departIdx, GetVertexID(clockwise_pos, 0), _minTransitTime);   // Clockwise
        _edges[departIdx].emplace_back(departIdx, GetVertexID(counter_pos, 0), _minTransitTime);     // Counterclockwise
    }
}

void Ring_1D::IO_SVs() {
    for (size_t i = 0; i < _ringSize; i++) {
        _arriveVertices[i]->IO_SVs(_Is, _Os);
        _departVertices[i]->IO_SVs(_Is, _Os);
    }
}

void Ring_1D::InitEvents() {
    TriangularDist init_arrive_delays(_minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime, _distSeed);

    for (size_t i = 0; i < _ringSize; i++) {
        AppendInitEvent(_arriveVertices[i]->getVertexID(),
                        init_arrive_delays.GenRV(),
                        -1);
    }
}

size_t Ring_1D::GetVertexID(size_t pos, size_t type) const {
    return pos * 2 + type;
}

void Ring_1D::PrintMeanPacketNetworkTime() const {
    double total_network_time = 0;
    for (const auto& packet : _packets) {
        total_network_time += packet.GetTimeInNetwork();
    }
    printf("Mean packet network time: %.*lf\n", DBL_DIG, total_network_time / _packets.size());
}

void Ring_1D::PrintSVs() const {
    printf("\n");
    for (size_t i = 0; i < _ringSize; i++) {
        printf("%d ", _packetQueueSVs[i].get());
    }
    printf("\n");
}

void Ring_1D::PrintNumVertexExecs() const {
    printf("\n");
    for (size_t i = 0; i < _ringSize; i++) {
        printf("(%lu %lu) ", _arriveVertices[i]->getNumExecs(),
               _departVertices[i]->getNumExecs());
    }
    printf("\n");
}

void Ring_1D::PrintFinishedPackets() const {
    printf("\n");
    for (const auto& packet : _packets) {
        packet.PrintData();
    }
}
