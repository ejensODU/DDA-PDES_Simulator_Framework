#include "PHOLD_Model.h"
#include "../Dist.h"
#include <iostream>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

PHOLD_Model::PHOLD_Model(size_t networkSize,
                       size_t maxEventsPerNode,
                       double maxSimTime,
                       size_t numThreads, 
                       size_t distSeed,
                       double bucketWidth, 
                       size_t targetBinSize,
                       std::string traceFolderName,
                       std::string distParamsFile)
    : PDDA_SimModel(maxSimTime, numThreads, distSeed, bucketWidth, targetBinSize, traceFolderName),
      _networkSize(networkSize),
      _maxEventsPerNode(maxEventsPerNode) {

    // Read distribution parameters
    std::cout << "dist params file name: " << distParamsFile << std::endl;
    std::ifstream dist_params_file(distParamsFile);
    if (!dist_params_file) {
        throw std::runtime_error("Required input parameter file " + distParamsFile + " not found");
    }
    
    // In PHOLD, we only need one delay distribution parameter set
    dist_params_file >> _minDelay >> _modeDelay >> _maxDelay;
    std::cout << "Event Delay Times - Min: " << _minDelay
              << ", Mode: " << _modeDelay
              << ", Max: " << _maxDelay << std::endl;

    // Create trace folder if needed
    if (!_traceFolderName.empty() && !fs::exists(_traceFolderName)) {
        if (fs::create_directory(_traceFolderName)) {
            std::cout << "Folder '" << _traceFolderName << "' created successfully.\n";
        } else {
            std::cerr << "Error: Failed to create folder '" << _traceFolderName << "'.\n";
        }
    }

    // Build the model
    BuildModel();
    
    // Initialize PDDA
    std::string params;
    size_t configPos = distParamsFile.rfind("params_");
    if (configPos != std::string::npos) {
        size_t trialPos = distParamsFile.find("_trial", configPos);
        if (trialPos != std::string::npos) {
            params = distParamsFile.substr(configPos + 7, trialPos - (configPos + 7));
        }
    }
    std::string table_filename = "ITL_table_PDDA_PHOLD_network_size_" +
    std::to_string(_networkSize) + "_params_" + params + ".csv";
    Init_PDDA(table_filename);
}

void PHOLD_Model::BuildModel() {
    CreateVertices();
    
    // After creating all vertices, set the arrive vertices in each vertex
    for (auto& vertex : _arriveVertices) {
        vertex->SetArriveVertices(_arriveVertices);
    }
    
    CreateEdges();
    IO_SVs();
    InitEvents();
}

void PHOLD_Model::CreateVertices() {
    // Initialize SVs to track event counts at each node
    _eventCountSVs.reserve(_networkSize);
    for (size_t i = 0; i < _networkSize; i++) {
        std::string name = "Event Counter " + std::to_string(i);
        _eventCountSVs.emplace_back(name, 0, 0, _maxEventsPerNode);
    }

    // Create vertices for each node in the network
    _arriveVertices.reserve(_networkSize);
    for (size_t pos = 0; pos < _networkSize; pos++) {
        std::string trace_file_heading = "ts, EventCount" + std::to_string(pos);

        // Create arrive vertex
        _arriveVertices.push_back(std::make_shared<PHOLD_Arrive>(
            pos, _networkSize,
            "PHOLD_" + std::to_string(pos),
            _distSeed, _traceFolderName, trace_file_heading,
            _maxEventsPerNode,
            _minDelay, _modeDelay, _maxDelay,
            _eventCountSVs.at(pos)));

        // Add vertex to simulation execution
        AppendVertex(_arriveVertices.back());
    }

    setNumVertices(Vertex::getNumVertices());
}

void PHOLD_Model::CreateEdges() {
    _edges.resize(getNumVertices());

    // Create fully connected network with equal lookahead (min delay)
    for (size_t i = 0; i < _networkSize; i++) {
        for (size_t j = 0; j < _networkSize; j++) {
            _edges[i].emplace_back(i, j, _minDelay);
        }
    }
}

void PHOLD_Model::IO_SVs() {
    for (size_t i = 0; i < _networkSize; i++) {
        _arriveVertices[i]->IO_SVs(_Is, _Os);
    }
}

void PHOLD_Model::InitEvents() {
    // Initialize with one event per node
    TriangularDist init_delays(_minDelay, _modeDelay, _maxDelay, _distSeed);

    for (size_t i = 0; i < _networkSize; i++) {
        AppendInitEvent(_arriveVertices[i]->getVertexID(),
                        init_delays.GenRV(),
                        -1);  // Using -1 as a placeholder ID, not a packet now
    }
}

void PHOLD_Model::PrintSVs() const {
    printf("\nEvent Counters:\n");
    for (size_t i = 0; i < _networkSize; i++) {
        printf("%d ", _eventCountSVs[i].get());
        if ((i + 1) & 31 == 0) printf("\n");  // Line break every 32 nodes
    }
    printf("\n");
}

void PHOLD_Model::PrintNumVertexExecs() const {
    printf("\nVertex Executions:\n");
    for (size_t i = 0; i < _networkSize; i++) {
        printf("(%lu) ", _arriveVertices[i]->getNumExecs());
        if ((i + 1) & 31 == 0) printf("\n");  // Line break every 32 nodes
    }
    printf("\n");
}