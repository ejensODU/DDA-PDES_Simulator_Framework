#pragma once

#include <vector>
#include <memory>
#include <string>
#include "../PDDA_SimModel.h"
#include "PHOLD_Arrive.h"

class PHOLD_Model : public PDDA_SimModel {
public:
    PHOLD_Model(size_t networkSize,
                size_t maxEventsPerNode, 
                double maxSimTime,
                size_t numThreads, 
                size_t distSeed,
                double bucketWidth, 
                size_t targetBinSize,
                std::string traceFolderName,
                std::string distParamsFile);

    virtual void PrintSVs() const override;
    virtual void PrintNumVertexExecs() const override;

private:
    // Internal methods
    void BuildModel();
    void CreateVertices();
    void CreateEdges();
    void IO_SVs();
    void InitEvents();

    // Network parameters
    const size_t _networkSize;
    const size_t _maxEventsPerNode;
    
    // Event delay times
    double _minDelay, _modeDelay, _maxDelay;
    
    // Vertices
    std::vector<std::shared_ptr<PHOLD_Arrive>> _arriveVertices;
    
    // State variables to track event counts
    std::vector<PDDA_SV<int>> _eventCountSVs;
};