#pragma once

#include <memory>
#include <vector>
#include "../Vertex.h"
#include "../PDDA_SV.h"

class PHOLD_Arrive : public Vertex, public std::enable_shared_from_this<PHOLD_Arrive> {
public:
    PHOLD_Arrive(size_t nodeID,
                size_t networkSize,
                const std::string& vertexName, 
                size_t distSeed,
                std::string traceFolderName, 
                std::string traceFileHeading,
                size_t maxEventsPerNode,
                double minDelay, 
                double modeDelay, 
                double maxDelay,
                PDDA_SV<int>& eventCountSV);

    size_t getNodeID() const;
    void SetArriveVertices(const std::vector<std::shared_ptr<PHOLD_Arrive>>& vertices);
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) override;
    virtual void Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) override;

private:
    // 8-byte aligned members first
    std::unique_ptr<class UniformIntDist> _randomNodeSelector;  // 8 bytes
    std::unique_ptr<class TriangularDist> _delayDistribution;   // 8 bytes
    std::vector<std::shared_ptr<PHOLD_Arrive>> _arriveVertices; // 24 bytes (vector)
    PDDA_SV<int>& _eventCountSV;                                // 8 bytes (reference)
    
    // 8-byte size_t members
    const size_t _nodeID;                                      // 8 bytes
    const size_t _networkSize;                                 // 8 bytes
    const size_t _maxEventsPerNode;                            // 8 bytes
    
    // 1-byte bool
    bool _canScheduleMoreEvents;                               // 1 byte
};