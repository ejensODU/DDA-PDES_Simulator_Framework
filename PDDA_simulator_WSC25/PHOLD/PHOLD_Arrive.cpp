#include "PHOLD_Arrive.h"
#include "../Dist.h"
#include "../PDDA_SimExec.h"

PHOLD_Arrive::PHOLD_Arrive(size_t nodeID,
                         size_t networkSize,
                         const std::string& vertexName, 
                         size_t distSeed,
                         std::string traceFolderName, 
                         std::string traceFileHeading,
                         size_t maxEventsPerNode,
                         double minDelay, 
                         double modeDelay, 
                         double maxDelay,
                         PDDA_SV<int>& eventCountSV)
    : Vertex(vertexName, nodeID, distSeed, traceFolderName, traceFileHeading),
      _nodeID(nodeID),
      _networkSize(networkSize),
      _maxEventsPerNode(maxEventsPerNode),
      _eventCountSV(eventCountSV),
      _canScheduleMoreEvents(true) {

    // Initialize distribution for random node selection
    _randomNodeSelector = std::make_unique<UniformIntDist>(0, _networkSize - 1, distSeed + nodeID);
    
    // Initialize distribution for event delay times
    _delayDistribution = std::make_unique<TriangularDist>(minDelay, modeDelay, maxDelay, distSeed + nodeID);
}

size_t PHOLD_Arrive::getNodeID() const {
    return _nodeID;
}

void PHOLD_Arrive::SetArriveVertices(const std::vector<std::shared_ptr<PHOLD_Arrive>>& vertices) {
    _arriveVertices = vertices;
}

void PHOLD_Arrive::IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) {
    // Register this vertex's state variable for both input and output
    std::vector<size_t> sv_indices;
    sv_indices.push_back(_eventCountSV.getModelIndex());
    
    Is.push_back(sv_indices);
    Os.push_back(sv_indices);
}

void PHOLD_Arrive::Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) {
    // Check if we can execute more events at this node
    if (_eventCountSV.get() < _maxEventsPerNode - 1) {

		// Update event counter
		_eventCountSV.inc();
		
		// Select a random node to send the next event to
		size_t targetNodeID = _randomNodeSelector->GenRV();
		
		// Schedule a new arrival event
		double delay = _delayDistribution->GenRV();
		CQ.AddEvent(_arriveVertices[targetNodeID]->getVertexID(), simTime + delay, -1);
		
		// Trace execution if enabled
		if (!_traceFolderName.empty()) {
			std::string trace_string = std::to_string(simTime) + ", " + std::to_string(_eventCountSV.get());
			WriteToTrace(trace_string);
		}
		
		// Increment execution counter
		_numExecutions++;
	}
}