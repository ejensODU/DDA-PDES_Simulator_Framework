#pragma once

#include "Vertex.h"
#include "PDDA_SimExec.h"
#include "PDDA_SV.h"

class TriangularDist;
class UniformIntDist;

class PDDA_SimModel {
public:
    PDDA_SimModel(double maxSimTime, size_t numThreads, size_t distSeed, double bucketWidth, size_t targetBinSize, std::string traceFolderName);
    void AppendVertex(std::shared_ptr<Vertex> vertex);
    void AppendInitEvent(int vertexID, double time, int entityID);
    void setNumVertices(size_t numVertices);
    size_t getNumVertices();
    void SimulateModel();
    virtual void PrintSVs() const = 0;
    virtual void PrintNumVertexExecs() const = 0;
    void Init_PDDA(std::string tableFilename);
protected:
    std::vector<std::vector<size_t>> _Is;
    std::vector<std::vector<size_t>> _Os;
    std::vector<std::vector<Edge>> _edges;
	const std::string _traceFolderName;
    const size_t _distSeed;
private:
	void WriteITLTableToCSV(std::vector<std::vector<float>>& ITL, std::string tableFilename) const;
    std::vector<std::vector<float>> FloydWarshall();
    std::vector<std::vector<float>> MakeITL(std::string tableFilename);
    std::vector<std::vector<float>> ReadITLTableFromCSV(const std::string& tableFilename) const;
    std::vector<std::shared_ptr<Vertex>> _allVertices;
	std::list<PDDA_Event> _initEvents;
    std::unique_ptr<PDDA_SimExec> _simExec;
    const double _maxSimTime;
	const double _bucketWidth;
    const size_t _numThreads;
    const size_t _targetBinSize;
    size_t _numVertices;
    bool _useGPU;  // Flag to indicate if GPU is available and should be used
};
