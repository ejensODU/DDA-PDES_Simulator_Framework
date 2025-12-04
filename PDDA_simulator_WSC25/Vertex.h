#pragma once

#include <vector>
#include <list>
#include <memory>
#include <string>
#include <fstream>

class PDDA_CalendarQueue;
#include <chrono>

class Edge;
class Entity;
class ExpoDist;

class Vertex {
public:
    Vertex(std::string vertexName, int extraWork, size_t distSeed, std::string traceFolderName, std::string traceFileHeading);
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os) = 0;
    virtual void Run(PDDA_CalendarQueue& CQ, double simTime, int entityID) = 0;
    //size_t ExtraWork();
    static size_t getNumVertices()  { return _numVertices; }
    size_t getVertexID() const  { return _vertexID; }
    //std::vector<Edge> const getEdges();
    std::string getVertexName() const  { return _vertexName; }
    size_t getNumExecs() const  { return _numExecutions; }
    void WriteToTrace(std::string traceSnapshot);
protected:
    static size_t _numVertices;          // static - doesn't count
    const size_t _vertexID;              // 8 bytes
    const size_t _distSeed;              // 8 bytes
    size_t _numExecutions;               // 8 bytes
    const std::string _vertexName;       // ~24-32 bytes
    const std::string _traceFolderName;  // ~24-32 bytes
    std::string _traceFilePath;          // ~24-32 bytes
    const int _extraWork;                // 4 bytes
private:
	std::ofstream _traceFile;            // typically 8 bytes
    std::unique_ptr<ExpoDist> _workDist; // 8 bytes
    double _expoRV;                      // 8 bytes
    static size_t _runID;                // static - doesn't count
};


class Edge {
public:
    Edge(size_t origVertexID, size_t termVertexID, const float& minDist);
    size_t getOrigVertexID() const  { return _origVertexID; }
    size_t getTermVertexID() const  { return _termVertexID; }
    float getMinDist() const  { return _minDist; }
private:
    const size_t _origVertexID;
    const size_t _termVertexID;
    const float _minDist;
};
