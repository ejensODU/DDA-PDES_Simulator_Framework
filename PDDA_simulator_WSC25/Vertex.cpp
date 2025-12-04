#include "Vertex.h"
#include "Dist.h"

#include <iostream>
#include <fstream>
#include <cmath>

size_t Vertex::_numVertices = 0;

Vertex::Vertex(std::string vertexName, int extraWork, size_t distSeed, std::string traceFolderName, std::string traceFileHeading)
: _vertexID(_numVertices++), _vertexName(vertexName), _extraWork(extraWork), _distSeed(distSeed + _vertexID), _traceFolderName(traceFolderName), _numExecutions(0)
{
	if (!_traceFolderName.empty()) {
		std::string file_vertex_name = _vertexName;
		std::replace(file_vertex_name.begin(), file_vertex_name.end(), ' ', '_');
		_traceFilePath = _traceFolderName + "/" + file_vertex_name + ".txt";
		_traceFile.open(_traceFilePath);
		if (_traceFile.is_open()) {
			_traceFile << traceFileHeading << std::endl;
		} else {
			std::cerr << "Error opening trace file! Path: " << _traceFilePath << std::endl;
		}
		_traceFile.close();
	}

    /* if (_extraWork < 0) {
        if (-1 == _extraWork) _workDist = std::make_unique<ExpoDist>(2, _distSeed);
        if (-2 == _extraWork) _workDist = std::make_unique<ExpoDist>(1, _distSeed);
        if (-3 == _extraWork) _workDist = std::make_unique<ExpoDist>(0.8, _distSeed);
        if (-4 == _extraWork) _workDist = std::make_unique<ExpoDist>(0.6, _distSeed);
        if (-5 == _extraWork) _workDist = std::make_unique<ExpoDist>(0.5, _distSeed);
        if (-6 == _extraWork) _workDist = std::make_unique<ExpoDist>(0.4, _distSeed);
    } */
}

size_t Vertex::_runID = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();


/* size_t Vertex::ExtraWork()
{
    int num_primes = 0;
    if (_extraWork > 0) num_primes = int(std::pow(2, _extraWork));
    else if (_extraWork < 0) {
		_expoRV = ceil(_workDist->GenRV());
		if (_expoRV > 20) {
			printf("expo rv: %lf\n", _expoRV);
			_expoRV = 20;
		}
		num_primes = int(std::pow(2, _expoRV));
	}
    
    if (num_primes < 1) return 0; // Handle edge case

    std::vector<int> primes;
    primes.reserve(num_primes); // Reserve space to avoid reallocations

    if (num_primes >= 1) primes.push_back(2);
    if (num_primes >= 2) primes.push_back(3);

    int ind = 2;
    for (int prime_cand = 5; ind < num_primes; prime_cand += 2) {
        bool is_prime = true;
        for (int i = 1; i < ind && primes[i] * primes[i] <= prime_cand; ++i) {
            if (prime_cand % primes[i] == 0) {
                is_prime = false;
                break;
            }
        }
        if (is_prime) {
            primes.push_back(prime_cand);
            ind++;
        }
    }
    return primes.back(); // Return the last prime found
} */


void Vertex::WriteToTrace(std::string traceSnapshot)
{
	if (!_traceFolderName.empty()) {
		_traceFile.open(_traceFilePath, std::ios::app);
		_traceFile << traceSnapshot << std::endl;
		_traceFile.close();
	}
}



Edge::Edge(size_t origVertexID, size_t termVertexID, const float& minDist)
:_origVertexID(origVertexID), _termVertexID(termVertexID), _minDist(minDist)
{}
