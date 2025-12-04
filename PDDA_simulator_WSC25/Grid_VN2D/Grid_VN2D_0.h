#pragma once

#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <algorithm>

#include "../PDDA_SimModel.h"

class Grid_VN2D_Depart;

class Packet : public Entity {
public:
    Packet(double genTime, size_t originNetworkNodeID, size_t destNetworkNodeID, size_t destX, size_t destY, size_t gridSizeX, size_t gridSizeY);
    void AddNetworkNodeData(double arrivalTime, size_t networkNodeID);
    const std::list<size_t>& getVisitedNetworkNodes() const;
    size_t getDestNetworkNodeID() const;
    double GetTimeInNetwork() const;
    void PrintData() const override;
private:
    const size_t _originNetworkNodeID;
    const size_t _destNetworkNodeID;
    std::list<double> _networkNodeArrivalTimes;
    std::list<size_t> _visitedNetworkNodes;
    const size_t _destX;
    const size_t _destY;
    const size_t _gridSizeX;
    const size_t _gridSizeY;
    size_t _minManhattanDist;
};


// Arrive vertex class
class Grid_VN2D_Arrive : public Vertex, public std::enable_shared_from_this<Grid_VN2D_Arrive> {
public:
    Grid_VN2D_Arrive(size_t networkNodeID, size_t gridSizeX, size_t gridSizeY, const std::string& vertexName, size_t distSeed, std::string traceFolderName, std::string traceFileHeading, size_t maxNumArriveEvents, size_t minIntraArrivalTime, size_t modeIntraArrivalTime, size_t maxIntraArrivalTime, size_t minServiceTime, size_t modeServiceTime, size_t maxServiceTime, PDDA_SV<int>& packetQueueSV, std::queue<std::shared_ptr<Packet>>& packetQueue, std::list<std::shared_ptr<Packet>>& finishedPackets, std::atomic<int>& finishedPacketListLock);
    size_t getNetworkNodeID() const;
    PDDA_SV<int>& getPacketQueueSV();
    void AddDepartVertex(std::shared_ptr<Grid_VN2D_Depart> departVertex);
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os);
    virtual void Run(std::list<PDDA_Event*>& newEvents, double simTime, std::shared_ptr<Entity> entity);
private:
    const size_t _networkNodeID;
    const size_t _gridSizeX;
    const size_t _gridSizeY;
    std::unique_ptr<UniformIntDist> _randomNodeID;
    int _numIntraArriveEvents;
    const int _maxNumIntraArriveEvents;
    // conditions
    bool _atDestination;
    bool _intraArrival;
    bool _serverAvailable;
    // SVs
    PDDA_SV<int>& _packetQueueSV;
    std::queue<std::shared_ptr<Packet>>& _packetQueue;
    // vertices
    std::shared_ptr<Grid_VN2D_Depart> _departVertex;
    // delays
    std::unique_ptr<TriangularDist> _intraArrivalDelay;
    std::unique_ptr<TriangularDist> _serviceDelay;
    // finished packets
    std::list<std::shared_ptr<Packet>>& _finishedPackets;
    std::atomic<int>& _finishedPacketListLock;
};

// // Service vertex class
// class Grid_VN2DService : public Vertex {
// public:
//     Grid_VN2DService(size_t networkNodeID, const std::string& vertexName, int distSeed, std::string traceFolderName, std::string traceFileHeading, size_t minServiceTime, size_t modeServiceTime, size_t maxServiceTime);
//     void AddDepartVertex(std::shared_ptr<Depart> departVertex);
//     virtual void IO_SVs(std::vector<std::vector<int>>& Is, std::vector<std::vector<int>>& Os);
//     virtual void Run(std::list<PDDA_Event*>& newEvents, double simTime);
// private:
//     const size_t _networkNodeID;
//     // SVs
//     PDDA_SV<int>& _packetQueueSV;
//     std::queue<std::shared_ptr<Packet>>& _packetQueue;
//     // vertices
//     std::shared_ptr<Depart> _departVertex;
//     // delays
//     std::unique_ptr<TriangularDist> _serviceDelay;
// };

struct Path {
    std::vector<size_t> nodeIndices;  // Sequence of node IDs in path
    double totalCost;                 // Sum of queue sizes along path

    Path() : totalCost(std::numeric_limits<double>::max()) {}
    void PrintPath() {
        printf("%lf: ", totalCost);
        for (auto index: nodeIndices) printf("%lu,", index);
        printf("\n");
    }
};

struct NeighborInfo {
    std::vector<std::vector<size_t>> I;  // Hierarchical indices
    std::vector<std::shared_ptr<Grid_VN2D_Arrive>> N;  // Flat array of neighbor pointers
    std::unordered_map<size_t, size_t> globalToLocalIdx;  // Maps network node IDs to indices in N

    // Helper function to get grid coordinates for a node ID
    std::pair<size_t, size_t> getCoordinates(size_t nodeID, size_t gridSizeX) const {
        return {nodeID % gridSizeX, nodeID / gridSizeX};
    }

    // Helper function to get node ID from coordinates
    size_t getNodeID(size_t x, size_t y, size_t gridSizeX) const {
        return y * gridSizeX + x;
    }

    // Get actual grid neighbors for any node
    std::vector<size_t> getGridNeighbors(size_t nodeID, size_t gridSizeX, size_t gridSizeY) const {
        std::vector<size_t> neighbors;
        auto [x, y] = getCoordinates(nodeID, gridSizeX);

        // Check each direction
        if (x > 0) {  // West
            neighbors.push_back(getNodeID(x-1, y, gridSizeX));
        }
        if (x < gridSizeX - 1) {  // East
            neighbors.push_back(getNodeID(x+1, y, gridSizeX));
        }
        if (y > 0) {  // North
            neighbors.push_back(getNodeID(x, y-1, gridSizeX));
        }
        if (y < gridSizeY - 1) {  // South
            neighbors.push_back(getNodeID(x, y+1, gridSizeX));
        }

        return neighbors;
    }

    // Get valid neighbors for pathfinding
    std::vector<size_t> getValidNeighbors(size_t nodeID, size_t targetNodeID, size_t hopsFromStart, size_t gridSizeX, size_t gridSizeY) const {
        // Get all possible grid neighbors
        auto allNeighbors = getGridNeighbors(nodeID, gridSizeX, gridSizeY);
        std::vector<size_t> validNeighbors;

        //printf("debug I:\n");
        //for (size_t i=0; i<I.size(); i++) {
        //    for (size_t j=0; j<I.at(i).size(); j++) {
        //        printf("%lu,", I.at(i).at(j));
        //    }
        //    printf("\n");
        //}
        //printf("hops from start: %lu\n", hopsFromStart);
        const auto& hopLevelNeighbors = I.at(hopsFromStart);
        //printf("neighbors:");
        //for (const auto& neighbor : hopLevelNeighbors) printf("%lu,", neighbor);
        //printf("\n");

        size_t node_x = nodeID % gridSizeX;
        size_t node_y = nodeID / gridSizeX;
        size_t target_node_x = targetNodeID % gridSizeX;
        size_t target_node_y = targetNodeID / gridSizeX;
        size_t current_distance = abs(static_cast<int>(node_x) - static_cast<int>(target_node_x)) +
        abs(static_cast<int>(node_y) - static_cast<int>(target_node_y));

        // Check each neighbor to see if it's in the current hop level
        for (size_t neighborID : allNeighbors) {

            size_t neighbor_node_x = neighborID % gridSizeX;
            size_t neighbor_node_y = neighborID / gridSizeX;
            size_t neighbor_distance = abs(static_cast<int>(neighbor_node_x) - static_cast<int>(target_node_x)) +
            abs(static_cast<int>(neighbor_node_y) - static_cast<int>(target_node_y));
            //printf("neighborID: %lu, neighbor_distance: %lu, current_distance: %lu\n", neighborID, neighbor_distance, current_distance);
            if (neighbor_distance >= current_distance) continue;

            // Check if this neighbor is in the current hop level
            //printf("\tneighborID: %lu\n", neighborID);
            if (std::find(hopLevelNeighbors.begin(), hopLevelNeighbors.end(), neighborID) != hopLevelNeighbors.end()) {
            //    printf("\t\tneighborID %lu is valid\n", neighborID);
                validNeighbors.push_back(neighborID);
            }
        }
        return validNeighbors;
    }

    Path findShortestPath(size_t finalDestNodeID, size_t intermedTargetNodeID, size_t current_x, size_t current_y,
                          size_t gridSizeX, size_t gridSizeY, size_t hopRadius, const std::list<size_t>& visitedNodes) const {
                              struct QueueEntry {
                                  size_t nodeID;
                                  std::vector<size_t> path;
                                  double costSoFar;
                                  size_t hopsFromStart;

                                  bool operator>(const QueueEntry& other) const {
                                      return costSoFar > other.costSoFar;
                                  }
                              };

                              std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
                              std::unordered_map<size_t, double> bestCosts;
                              size_t currentNodeID = current_y * gridSizeX + current_x;

                              size_t FD_x = finalDestNodeID % gridSizeX;
                              size_t FD_y = finalDestNodeID / gridSizeX;

                              size_t current_distance_FD_node = abs(static_cast<int>(FD_x) - static_cast<int>(current_x)) +
                              abs(static_cast<int>(FD_y) - static_cast<int>(current_y));

                              //printf("Finding path from %lu to %lu\n", currentNodeID, targetNodeID);

                              // Start from current node
                              queue.push({currentNodeID, {currentNodeID}, 0.0, 0});
                              bestCosts[currentNodeID] = 0.0;

                              while (!queue.empty()) {
                                  auto current = queue.top();
                                  queue.pop();

                                  //printf("\nProcessing node %lu (cost: %.1f, hops: %lu)\n",
                                  //       current.nodeID, current.costSoFar, current.hopsFromStart);

                                  // If we found target, return the path
                                  if (current.nodeID == intermedTargetNodeID) {
                                      //printf("Found target! Path: ");
                                      //for (size_t id : current.path) printf("%lu ", id);
                                      //printf("\n");

                                      Path result;
                                      result.nodeIndices = current.path;
                                      result.totalCost = current.costSoFar;
                                      return result;
                                  }

                                  // Skip if we've exceeded hop limit
                                  //if (current.hopsFromStart >= I.size()) {
                                  //    continue;
                                  //}

                                  // In NeighborInfo::findShortestPath:
                                  // Change the hop limit check to use hopRadius directly
                                  if (current.hopsFromStart == hopRadius) {  // Allow up to hopRadius hops
                                      continue;
                                  }

                                  // Get valid neighbors for current node
                                  auto neighbors = getValidNeighbors(current.nodeID, intermedTargetNodeID, current.hopsFromStart, gridSizeX, gridSizeY);

                                  //printf("  Node %lu grid neighbors: ", current.nodeID);
                                  //for (size_t n : neighbors) printf("%lu ", n);
                                  //printf("\n");

                                  for (size_t neighborID : neighbors) {
                                      // Skip if already in path
                                      if (std::find(current.path.begin(), current.path.end(), neighborID) != current.path.end()) {
                                          //printf("  Skipping %lu (already in path)\n", neighborID);
                                          continue;
                                      }
                                      // Skip if already visited
                                      if (std::find(visitedNodes.begin(), visitedNodes.end(), neighborID) != visitedNodes.end()) {
                                          //printf("  Skipping %lu (already visited)\n", neighborID);
                                          continue;
                                      }

                                      size_t neighbor_x = neighborID % gridSizeX;
                                      size_t neighbor_y = neighborID / gridSizeX;

                                      size_t neighbor_distance_FD_node = abs(static_cast<int>(FD_x) - static_cast<int>(neighbor_x)) +
                                      abs(static_cast<int>(FD_y) - static_cast<int>(neighbor_y));

                                      //printf("target node ID: %lu, current distance FD node: %lu, neighbor distance FD node: %lu\n", finalDestNodeID, current_distance_FD_node, neighbor_distance_FD_node);

                                      if (neighbor_distance_FD_node >= current_distance_FD_node) {
                                          //printf("current distance FD node: %lu, neighbor distance FD node: %lu\n", current_distance_FD_node, neighbor_distance_FD_node);
                                          continue;
                                      }

                                      auto local_idx = globalToLocalIdx.at(neighborID);
                                      double queueSize = std::max(0.0, static_cast<double>(N[local_idx]->getPacketQueueSV().get()));
                                      double newCost = current.costSoFar + queueSize;

                                      //printf("  Checking neighbor %lu (cost: %.1f)\n", neighborID, newCost);

                                      if (bestCosts.find(neighborID) == bestCosts.end() || newCost < bestCosts[neighborID]) {
                                          bestCosts[neighborID] = newCost;
                                          std::vector<size_t> newPath = current.path;
                                          newPath.push_back(neighborID);
                                          queue.push({neighborID, newPath, newCost, current.hopsFromStart + 1});
                                          //printf("check successful\n");
                                      }
                                  }
                              }

                              //printf("No path found!\n");
                              //exit(1);
                              return Path();
                          }
};

// Depart vertex class
class Grid_VN2D_Depart : public Vertex, public std::enable_shared_from_this<Grid_VN2D_Depart> {
public:
    Grid_VN2D_Depart(size_t networkNodeID, size_t x, size_t y, std::vector<bool> neighbors, size_t gridSizeX, size_t gridSizeY, size_t hopRadius, const std::string& vertexName, size_t distSeed, std::string traceFolderName, std::string traceFileHeading, size_t minServiceTime, size_t modeServiceTime, size_t maxServiceTime, size_t minTransitTime, size_t modeTransitTime, size_t maxTransitTime, PDDA_SV<int>& packetQueueSV, std::queue<std::shared_ptr<Packet>>& packetQueue);
    void AddNeighborInfo(const NeighborInfo& info);
    void AddArriveVertices(std::vector<std::shared_ptr<Grid_VN2D_Arrive>> arriveVertices);
    virtual void IO_SVs(std::vector<std::vector<size_t>>& Is, std::vector<std::vector<size_t>>& Os);
    virtual void Run(std::list<PDDA_Event*>& newEvents, double simTime, std::shared_ptr<Entity> entity);
    void PrintNeighborInfo() const;
private:
    const size_t _networkNodeID;
    const size_t _x;
    const size_t _y;
    const size_t _hopRadius;
    std::vector<bool> _neighbors;
    const size_t _gridSizeX;
    const size_t _gridSizeY;
    NeighborInfo _neighborInfo;
    //Path findShortestPath(size_t targetIdx);
    // conditions
    bool _packetInQueue;
    // SVs
    PDDA_SV<int>& _packetQueueSV;
    std::queue<std::shared_ptr<Packet>>& _packetQueue;
    // vertices
    std::vector<std::shared_ptr<Grid_VN2D_Arrive>> _arriveVertices;
    std::vector<std::vector<std::shared_ptr<Grid_VN2D_Arrive>>> _hopNeighborVertices;
    // delays
    std::unique_ptr<TriangularDist> _serviceDelay;
    std::unique_ptr<TriangularDist> _transitDelay;
};

// Grid_VN2D class representing a 2D locally-connected grid of vertices
class Grid_VN2D : public PDDA_SimModel {
public:
    Grid_VN2D(size_t gridSizeX, size_t gridSizeY, size_t hopRadius, size_t numServersPerNetworkNode, size_t maxNumArriveEvents, double maxSimTime, size_t numThreads, size_t distSeed, int numSerialPDDA_Execs, std::string traceFolderName, std::string distParamsFile);
    NeighborInfo GetHopNeighborStructures(size_t x, size_t y);
    void PrintMeanPacketNetworkTime() const;
    virtual void PrintSVs() const;
    virtual void PrintNumVertexExecs() const;
    void PrintFinishedPackets() const;
private:
    // SVs
    std::vector<PDDA_SV<int>> _packetQueueSVs;
    std::vector<std::queue<std::shared_ptr<Packet>>> _packetQueues;
    // vertices
    std::vector<std::shared_ptr<Grid_VN2D_Arrive>> _arriveVertices;
    //std::vector<std::shared_ptr<Service>> _serviceVertices;
    std::vector<std::shared_ptr<Grid_VN2D_Depart>> _departVertices;
    // delay times
    size_t _minIntraArrivalTime, _modeIntraArrivalTime, _maxIntraArrivalTime;
    size_t _minServiceTime, _modeServiceTime, _maxServiceTime;
    size_t _minTransitTime, _modeTransitTime, _maxTransitTime;
    // other model parameters
    const size_t _gridSizeX;
    const size_t _gridSizeY;
    const size_t _hopRadius;
    const size_t _numServersPerNetworkNode;
    const size_t _maxNumArriveEvents;
    // methods
    void BuildModel();
    void CreateVertices();
    //std::vector<size_t> getNextHopNeighbors(size_t node, const std::unordered_set<size_t>& seenNodes) const;
    void AddVertices();
    void CreateEdges();
    void IO_SVs();
    void InitEvents();
    size_t GetVertexIndex(size_t x, size_t y, size_t type) const;
    // finished packets
    std::list<std::shared_ptr<Packet>> _finishedPackets;
    std::atomic<int> _finishedPacketListLock;
};
