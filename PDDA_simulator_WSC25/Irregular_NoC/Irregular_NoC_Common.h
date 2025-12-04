#pragma once

#include <cstddef>

// Constants for fixed-size arrays
const size_t NOC_MAX_PATH_LENGTH = 64;
const size_t NOC_MAX_HOP_RADIUS = 4;
const size_t NOC_MAX_NEIGHBORS_PER_LEVEL = 66;
const size_t NOC_MAX_TOTAL_NEIGHBORS = 129;

// Link type enumeration
enum class LinkType {
    MESH_LINK,     // Regular connection in a partial mesh
    EXPRESS_LINK   // Long-range express connection
};

// Link structure
struct NetworkLink {
    size_t targetNodeID;    // Destination node
    LinkType type;          // Type of link
    double latencyFactor;   // Multiplier for transit time (1.0 for mesh, lower for express)
    double bandwidthFactor; // Multiplier for bandwidth (1.0 for mesh, higher for express)
};