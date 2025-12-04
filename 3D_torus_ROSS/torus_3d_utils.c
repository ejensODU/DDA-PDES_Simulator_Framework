/**
 * torus_3d_utils.c
 * 
 * Helper functions for the Torus 3D ROSS simulator model
 */

#include "torus_3d_ross.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <float.h>

// Function to get the LP ID for a given node ID (declaration)
extern tw_lpid get_lp_for_node(size_t node_id);

/**
 * Initialize neighbor information for a node
 */
void init_neighbor_info(torus_3d_node_state* s) {
    // Calculate total nodes
    size_t total_nodes = g_params.grid_size_x * g_params.grid_size_y * g_params.grid_size_z;
    
    // Initialize all counts to 0
    s->neighbor_info.all_neighbor_count = 0;
    
    for (size_t i = 0; i <= T3D_MAX_HOP_RADIUS; i++) {
        s->neighbor_info.hop_level_counts[i] = 0;
    }
    
    // Initialize all direct neighbor indices to invalid
    for (int i = 0; i < 6; i++) {
        s->neighbor_info.direct_neighbors[i] = UINT_MAX;
    }
    
    // Create a visited bitmap to avoid duplicates
    bool* visited = (bool*)calloc(total_nodes, sizeof(bool));
    if (visited == NULL) {
        fprintf(stderr, "Error: Failed to allocate visited bitmap for node %zu\n", s->node_id);
        return;
    }
    
    // Mark self as visited
    visited[s->node_id] = true;
    
    // Calculate direct neighbors (6 directions in 3D torus)
    // Use coordinates for reliable neighbor calculation
    size_t west_x = wrap_coordinate(s->x - 1, g_params.grid_size_x);
    size_t east_x = wrap_coordinate(s->x + 1, g_params.grid_size_x);
    size_t north_y = wrap_coordinate(s->y - 1, g_params.grid_size_y);
    size_t south_y = wrap_coordinate(s->y + 1, g_params.grid_size_y);
    size_t down_z = wrap_coordinate(s->z - 1, g_params.grid_size_z);
    size_t up_z = wrap_coordinate(s->z + 1, g_params.grid_size_z);
    
    // Calculate neighbor node IDs directly from coordinates
    size_t neighbors[6];
    neighbors[0] = get_node_id_from_coords(west_x, s->y, s->z);  // West
    neighbors[1] = get_node_id_from_coords(east_x, s->y, s->z);  // East
    neighbors[2] = get_node_id_from_coords(s->x, north_y, s->z); // North
    neighbors[3] = get_node_id_from_coords(s->x, south_y, s->z); // South
    neighbors[4] = get_node_id_from_coords(s->x, s->y, down_z);  // Down
    neighbors[5] = get_node_id_from_coords(s->x, s->y, up_z);    // Up
    
    //printf("Node %zu (%zu,%zu,%zu) direct neighbors: W:%zu E:%zu N:%zu S:%zu D:%zu U:%zu\n",
    //       s->node_id, s->x, s->y, s->z, 
    //       neighbors[0], neighbors[1], neighbors[2], neighbors[3], neighbors[4], neighbors[5]);
    
    // Add immediate neighbors (hop 1)
    s->neighbor_info.hop_level_counts[1] = 0;  // Start with 0 and increment as we add
    
    for (int i = 0; i < 6; i++) {
        // Validate neighbor ID
        if (neighbors[i] >= total_nodes) {
            fprintf(stderr, "Warning: Invalid neighbor ID %zu for node %zu\n", neighbors[i], s->node_id);
            continue;
        }
        
        // Store in the hop level structure
        if (s->neighbor_info.hop_level_counts[1] < T3D_MAX_NEIGHBORS_PER_LEVEL) {
            size_t idx = s->neighbor_info.hop_level_counts[1];
            s->neighbor_info.hop_level_nodes[1][idx] = neighbors[i];
            s->neighbor_info.hop_level_counts[1]++;
        }
        
        // Store direct neighbor
        s->neighbor_info.direct_neighbors[i] = neighbors[i];
        
        // Mark as visited
        visited[neighbors[i]] = true;
        
        // Add to flat array
        if (s->neighbor_info.all_neighbor_count < T3D_MAX_TOTAL_NEIGHBORS) {
            s->neighbor_info.all_neighbors[s->neighbor_info.all_neighbor_count++] = neighbors[i];
        }
    }
    
    // BFS for remaining hop levels
    if (g_params.hop_radius > 1) {
        // For each hop level, starting at 2
        for (size_t level = 2; level <= g_params.hop_radius && level <= T3D_MAX_HOP_RADIUS; level++) {
            // Process previous level nodes to find their neighbors
            size_t prev_level_count = s->neighbor_info.hop_level_counts[level-1];
            for (size_t i = 0; i < prev_level_count && i < T3D_MAX_NEIGHBORS_PER_LEVEL; i++) {
                size_t current_node = s->neighbor_info.hop_level_nodes[level-1][i];
                
                // Skip invalid nodes
                if (current_node >= total_nodes) {
                    continue;
                }
                
                size_t nx, ny, nz;
                get_coords_from_node_id(current_node, &nx, &ny, &nz);
                
                // Calculate neighbors
                size_t w_x = wrap_coordinate(nx - 1, g_params.grid_size_x);
                size_t e_x = wrap_coordinate(nx + 1, g_params.grid_size_x);
                size_t n_y = wrap_coordinate(ny - 1, g_params.grid_size_y);
                size_t s_y = wrap_coordinate(ny + 1, g_params.grid_size_y);
                size_t d_z = wrap_coordinate(nz - 1, g_params.grid_size_z);
                size_t u_z = wrap_coordinate(nz + 1, g_params.grid_size_z);
                
                size_t next_neighbors[6];
                next_neighbors[0] = get_node_id_from_coords(w_x, ny, nz);  // West
                next_neighbors[1] = get_node_id_from_coords(e_x, ny, nz);  // East
                next_neighbors[2] = get_node_id_from_coords(nx, n_y, nz);  // North
                next_neighbors[3] = get_node_id_from_coords(nx, s_y, nz);  // South
                next_neighbors[4] = get_node_id_from_coords(nx, ny, d_z);  // Down
                next_neighbors[5] = get_node_id_from_coords(nx, ny, u_z);  // Up
                
                // Process each neighbor
                for (int j = 0; j < 6; j++) {
                    size_t neighbor = next_neighbors[j];
                    
                    // Skip invalid neighbors
                    if (neighbor >= total_nodes) {
                        continue;
                    }
                    
                    // Skip if already visited
                    if (visited[neighbor]) {
                        continue;
                    }
                    
                    // Add to hop level
                    if (s->neighbor_info.hop_level_counts[level] < T3D_MAX_NEIGHBORS_PER_LEVEL) {
                        size_t idx = s->neighbor_info.hop_level_counts[level];
                        s->neighbor_info.hop_level_nodes[level][idx] = neighbor;
                        s->neighbor_info.hop_level_counts[level]++;
                        visited[neighbor] = true;
                        
                        // Add to flat array
                        if (s->neighbor_info.all_neighbor_count < T3D_MAX_TOTAL_NEIGHBORS) {
                            s->neighbor_info.all_neighbors[s->neighbor_info.all_neighbor_count++] = neighbor;
                        } else {
                            // Max neighbors reached
                            break;
                        }
                    } else {
                        // Max neighbors for this level reached
                        break;
                    }
                }
                
                // Stop if max total neighbors reached
                if (s->neighbor_info.all_neighbor_count >= T3D_MAX_TOTAL_NEIGHBORS) {
                    break;
                }
            }
        }
    }
    
    free(visited);
}

/**
 * Initialize the queue knowledge structure
 */
void init_queue_knowledge(torus_3d_node_state* s) {
    // Calculate total nodes in the network
    size_t total_nodes = g_params.grid_size_x * g_params.grid_size_y * g_params.grid_size_z;
    
    // Allocate the queue knowledge structure
    s->queue_knowledge = (torus_3d_queue_knowledge*)malloc(sizeof(torus_3d_queue_knowledge));
    if (s->queue_knowledge == NULL) {
        fprintf(stderr, "Error: Failed to allocate queue knowledge structure for node %zu\n", s->node_id);
        return;
    }
    
    // Allocate arrays
    s->queue_knowledge->queue_sizes = (short*)malloc(total_nodes * sizeof(short));
    s->queue_knowledge->queue_timestamps = (float*)malloc(total_nodes * sizeof(float));
    s->queue_knowledge->known_queues = (bool*)malloc(total_nodes * sizeof(bool));
    
    // Check for allocation failures
    if (s->queue_knowledge->queue_sizes == NULL || 
        s->queue_knowledge->queue_timestamps == NULL || 
        s->queue_knowledge->known_queues == NULL) {
        fprintf(stderr, "Error: Failed to allocate queue knowledge arrays for node %zu\n", s->node_id);
        
        // Clean up any allocated memory
        if (s->queue_knowledge->queue_sizes) free(s->queue_knowledge->queue_sizes);
        if (s->queue_knowledge->queue_timestamps) free(s->queue_knowledge->queue_timestamps);
        if (s->queue_knowledge->known_queues) free(s->queue_knowledge->known_queues);
        free(s->queue_knowledge);
        s->queue_knowledge = NULL;
        return;
    }
    
    // Initialize all to default values
    for (size_t i = 0; i < total_nodes; i++) {
        s->queue_knowledge->queue_sizes[i] = -1 * g_params.num_servers_per_node; // Default state
        s->queue_knowledge->queue_timestamps[i] = 0.0f;
        s->queue_knowledge->known_queues[i] = false;
    }
    
    // Mark self and neighbors as known
    s->queue_knowledge->known_queues[s->node_id] = true;
    s->queue_knowledge->queue_sizes[s->node_id] = s->packet_queue_size;
    
    // Only access all_neighbors if properly initialized
    if (s->neighbor_info.all_neighbor_count > 0) {
        for (size_t i = 0; i < s->neighbor_info.all_neighbor_count && i < T3D_MAX_TOTAL_NEIGHBORS; i++) {
            size_t neighbor_id = s->neighbor_info.all_neighbors[i];
            
            // Bounds check for neighbor_id
            if (neighbor_id < total_nodes) {
                s->queue_knowledge->known_queues[neighbor_id] = true;
                s->queue_knowledge->queue_sizes[neighbor_id] = g_queue_sizes[neighbor_id];
            }
        }
    }
}

/**
 * Get a packet by ID
 */
torus_3d_packet* get_packet(tw_lpid packet_id) {
    // Check if packet ID is valid
    size_t total_nodes = g_params.grid_size_x * g_params.grid_size_y * g_params.grid_size_z;
    size_t total_packets = total_nodes * g_params.max_num_arrive_events;
    
    if (packet_id >= 0 && packet_id < total_packets) {
        return &g_packets[packet_id];
    }
    
    return NULL;
}

/**
 * Add network node data to a packet's path
 */
bool add_network_node_data(torus_3d_packet* packet, float arrival_time, size_t network_node_id, short queue_size) {
    // Check for revisits
    /* bool already_visited = false;
    for (size_t i = 0; i < packet->visited_node_cnt; i++) {
        if (packet->visited_nodes[i] == network_node_id) {
            already_visited = true;
            break;
        }
    }
    
    // Add to path history
    if (packet->visited_node_cnt < T3D_MAX_PATH_LENGTH) {
        packet->node_arrival_times[packet->visited_node_cnt] = arrival_time;
        packet->visited_nodes[packet->visited_node_cnt] = network_node_id;
        //packet->queue_sizes[packet->visited_node_cnt] = queue_size;
        //packet->visited_node_cnt++;
    } */
	packet->visited_node_cnt++;
	
	if (packet->visited_node_cnt == 2*T3D_MAX_PATH_LENGTH) {
		//printf("ERROR: Packet %lu has exceeded 2 * T3D_MAX_PATH_LENGTH-1\n", packet->id);
		return true;
	}
    
    // Add to circular buffer for recent history
    packet->recent_queue_sizes[packet->queue_history_index] = queue_size;
    packet->recent_queue_nodes[packet->queue_history_index] = network_node_id;
    packet->recent_queue_times[packet->queue_history_index] = arrival_time;
    
    // Update circular buffer index and count
    packet->queue_history_index = (packet->queue_history_index + 1) & T3D_QUEUE_HISTORY_MASK;
    if (packet->queue_history_count < T3D_QUEUE_HISTORY_SIZE) {
        packet->queue_history_count++;
    }
	
	return false;
}

/**
 * Convert coordinates to node ID
 */
size_t get_node_id_from_coords(size_t x, size_t y, size_t z) {
    return z * g_params.grid_size_x * g_params.grid_size_y + 
           y * g_params.grid_size_x + x;
}

/**
 * Convert node ID to coordinates
 */
void get_coords_from_node_id(size_t node_id, size_t* x, size_t* y, size_t* z) {
    *x = node_id % g_params.grid_size_x;
    *y = (node_id / g_params.grid_size_x) % g_params.grid_size_y;
    *z = node_id / (g_params.grid_size_x * g_params.grid_size_y);
}

/**
 * Wrap coordinate for torus topology
 */
size_t wrap_coordinate(size_t coord, size_t size) {
    return (coord + size) % size;
}

/**
 * Calculate wrapped distance in torus
 */
size_t get_wrapped_distance(size_t a, size_t b, size_t grid_size) {
    size_t direct_dist = (a > b) ? (a - b) : (b - a);
    size_t wrapped_dist = grid_size - direct_dist;
    return (direct_dist < wrapped_dist) ? direct_dist : wrapped_dist;
}

/**
 * Generate a random value from triangular distribution
 * Changed from double to float
 */
float generate_triangular_rv(float min, float mode, float max, tw_lp* lp) {
    float U = (float)tw_rand_unif(lp->rng);
    float F = (mode - min) / (max - min);
    
    if (U <= F) {
        return min + sqrtf(U * (max - min) * (mode - min));
    } else {
        return max - sqrtf((1.0f - U) * (max - min) * (max - mode));
    }
}

/**
 * Get a random node ID (for packet destinations)
 */
size_t get_random_node_id(torus_3d_node_state* s, tw_lp* lp) {
    size_t total_nodes = g_params.grid_size_x * g_params.grid_size_y * g_params.grid_size_z;
    return (size_t)(tw_rand_unif(lp->rng) * total_nodes);
}

/**
 * Deterministic shortest path implementation to ensure consistent routing
 * between serial and parallel execution
 * Updated to use float instead of double
 */
torus_3d_path find_shortest_path(torus_3d_node_state* s, size_t dest_node_id, size_t visited_node_cnt, float current_time, tw_lp* lp) {
    // Initialize path result
    torus_3d_path path;
    memset(&path, 0, sizeof(torus_3d_path));
    
    // Direct path for self-destination
    if (s->node_id == dest_node_id) {
        path.node_indices[path.length++] = s->node_id;
        return path;
    }
	
	// Revert to greedy path if out of hops
	if (visited_node_cnt >= T3D_MAX_PATH_LENGTH) {
		return create_greedy_path(s, dest_node_id, current_time);
	}
    
    // Get destination coordinates
    size_t dest_x, dest_y, dest_z;
    get_coords_from_node_id(dest_node_id, &dest_x, &dest_y, &dest_z);
    
    // If path finding arrays aren't allocated, fall back to greedy approach
    if (s->path_max_nodes == 0 || !s->path_g_scores) {
        return create_greedy_path(s, dest_node_id, current_time);
    }
    
    // Calculate total nodes
    size_t total_nodes = g_params.grid_size_x * g_params.grid_size_y * g_params.grid_size_z;
    size_t max_nodes = s->path_max_nodes;
    
    // Access path arrays
    float* g_scores = s->path_g_scores;
    float* f_scores = s->path_f_scores;
    size_t* came_from = s->path_came_from;
    bool* in_closed_set = s->path_in_closed_set;
    bool* in_open_set = s->path_in_open_set;
    
    // Reset arrays - only as needed
    for (size_t i = 0; i < max_nodes; i++) {
        g_scores[i] = FLT_MAX;
        f_scores[i] = FLT_MAX;
        came_from[i] = SIZE_MAX;
        in_closed_set[i] = false;
        in_open_set[i] = false;
    }
    
    // Simple manual priority queue using the pre-allocated array
    size_t* open_set = s->path_open_set;
    size_t open_set_size = 0;
    
    // Best partial path tracking
    size_t best_node_id = s->node_id;
    float best_dist = FLT_MAX;
    
    // Initialize start node
    size_t src_idx = s->node_id % max_nodes;
    g_scores[src_idx] = 0.0f;
    
    // Calculate initial heuristic with integer math to avoid FP precision issues
    size_t dx = get_wrapped_distance(s->x, dest_x, g_params.grid_size_x);
    size_t dy = get_wrapped_distance(s->y, dest_y, g_params.grid_size_y);
    size_t dz = get_wrapped_distance(s->z, dest_z, g_params.grid_size_z);
    float h_score = (float)(dx + dy + dz); 
    
    f_scores[src_idx] = h_score;
    best_dist = h_score;
    
    // Add to open set
    open_set[open_set_size++] = s->node_id;
    in_open_set[src_idx] = true;
    
    // Limit exploration
    const size_t MAX_ITERATIONS = 64;
    size_t iterations = 0;
    
    // Main A* loop
    while (open_set_size > 0 && iterations < MAX_ITERATIONS) {
        iterations++;
        
        // Find node with lowest f-score
        size_t current_idx = 0;
        float lowest_f = FLT_MAX;
        
        // First, find the lowest f-score
        for (size_t i = 0; i < open_set_size; i++) {
            size_t node_id = open_set[i];
            size_t idx = node_id % max_nodes;
            
            if (f_scores[idx] < lowest_f) {
                lowest_f = f_scores[idx];
            }
        }
        
        // Then, find the node with lowest ID among those with the lowest f-score
        // This provides deterministic tie-breaking
        size_t lowest_id = SIZE_MAX;
        for (size_t i = 0; i < open_set_size; i++) {
            size_t node_id = open_set[i];
            size_t idx = node_id % max_nodes;
            
            // Use a small epsilon to handle floating point comparisons
            if (fabsf(f_scores[idx] - lowest_f) < 0.0001f) {
                if (node_id < lowest_id) {
                    lowest_id = node_id;
                    current_idx = i;
                }
            }
        }
        
        // Get the current node
        size_t current_node_id = open_set[current_idx];
        size_t node_idx = current_node_id % max_nodes;
        
        // Remove from open set
        open_set[current_idx] = open_set[--open_set_size];
        in_open_set[node_idx] = false;
        
        // Add to closed set
        in_closed_set[node_idx] = true;
        
        // Check if reached destination
        if (current_node_id == dest_node_id) {
            // Reconstruct path (safely)
            size_t temp_path[T3D_MAX_PATH_LENGTH];
            size_t temp_len = 0;
            size_t current = dest_node_id;
            
            // Build path in reverse order (with loop protection)
            size_t max_steps = T3D_MAX_PATH_LENGTH;
            size_t step = 0;
            
            while (current != s->node_id && step < max_steps) {
                if (temp_len < T3D_MAX_PATH_LENGTH) {
                    temp_path[temp_len++] = current;
                }
                current = came_from[current % max_nodes];
                step++;
            }
            
            // Add source node
            if (temp_len < T3D_MAX_PATH_LENGTH) {
                temp_path[temp_len++] = s->node_id;
            }
            
            // Copy to result path in correct order
            for (size_t i = 0; i < temp_len; i++) {
                if (path.length < T3D_MAX_PATH_LENGTH) {
                    path.node_indices[path.length++] = temp_path[temp_len - 1 - i];
                }
            }
            
            path.total_cost = g_scores[dest_node_id % max_nodes];
            return path;
        }
        
        // Get current coordinates
        size_t cx, cy, cz;
        get_coords_from_node_id(current_node_id, &cx, &cy, &cz);
        
        // Update best partial path
        size_t cdx = get_wrapped_distance(cx, dest_x, g_params.grid_size_x);
        size_t cdy = get_wrapped_distance(cy, dest_y, g_params.grid_size_y);
        size_t cdz = get_wrapped_distance(cz, dest_z, g_params.grid_size_z);
        float curr_dist = (float)(cdx + cdy + cdz);
        
        if (curr_dist < best_dist) {
            best_dist = curr_dist;
            best_node_id = current_node_id;
        }
        
        // Process neighbors in a deterministic order (always W,E,N,S,D,U)
        int dir_order[6] = {0, 1, 2, 3, 4, 5}; // West, East, North, South, Down, Up
        
        for (int dir_idx = 0; dir_idx < 6; dir_idx++) {
            int dir = dir_order[dir_idx];
            
            // Calculate neighbor based on direction
            size_t nx = cx, ny = cy, nz = cz;
            
            switch (dir) {
                case 0: nx = wrap_coordinate(cx - 1, g_params.grid_size_x); break; // West
                case 1: nx = wrap_coordinate(cx + 1, g_params.grid_size_x); break; // East
                case 2: ny = wrap_coordinate(cy - 1, g_params.grid_size_y); break; // North
                case 3: ny = wrap_coordinate(cy + 1, g_params.grid_size_y); break; // South
                case 4: nz = wrap_coordinate(cz - 1, g_params.grid_size_z); break; // Down
                case 5: nz = wrap_coordinate(cz + 1, g_params.grid_size_z); break; // Up
            }
            
            // Get neighbor ID
            size_t neighbor_id = get_node_id_from_coords(nx, ny, nz);
            size_t neighbor_idx = neighbor_id % max_nodes;
            
            // Skip if already in closed set
            if (in_closed_set[neighbor_idx]) {
                continue;
            }
            
            // Calculate cost to move to neighbor
			float queue_size = estimate_queue_size(s, neighbor_id, current_time);
			float move_cost = queue_size + 1.0f;
            
            float tentative_g = g_scores[node_idx] + move_cost;
            
            // Check if this path is better
            if (!in_open_set[neighbor_idx] || tentative_g < g_scores[neighbor_idx]) {
                // Update path
                came_from[neighbor_idx] = current_node_id;
                g_scores[neighbor_idx] = tentative_g;
                
                // Calculate heuristic with integer math
                size_t ndx = get_wrapped_distance(nx, dest_x, g_params.grid_size_x);
                size_t ndy = get_wrapped_distance(ny, dest_y, g_params.grid_size_y);
                size_t ndz = get_wrapped_distance(nz, dest_z, g_params.grid_size_z);
                float h = (float)(ndx + ndy + ndz);
                
                // Update f-score
                f_scores[neighbor_idx] = tentative_g + h;
                
                // Add to open set if not already there
                if (!in_open_set[neighbor_idx]) {
                    if (open_set_size < max_nodes) {
                        open_set[open_set_size++] = neighbor_id;
                        in_open_set[neighbor_idx] = true;
                    }
                }
            }
        }
    }
    
    // If best node is still source or we couldn't find a path, use greedy approach
    if (best_node_id == s->node_id) {
        return create_greedy_path(s, dest_node_id, current_time);
    }
    
    // Reconstruct path to best node
    size_t temp_path[T3D_MAX_PATH_LENGTH];
    size_t temp_len = 0;
    size_t current = best_node_id;
    
    // Build path in reverse order (with loop protection)
    size_t max_steps = T3D_MAX_PATH_LENGTH;
    size_t step = 0;
    
    while (current != s->node_id && step < max_steps) {
        if (temp_len < T3D_MAX_PATH_LENGTH) {
            temp_path[temp_len++] = current;
        }
        current = came_from[current % max_nodes];
        step++;
    }
    
    // Add source node
    if (temp_len < T3D_MAX_PATH_LENGTH) {
        temp_path[temp_len++] = s->node_id;
    }
    
    // Copy to result path in correct order
    for (size_t i = 0; i < temp_len; i++) {
        if (path.length < T3D_MAX_PATH_LENGTH) {
            path.node_indices[path.length++] = temp_path[temp_len - 1 - i];
        }
    }
    
    path.total_cost = g_scores[best_node_id % max_nodes];
    return path;
}

/**
 * Make the greedy path creation deterministic as well
 * Updated to use float instead of double
 */
torus_3d_path create_greedy_path(torus_3d_node_state* s, size_t dest_node_id, float current_time) {
    torus_3d_path path;
    path.length = 0;
    path.total_cost = 0.0f;
    
    // Start with current node
    path.node_indices[path.length++] = s->node_id;
    
    // If already at destination, return
    if (s->node_id == dest_node_id) {
        return path;
    }
    
    // Get destination coordinates
    size_t dest_x, dest_y, dest_z;
    get_coords_from_node_id(dest_node_id, &dest_x, &dest_y, &dest_z);
    
    // Calculate wrapped distances in each dimension
    size_t dx = get_wrapped_distance(s->x, dest_x, g_params.grid_size_x);
    size_t dy = get_wrapped_distance(s->y, dest_y, g_params.grid_size_y);
    size_t dz = get_wrapped_distance(s->z, dest_z, g_params.grid_size_z);
    
    // Get the next hop based on dimension with largest distance
    // Break ties deterministically by prioritizing X > Y > Z axes
    size_t next_node_id;
    
    if (dx >= dy && dx >= dz) {
        // X dimension has biggest distance
        if ((s->x < dest_x && dest_x - s->x <= g_params.grid_size_x/2) ||
            (s->x > dest_x && s->x - dest_x > g_params.grid_size_x/2)) {
            // Need to go east
            next_node_id = s->neighbor_info.direct_neighbors[1];
        } else {
            // Need to go west
            next_node_id = s->neighbor_info.direct_neighbors[0];
        }
    } else if (dy > dx && dy >= dz) {  // Changed >= to > to break ties deterministically
        // Y dimension has biggest distance
        if ((s->y < dest_y && dest_y - s->y <= g_params.grid_size_y/2) ||
            (s->y > dest_y && s->y - dest_y > g_params.grid_size_y/2)) {
            // Need to go south
            next_node_id = s->neighbor_info.direct_neighbors[3];
        } else {
            // Need to go north
            next_node_id = s->neighbor_info.direct_neighbors[2];
        }
    } else {
        // Z dimension has biggest distance
        if ((s->z < dest_z && dest_z - s->z <= g_params.grid_size_z/2) ||
            (s->z > dest_z && s->z - dest_z > g_params.grid_size_z/2)) {
            // Need to go up
            next_node_id = s->neighbor_info.direct_neighbors[5];
        } else {
            // Need to go down
            next_node_id = s->neighbor_info.direct_neighbors[4];
        }
    }
    
    // Add next hop and basic cost
    if (next_node_id != UINT_MAX) {
        path.node_indices[path.length++] = next_node_id;
        
        // Use a fixed cost to maintain determinism
        path.total_cost = 1.0f;
    } else {
        // Fallback if neighbor not found
        printf("ERROR: Failed to find valid neighbor for node %zu to destination %zu (%zu,%zu,%zu)\n",
               s->node_id, dest_node_id, dest_x, dest_y, dest_z);
    }
    
    return path;
}

/**
 * Update the queue size estimation to match C++ floating-point and dynamic sampling behavior
 * Updated to use float instead of double
 */
float estimate_queue_size(torus_3d_node_state* s, size_t node_id, float current_time) {
    // If this is our node, return actual queue size
    if (node_id == s->node_id) {
        return (float)s->packet_queue_size;
    }
    
    // Static variable to store computed average (calculated once per node)
    static float avg_queue_size = -1.0f;
    
    // Calculate the average queue size dynamically if not done yet
    if (avg_queue_size < 0.0f && s->queue_knowledge) {
        size_t total_nodes = g_params.grid_size_x * g_params.grid_size_y * g_params.grid_size_z;
        float sum = 0.0f;
        int count = 0;
        
        // Use sampling approach like C++ version (check every Nth node)
        size_t step = (total_nodes < 512) ? 1 : (total_nodes / 512);
        
        for (size_t i = 0; i < total_nodes; i += step) {
            if (s->queue_knowledge->known_queues && 
                s->queue_knowledge->known_queues[i] && 
                s->queue_knowledge->queue_timestamps[i] > 0) {
                sum += s->queue_knowledge->queue_sizes[i];
                count++;
            }
        }
        
        // Calculate average plus buffer (like C++ version)
        avg_queue_size = (count > 0) ? (sum / count) + 15.0f : 20.0f;
    } else if (avg_queue_size < 0.0f) {
        // If no queue knowledge is available
        avg_queue_size = 20.0f;
    }
    
    // Look up known queue size using same approach as C++
    if (s->queue_knowledge && node_id < g_params.grid_size_x * g_params.grid_size_y * g_params.grid_size_z &&
        s->queue_knowledge->known_queues && s->queue_knowledge->known_queues[node_id]) {
        
        float age = current_time - s->queue_knowledge->queue_timestamps[node_id];
        short queue_size = s->queue_knowledge->queue_sizes[node_id];
        
        // Fast path for very recent data (same as C++)
        if (age < 0.1f) {
            return (float)queue_size;
        }
        
        // Use C++ age penalty calculation: age/5.0 with cap at 20.0
        float age_penalty = (age / 5.0f);
        if (age_penalty > 20.0f) age_penalty = 20.0f;
        
        return (float)queue_size + age_penalty;
    }
    
    // Return precomputed average for unknown nodes
    return avg_queue_size;
}

/**
 * Fix the next_hop_direction function to be deterministic
 */
size_t next_hop_direction(torus_3d_node_state* s, torus_3d_path* path) {
    // If path has at least two nodes (current + next)
    if (path->length >= 2) {
        // Current node should be first in path
        size_t next_node_id = path->node_indices[1];
        
        // Find which direct neighbor this is
        for (int i = 0; i < 6; i++) {
            if (s->neighbor_info.direct_neighbors[i] == next_node_id) {
                return i;
            }
        }
    }
    
    // Default: choose based on destination coordinates with deterministic tie-breaking
    size_t dest_node_id = path->node_indices[path->length - 1];
    size_t dest_x, dest_y, dest_z;
    get_coords_from_node_id(dest_node_id, &dest_x, &dest_y, &dest_z);
    
    // Calculate wrapped distances
    size_t dx = get_wrapped_distance(s->x, dest_x, g_params.grid_size_x);
    size_t dy = get_wrapped_distance(s->y, dest_y, g_params.grid_size_y);
    size_t dz = get_wrapped_distance(s->z, dest_z, g_params.grid_size_z);
    
    // Choose direction with largest distance
    // Break ties deterministically by prioritizing X > Y > Z axes
    if (dx >= dy && dx >= dz) {
        // X dimension has biggest distance
        if ((s->x < dest_x && dest_x - s->x <= g_params.grid_size_x/2) ||
            (s->x > dest_x && s->x - dest_x > g_params.grid_size_x/2)) {
            return 1; // East
        } else {
            return 0; // West
        }
    } else if (dy > dx && dy >= dz) {  // Changed >= to > to break ties deterministically
        // Y dimension has biggest distance
        if ((s->y < dest_y && dest_y - s->y <= g_params.grid_size_y/2) ||
            (s->y > dest_y && s->y - dest_y > g_params.grid_size_y/2)) {
            return 3; // South
        } else {
            return 2; // North
        }
    } else {
        // Z dimension has biggest distance
        if ((s->z < dest_z && dest_z - s->z <= g_params.grid_size_z/2) ||
            (s->z > dest_z && s->z - dest_z > g_params.grid_size_z/2)) {
            return 5; // Up
        } else {
            return 4; // Down
        }
    }
}

/**
 * Determine if two nodes are in the same LP using linear mapping
 */
bool is_node_in_same_lp(size_t node_id1, size_t node_id2) {
    // If running in serial mode, all nodes are in the same LP
    if (tw_nnodes() == 1) {
        return true;
    }
    
    // Using linear mapping, nodes are in the same LP if they map to the same PE
    size_t pe1 = node_id1 / g_params.nodes_per_pe;
    size_t pe2 = node_id2 / g_params.nodes_per_pe;
    
    // Ensure valid PE IDs
    if (pe1 >= tw_nnodes()) pe1 = tw_nnodes() - 1;
    if (pe2 >= tw_nnodes()) pe2 = tw_nnodes() - 1;
    
    // Nodes are in the same LP if they belong to the same PE
    return (pe1 == pe2);
}
 
/**
 * Update local queue sizes from shared memory
 * Updated to use float instead of double
 */
void update_local_queue_sizes(torus_3d_node_state* s, float current_time) {
    // Update queue sizes for all neighbors within hop radius
    for (size_t i = 0; i < s->neighbor_info.all_neighbor_count; i++) {
        size_t neighbor_id = s->neighbor_info.all_neighbors[i];
        
        // Only access shared memory directly if in same LP
        if (is_node_in_same_lp(s->node_id, neighbor_id)) {
            s->queue_knowledge->queue_sizes[neighbor_id] = g_queue_sizes[neighbor_id];
            s->queue_knowledge->queue_timestamps[neighbor_id] = current_time;
            s->queue_knowledge->known_queues[neighbor_id] = true;
        }
        // Otherwise updates come through QUEUE_UPDATE events
    }
    
    // Update own node's data
    s->queue_knowledge->queue_sizes[s->node_id] = s->packet_queue_size;
    s->queue_knowledge->queue_timestamps[s->node_id] = current_time;
    s->queue_knowledge->known_queues[s->node_id] = true;
}

/**
 * Update global queue size knowledge from packet data
 * Uses the recent queue history circular buffer for efficiency
 */
void update_queue_knowledge(torus_3d_node_state* s, torus_3d_packet* packet) {
    // Make sure queue_knowledge is valid
    if (!s->queue_knowledge) {
        return;
    }
    
    // Get total nodes in network
    size_t total_nodes = g_params.grid_size_x * g_params.grid_size_y * g_params.grid_size_z;
    
    // Loop through the packet's recent queue history (circular buffer)
    for (size_t i = 0; i < packet->queue_history_count; i++) {
        // Calculate actual index in circular buffer
        size_t actual_idx = (packet->queue_history_index - 1 - i) & T3D_QUEUE_HISTORY_MASK;
        
        // Get node, queue size, and timestamp
        size_t node_id = packet->recent_queue_nodes[actual_idx];
        short queue_size = packet->recent_queue_sizes[actual_idx];
        float timestamp = packet->recent_queue_times[actual_idx];
        
        // Only update if node_id is valid and information is newer
        if (node_id < total_nodes && 
            (!s->queue_knowledge->known_queues[node_id] || 
             timestamp > s->queue_knowledge->queue_timestamps[node_id])) {
            
            s->queue_knowledge->queue_sizes[node_id] = queue_size;
            s->queue_knowledge->queue_timestamps[node_id] = timestamp;
            s->queue_knowledge->known_queues[node_id] = true;
        }
    }
}