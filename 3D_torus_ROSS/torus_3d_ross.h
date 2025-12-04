/**
 * torus_3d_ross.h
 * 
 * Main header file for the Torus 3D ROSS simulator model
 */

#ifndef _TORUS_3D_ROSS_H
#define _TORUS_3D_ROSS_H

#include <ross.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

/** 
 * Constants from the original C++ model 
 */
#define T3D_MAX_PATH_LENGTH 64
#define T3D_MAX_HOP_RADIUS 4
#define T3D_MAX_NEIGHBORS_PER_LEVEL 66
#define T3D_MAX_TOTAL_NEIGHBORS 129
#define T3D_QUEUE_HISTORY_SIZE 16
#define T3D_QUEUE_HISTORY_MASK (T3D_QUEUE_HISTORY_SIZE - 1)

/**
 * Global simulation parameters
 */
typedef struct {
    size_t grid_size_x;
    size_t grid_size_y; 
    size_t grid_size_z;
    size_t hop_radius;
    size_t num_servers_per_node;
    size_t max_num_arrive_events;
    size_t queue_capacity_per_node;  // Max queue size per node
    
    // Distribution parameters - changed from double to float
    float min_intra_arrival_time;
    float mode_intra_arrival_time;
    float max_intra_arrival_time;
    
    float min_service_time;
    float mode_service_time;
    float max_service_time;
    
    float min_transit_time;
    float mode_transit_time;
    float max_transit_time;
	
	tw_stime lookahead;        // Lookahead value for ROSS
	unsigned int update_delta; // Update delta threshold for queue updates
    
    // LP mapping parameters
    size_t nodes_per_kp;  // Number of nodes per kernel process
    size_t nodes_per_pe;  // Number of nodes per processing element
} torus_3d_params;

// Global parameters instance
extern torus_3d_params g_params;

/**
 * Packet structure (analogous to Torus_3D_Packet)
 */
typedef struct {
    tw_lpid id;  // Packet ID (ROSS LP ID used for unique identification)
    tw_stime gen_time;
    tw_stime exit_time;
    
    size_t origin_node_id;
    size_t dest_node_id;
    short dest_x;
    short dest_y;
    short dest_z;
    
    // Path tracking
    short visited_node_cnt;
    //float node_arrival_times[T3D_MAX_PATH_LENGTH];
    //size_t visited_nodes[T3D_MAX_PATH_LENGTH];
    //int queue_sizes[T3D_MAX_PATH_LENGTH];
    
    // Circular buffer for recent queue size history
    short recent_queue_sizes[T3D_QUEUE_HISTORY_SIZE];
    size_t recent_queue_nodes[T3D_QUEUE_HISTORY_SIZE];
    float recent_queue_times[T3D_QUEUE_HISTORY_SIZE];
    short queue_history_count;
    short queue_history_index;
} torus_3d_packet;

/**
 * Neighbor information structure (based on Torus_3D_NeighborInfo)
 */
typedef struct {
    // Hierarchical hop-level structures
    size_t hop_level_nodes[T3D_MAX_HOP_RADIUS+1][T3D_MAX_NEIGHBORS_PER_LEVEL];
    short hop_level_counts[T3D_MAX_HOP_RADIUS+1];
    size_t all_neighbors[T3D_MAX_TOTAL_NEIGHBORS];
    size_t all_neighbor_count;
    
    // Direct neighbors in the 6 directions (west, east, north, south, down, up)
    size_t direct_neighbors[6];
    
    // Node ID for reference
    size_t node_id;
    short x, y, z;
} torus_3d_neighbor_info;

/**
 * Queue size knowledge structure
 */
typedef struct {
    short* queue_sizes;
    float* queue_timestamps;
    bool* known_queues;
} torus_3d_queue_knowledge;

/**
 * Path structure for routing
 */
typedef struct {
    size_t node_indices[T3D_MAX_PATH_LENGTH];
    float total_cost;
    short length;
} torus_3d_path;

/**
 * LP state structure for a network node
 */
typedef struct {
    // Node identification and location
    tw_lpid node_id;
    size_t x, y, z;
    bool slow_node;  // Flag for slow processing nodes
    
    // Queue and packet management - queue size is short, but packet IDs must stay as tw_lpid
    short packet_queue_size;
    tw_lpid* packet_queue;
    size_t queue_head;
    size_t queue_tail;
    size_t queue_capacity;
    size_t queue_count;
    
    // Statistics - changed some from double to float
    size_t arrive_count;
    size_t depart_count;
    size_t max_queue_length;
    float total_waiting_time;
    size_t num_packets_completed;  // Count of packets that reached destination at this node
    float total_packet_time;      // Sum of packet lifespans (exit_time - gen_time)
    
    // Packet generation parameters for source nodes
    size_t num_intra_arrive_events;
    
    // Neighbor information
    torus_3d_neighbor_info neighbor_info;
    
    // Global queue size knowledge
    torus_3d_queue_knowledge* queue_knowledge;
    
    // Mapping information for communication
    tw_lpid lp_id;  // ROSS LP ID
    tw_lpid kp_id;  // Kernel process ID
    
    // Save state for message handling
    short saved_queue_size;
    size_t saved_packet_id;  // For packets being processed
    tw_lpid saved_dest_lp;   // Destination for rollbacks
    
    // Persistent path finding data structures
    // These arrays are allocated once and reused for all path calculations
    // Changed from double* to float*
    float* path_g_scores;      // Cost from start to each node
    float* path_f_scores;      // Estimated total cost (g + h) for each node
    size_t* path_came_from;     // Parent node in the path
    bool* path_in_closed_set;   // Closed set (nodes already evaluated)
    bool* path_in_open_set;     // Open set (nodes discovered but not evaluated)
    size_t* path_open_set;      // Nodes in the open set (for priority queue)
    size_t path_max_nodes;      // Maximum number of nodes these arrays can handle
} torus_3d_node_state;

/**
 * Event types in the model
 */
typedef enum {
    ARRIVE_EVENT = 1,        // Packet arrives at a node
    DEPART_EVENT = 2,        // Packet departs from a node after service
    QUEUE_UPDATE_EVENT = 3,  // Update of queue size information
    GEN_PACKET_EVENT = 4     // Generate a new packet
} torus_3d_event_type;

/**
 * Event message structure
 */
typedef struct {
    torus_3d_event_type type;
    
    // For packet events
    tw_lpid packet_id;  // Using ROSS LP IDs for unique packet identification
    size_t origin_node_id;
    size_t dest_node_id;
    size_t dest_x, dest_y, dest_z;
    
    // For queue updates
    short queue_size;
    tw_stime update_time;
    size_t source_node_id;
    
    // Saved state for reverse computation
    short prev_queue_size;
    tw_lpid prev_dest_lp;
    
    // For rollbacks
    size_t packet_path_size_before;
    short queue_size_before;
    size_t queue_count_before;
	
	// Path tracking preservation for cross-PE transfers
	short visited_node_cnt;          // Number of nodes visited
	tw_stime orig_gen_time;           // Original generation time
	tw_stime first_arrival_time;      // Time of first arrival (for delay calculation)
	short hop_count;                 // For statistics
	
	// Path tracking
    //float node_arrival_times[T3D_MAX_PATH_LENGTH];
    //size_t visited_nodes[T3D_MAX_PATH_LENGTH];
    //int queue_sizes[T3D_MAX_PATH_LENGTH];
    
    // Circular buffer for recent queue size history
    short recent_queue_sizes[T3D_QUEUE_HISTORY_SIZE];
    size_t recent_queue_nodes[T3D_QUEUE_HISTORY_SIZE];
    float recent_queue_times[T3D_QUEUE_HISTORY_SIZE];
    short queue_history_count;
    short queue_history_index;
} torus_3d_message;

/**
 * LP and event handler function declarations
 */
void torus_3d_init(torus_3d_node_state* s, tw_lp* lp);
void torus_3d_prerun(torus_3d_node_state* s, tw_lp* lp);
void torus_3d_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp);
void torus_3d_reverse_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp);
void torus_3d_final(torus_3d_node_state* s, tw_lp* lp);

/**
 * LP type definition
 */
extern tw_lptype torus_3d_lp_type;

/**
 * Event handler function declarations for specific events
 */
void handle_arrive_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp);
void handle_depart_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp);
void handle_queue_update_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp);
void handle_gen_packet_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp);

void reverse_arrive_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp);
void reverse_depart_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp);
void reverse_queue_update_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp);
void reverse_gen_packet_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp);

/**
 * Helper functions
 */
void init_neighbor_info(torus_3d_node_state* s);
void init_queue_knowledge(torus_3d_node_state* s);
torus_3d_packet* get_packet(tw_lpid packet_id);
bool add_network_node_data(torus_3d_packet* packet, float arrival_time, size_t network_node_id, short queue_size);
size_t get_node_id_from_coords(size_t x, size_t y, size_t z);
void get_coords_from_node_id(size_t node_id, size_t* x, size_t* y, size_t* z);
size_t wrap_coordinate(size_t coord, size_t size);
size_t get_wrapped_distance(size_t a, size_t b, size_t grid_size);
float generate_triangular_rv(float min, float mode, float max, tw_lp* lp);
size_t get_random_node_id(torus_3d_node_state* s, tw_lp* lp);
torus_3d_path find_shortest_path(torus_3d_node_state* s, size_t dest_node_id, size_t visited_node_cnt, float current_time, tw_lp* lp);
torus_3d_path create_greedy_path(torus_3d_node_state* s, size_t dest_node_id, float current_time);
bool is_node_in_same_lp(size_t node_id1, size_t node_id2);
void update_local_queue_sizes(torus_3d_node_state* s, float current_time);
void update_queue_knowledge(torus_3d_node_state* s, torus_3d_packet* packet);
float estimate_queue_size(torus_3d_node_state* s, size_t node_id, float current_time);
size_t next_hop_direction(torus_3d_node_state* s, torus_3d_path* path);

/**
 * Function declarations
 */
tw_peid torus_3d_map(tw_lpid gid);
tw_lpid torus_3d_typemap(tw_lpid gid);

/**
 * Global helper variables
 */
extern torus_3d_packet* g_packets;  // Global packet array (analogous to _packets in C++ code)
extern short* g_queue_sizes;          // Global queue sizes (analogous to _queueSizes in C++ code)
extern short* g_last_reported_size;

#endif /* _TORUS_3D_ROSS_H */