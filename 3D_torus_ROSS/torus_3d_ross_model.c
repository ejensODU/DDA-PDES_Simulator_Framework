/**
 * torus_3d_ross_model.c
 * 
 * Main implementation of the Torus 3D ROSS simulator model
 */

#include "torus_3d_ross.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/**
 * Global parameters and variables
 */
torus_3d_params g_params;
torus_3d_packet* g_packets;  // Global packet array
short* g_queue_sizes;          // Global queue sizes for shared memory access
short* g_last_reported_size = NULL;

/**
 * Option definitions for command line parameters
 */
static tw_optdef torus_options[] = {
    TWOPT_GROUP("Torus 3D Model"),
    TWOPT_UINT("grid-x", g_params.grid_size_x, "X dimension of torus"),
    TWOPT_UINT("grid-y", g_params.grid_size_y, "Y dimension of torus"),
    TWOPT_UINT("grid-z", g_params.grid_size_z, "Z dimension of torus"),
    TWOPT_UINT("hop-radius", g_params.hop_radius, "Hop radius for routing"),
    TWOPT_UINT("servers-per-node", g_params.num_servers_per_node, "Servers per network node"),
    TWOPT_UINT("max-arrive-events", g_params.max_num_arrive_events, "Maximum arrive events per node"),
    TWOPT_UINT("queue-capacity", g_params.queue_capacity_per_node, "Maximum queue capacity per node"),
	TWOPT_STIME("lookahead", g_params.lookahead, "Lookahead value for ROSS"),
	TWOPT_UINT("update-delta", g_params.update_delta, "Update delta threshold for queue updates"),
    TWOPT_UINT("nodes-per-kp", g_params.nodes_per_kp, "Nodes per kernel process"),
    TWOPT_UINT("nodes-per-pe", g_params.nodes_per_pe, "Nodes per processing element"),
    TWOPT_ULONG("events-per-pe", g_tw_events_per_pe, "Number of events per PE (increase if running out of event memory)"),
    TWOPT_END()
};

/**
 * Simple linear mapping function that replaces the complex domain decomposition
 * This determines which PE handles which LP
 */
tw_peid torus_3d_map(tw_lpid gid) {
    // Simple linear mapping: divide gid by number of LPs per PE
    // This evenly distributes LPs across PEs
    return (tw_peid)(gid / g_tw_nlp);
}

/**
 * Get the LP ID for a given node ID using linear mapping
 * This must be consistent with the torus_3d_map function
 */
tw_lpid get_lp_for_node(size_t node_id) {
    size_t total_nodes = g_params.grid_size_x * g_params.grid_size_y * g_params.grid_size_z;
    
    // Validate node ID
    if (node_id >= total_nodes) {
        fprintf(stderr, "Error: Invalid node ID %zu in get_lp_for_node\n", node_id);
        return (tw_lpid)-1;
    }
    
    // For sequential mode, LP ID is the same as node ID
    if (tw_nnodes() == 1) {
        return (tw_lpid)node_id;
    }
    
    // Calculate PE for this node using linear mapping
    tw_peid pe = node_id / g_params.nodes_per_pe;
    
    // Ensure valid PE ID
    if (pe >= tw_nnodes()) {
        pe = tw_nnodes() - 1;
    }
    
    // Calculate the global LP ID
    return (pe * g_tw_nlp) + node_id % g_tw_nlp;
}

/**
 * ROSS LP type mapping function
 */
tw_lpid torus_3d_typemap(tw_lpid gid) {
    // All nodes are of the same type in this model
    return 0;
}

/**
 * Initialize an LP (a network node)
 */
void torus_3d_init(torus_3d_node_state* s, tw_lp* lp) {
    // Zero out the state structure
    memset(s, 0, sizeof(torus_3d_node_state));

    // Get total nodes in simulation
    size_t total_nodes = g_params.grid_size_x * g_params.grid_size_y * g_params.grid_size_z;
    
    // Calculate node ID using consistent mapping
    size_t node_id = lp->gid % total_nodes;
    
    // Initialize state with proper node ID
    s->node_id = node_id;
    s->lp_id = lp->gid;
    s->kp_id = lp->gid / g_params.nodes_per_kp;
    
    // Get coordinates from node ID
    get_coords_from_node_id(s->node_id, &s->x, &s->y, &s->z);
    
    //printf("Initializing node %zu at coords (%zu,%zu,%zu) on LP %lu\n", 
    //       s->node_id, s->x, s->y, s->z, lp->gid);
    
    // Read node status from CSV file instead of using random numbers
    s->slow_node = false; // Default to normal node
    
    // Open the node status CSV file
    char filepath[512];
    snprintf(filepath, sizeof(filepath), "node_status.csv");
    FILE* file = fopen(filepath, "r");
    
    if (file != NULL) {
        char line[256];
        // Skip header line
        if (fgets(line, sizeof(line), file) != NULL) {
            // Read data lines
            while (fgets(line, sizeof(line), file) != NULL) {
                size_t id, x, y, z;
                int slow;
                if (sscanf(line, "%zu,%zu,%zu,%zu,%d", &id, &x, &y, &z, &slow) == 5) {
                    if (id == node_id) {
                        s->slow_node = (slow == 1);
                        break; // Found the node we're looking for
                    }
                }
            }
        }
        fclose(file);
    } else {
        // If file can't be opened, fall back to random number generator (original logic)
        fprintf(stderr, "Warning: Could not open node_status.csv, using random values\n");
        double rng = tw_rand_unif(lp->rng);
        s->slow_node = (rng < 0.16667); // 1/6 probability
    }
    
    // Initialize queue
    s->packet_queue_size = -1 * g_params.num_servers_per_node;
    s->queue_capacity = g_params.queue_capacity_per_node;  // Use the configurable parameter
    s->packet_queue = (tw_lpid*)malloc(s->queue_capacity * sizeof(tw_lpid));
    if (s->packet_queue == NULL) {
        fprintf(stderr, "Error: Failed to allocate packet queue for node %zu\n", s->node_id);
        // No need to clean up anything yet as this is the first allocation
        return;
    }
    memset(s->packet_queue, 0, s->queue_capacity * sizeof(tw_lpid));
    s->queue_head = 0;
    s->queue_tail = 0;
    s->queue_count = 0;
    
    // Initialize statistics
    s->arrive_count = 0;
    s->depart_count = 0;
    s->max_queue_length = 0;
    s->total_waiting_time = 0.0f;
    s->num_packets_completed = 0;
    s->total_packet_time = 0.0f;
    
    // Initialize packet generation
    s->num_intra_arrive_events = 0;
    
    // Initialize saved state
    s->saved_queue_size = s->packet_queue_size;
    s->saved_packet_id = -1;
    s->saved_dest_lp = -1;
    
    // Initialize neighbor info
    init_neighbor_info(s);
    
    // Initialize queue knowledge
    init_queue_knowledge(s);
    
    // Initialize path finding data structures (persistent)
    // For smaller simulations, allocate enough for all nodes
    // For larger simulations, allocate a reasonable subset
    size_t path_array_size = (total_nodes <= 4096) ? total_nodes : 4096;
    s->path_max_nodes = path_array_size;
    
    s->path_g_scores = (float*)calloc(path_array_size, sizeof(float));
    s->path_f_scores = (float*)calloc(path_array_size, sizeof(float));
    s->path_came_from = (size_t*)malloc(path_array_size * sizeof(size_t));
    s->path_in_closed_set = (bool*)calloc(path_array_size, sizeof(bool));
    s->path_in_open_set = (bool*)calloc(path_array_size, sizeof(bool));
    s->path_open_set = (size_t*)malloc(path_array_size * sizeof(size_t));
    
    if (!s->path_g_scores || !s->path_f_scores || !s->path_came_from || 
        !s->path_in_closed_set || !s->path_in_open_set || !s->path_open_set) {
        fprintf(stderr, "Error: Failed to allocate path finding arrays for node %zu\n", s->node_id);
        
        // Free any allocated arrays
        if (s->path_g_scores) free(s->path_g_scores);
        if (s->path_f_scores) free(s->path_f_scores);
        if (s->path_came_from) free(s->path_came_from);
        if (s->path_in_closed_set) free(s->path_in_closed_set);
        if (s->path_in_open_set) free(s->path_in_open_set);
        if (s->path_open_set) free(s->path_open_set);
        
        s->path_max_nodes = 0;
    }
    
    // Update global queue size array
    g_queue_sizes[s->node_id] = s->packet_queue_size;
    
    // Schedule first packet generation event for each node
    // This creates the initial workload for the simulation
    float arrival_delay = generate_triangular_rv(
            g_params.min_intra_arrival_time,
            g_params.mode_intra_arrival_time,
            g_params.max_intra_arrival_time,
            lp
        );
    
    tw_event* e = tw_event_new(s->lp_id, arrival_delay - 1.00001*g_tw_lookahead, lp);
    if (e == NULL) {
        fprintf(stderr, "Error: Failed to create initial event for node %zu\n", s->node_id);
        return;
    }
    
    torus_3d_message* msg = tw_event_data(e);
    memset(msg, 0, sizeof(torus_3d_message));  // Initialize all fields to 0
    msg->type = GEN_PACKET_EVENT;
    msg->packet_id = -1; // Special flag for generation events
    tw_event_send(e);
}

/**
 * Pre-run function (called after init but before event processing)
 */
void torus_3d_prerun(torus_3d_node_state* s, tw_lp* lp) {
    // Any additional setup needed before running the simulation
}

/**
 * Main event handler that dispatches to specific event handlers
 */
void torus_3d_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp) {
    // Skip invalid nodes (LPs that don't map to a node in our model)
    if (s->node_id == UINT_MAX) {
        return;
    }
    
    // Set saved state for potential rollbacks
    s->saved_queue_size = s->packet_queue_size;
    
    // Save message state for reverse computation
    msg->prev_queue_size = s->packet_queue_size;
    msg->queue_size_before = s->packet_queue_size;
    msg->queue_count_before = s->queue_count;
    
    if (msg->packet_id >= 0) {
        torus_3d_packet* packet = get_packet(msg->packet_id);
        if (packet) {
            msg->packet_path_size_before = packet->visited_node_cnt;
        }
    }
    
    // Dispatch to specific event handler based on message type
    switch (msg->type) {
        case ARRIVE_EVENT:
            handle_arrive_event(s, bf, msg, lp);
            break;
        case DEPART_EVENT:
            handle_depart_event(s, bf, msg, lp);
            break;
        case QUEUE_UPDATE_EVENT:
            handle_queue_update_event(s, bf, msg, lp);
            break;
        case GEN_PACKET_EVENT:
            handle_gen_packet_event(s, bf, msg, lp);
            break;
        default:
            printf("Error: Unknown event type %d\n", msg->type);
            break;
    }
}

/**
 * Reverse event handler for rollbacks
 */
void torus_3d_reverse_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp) {
    // Skip invalid nodes (LPs that don't map to a node in our model)
    if (s->node_id == UINT_MAX) {
        return;
    }
    // Dispatch to specific reverse handler based on message type
    switch (msg->type) {
        case ARRIVE_EVENT:
            reverse_arrive_event(s, bf, msg, lp);
            break;
        case DEPART_EVENT:
            reverse_depart_event(s, bf, msg, lp);
            break;
        case QUEUE_UPDATE_EVENT:
            reverse_queue_update_event(s, bf, msg, lp);
            break;
        case GEN_PACKET_EVENT:
            reverse_gen_packet_event(s, bf, msg, lp);
            break;
        default:
            printf("Error: Unknown event type in reverse %d\n", msg->type);
            break;
    }
    
    // Restore previous state
    s->packet_queue_size = msg->prev_queue_size;
}

/**
 * Finalization function
 */
void torus_3d_final(torus_3d_node_state* s, tw_lp* lp) {
    // Skip invalid nodes (LPs that don't map to a node in our model)
    if (s->node_id == UINT_MAX) {
        return;
    }
    
    // Clean up node resources
    free(s->packet_queue);
    
    if (s->queue_knowledge) {
        free(s->queue_knowledge->queue_sizes);
        free(s->queue_knowledge->queue_timestamps);
        free(s->queue_knowledge->known_queues);
        free(s->queue_knowledge);
    }
    
    // Clean up path finding arrays
    free(s->path_g_scores);
    free(s->path_f_scores);
    free(s->path_came_from);
    free(s->path_in_closed_set);
    free(s->path_in_open_set);
    free(s->path_open_set);
    
    // Collect and report statistics
    float avg_wait = s->depart_count > 0 ? s->total_waiting_time / s->depart_count : 0.0f;
    float avg_packet_time = s->num_packets_completed > 0 ? s->total_packet_time / s->num_packets_completed : 0.0f;
    
    /* if (s->arrive_count > 0 || s->depart_count > 0) {
        printf("Node %zu: Arrivals: %zu, Departures: %zu, Max Queue: %zu, Completed Packets: %zu, Avg Wait: %f, Avg Packet Delay: %f\n",
              s->node_id, s->arrive_count, s->depart_count, 
              s->max_queue_length, s->num_packets_completed, 
              avg_wait, avg_packet_time);
    } */
    
    // Final statistics will be aggregated by ROSS
}

/**
 * Main function to start the simulation
 */
int main(int argc, char** argv) {
    // Set default parameters
    g_params.grid_size_x = 16;
    g_params.grid_size_y = 16;
    g_params.grid_size_z = 16;
    g_params.hop_radius = 4;
    g_params.num_servers_per_node = 1;
    g_params.max_num_arrive_events = 100;
    g_params.queue_capacity_per_node = 512;  // Much larger default queue capacity
    
    g_params.min_intra_arrival_time = 10.0f;
    g_params.mode_intra_arrival_time = 12.0f;
    g_params.max_intra_arrival_time = 16.0f;
    
    g_params.min_service_time = 2.0f;
    g_params.mode_service_time = 3.0f;
    g_params.max_service_time = 4.0f;
    
    g_params.min_transit_time = 1.0f;
    g_params.mode_transit_time = 1.5f;
    g_params.max_transit_time = 2.0f;
	
	// Add near the other default parameter initializations
	g_params.lookahead = 1.0;       // Default lookahead
	g_params.update_delta = 1;      // Default update delta
    
    //g_params.nodes_per_kp = 64;  // Default: 64 nodes per kernel process
    //g_params.nodes_per_pe = 512; // Default: 512 nodes per processing element
    
    g_tw_lookahead = 1.0;  // Set lookahead to 1.0 time units
    
    // Add our command line options
    tw_opt_add(torus_options);
    
    // Initialize ROSS
    tw_init(&argc, &argv);
	
	// Use the lookahead parameter
	g_tw_lookahead = g_params.lookahead;
    
    // Set a much larger number of events per PE
    // This is needed for our dense torus network with many events
    g_tw_events_per_pe = 1000000;  // Default may be too small for our model
	
	// Spatial decomposition
	size_t total_nodes = g_params.grid_size_x * g_params.grid_size_y * g_params.grid_size_z;
	g_params.nodes_per_pe = (total_nodes + tw_nnodes() - 1) / tw_nnodes();
	g_params.nodes_per_kp = g_params.nodes_per_pe / 4;
    
    // Print configuration info
    printf("Torus 3D Configuration:\n");
    printf("Grid Size: %zu x %zu x %zu\n", 
           g_params.grid_size_x, g_params.grid_size_y, g_params.grid_size_z);
    printf("Hop Radius: %zu\n", g_params.hop_radius);
    printf("Servers per Node: %zu\n", g_params.num_servers_per_node);
    printf("Max Arrive Events: %zu\n", g_params.max_num_arrive_events);
    printf("Queue Capacity per Node: %zu\n", g_params.queue_capacity_per_node);
    printf("Events per PE: %u\n", g_tw_events_per_pe);
	printf("Nodes per PE: %lu\n", g_params.nodes_per_pe);
	printf("Nodes per KP: %lu\n", g_params.nodes_per_kp);
    
    // Allocate global resources
    size_t total_packets = total_nodes * g_params.max_num_arrive_events;
    
    g_packets = (torus_3d_packet*)calloc(total_packets, sizeof(torus_3d_packet));
    if (!g_packets) {
        fprintf(stderr, "Error: Failed to allocate packet array\n");
        tw_end();
        return 1;
    }
	
    // Initialize all packets
    for (size_t i = 0; i < total_packets; i++) {
        g_packets[i].id = i;
        g_packets[i].gen_time = -1.0;
        g_packets[i].exit_time = -1.0;
        g_packets[i].visited_node_cnt = 0;
        g_packets[i].queue_history_count = 0;
        g_packets[i].queue_history_index = 0;
    }
    
    g_queue_sizes = (short*)calloc(total_nodes, sizeof(short));
    if (!g_queue_sizes) {
        fprintf(stderr, "Error: Failed to allocate queue sizes array\n");
        free(g_packets);
        tw_end();
        return 1;
    }
	
	// In torus_3d_ross_model.c, add after allocating g_queue_sizes
	g_last_reported_size = (short*)calloc(total_nodes, sizeof(short));
	if (!g_last_reported_size) {
		fprintf(stderr, "Error: Failed to allocate last reported size array\n");
		free(g_packets);
		free(g_queue_sizes);
		tw_end();
		return 1;
	}
	// Initialize each element to -1
	for (size_t i = 0; i < total_nodes; i++) {
		g_last_reported_size[i] = -1;
	}
    
    // Initialize all queue sizes to -num_servers_per_node (available servers)
    short init_queue_val = -1 * g_params.num_servers_per_node;
    for (size_t i = 0; i < total_nodes; i++) {
        g_queue_sizes[i] = init_queue_val;
    }
    
    // Set up ROSS model
    // Calculate correct LPs per PE - divide total nodes evenly by number of PEs
    size_t nlp_per_pe = (total_nodes + tw_nnodes() - 1) / tw_nnodes();
    
    // Set up ROSS model with correct number of LPs
    tw_define_lps(nlp_per_pe, sizeof(torus_3d_message));
    
    // Set LP type for all LPs
    for (tw_lpid i = 0; i < g_tw_nlp; i++) {
        tw_lp_settype(i, &torus_3d_lp_type);
    }
	
    // Run the simulation
    tw_run();
    
    // Calculate local statistics
    float local_time = 0.0f;
    float global_max_time = 0.0f;
    unsigned long local_packet_count = 0;
    unsigned long global_packet_count = 0;
    float local_total_delay = 0.0f;
    float global_total_delay = 0.0f;
    
    // Collect statistics from local LPs
    if (g_tw_nlp > 0) {
        local_time = 0.0f;
        // Sum up finished_packet_count and total_delay across all local LPs
        for (int i = 0; i < g_tw_nlp; i++) {
			float lp_time = (float)tw_now(g_tw_lp[i]);
			if (lp_time > local_time) {
				local_time = lp_time;
			}
            torus_3d_node_state *s = (torus_3d_node_state *)tw_getstate(g_tw_lp[i]);
            if (s->node_id != UINT_MAX) {
                local_packet_count += s->num_packets_completed;
                local_total_delay += s->total_packet_time;
            }
        }
    }
    
    // Perform MPI reductions to gather global statistics
    MPI_Reduce(&local_time, &global_max_time, 1, MPI_FLOAT, MPI_MAX, 0, MPI_COMM_WORLD);
    MPI_Reduce(&local_packet_count, &global_packet_count, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, MPI_COMM_WORLD);
    MPI_Reduce(&local_total_delay, &global_total_delay, 1, MPI_FLOAT, MPI_SUM, 0, MPI_COMM_WORLD);
    
    if (tw_ismaster()) {
        printf("\n3D Torus Network Model Statistics:\n");
        printf("\t%-50s %11.4f\n", "Final Simulation Time (global max)", global_max_time);
        printf("\t%-50s %11lu\n", "Total Finished Packets", global_packet_count);
        
        // Calculate and print global average packet delay
        if (global_packet_count > 0) {
            float global_avg_delay = global_total_delay / global_packet_count;
            printf("\t%-50s %11.4f\n", "Global Average Packet Delay", global_avg_delay);
        } else {
            printf("\t%-50s %11s\n", "Global Average Packet Delay", "N/A (no packets)");
        }
    }
    
    // Clean up global resources
    free(g_packets);
    free(g_queue_sizes);
	free(g_last_reported_size);
    
    // Finalize ROSS
    tw_end();
    
    return 0;
}

/**
 * ROSS LP type definition - must be after all function definitions
 */
tw_lptype torus_3d_lp_type = {
    (init_f) torus_3d_init,
    (pre_run_f) torus_3d_prerun,
    (event_f) torus_3d_event,
    (revent_f) torus_3d_reverse_event,
    (commit_f) NULL,
    (final_f) torus_3d_final,
    (map_f) torus_3d_map,
    sizeof(torus_3d_node_state)
};

/**
 * LP type forward declaration
 */
tw_lptype torus_3d_lp_type;