/**
 * torus_3d_events.c
 * 
 * Implementation of event handlers for the Torus 3D ROSS model
 */

#include "torus_3d_ross.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define TORUS_3D_PACKET_DEBUG 0    // Set to 1 to enable packet debugging

// Function to get the LP ID for a given node ID (declaration)
extern tw_lpid get_lp_for_node(size_t node_id);

/**
 * Handle packet generation event
 */
void handle_gen_packet_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp) {
    // Save LP state before modification for rollback
    msg->prev_queue_size = s->packet_queue_size;
    
    // Generate a new packet
    size_t total_nodes = g_params.grid_size_x * g_params.grid_size_y * g_params.grid_size_z;
    size_t dest_node_id = get_random_node_id(s, lp);
    
    // Ensure destination isn't the same as source
    while (dest_node_id == s->node_id) {
        dest_node_id = get_random_node_id(s, lp);
    }
    
    // Calculate destination coordinates
    size_t dest_x, dest_y, dest_z;
    get_coords_from_node_id(dest_node_id, &dest_x, &dest_y, &dest_z);
    
    // Create a new packet
    size_t packet_id = s->node_id * g_params.max_num_arrive_events + s->num_intra_arrive_events;
    torus_3d_packet* packet = &g_packets[packet_id];
    
    // Initialize packet data
    packet->id = packet_id;
    packet->gen_time = tw_now(lp);
    packet->exit_time = -1.0; // Not yet exited
    packet->origin_node_id = s->node_id;
    packet->dest_node_id = dest_node_id;
    packet->dest_x = dest_x;
    packet->dest_y = dest_y;
    packet->dest_z = dest_z;
    
    #if TORUS_3D_PACKET_DEBUG
    printf("PACKET_DEBUG: Created packet %zu from node %zu to dest %zu (%zu,%zu,%zu)\n",
           packet_id, s->node_id, packet->dest_node_id, packet->dest_x, packet->dest_y, packet->dest_z);
    #endif
    
    // Initialize path tracking
    packet->visited_node_cnt = 0;
    packet->queue_history_count = 0;
    packet->queue_history_index = 0;
    
    // Record first node visit
    add_network_node_data(packet, (float)tw_now(lp), s->node_id, s->packet_queue_size);

    //printf("Node %zu generating packet %zu to dest %zu (%zu,%zu,%zu) at time %f\n", 
    //       s->node_id, packet_id, dest_node_id, dest_x, dest_y, dest_z, packet->gen_time);
    
    // Create an arrive event for this node with the new packet
    // Use the LP ID of the current node
    tw_event* e = tw_event_new(s->lp_id, 1.00001*g_tw_lookahead, lp);
    torus_3d_message* new_msg = tw_event_data(e);
    new_msg->type = ARRIVE_EVENT;
    new_msg->packet_id = packet_id;
	new_msg->source_node_id = s->node_id;
    new_msg->origin_node_id = s->node_id;
    new_msg->dest_node_id = dest_node_id;
    new_msg->dest_x = dest_x;
    new_msg->dest_y = dest_y;
    new_msg->dest_z = dest_z;
    tw_event_send(e);
    
    // Schedule next packet generation if not at limit
    s->num_intra_arrive_events++;
    
    if (s->num_intra_arrive_events < g_params.max_num_arrive_events) {
        // Calculate next arrival delay using triangular distribution
        float arrival_delay = generate_triangular_rv(
            g_params.min_intra_arrival_time,
            g_params.mode_intra_arrival_time,
            g_params.max_intra_arrival_time,
            lp
        );
        
        tw_event* next_gen = tw_event_new(s->lp_id, arrival_delay - 1.00001*g_tw_lookahead, lp);
        torus_3d_message* next_gen_msg = tw_event_data(next_gen);
        next_gen_msg->type = GEN_PACKET_EVENT;
        next_gen_msg->packet_id = -1; // Flag for generation events
        tw_event_send(next_gen);
        
        //printf("Node %zu scheduling next packet generation at time %f\n", 
        //       s->node_id, tw_now(lp) + arrival_delay);
    } else {
        //printf("Node %zu reached max packet generation limit (%zu)\n", 
        //       s->node_id, g_params.max_num_arrive_events);
    }
}

/**
 * Reverse the packet generation event
 */
void reverse_gen_packet_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp) {
    // Decrement packet count
    s->num_intra_arrive_events--;
    
    // The packet will be handled by reverse_arrive_event
    // We just need to restore model state
    s->packet_queue_size = msg->prev_queue_size;
    
    // Restore RNG state
    tw_rand_reverse_unif(lp->rng);
    tw_rand_reverse_unif(lp->rng);
}

/**
 * Handle packet arrival
 */
void handle_arrive_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp) {
    // Get the packet
    torus_3d_packet* packet = get_packet(msg->packet_id);
    if (!packet) {
        printf("Error: Invalid packet ID %zu in arrive event\n", msg->packet_id);
        return;
    }
	
	// Preserve packet history data in the message for cross-PE transfer
	size_t prev_hop_node = msg->source_node_id;
	if (!is_node_in_same_lp(s->node_id, prev_hop_node)) {
		// Save critical packet metadata to message
		packet->origin_node_id = msg->origin_node_id;
		packet->dest_node_id = msg->dest_node_id;
		packet->dest_x = msg->dest_x;
		packet->dest_y = msg->dest_y;
		packet->dest_z = msg->dest_z;
		packet->visited_node_cnt = msg->visited_node_cnt;
		packet->gen_time = msg->orig_gen_time;
		//packet->node_arrival_times[0] = msg->first_arrival_time;
		packet->visited_node_cnt = msg->hop_count;
		//memcpy(packet->node_arrival_times, msg->node_arrival_times, T3D_MAX_PATH_LENGTH * sizeof(double));
		//memcpy(packet->visited_nodes, msg->visited_nodes, T3D_MAX_PATH_LENGTH * sizeof(size_t));
		memcpy(packet->recent_queue_sizes, msg->recent_queue_sizes, T3D_QUEUE_HISTORY_SIZE * sizeof(short));
		memcpy(packet->recent_queue_nodes, msg->recent_queue_nodes, T3D_QUEUE_HISTORY_SIZE * sizeof(size_t));
		memcpy(packet->recent_queue_times, msg->recent_queue_times, T3D_QUEUE_HISTORY_SIZE * sizeof(float));
		packet->queue_history_count = msg->queue_history_count;
		packet->queue_history_index = msg->queue_history_index;
		
		//printf("CROSS-PE: Updating packet %zu metadata: hops=%zu, gen_time=%f\n", 
		//	   packet->id, packet->visited_node_cnt, packet->gen_time);
	}
    
    // Update statistics
    s->arrive_count++;
    
    // Check if packet has reached destination
    bf->c0 = (s->node_id == packet->dest_node_id); // At destination flag
    bf->c1 = (s->packet_queue_size < 0);           // Server available flag
    
    /* // Fix corrupted packet destination if necessary (safety check)
    if (packet->dest_node_id != msg->dest_node_id) {
        printf("ERROR: Packet %zu destination corrupted! Was %zu, fixing to %zu\n", 
               packet->id, packet->dest_node_id, msg->dest_node_id);
        packet->dest_node_id = msg->dest_node_id;
        get_coords_from_node_id(packet->dest_node_id, &packet->dest_x, &packet->dest_y, &packet->dest_z);
        
        // Update destination flag
        bf->c0 = (s->node_id == packet->dest_node_id);
    } */
    
    //printf("Packet %zu arrived at node %zu, destination=%zu, at_dest=%d, server_avail=%d, time=%f\n", 
    //       packet->id, s->node_id, packet->dest_node_id, bf->c0, bf->c1, tw_now(lp));
    
    // Update global queue size knowledge from incoming packet
    update_queue_knowledge(s, packet);
    
    // Add current node to packet's path with queue size
    bool quit_packet = add_network_node_data(packet, (float)tw_now(lp), s->node_id, s->packet_queue_size);
	
	if (quit_packet) {
		// Packet has reached destination
        packet->exit_time = tw_now(lp);
        
        // Update packet completion statistics
        s->num_packets_completed++;
        s->total_packet_time += (float)(packet->exit_time - packet->gen_time);
        
        // Record statistics
        float time_in_network = (float)(packet->exit_time - packet->gen_time);
        s->total_waiting_time += time_in_network;
		
		printf("Packet %zu QUITTING at node %zu (destination), delay=%f, hops=%d\n", 
               packet->id, s->node_id, time_in_network, packet->visited_node_cnt);
			   
		return;
	}
    
    if (!bf->c0) {
        // Packet needs to continue through network
        s->packet_queue_size++;
        // Update shared memory queue sizes for routing decisions
        g_queue_sizes[s->node_id] = s->packet_queue_size;
        
        if (!bf->c1) {
            // Server not available, add to queue
            if (s->queue_count < s->queue_capacity) {
                s->packet_queue[s->queue_tail] = msg->packet_id;
                s->queue_tail = (s->queue_tail + 1) % s->queue_capacity;
                s->queue_count++;
                
                //printf("Packet %zu queued at node %zu, queue size now %zu\n", 
                //       packet->id, s->node_id, s->queue_count);
                
                // Update max queue statistics
                if (s->queue_count > s->max_queue_length) {
                    s->max_queue_length = s->queue_count;
                }
            } else {
                printf("Error: Queue overflow at node %zu\n", s->node_id);
            }
        }
    } else {
        // Packet has reached destination
        packet->exit_time = tw_now(lp);
        
        // Update packet completion statistics
        s->num_packets_completed++;
        s->total_packet_time += (float)(packet->exit_time - packet->gen_time);
        
        // Record statistics
        float time_in_network = (float)(packet->exit_time - packet->gen_time);
        s->total_waiting_time += time_in_network;
        
        //printf("Packet %zu COMPLETED at node %zu (destination), delay=%f, hops=%zu\n", 
        //       packet->id, s->node_id, time_in_network, packet->visited_node_cnt);
    }
    
    // If server is available and packet needs to continue, schedule departure
    if (!bf->c0 && bf->c1) {
        // Calculate service time
        float base_service_time;
        if (s->slow_node) {
            // Slow nodes have 10x service time
            base_service_time = generate_triangular_rv(
                g_params.min_service_time * 10.0f,
                g_params.mode_service_time * 10.0f,
                g_params.max_service_time * 10.0f,
                lp
            );
        } else {
            base_service_time = generate_triangular_rv(
                g_params.min_service_time,
                g_params.mode_service_time,
                g_params.max_service_time,
                lp
            );
        }
        
        // Create departure event
        tw_event* e = tw_event_new(s->lp_id, base_service_time, lp);
        torus_3d_message* new_msg = tw_event_data(e);
        new_msg->type = DEPART_EVENT;
        new_msg->packet_id = msg->packet_id;
        new_msg->origin_node_id = msg->origin_node_id;
        new_msg->dest_node_id = msg->dest_node_id;
        new_msg->dest_x = msg->dest_x;
        new_msg->dest_y = msg->dest_y;
        new_msg->dest_z = msg->dest_z;
        tw_event_send(e);
        
        //printf("Packet %zu immediately processing at node %zu, service time=%f\n", 
        //       packet->id, s->node_id, base_service_time);
        
        // Save the packet being processed for potential rollback
        s->saved_packet_id = msg->packet_id;
    }
    
    // Only send queue updates if queue size changed significantly (at least 5)
    // This reduces unnecessary event traffic
    if (abs(s->packet_queue_size - g_last_reported_size[s->node_id]) >= g_params.update_delta) {
        // Send queue size updates to neighbors in different LPs
        for (size_t i = 0; i < s->neighbor_info.all_neighbor_count; i++) {
            size_t neighbor_id = s->neighbor_info.all_neighbors[i];
            
            // Only send update if neighbor is in a different LP
            if (!is_node_in_same_lp(s->node_id, neighbor_id)) {
                // Calculate LP ID for neighbor node
                tw_lpid neighbor_lp = get_lp_for_node(neighbor_id);
                
                tw_event* e = tw_event_new(neighbor_lp, 1.00001*g_tw_lookahead, lp); // Small delay for update
                torus_3d_message* update_msg = tw_event_data(e);
                memset(update_msg, 0, sizeof(torus_3d_message));  // Zero all fields
                update_msg->type = QUEUE_UPDATE_EVENT;
                update_msg->source_node_id = s->node_id;
                update_msg->queue_size = s->packet_queue_size;
                update_msg->update_time = tw_now(lp);
                tw_event_send(e);
            }
        }
        
        // Update the last reported size
        g_last_reported_size[s->node_id] = s->packet_queue_size;
    }
}

/**
 * Reverse packet arrival
 */
void reverse_arrive_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp) {
    torus_3d_packet* packet = get_packet(msg->packet_id);
    if (!packet) {
        printf("Error: Invalid packet ID %zu in reverse arrive\n", msg->packet_id);
        return;
    }
    
    // Reverse statistics
    s->arrive_count--;
    
    // If server is available and packet needs to continue, need to reverse the departure event
    if (!bf->c0 && bf->c1) {
        // Reverse RNG calls
        tw_rand_reverse_unif(lp->rng);
    }
    
    // Undo packet path updates
    if (packet->visited_node_cnt > msg->packet_path_size_before) {
        packet->visited_node_cnt = msg->packet_path_size_before;
    }
    
    // Undo queue updates
    if (!bf->c0) {
        // Packet was in transit
        s->packet_queue_size--;
        g_queue_sizes[s->node_id] = s->packet_queue_size;
        
        if (!bf->c1) {
            // Packet was added to queue, remove it
            s->queue_count--;
            s->queue_tail = (s->queue_tail == 0) ? 
                            s->queue_capacity - 1 : s->queue_tail - 1;
        }
    } else {
        // Packet was at destination, undo exit time
        packet->exit_time = -1.0;
        
        // Reverse packet completion statistics
        s->num_packets_completed--;
        s->total_packet_time -= (float)(tw_now(lp) - packet->gen_time);
        
        // Reverse statistics
        float time_in_network = (float)(packet->exit_time - packet->gen_time);
        s->total_waiting_time -= time_in_network;
    }
    
    // Any RNG calls made need to be reversed
    tw_rand_reverse_unif(lp->rng);
}

/**
 * Handle packet departure
 */
void handle_depart_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp) {
    // Get the packet
    torus_3d_packet* packet = get_packet(msg->packet_id);
    if (!packet) {
        printf("Error: Invalid packet ID %zu in depart event\n", msg->packet_id);
        return;
    }
    
    // Update statistics
    s->depart_count++;
    
    // Update local queue size data
    update_local_queue_sizes(s, (float)tw_now(lp));
    
    // Check if queue has packets
    bf->c0 = (s->queue_count > 0);
    
    // Decrease queue size for this departure
    s->packet_queue_size--;
    g_queue_sizes[s->node_id] = s->packet_queue_size;
    
    //printf("Packet %zu departing from node %zu, queue_size=%d, queued_packets=%zu, time=%f, destination=%zu\n", 
    //       packet->id, s->node_id, s->packet_queue_size, s->queue_count, tw_now(lp), packet->dest_node_id);
    
    // Get next packet from queue if available
    tw_lpid queue_packet_id = -1;
    if (bf->c0) {
        queue_packet_id = s->packet_queue[s->queue_head];
        s->queue_head = (s->queue_head + 1) % s->queue_capacity;
        s->queue_count--;
        
        //printf("Node %zu dequeuing next packet %d, remaining queue=%zu\n", 
        //       s->node_id, queue_packet_id, s->queue_count);
    }
    
    // Add current node to packet's path with updated queue size
    add_network_node_data(packet, (float)tw_now(lp), s->node_id, s->packet_queue_size);
    
    // Find the best path to destination
    torus_3d_path path = find_shortest_path(
        s, 
        packet->dest_node_id,
		packet->visited_node_cnt,
        (float)tw_now(lp),
        lp
    );
    
    // Double-check destination hasn't been corrupted
    /* if (path.node_indices[path.length-1] != packet->dest_node_id) {
        printf("ERROR: Path destination %zu doesn't match packet destination %zu - FIXING\n", 
               path.node_indices[path.length-1], packet->dest_node_id);
        // Fix the path by ensuring last node is the actual destination
        path.node_indices[path.length-1] = packet->dest_node_id;
    } */
    
    // Determine next hop direction
    size_t next_hop_dir = next_hop_direction(s, &path);
    size_t next_hop_node = s->neighbor_info.direct_neighbors[next_hop_dir];
    
    // Check if the next hop node is valid
    if (next_hop_node == UINT_MAX) {
        printf("ERROR: Invalid next hop for packet %zu from node %zu to destination %zu\n", 
               packet->id, s->node_id, packet->dest_node_id);
        return;
    }
    
    // Schedule arrival at next node
    float transit_time = generate_triangular_rv(
        g_params.min_transit_time,
        g_params.mode_transit_time,
        g_params.max_transit_time,
        lp
    );
    
    // Get the correct LP ID for the next hop node
    tw_lpid next_hop_lp = get_lp_for_node(next_hop_node);
    
    //printf("Packet %zu routing from node %zu to next hop %zu (dir=%zu) on LP %lu, transit time=%f, dest=%zu\n", 
    //       packet->id, s->node_id, next_hop_node, next_hop_dir, next_hop_lp, transit_time, 
    //       packet->dest_node_id);
    
    tw_event* e = tw_event_new(next_hop_lp, transit_time, lp);
    torus_3d_message* new_msg = tw_event_data(e);
    new_msg->type = ARRIVE_EVENT;
    new_msg->packet_id = msg->packet_id;
	new_msg->source_node_id = s->node_id;
	
	// Preserve packet history data in the message for cross-PE transfer
	if (!is_node_in_same_lp(s->node_id, next_hop_node)) {
		// Save critical packet metadata to message
		new_msg->origin_node_id = packet->origin_node_id;
		new_msg->dest_node_id = packet->dest_node_id;
		new_msg->dest_x = packet->dest_x;
		new_msg->dest_y = packet->dest_y;
		new_msg->dest_z = packet->dest_z;
		new_msg->visited_node_cnt = packet->visited_node_cnt;
		new_msg->orig_gen_time = packet->gen_time;
		//new_msg->first_arrival_time = packet->node_arrival_times[0];
		new_msg->hop_count = packet->visited_node_cnt;
		//memcpy(new_msg->node_arrival_times, packet->node_arrival_times, T3D_MAX_PATH_LENGTH * sizeof(double));
		//memcpy(new_msg->visited_nodes, packet->visited_nodes, T3D_MAX_PATH_LENGTH * sizeof(size_t));
		memcpy(new_msg->recent_queue_sizes, packet->recent_queue_sizes, T3D_QUEUE_HISTORY_SIZE * sizeof(short));
		memcpy(new_msg->recent_queue_nodes, packet->recent_queue_nodes, T3D_QUEUE_HISTORY_SIZE * sizeof(size_t));
		memcpy(new_msg->recent_queue_times, packet->recent_queue_times, T3D_QUEUE_HISTORY_SIZE * sizeof(float));
		new_msg->queue_history_count = packet->queue_history_count;
		new_msg->queue_history_index = packet->queue_history_index;
		
		//printf("CROSS-PE: Saving packet %zu metadata: hops=%zu, gen_time=%f\n", 
		//	   packet->id, packet->visited_node_cnt, packet->gen_time);
	}
	
    tw_event_send(e);
    
    // Save destination for rollback
    s->saved_dest_lp = next_hop_lp;
    msg->prev_dest_lp = next_hop_lp;
    
    // If queue had packets, schedule service for the next packet
    if (bf->c0) {
        float service_time;
        if (s->slow_node) {
            service_time = generate_triangular_rv(
                g_params.min_service_time * 10.0f,
                g_params.mode_service_time * 10.0f,
                g_params.max_service_time * 10.0f,
                lp
            );
        } else {
            service_time = generate_triangular_rv(
                g_params.min_service_time,
                g_params.mode_service_time,
                g_params.max_service_time,
                lp
            );
        }
        
        tw_event* next_e = tw_event_new(s->lp_id, service_time, lp);
        torus_3d_message* next_msg = tw_event_data(next_e);
        next_msg->type = DEPART_EVENT;
        next_msg->packet_id = queue_packet_id;
        
        // Get this packet's destination info
        torus_3d_packet* next_packet = get_packet(queue_packet_id);
        if (next_packet) {
            next_msg->origin_node_id = next_packet->origin_node_id;
            next_msg->dest_node_id = next_packet->dest_node_id;
            next_msg->dest_x = next_packet->dest_x;
            next_msg->dest_y = next_packet->dest_y;
            next_msg->dest_z = next_packet->dest_z;
            
            //printf("Node %zu scheduling service for next packet %d to destination %zu, service time=%f\n", 
            //       s->node_id, queue_packet_id, next_packet->dest_node_id, service_time);
        }
        
        tw_event_send(next_e);
    }
    
    // Only send queue updates if queue size changed significantly (at least 5)
    // This reduces unnecessary event traffic
    if (abs(s->packet_queue_size - g_last_reported_size[s->node_id]) >= g_params.update_delta) {
        // Send queue size updates to neighbors in different LPs
        for (size_t i = 0; i < s->neighbor_info.all_neighbor_count; i++) {
            size_t neighbor_id = s->neighbor_info.all_neighbors[i];
            
            // Only send update if neighbor is in a different LP
            if (!is_node_in_same_lp(s->node_id, neighbor_id)) {
                // Calculate LP ID for neighbor node
				tw_lpid neighbor_lp = get_lp_for_node(neighbor_id);
                
                tw_event* update_e = tw_event_new(neighbor_lp, 1.00001*g_tw_lookahead, lp); // Small delay
                torus_3d_message* update_msg = tw_event_data(update_e);
                memset(update_msg, 0, sizeof(torus_3d_message));  // Zero all fields
                update_msg->type = QUEUE_UPDATE_EVENT;
                update_msg->source_node_id = s->node_id;
                update_msg->queue_size = s->packet_queue_size;
                update_msg->update_time = tw_now(lp);
                tw_event_send(update_e);
            }
        }
        
        // Update the last reported size
        g_last_reported_size[s->node_id] = s->packet_queue_size;
    }
}

/**
 * Reverse packet departure
 */
void reverse_depart_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp) {
    // Undo statistics update
    s->depart_count--;
    
    // Restore queue state
    s->packet_queue_size++;
    g_queue_sizes[s->node_id] = s->packet_queue_size;
    
    // Restore queue head if there was a packet in queue
    if (bf->c0) {
        s->queue_head = (s->queue_head == 0) ? 
                        s->queue_capacity - 1 : s->queue_head - 1;
        s->queue_count++;
        
        // Need to reverse the service time RNG
        tw_rand_reverse_unif(lp->rng);
    }
    
    // Reverse the transit time RNG
    tw_rand_reverse_unif(lp->rng);
    
    // Reverse any path calculation RNGs
    // Number depends on implementation of find_shortest_path
    // Simplified for this example
    tw_rand_reverse_unif(lp->rng);
    
    // Get the packet and restore state
    torus_3d_packet* packet = get_packet(msg->packet_id);
    if (packet) {
        if (packet->visited_node_cnt > msg->packet_path_size_before) {
            packet->visited_node_cnt = msg->packet_path_size_before;
        }
    }
}

/**
 * Handle queue size update event
 */
void handle_queue_update_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp) {
    // Save previous state for potential rollback
    msg->prev_queue_size = -1;
    
    // Only store if we have a valid queue_knowledge
    if (s->queue_knowledge) {
        size_t source_id = msg->source_node_id;
        short queue_size = msg->queue_size;
        float update_time = (float)msg->update_time;
        
        // Save the previous value
        msg->prev_queue_size = s->queue_knowledge->queue_sizes[source_id];
        
        // Update queue knowledge if newer
        if (update_time > s->queue_knowledge->queue_timestamps[source_id]) {
            s->queue_knowledge->queue_sizes[source_id] = queue_size;
            s->queue_knowledge->queue_timestamps[source_id] = update_time;
            s->queue_knowledge->known_queues[source_id] = true;
        }
    }
}

/**
 * Reverse queue size update
 */
void reverse_queue_update_event(torus_3d_node_state* s, tw_bf* bf, torus_3d_message* msg, tw_lp* lp) {
    // Restore previous queue size knowledge
    if (s->queue_knowledge && msg->prev_queue_size != -1) {
        size_t source_id = msg->source_node_id;
        s->queue_knowledge->queue_sizes[source_id] = msg->prev_queue_size;
    }
}