#include "PHOLD/PHOLD_Model.h"
#include "Ring_1D/Ring_1D.h"
#include "Irregular_NoC/Irregular_NoC.h"
#include "Grid_VN2D/Grid_VN2D.h"
#include "Grid_VN3D/Grid_VN3D.h"
#include "Torus_3D/Torus_3D.h"

#include <iostream>
#include <fstream>
#include <string>

int main(int argc, char* argv[])
{
    std::string model_name;
	size_t network_size;
    size_t ring_size;
    size_t grid_size_x;
    size_t grid_size_y;
    size_t grid_size_z;
    size_t hop_radius;
    size_t num_servers_per_network_node;
	size_t max_num_arrive_events;
    size_t max_num_intra_arrive_events;
    double max_sim_time;
    size_t num_threads;
    size_t dist_seed;
    double bucket_width;
    size_t target_bin_size;
    std::string dist_params_file;
	std::string network_topology_file;
    
    std::string line;
	std::ifstream in_file(argv[1]);
	if (in_file.is_open()) {
        getline (in_file, line, ':');  	in_file >> model_name;  			    std::cout << line << ": " << model_name;

		if ("PHOLD_benchmark" == model_name) {
            getline (in_file, line, ':');  	in_file >> network_size;  			    std::cout << line << ": " << network_size;
            getline (in_file, line, ':');  	in_file >> max_num_arrive_events; 		std::cout << line << ": " << max_num_arrive_events;
            getline (in_file, line, ':');  	in_file >> max_sim_time;                std::cout << line << ": " << max_sim_time;
            getline (in_file, line, ':');	in_file >> num_threads;  		        std::cout << line << ": " << num_threads;
            getline (in_file, line, ':');  	in_file >> dist_seed;                   std::cout << line << ": " << dist_seed;
            getline (in_file, line, ':');  	in_file >> bucket_width;                std::cout << line << ": " << bucket_width;
            getline (in_file, line, ':');  	in_file >> target_bin_size;             std::cout << line << ": " << target_bin_size;
            getline (in_file, line, ':');  	in_file >> dist_params_file;            std::cout << line << ": " << dist_params_file;

            std::cout << "\nPHOLD Benchmark" << std::endl;

            std::string params;
            size_t configPos = dist_params_file.rfind("params_");  // Use rfind instead of find
            if (configPos != std::string::npos) {
                size_t trialPos = dist_params_file.find("_trial", configPos);
                if (trialPos != std::string::npos) {
                    params = dist_params_file.substr(configPos + 7, trialPos - (configPos + 7));
                }
            }

            /* std::string trace_folder_name = "traces/trace_PHOLD_benchmark_network_size_" + std::to_string(network_size) +
            "_seed_" + std::to_string(dist_seed) +
            "_params_" + params +
            "_threads_" + std::to_string(num_threads) +
			"_bin_size_" + std::to_string(target_bin_size); */
			
			std::string trace_folder_name = "";

            PHOLD_Model PHOLD(network_size, max_num_arrive_events, max_sim_time, num_threads, dist_seed, bucket_width, target_bin_size, trace_folder_name, dist_params_file);
            PHOLD.SimulateModel();
            //PHOLD.PrintMeanPacketNetworkTime();
			//printf("\nHigh-level execution data:\n");
			//PHOLD.PrintSVs();
            //PHOLD.PrintNumVertexExecs();
        }

        else if ("1D_ring_network" == model_name) {
            getline (in_file, line, ':');  	in_file >> ring_size;  			    	std::cout << line << ": " << ring_size;
            getline (in_file, line, ':');  	in_file >> num_servers_per_network_node;std::cout << line << ": " << num_servers_per_network_node;
            getline (in_file, line, ':');  	in_file >> max_num_intra_arrive_events; std::cout << line << ": " << max_num_intra_arrive_events;
            getline (in_file, line, ':');  	in_file >> max_sim_time;                std::cout << line << ": " << max_sim_time;
            getline (in_file, line, ':');	in_file >> num_threads;  		        std::cout << line << ": " << num_threads;
            getline (in_file, line, ':');  	in_file >> dist_seed;                   std::cout << line << ": " << dist_seed;
            getline (in_file, line, ':');  	in_file >> bucket_width;                std::cout << line << ": " << bucket_width;
            getline (in_file, line, ':');  	in_file >> target_bin_size;             std::cout << line << ": " << target_bin_size;
            getline (in_file, line, ':');  	in_file >> dist_params_file;            std::cout << line << ": " << dist_params_file;

            std::cout << "\n1D Ring Network" << std::endl;

            std::string params;
            size_t configPos = dist_params_file.rfind("params_");  // Use rfind instead of find
            if (configPos != std::string::npos) {
                size_t trialPos = dist_params_file.find("_trial", configPos);
                if (trialPos != std::string::npos) {
                    params = dist_params_file.substr(configPos + 7, trialPos - (configPos + 7));
                }
            }

            /* std::string trace_folder_name = "traces/trace_1D_ring_network_size_" + std::to_string(ring_size) +
            "_seed_" + std::to_string(dist_seed) +
            "_params_" + params +
            "_threads_" + std::to_string(num_threads) +
            "_servers_" + std::to_string(num_servers_per_network_node) +
			"_bin_size_" + std::to_string(target_bin_size); */
			
			std::string trace_folder_name = "";

            Ring_1D ring_sim(ring_size, num_servers_per_network_node, max_num_intra_arrive_events, max_sim_time, num_threads, dist_seed, bucket_width, target_bin_size, trace_folder_name, dist_params_file);
            ring_sim.SimulateModel();
            ring_sim.PrintMeanPacketNetworkTime();
			//printf("\nHigh-level execution data:\n");
            //ring_sim.PrintSVs();
            //ring_sim.PrintNumVertexExecs();
            //ring_sim.PrintFinishedPackets();
        }

        else if ("VN2D_grid_network" == model_name) {
            getline (in_file, line, ':');  	in_file >> grid_size_x;  			    std::cout << line << ": " << grid_size_x;
            getline (in_file, line, ':');  	in_file >> grid_size_y;  			    std::cout << line << ": " << grid_size_y;
            getline (in_file, line, ':');  	in_file >> hop_radius;  			    std::cout << line << ": " << hop_radius;
            getline (in_file, line, ':');  	in_file >> num_servers_per_network_node;std::cout << line << ": " << num_servers_per_network_node;
            getline (in_file, line, ':');  	in_file >> max_num_intra_arrive_events; std::cout << line << ": " << max_num_intra_arrive_events;
            getline (in_file, line, ':');  	in_file >> max_sim_time;                std::cout << line << ": " << max_sim_time;
            getline (in_file, line, ':');	in_file >> num_threads;  		        std::cout << line << ": " << num_threads;
            getline (in_file, line, ':');  	in_file >> dist_seed;                   std::cout << line << ": " << dist_seed;
            getline (in_file, line, ':');  	in_file >> bucket_width;                std::cout << line << ": " << bucket_width;
            getline (in_file, line, ':');  	in_file >> target_bin_size;             std::cout << line << ": " << target_bin_size;
            getline (in_file, line, ':');  	in_file >> dist_params_file;            std::cout << line << ": " << dist_params_file;
        
            std::cout << "\n2D von Neumann Grid Network" << std::endl;
        
            std::string params;
            size_t configPos = dist_params_file.rfind("params_");  // Use rfind instead of find
            if (configPos != std::string::npos) {
                size_t trialPos = dist_params_file.find("_trial", configPos);
                if (trialPos != std::string::npos) {
                    params = dist_params_file.substr(configPos + 7, trialPos - (configPos + 7));
                }
            }

            /* std::string trace_folder_name = "traces/trace_VN2D_grid_network_size_" +
			std::to_string(grid_size_x) + "_" + std::to_string(grid_size_y) +
			"_hop_" + std::to_string(hop_radius) +
            "_seed_" + std::to_string(dist_seed) +
            "_params_" + params +
            "_threads_" + std::to_string(num_threads) +
            "_servers_" + std::to_string(num_servers_per_network_node) +
			"_bin_size_" + std::to_string(target_bin_size); */
			
			std::string trace_folder_name = "";
        
            Grid_VN2D grid_sim(grid_size_x, grid_size_y, hop_radius, num_servers_per_network_node, max_num_intra_arrive_events, max_sim_time, num_threads, dist_seed, bucket_width, target_bin_size, trace_folder_name, dist_params_file);
            grid_sim.SimulateModel();
            grid_sim.PrintMeanPacketNetworkTime();
			//printf("\nHigh-level execution data:\n");
            //grid_sim.PrintSVs();
            //grid_sim.PrintNumVertexExecs();
            //grid_sim.PrintFinishedPackets();
        }
        
        else if ("VN3D_grid_network" == model_name) {
            getline (in_file, line, ':');  	in_file >> grid_size_x;  			    std::cout << line << ": " << grid_size_x;
            getline (in_file, line, ':');  	in_file >> grid_size_y;  			    std::cout << line << ": " << grid_size_y;
            getline (in_file, line, ':');  	in_file >> grid_size_z;  			    std::cout << line << ": " << grid_size_z;
            getline (in_file, line, ':');  	in_file >> hop_radius;  			    std::cout << line << ": " << hop_radius;
            getline (in_file, line, ':');  	in_file >> num_servers_per_network_node;std::cout << line << ": " << num_servers_per_network_node;
            getline (in_file, line, ':');  	in_file >> max_num_intra_arrive_events; std::cout << line << ": " << max_num_intra_arrive_events;
            getline (in_file, line, ':');  	in_file >> max_sim_time;                std::cout << line << ": " << max_sim_time;
            getline (in_file, line, ':');	in_file >> num_threads;  		        std::cout << line << ": " << num_threads;
			getline (in_file, line, ':');  	in_file >> dist_seed;                   std::cout << line << ": " << dist_seed;
            getline (in_file, line, ':');  	in_file >> bucket_width;                std::cout << line << ": " << bucket_width;
            getline (in_file, line, ':');  	in_file >> target_bin_size;             std::cout << line << ": " << target_bin_size;
            getline (in_file, line, ':');  	in_file >> dist_params_file;            std::cout << line << ": " << dist_params_file;
        
            std::cout << "\n3D von Neumann Grid Network" << std::endl;
        
            std::string params;
            size_t configPos = dist_params_file.rfind("params_");  // Use rfind instead of find
            if (configPos != std::string::npos) {
                size_t trialPos = dist_params_file.find("_trial", configPos);
                if (trialPos != std::string::npos) {
                    params = dist_params_file.substr(configPos + 7, trialPos - (configPos + 7));
                }
            }
			
            /* std::string trace_folder_name = "traces/trace_VN3D_grid_network_size_" +
            std::to_string(grid_size_x) + "_" + std::to_string(grid_size_y) + "_" + std::to_string(grid_size_z) +
            "_hop_" + std::to_string(hop_radius) +
            "_seed_" + std::to_string(dist_seed) +
            "_params_" + params +
            "_threads_" + std::to_string(num_threads) +
            "_servers_" + std::to_string(num_servers_per_network_node) +
			"_bin_size_" + std::to_string(target_bin_size); */
			
			std::string trace_folder_name = "";
        
            Grid_VN3D grid_sim(grid_size_x, grid_size_y, grid_size_z, hop_radius, num_servers_per_network_node, max_num_intra_arrive_events, max_sim_time, num_threads, dist_seed, bucket_width, target_bin_size, trace_folder_name, dist_params_file);
            grid_sim.SimulateModel();
            grid_sim.PrintMeanPacketNetworkTime();
			//printf("\nHigh-level execution data:\n");
            //grid_sim.PrintSVs();
            //grid_sim.PrintNumVertexExecs();
            //grid_sim.PrintFinishedPackets();
        }
        
        else if ("3D_torus_network" == model_name) {
            getline (in_file, line, ':');  	in_file >> grid_size_x;  			    std::cout << line << ": " << grid_size_x;
            getline (in_file, line, ':');  	in_file >> grid_size_y;  			    std::cout << line << ": " << grid_size_y;
            getline (in_file, line, ':');  	in_file >> grid_size_z;  			    std::cout << line << ": " << grid_size_z;
            getline (in_file, line, ':');  	in_file >> hop_radius;  			    std::cout << line << ": " << hop_radius;
            getline (in_file, line, ':');  	in_file >> num_servers_per_network_node;std::cout << line << ": " << num_servers_per_network_node;
            getline (in_file, line, ':');  	in_file >> max_num_intra_arrive_events; std::cout << line << ": " << max_num_intra_arrive_events;
            getline (in_file, line, ':');  	in_file >> max_sim_time;                std::cout << line << ": " << max_sim_time;
            getline (in_file, line, ':');	in_file >> num_threads;  		        std::cout << line << ": " << num_threads;
            getline (in_file, line, ':');  	in_file >> dist_seed;                   std::cout << line << ": " << dist_seed;
            getline (in_file, line, ':');  	in_file >> bucket_width;                std::cout << line << ": " << bucket_width;
            getline (in_file, line, ':');  	in_file >> target_bin_size;             std::cout << line << ": " << target_bin_size;
            getline (in_file, line, ':');  	in_file >> dist_params_file;            std::cout << line << ": " << dist_params_file;
        
            std::cout << "\n3D Torus Network" << std::endl;
        
            std::string params;
            size_t configPos = dist_params_file.rfind("params_");  // Use rfind instead of find
            if (configPos != std::string::npos) {
                size_t trialPos = dist_params_file.find("_trial", configPos);
                if (trialPos != std::string::npos) {
                    params = dist_params_file.substr(configPos + 7, trialPos - (configPos + 7));
                }
            }
			
            /* std::string trace_folder_name = "traces/trace_3D_torus_network_size_" +
            std::to_string(grid_size_x) + "_" + std::to_string(grid_size_y) + "_" + std::to_string(grid_size_z) +
            "_hop_" + std::to_string(hop_radius) +
            "_seed_" + std::to_string(dist_seed) +
            "_params_" + params +
            "_threads_" + std::to_string(num_threads) +
            "_servers_" + std::to_string(num_servers_per_network_node) +
			"_bin_size_" + std::to_string(target_bin_size); */
			
			std::string trace_folder_name = "";
        
            Torus_3D torus_sim(grid_size_x, grid_size_y, grid_size_z, hop_radius, num_servers_per_network_node, max_num_intra_arrive_events, max_sim_time, num_threads, dist_seed, bucket_width, target_bin_size, trace_folder_name, dist_params_file);
            torus_sim.SimulateModel();
            torus_sim.PrintMeanPacketNetworkTime();
			//printf("\nHigh-level execution data:\n");
            //torus_sim.PrintSVs();
            //torus_sim.PrintNumVertexExecs();
            //torus_sim.PrintFinishedPackets();
        }
	
		else if ("Irregular_2D_NoC_grid_network" == model_name) {
            getline (in_file, line, ':');  	in_file >> grid_size_x;  			    std::cout << line << ": " << grid_size_x;
            getline (in_file, line, ':');  	in_file >> grid_size_y;  			    std::cout << line << ": " << grid_size_y;
            getline (in_file, line, ':');  	in_file >> hop_radius;  			    std::cout << line << ": " << hop_radius;
			getline (in_file, line, ':');  	in_file >> network_topology_file;       std::cout << line << ": " << network_topology_file;
            getline (in_file, line, ':');  	in_file >> num_servers_per_network_node;std::cout << line << ": " << num_servers_per_network_node;
            getline (in_file, line, ':');  	in_file >> max_num_intra_arrive_events; std::cout << line << ": " << max_num_intra_arrive_events;
            getline (in_file, line, ':');  	in_file >> max_sim_time;                std::cout << line << ": " << max_sim_time;
            getline (in_file, line, ':');	in_file >> num_threads;  		        std::cout << line << ": " << num_threads;
            getline (in_file, line, ':');  	in_file >> dist_seed;                   std::cout << line << ": " << dist_seed;
            getline (in_file, line, ':');  	in_file >> bucket_width;                std::cout << line << ": " << bucket_width;
            getline (in_file, line, ':');  	in_file >> target_bin_size;             std::cout << line << ": " << target_bin_size;
            getline (in_file, line, ':');  	in_file >> dist_params_file;            std::cout << line << ": " << dist_params_file;
        
            std::cout << "\n2D Irregular NoC" << std::endl;
        
            std::string params;
            size_t configPos = dist_params_file.rfind("params_");  // Use rfind instead of find
            if (configPos != std::string::npos) {
                size_t trialPos = dist_params_file.find("_trial", configPos);
                if (trialPos != std::string::npos) {
                    params = dist_params_file.substr(configPos + 7, trialPos - (configPos + 7));
                }
            }
			
            /*std::string trace_folder_name = "traces/trace_2D_irregular_NoC_size_" +
            std::to_string(grid_size_x) + "_" + std::to_string(grid_size_y) + "_" +
            "_hop_" + std::to_string(hop_radius) +
            "_seed_" + std::to_string(dist_seed) +
            "_params_" + params +
            "_threads_" + std::to_string(num_threads) +
            "_servers_" + std::to_string(num_servers_per_network_node) +
			"_bin_size_" + std::to_string(target_bin_size);*/
			
			std::string trace_folder_name = "";
        
            Irregular_NoC irregular_NoC_sim(grid_size_x*grid_size_y, hop_radius, num_servers_per_network_node, max_num_intra_arrive_events, max_sim_time, num_threads, dist_seed, bucket_width, target_bin_size, trace_folder_name, dist_params_file, network_topology_file);
            irregular_NoC_sim.SimulateModel();
            irregular_NoC_sim.PrintMeanPacketNetworkTime();
			//printf("\nHigh-level execution data:\n");
            //irregular_NoC_sim.PrintSVs();
            //irregular_NoC_sim.PrintNumVertexExecs();
            //irregular_NoC_sim.PrintFinishedPackets();
        }

        std::cout << std::endl;
    }
	
    in_file.close();
    
    std::cout << std::endl;
}
