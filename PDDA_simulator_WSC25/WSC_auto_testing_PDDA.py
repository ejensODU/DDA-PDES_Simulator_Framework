import subprocess
import re
import time
import os
import threading
import filecmp
import csv
import sys
import hashlib
from datetime import datetime

# Network configurations
NETWORK_CONFIGS = [
   {'model': 'PHOLD_benchmark', 'size': [4096], 'max_events': 10000},
   {'model': '1D_ring_network', 'size': [4096], 'max_events': 5},
   {'model': 'Irregular_2D_NoC_grid_network', 'size': [64, 64], 'max_events': 25, 'network_topology_file': 'network_topology_50.txt'},
   {'model': 'Irregular_2D_NoC_grid_network', 'size': [64, 64], 'max_events': 25, 'network_topology_file': 'network_topology_80.txt'},
   {'model': '3D_torus_network', 'size': [16, 16, 16], 'max_events': 100}
]

# Hop radius values for non-1D models
HOP_RADIUS_VALUES = [1, 4]

# Base delay parameters
INTRA_ARRIVAL_DELAYS = [10, 12, 16]
SERVICE_DELAYS = [2, 3, 4]
TRANSIT_DELAYS = [1, 1.5, 2]

# Results file setup
RESULTS_FILE = 'WSC_network_simulation_results_4096.csv'

# Execution method: 'os_system' or 'subprocess_optimized'
EXECUTION_METHOD = 'os_system'  # Change this to 'subprocess_optimized' if needed

def get_base_filename(config, bucket_cap, dist_seed, num_threads, trial, hop_radius=None):
    """Generate consistent base filename pattern"""
    size_str = '_'.join(map(str, config['size']))
    # Don't include hop_radius in filename for 1D_ring_network and PHOLD_benchmark
    hop_str = "" if config['model'] in ['1D_ring_network', 'PHOLD_benchmark'] else (f"_hop_{hop_radius}" if hop_radius is not None else "")
    
    # Include network topology file for Irregular_2D_NoC_grid_network
    topology_str = ""
    if config['model'] == 'Irregular_2D_NoC_grid_network' and 'network_topology_file' in config:
        topology_str = f"_topology_{config['network_topology_file'].split('.')[0]}"
        
    return f"{config['model']}_size_{size_str}{hop_str}{topology_str}_bucketcap_{bucket_cap}_seed_{dist_seed}_runtype_{num_threads}_params_1_trial_{trial}"

def setup_results_file():
    """Create results file with headers if it doesn't exist"""
    headers = [
        'model', 'size', 'trial', 'num_threads', 'dist_seed', 'hop_radius', 'bucket_cap',
        'run_time', 'events_executed', 'mean_packet_time', 'end_sim_time', 'RE_efficiency', 
        'mean_bucket_size', 'adjust_count', 'sim_data_hash', 'serial_match', 'network_topology_file', 'timestamp'
    ]
    if not os.path.exists(RESULTS_FILE):
        with open(RESULTS_FILE, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()

def write_result(result):
    """Write a single result to the CSV file with lock protection"""
    headers = [
        'model', 'size', 'trial', 'num_threads', 'dist_seed', 'hop_radius', 'bucket_cap',
        'run_time', 'events_executed', 'mean_packet_time', 'end_sim_time', 'RE_efficiency', 
        'mean_bucket_size', 'adjust_count', 'sim_data_hash', 'serial_match', 'network_topology_file', 'timestamp'
    ]

    with open(RESULTS_FILE, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writerow(result)

def extract_metrics(output, is_parallel):
    """Extract all required metrics from simulation output"""
    metrics = {}

    # Extract run time
    run_time_match = re.search(r"runtime: ([\d\.]+)", output)
    metrics['run_time'] = float(run_time_match.group(1)) if run_time_match else None

    # Extract end sim time
    sim_time_match = re.search(r"sim time: ([\d\.]+)", output)
    metrics['end_sim_time'] = float(sim_time_match.group(1)) if sim_time_match else None

    # Extract events executed
    events_match = re.search(r"events executed: ([\d\.]+)", output)
    metrics['events_executed'] = float(events_match.group(1)) if events_match else None

    # Extract mean packet time
    time_match = re.search(r"Mean packet network time: ([\d\.]+)", output)
    metrics['mean_packet_time'] = float(time_match.group(1)) if time_match else None
    
    # Calculate hash based on everything after "High-level execution data" marker
    exec_data_marker = "\nHigh-level execution data:\n"
    marker_pos = output.find(exec_data_marker)
    
    if marker_pos != -1:
        # Get everything after the marker
        start_pos = marker_pos + len(exec_data_marker)
        rest_of_data = output[start_pos:].strip()
        # Calculate hash
        metrics['sim_data_hash'] = hashlib.md5(rest_of_data.encode()).hexdigest()
    else:
        metrics['sim_data_hash'] = None

    # For parallel runs, extract additional metrics
    if is_parallel:
        RE_eff_match = re.search(r"RE efficiency: ([\d\.]+)", output)
        metrics['RE_efficiency'] = float(RE_eff_match.group(1)) if RE_eff_match else None

        bucket_size_match = re.search(r"Mean first-bucket size: ([\d\.]+)", output)
        metrics['mean_bucket_size'] = float(bucket_size_match.group(1)) if bucket_size_match else None

        adjust_count_match = re.search(r"adjust count: ([\d\.]+)", output)
        metrics['adjust_count'] = float(adjust_count_match.group(1)) if adjust_count_match else None
    else:
        # For serial runs, these metrics don't apply
        metrics['RE_efficiency'] = None
        metrics['mean_bucket_size'] = None
        metrics['adjust_count'] = None

    return metrics

def get_serial_results(model, size, dist_seed, hop_radius, network_topology_file=None):
    """Get all serial results matching the given parameters"""
    serial_results = []
    
    # For PHOLD_benchmark and 1D_ring_network, always use hop_radius '0'
    if model in ['PHOLD_benchmark', '1D_ring_network']:
        hop_radius = 0
    
    # We'll try a simpler, more direct approach reading the CSV
    try:
        print(f"\nLooking for serial runs in {os.path.abspath(RESULTS_FILE)}")
        
        if not os.path.exists(RESULTS_FILE):
            print(f"Results file does not exist!")
            return []
            
        # Read the CSV file manually to avoid type conversion issues
        with open(RESULTS_FILE, 'r', newline='') as f:
            reader = csv.DictReader(f)
            all_rows = list(reader)  # Convert to list to process multiple times
            
        print(f"Successfully read {len(all_rows)} rows from CSV")
        
        # Get the serial runs
        size_str = '_'.join(map(str, size))
        hop_radius_val = '0' if model in ['PHOLD_benchmark', '1D_ring_network'] else (str(hop_radius) if hop_radius is not None else 'N/A')
        
        print(f"Looking for: model={model}, size={size_str}, dist_seed={dist_seed}, hop_radius={hop_radius_val}")
        if network_topology_file:
            print(f"With network_topology_file={network_topology_file}")
        
        for row in all_rows:
            # Convert num_threads to int for comparison
            try:
                num_threads = int(float(row['num_threads']))
            except (ValueError, TypeError):
                num_threads = -1  # Invalid value
                
            # Manual string matching to avoid type conversion issues
            topology_match = True
            if model == 'Irregular_2D_NoC_grid_network':
                topology_match = (str(row.get('network_topology_file', '')).strip() == 
                                 str(network_topology_file).strip())
                
            if (row['model'] == model and 
                row['size'] == size_str and
                str(row['dist_seed']).strip() == str(dist_seed).strip() and
                str(row['hop_radius']).strip() == str(hop_radius_val).strip() and
                topology_match and
                (num_threads == 0 or num_threads == 1)):
                
                # Add with standardized types
                serial_row = {k: str(v).strip() if v is not None else '' for k, v in row.items()}
                serial_results.append(serial_row)
        
        print(f"Found {len(serial_results)} matching serial runs")
        for i, run in enumerate(serial_results[:2]):  # Show first 2 matches
            print(f"Match {i+1}: hop_radius={run['hop_radius']}, num_threads={run['num_threads']}")
        
    except Exception as e:
        print(f"Error reading serial results: {e}")
        import traceback
        traceback.print_exc()
        
    return serial_results

def check_serial_match(result_dict):
    """Check if the parallel run matches all corresponding serial runs"""
    # Only applicable to parallel runs
    try:
        # Convert num_threads to integer for comparison
        num_threads = int(float(result_dict['num_threads']))
        if num_threads <= 1:
            return 'N/A'
    except (ValueError, TypeError):
        # If conversion fails, assume it's a parallel run
        pass
    
    # Get all serial results for this configuration
    size_array = [int(x) for x in result_dict['size'].split('_')]
    hop_radius = result_dict['hop_radius'] if result_dict['hop_radius'] != 'N/A' else None
    network_topology_file = result_dict.get('network_topology_file', 'N/A')
    if network_topology_file == 'N/A':
        network_topology_file = None
    
    # Print what we're checking
    print(f"\nChecking serial match for parallel run:")
    print(f"  Model: {result_dict['model']}")
    print(f"  Size: {result_dict['size']}")
    print(f"  Dist seed: {result_dict['dist_seed']}")
    print(f"  Hop radius: {result_dict['hop_radius']}")
    if network_topology_file:
        print(f"  Network topology file: {network_topology_file}")
    
    # Force all values to be strings to avoid type comparison issues
    parallel_run = {k: str(v).strip() if v is not None else '' for k, v in result_dict.items()}
    
    serial_results = get_serial_results(
        result_dict['model'],
        size_array,
        result_dict['dist_seed'],
        hop_radius,
        network_topology_file
    )
    
    # If there are no serial results, can't determine match
    if not serial_results:
        print(f"  Result: NO_SERIAL_RUNS (no matching serial runs found)")
        return 'NO_SERIAL_RUNS'
    
    print(f"  Found {len(serial_results)} matching serial runs")
    
    # For each serial run found, check for match
    all_pass = True
    
    for i, serial in enumerate(serial_results):
        print(f"  Checking serial run {i+1}:")
        
        # Standardize serial run values
        serial_run = {k: str(v).strip() if v is not None else '' for k, v in serial.items()}
        
        # Check each field that needs to match
        fields_to_check = ['events_executed', 'mean_packet_time', 'end_sim_time', 'sim_data_hash']
        field_matches = {}
        
        for field in fields_to_check:
            serial_value = serial_run.get(field, '')
            parallel_value = parallel_run.get(field, '')
            field_matches[field] = serial_value == parallel_value
            print(f"    {field}: '{serial_value}' vs '{parallel_value}' - {'Match' if field_matches[field] else 'MISMATCH'}")
            
        # Check if all required fields match
        if not all(field_matches.values()):
            all_pass = False
            print(f"    Not all fields match for serial run {i+1}")
    
    result = 'PASS' if all_pass else 'FAIL'
    print(f"  Final result: {result}")
    return result

def create_delay_params_file(config, bucket_cap, dist_seed, num_threads, trial, hop_radius=None):
    """Create distribution parameter file with given configuration"""
    base_name = get_base_filename(config, bucket_cap, dist_seed, num_threads, trial, hop_radius)
    filename = f"params_files/params_{base_name}.txt"

    # Ensure directory exists
    os.makedirs("params_files", exist_ok=True)

    with open(filename, 'w') as f:
        # For PHOLD_benchmark, only write TRANSIT_DELAYS
        if config['model'] == 'PHOLD_benchmark':
            f.write(f"{TRANSIT_DELAYS[0]} {TRANSIT_DELAYS[1]} {TRANSIT_DELAYS[2]}\n")
        else:
            # For other models, write all three types of delays
            f.write(f"{INTRA_ARRIVAL_DELAYS[0]} {INTRA_ARRIVAL_DELAYS[1]} {INTRA_ARRIVAL_DELAYS[2]}\n")
            f.write(f"{SERVICE_DELAYS[0]} {SERVICE_DELAYS[1]} {SERVICE_DELAYS[2]}\n")
            f.write(f"{TRANSIT_DELAYS[0]} {TRANSIT_DELAYS[1]} {TRANSIT_DELAYS[2]}\n")

    return filename

def create_network_input_file(config, bucket_cap, dist_seed, num_threads, trial, hop_radius, params_file):
    """Create input file based on network configuration"""
    input_content = f"model_name : {config['model']}\n"

    # Model-specific sizing parameters
    if config['model'] == 'PHOLD_benchmark':
        input_content += f"network_size : {config['size'][0]}\n"
        # PHOLD_benchmark doesn't use hop_radius in input file
    elif config['model'] == '1D_ring_network':
        input_content += f"ring_size : {config['size'][0]}\n"
        # Do NOT add hop_radius to input file for 1D_ring_network
    else:
        input_content += f"grid_size_x : {config['size'][0]}\n"
        input_content += f"grid_size_y : {config['size'][1]}\n"
        if len(config['size']) > 2:
            input_content += f"grid_size_z : {config['size'][2]}\n"
        input_content += f"hop_radius : {hop_radius}\n"

    # Add network topology file for Irregular_2D_NoC_grid_network
    if config['model'] == 'Irregular_2D_NoC_grid_network':
        input_content += f"network_topology_file : {config['network_topology_file']}\n"

    # Add common parameters, with model-specific variations
    if config['model'] == 'PHOLD_benchmark':
        input_content += f"max_num_arrive_events : {config['max_events']}\n"
    else:
        input_content += f"num_servers_per_network_node : 1\n"
        input_content += f"max_num_intra_arrive_events : {config['max_events']}\n"

    input_content += f"""max_sim_time : 10000000
num_threads : {num_threads}
dist_seed : {dist_seed}
bucket_width : 0.001
target_bin_size : {bucket_cap}
dist_params_file : {params_file}
"""

    base_name = get_base_filename(config, bucket_cap, dist_seed, num_threads, trial, hop_radius)
    input_file = f"input_files/input_{base_name}.txt"

    # Ensure directory exists
    os.makedirs("input_files", exist_ok=True)
    os.makedirs("output_files", exist_ok=True)

    with open(input_file, 'w') as f:
        f.write(input_content)
    return input_file

def run_single_simulation(config, bucket_cap, dist_seed, num_threads, trial, hop_radius=None):
    """Run a single simulation with given parameters using os.system()"""
    # Create params file for this execution
    params_file = create_delay_params_file(config, bucket_cap, dist_seed, num_threads, trial, hop_radius)

    # Create input file
    input_file = create_network_input_file(config, bucket_cap, dist_seed, num_threads, trial, hop_radius, params_file)

    # Create unique output filename
    base_name = get_base_filename(config, bucket_cap, dist_seed, num_threads, trial, hop_radius)
    output_file = f"output_files/output_{base_name}.txt"

    try:
        # Use os.system instead of subprocess.run - less overhead
        command = f'./PDDA_Sim {input_file} > {output_file}'
        print(f"Executing: {command}")
        ret_code = os.system(command)

        if ret_code == 0:
            with open(output_file, 'r') as f:
                output = f.read()
            is_parallel = num_threads > 1
            metrics = extract_metrics(output, is_parallel)

            if metrics:
                # Add additional info to the metrics
                result_dict = {
                    'model': config['model'],
                    'size': '_'.join(map(str, config['size'])),
                    'trial': trial,
                    'num_threads': num_threads,
                    'dist_seed': dist_seed,
                    # Always use 0 for hop_radius in results file for PHOLD_benchmark and 1D_ring_network
                    'hop_radius': '0' if config['model'] in ['PHOLD_benchmark', '1D_ring_network'] else (hop_radius if hop_radius is not None else 'N/A'),
                    'bucket_cap': bucket_cap if num_threads > 1 else 'N/A',
                    'run_time': metrics['run_time'],
                    'events_executed': metrics['events_executed'],
                    'mean_packet_time': metrics['mean_packet_time'],
                    'end_sim_time': metrics['end_sim_time'],
                    'RE_efficiency': metrics['RE_efficiency'],
                    'mean_bucket_size': metrics['mean_bucket_size'],
                    'adjust_count': metrics['adjust_count'],
                    'sim_data_hash': metrics['sim_data_hash'],
                    'serial_match': 'N/A',  # Will be updated later for parallel runs
                    'network_topology_file': config.get('network_topology_file', 'N/A'),
                    'timestamp': datetime.now().isoformat()
                }
                
                # Store original num_threads value before converting to string
                original_num_threads = num_threads
                
                # Convert all values to strings to ensure consistent data types
                for key in result_dict:
                    if result_dict[key] is not None:
                        result_dict[key] = str(result_dict[key])
                
                # Check serial match for parallel runs
                if original_num_threads > 1:
                    result_dict['serial_match'] = check_serial_match(result_dict)

                return result_dict
        else:
            print(f"Error in simulation: return code {ret_code}")
            return None
    except Exception as e:
        print(f"Exception in simulation: {e}")
        import traceback
        traceback.print_exc()
        return None

def run_single_simulation_subprocess(config, bucket_cap, dist_seed, num_threads, trial, hop_radius=None):
    """Run a single simulation with optimized subprocess settings"""
    # Create params file for this execution
    params_file = create_delay_params_file(config, bucket_cap, dist_seed, num_threads, trial, hop_radius)

    # Create input file
    input_file = create_network_input_file(config, bucket_cap, dist_seed, num_threads, trial, hop_radius, params_file)

    # Create unique output filename
    base_name = get_base_filename(config, bucket_cap, dist_seed, num_threads, trial, hop_radius)
    output_file = f"output_files/output_{base_name}.txt"

    try:
        # Optimized subprocess: Don't capture output, use preexec_fn to lower priority if on Unix
        command = f'./PDDA_Sim {input_file} > {output_file}'
        print(f"Executing: {command}")
        
        # Different approach based on platform
        if os.name == 'posix':  # Linux/Unix/WSL
            # Set process priority lower to give more resources to the simulator
            result = subprocess.run(
                command,
                shell=True,
                capture_output=False,  # Don't capture output in Python
                preexec_fn=lambda: os.nice(10)  # Lower Python priority
            )
        else:  # Windows
            # Windows doesn't support preexec_fn
            result = subprocess.run(
                command,
                shell=True,
                capture_output=False  # Don't capture output in Python
            )

        if result.returncode == 0:
            with open(output_file, 'r') as f:
                output = f.read()
            is_parallel = num_threads > 1
            metrics = extract_metrics(output, is_parallel)

            if metrics:
                # Add additional info to the metrics
                result_dict = {
                    'model': config['model'],
                    'size': '_'.join(map(str, config['size'])),
                    'trial': trial,
                    'num_threads': num_threads,
                    'dist_seed': dist_seed,
                    # Always use 0 for hop_radius in results file for PHOLD_benchmark and 1D_ring_network
                    'hop_radius': '0' if config['model'] in ['PHOLD_benchmark', '1D_ring_network'] else (hop_radius if hop_radius is not None else 'N/A'),
                    'bucket_cap': bucket_cap if num_threads > 1 else 'N/A',
                    'run_time': metrics['run_time'],
                    'events_executed': metrics['events_executed'],
                    'mean_packet_time': metrics['mean_packet_time'],
                    'end_sim_time': metrics['end_sim_time'],
                    'RE_efficiency': metrics['RE_efficiency'],
                    'mean_bucket_size': metrics['mean_bucket_size'],
                    'adjust_count': metrics['adjust_count'],
                    'sim_data_hash': metrics['sim_data_hash'],
                    'serial_match': 'N/A',  # Will be updated later for parallel runs
                    'network_topology_file': config.get('network_topology_file', 'N/A'),
                    'timestamp': datetime.now().isoformat()
                }
                
                # Store original num_threads value before converting to string
                original_num_threads = num_threads
                
                # Convert all values to strings to ensure consistent data types
                for key in result_dict:
                    if result_dict[key] is not None:
                        result_dict[key] = str(result_dict[key])
                
                # Check serial match for parallel runs
                if original_num_threads > 1:
                    result_dict['serial_match'] = check_serial_match(result_dict)

                return result_dict
        else:
            print(f"Error in simulation: return code {result.returncode}")
            return None
    except Exception as e:
        print(f"Exception in simulation: {e}")
        import traceback
        traceback.print_exc()
        return None

def run_experiment(config, bucket_cap, dist_seed, hop_radius=None):
    """Run all required simulation types for given configuration"""
    
    # For PHOLD_benchmark and 1D_ring_network, always display hop_radius as 0 in logging
    display_hop_radius = "0" if config['model'] in ['PHOLD_benchmark', '1D_ring_network'] else (hop_radius if hop_radius is not None else 'N/A')
    
    # Display topology file for Irregular_2D_NoC_grid_network
    topology_display = f", Topology={config['network_topology_file']}" if 'network_topology_file' in config else ""
    
    results = []

    # Select the simulation function based on global execution method
    sim_func = run_single_simulation if EXECUTION_METHOD == 'os_system' else run_single_simulation_subprocess
    method_name = "os.system()" if EXECUTION_METHOD == 'os_system' else "optimized subprocess.run()"
    print(f"Using execution method: {method_name}")

    if 64 == bucket_cap:
        # Serial runs with num_threads=0 (3 trials)
        for trial in range(1, 4):
            print(f"Running: Model={config['model']}, Size={config['size']}, "
                  f"Hop Radius={display_hop_radius}{topology_display}, "
                  f"Seed={dist_seed}, num_threads=0, Trial={trial}")

            result = sim_func(config, 0, dist_seed, 0, trial, hop_radius)
            if result:
                write_result(result)
                results.append(result)
                
            # Uncomment to add cooling period between runs
            # print("\tsleeping for 5 seconds...")
            # time.sleep(5)
            # print("\t...resuming")

        # Serial runs with num_threads=1 (3 trials)
        for trial in range(1, 4):
            print(f"Running: Model={config['model']}, Size={config['size']}, "
                  f"Hop Radius={display_hop_radius}{topology_display}, "
                  f"Seed={dist_seed}, num_threads=1, Trial={trial}")

            result = sim_func(config, 0, dist_seed, 1, trial, hop_radius)
            if result:
                write_result(result)
                results.append(result)
                
            # Uncomment to add cooling period between runs
            # print("\tsleeping for 5 seconds...")
            # time.sleep(5)
            # print("\t...resuming")

    # Parallel runs with num_threads=n*4 (5 trials each)
    for num_threads in [2, 4, 6, 8, 10, 11, 12]:
        for trial in range(1, 6):
            print(f"Running: Model={config['model']}, Size={config['size']}, "
                  f"Hop Radius={display_hop_radius}{topology_display}, "
                  f"Seed={dist_seed}, num_threads={num_threads}, BucketCap={bucket_cap}, Trial={trial}")

            result = sim_func(config, bucket_cap, dist_seed, num_threads, trial, hop_radius)
            if result:
                write_result(result)
                results.append(result)
            
            # Uncomment to add cooling period between runs
            #print("\tsleeping for 5 seconds...")
            #time.sleep(5)
            #print("\t...resuming")

    return results

def main():
    print(f"Starting WSC auto testing with {EXECUTION_METHOD} execution method")
    print(f"Current working directory: {os.getcwd()}")
    
    setup_results_file()

    # Run one experiment at a time (no parallel execution of experiments)
    for config in NETWORK_CONFIGS:
        for bucket_cap in [64, 128, 256, 512, 1024, 2048]:
            for dist_seed in range(1):  # 1 different seeds
                if config['model'] in ['PHOLD_benchmark', '1D_ring_network']:
                    # Run PHOLD benchmark and 1D ring network with hop radius explicitly set to 0
                    run_experiment(config, bucket_cap, dist_seed, 0)
                else:
                    # Run other models with different hop radius values
                    for hop_radius in HOP_RADIUS_VALUES:
                        run_experiment(config, bucket_cap, dist_seed, hop_radius)

if __name__ == "__main__":
    main()