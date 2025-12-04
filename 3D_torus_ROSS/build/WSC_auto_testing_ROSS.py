#!/usr/bin/env python3
"""
Automated testing script for the Torus 3D ROSS model.

This script runs the Torus 3D model with different parameters and extracts
performance metrics from the output, saving results to a CSV file.

For serial runs:
- lookahead = 0.0
- update_delta = very large number (999999)

For parallel runs:
- lookahead values: [1.0, 0.1, 0.01]
- update threshold values: [1, 2, 3, 4, 5, 6, 8, 10, 15, 20]
"""

import subprocess
import re
import csv
import os
import time
import sys

def run_command(command):
    """Run a command and return its output."""
    print(f"Running: {command}")
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    stdout, stderr = process.communicate()
    return stdout.decode('utf-8'), stderr.decode('utf-8')

def extract_metrics(output):
    """Extract the Global Average Packet Delay and Running Time from the output."""
    delay_match = re.search(r'Global Average Packet Delay\s+(\d+\.\d+)', output)
    time_match = re.search(r': Running Time = (\d+\.\d+)', output)
    
    delay = float(delay_match.group(1)) if delay_match else None
    run_time = float(time_match.group(1)) if time_match else None
    
    packets_match = re.search(r'Total Finished Packets\s+(\d+)', output)
    print(f"\tTotal Finished Packets: {packets_match.group(1)}")
    
    return delay, run_time

def append_to_csv(csv_file, row):
    """Append a row to the CSV file."""
    file_exists = os.path.isfile(csv_file)
    with open(csv_file, 'a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(['procs', 'lookahead', 'update_threshold', 'mean_packet_delay', 'runtimes'])
        writer.writerow(row)

def main():
    # Define parameters
    serial_command = "./torus3d --synch=1"
    parallel_command = "mpirun -np 8 ./torus3d --synch=2"
    
    lookahead_values = [1.0, 0.1, 0.01]
    update_delta_values = [1, 2, 3, 4, 5, 6, 8, 10, 15, 20]
    num_runs = 10  # Number of runs per configuration (increased from 5 to 10)
    
    csv_file = "torus3d_results.csv"
    
    # Serial run (lookahead=0.0, update_delta=very large number)
    runtimes = []
    delay = None
    
    print("Starting serial runs...")
    for i in range(num_runs):
        # Add a 60-second pause between runs
        print(f"Pausing for 60 seconds before next run...")
        time.sleep(60)
        
        command = f"{serial_command} --lookahead=0.0 --update-delta=999999"
        print(f"Run {i+1}/{num_runs}")
        stdout, stderr = run_command(command)
        combined_output = stdout + stderr
        current_delay, current_runtime = extract_metrics(combined_output)
        
        if current_delay is not None:
            delay = current_delay  # Use the last valid delay
        
        if current_runtime is not None:
            runtimes.append(f"{current_runtime:.4f}")
            print(f"Run completed in {current_runtime:.4f} seconds")
        else:
            print(f"Warning: Failed to extract runtime from output")
    
    # Format runtimes as a space-separated string
    runtimes_str = " ".join(runtimes)
    append_to_csv(csv_file, [1, 'N/A', 'N/A', delay, runtimes_str])
    print(f"Serial runs complete. Packet delay: {delay}, Runtimes: {runtimes_str}")
    print(f"Results saved to {csv_file}")
    
    # Parallel runs with different parameters
    for lookahead in lookahead_values:
        for update_delta in update_delta_values:
            runtimes = []
            delay = None
            
            print(f"\nStarting parallel runs with lookahead={lookahead}, update_delta={update_delta}")
            
            for i in range(num_runs):
                # Add a 60-second pause between runs
                print(f"Pausing for 60 seconds before next run...")
                time.sleep(60)
                
                command = f"{parallel_command} --lookahead={lookahead} --update-delta={update_delta}"
                print(f"Run {i+1}/{num_runs}")
                stdout, stderr = run_command(command)
                combined_output = stdout + stderr
                current_delay, current_runtime = extract_metrics(combined_output)
                
                if current_delay is not None:
                    delay = current_delay  # Use the last valid delay
                
                if current_runtime is not None:
                    runtimes.append(f"{current_runtime:.4f}")
                    print(f"Run completed in {current_runtime:.4f} seconds")
                else:
                    print(f"Warning: Failed to extract runtime from output")
            
            # Format runtimes as a space-separated string
            runtimes_str = " ".join(runtimes)
            append_to_csv(csv_file, [8, lookahead, update_delta, delay, runtimes_str])
            print(f"Completed lookahead={lookahead}, update_delta={update_delta}")
            print(f"Packet delay: {delay}, Runtimes: {runtimes_str}")
            print(f"Results appended to {csv_file}")

    print(f"\nAll tests completed. Final results saved to {csv_file}")

if __name__ == "__main__":
    main()