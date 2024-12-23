import os
import re
import psutil

def get_isolated_cores():
    # Read the kernel boot parameters
    with open('/proc/cmdline', 'r') as file:
        cmdline = file.read()

    # Find the isolcpus parameter
    match = re.search(r'isolcpus=([0-9,-]+)', cmdline)
    if not match:
        return "No isolated cores found.", []

    # Extract the core indices
    isolated_cores_str = match.group(1).split(',')
    isolated_cores = []

    for core_str in isolated_cores_str:
        if '-' in core_str:
            start, end = map(int, core_str.split('-'))
            isolated_cores.extend(range(start, end + 1))
        else:
            isolated_cores.append(int(core_str))

    return len(isolated_cores), isolated_cores

# Get and print the number of isolated cores and their IDs
num_isolated, isolated_core_ids = get_isolated_cores()
print(f"Number of isolated cores: {num_isolated}")
print(f"Isolated core IDs: {isolated_core_ids}")

def get_memory_usage(db_print:bool=True):
    process = psutil.Process()
    memory_info = process.memory_info()
    memory_usage_gb = memory_info.rss / (1024**3)  # Convert bytes to gigabytes
    if db_print:
        print(f"[PID {os.getpid()}] Memory usage now: {memory_usage_gb} GB")
    return memory_usage_gb

def get_system_memory(label="", prev=0.0):
    """Get system memory usage."""
    mem = psutil.virtual_memory()
    used=mem.used / (1024 ** 3)
    print(f"{label} System Memory: Used = {used} GB, Available = {mem.available / (1024 ** 3)} GB,differece {used-prev}")
    return used