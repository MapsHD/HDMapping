# HDMapping CLI Batch Processing Guide

## Overview

HDMapping v0.85.0 offers both interactive GUI applications and command-line interfaces for large-scale batch processing. This guide focuses on CLI capabilities essential for automated workflows, enterprise deployments, and research projects requiring processing of multiple datasets.

## Table of Contents

1. [Command-Line Applications](#command-line-applications)
2. [Batch Processing Workflow](#batch-processing-workflow)
3. [Parameter Files (TOML)](#parameter-files-toml)
4. [Integration Examples](#integration-examples)
5. [Performance Optimization](#performance-optimization)
6. [Error Handling](#error-handling)

## Command-Line Applications

### Pure CLI Applications

#### 1. split_multi_livox
**Purpose**: Splits multi-LiDAR Mandeye datasets by individual sensor serial numbers.

```bash
split_multi_livox <input_file_from_mandeye> <output_prefix>
```

**Example**:
```bash
split_multi_livox continousScanning_20241215_143022.laz ./separated/scan
```

**Output**: Creates separate LAZ files for each LiDAR sensor:
- `scan_SN12345.laz`
- `scan_SN67890.laz`
- etc.

**Batch Processing**:
```bash
# Process multiple sessions
for file in continousScanning_*.laz; do
    split_multi_livox "$file" "./separated/$(basename "$file" .laz)"
done
```

#### 2. matrix_mul
**Purpose**: Multiplies two 4x4 transformation matrices from files.

```bash
matrix_mul <matrix1_file> <matrix2_file>
```

**Example**:
```bash
matrix_mul pose1.txt pose2.txt
```

**Input Format**: Each matrix file contains 3 lines with 4 values each:
```
1.0 0.0 0.0 10.5
0.0 1.0 0.0 5.2
0.0 0.0 1.0 2.1
```

### Hybrid Applications (CLI + GUI)

#### 1. lidar_odometry_step_1
**Purpose**: First step of HDMapping workflow - processes raw Mandeye data to create trajectory and point clouds.

**CLI Usage**:
```bash
lidar_odometry_step_1 <input_folder> <parameters.toml> <output_folder>
```

**GUI Usage**: When called without arguments, opens interactive interface.

**Batch Example**:
```bash
# Process multiple sessions with same parameters
for session_dir in sessions/*/; do
    session_name=$(basename "$session_dir")
    lidar_odometry_step_1 "$session_dir" config/standard_params.toml "results/$session_name"
done
```

#### 2. compare_trajectories
**Purpose**: Compares ground truth and estimated trajectories.

**CLI Usage**:
```bash
compare_trajectories <ground_truth.csv> <estimated_trajectory.csv>
```

**CSV Format**:
```csv
timestamp,x,y,z,qx,qy,qz,qw
1640995200.123,10.5,5.2,2.1,0.0,0.0,0.0,1.0
```

### GUI-Only Applications

The following applications currently support only interactive GUI mode but accept command-line arguments for trajectory loading:

- **multi_view_tls_registration**: Multi-view TLS registration and optimization
- **hd_mapper**: High-definition mapping interface
- **precision_forestry_tools**: Specialized forestry analysis tools
- **mandeye_raw_data_viewer**: Raw Mandeye data visualization
- **quick_start_demo**: Interactive demonstration

## Batch Processing Workflow

### Complete HDMapping Pipeline

```bash
#!/bin/bash
# HDMapping Batch Processing Pipeline

INPUT_ROOT="/data/mandeye_sessions"
OUTPUT_ROOT="/data/processed"
PARAMS_DIR="/config"

# Step 1: Process each raw Mandeye session
for session_path in "$INPUT_ROOT"/*/; do
    session_name=$(basename "$session_path")
    echo "Processing session: $session_name"
    
    # Create output directory
    output_dir="$OUTPUT_ROOT/$session_name"
    mkdir -p "$output_dir"
    
    # Step 1: LiDAR Odometry
    lidar_odometry_step_1 \
        "$session_path" \
        "$PARAMS_DIR/lidar_odometry_params.toml" \
        "$output_dir/step1"
    
    if [ $? -eq 0 ]; then
        echo "✓ Step 1 completed for $session_name"
        
        # Optional: Split multi-LiDAR data
        if [ -f "$session_path"/*.laz ]; then
            for laz_file in "$session_path"/*.laz; do
                split_multi_livox "$laz_file" "$output_dir/split/$(basename "$laz_file" .laz)"
            done
        fi
    else
        echo "✗ Step 1 failed for $session_name"
        continue
    fi
done

echo "Batch processing completed"
```

### Parallel Processing

```bash
#!/bin/bash
# Parallel processing with GNU parallel

export -f process_session
export PARAMS_DIR="/config"
export OUTPUT_ROOT="/data/processed"

process_session() {
    session_path="$1"
    session_name=$(basename "$session_path")
    output_dir="$OUTPUT_ROOT/$session_name"
    mkdir -p "$output_dir"
    
    lidar_odometry_step_1 \
        "$session_path" \
        "$PARAMS_DIR/lidar_odometry_params.toml" \
        "$output_dir/step1"
}

# Process sessions in parallel (4 concurrent jobs)
find /data/mandeye_sessions -maxdepth 1 -type d | parallel -j4 process_session
```

## Parameter Files (TOML)

HDMapping uses TOML configuration files for batch processing parameters.

### Sample Configuration

```toml
# lidar_odometry_params.toml
[motion_model_correction]
om = 0.0    # Rotation around X-axis (degrees)
fi = 0.05   # Rotation around Y-axis (degrees)
ka = 0.0    # Rotation around Z-axis (degrees)

[motion_model_uncertainty]
x_1_sigma_m = 0.1
y_1_sigma_m = 0.1
z_1_sigma_m = 0.1
om_1_sigma_deg = 0.5
fi_1_sigma_deg = 0.5
ka_1_sigma_deg = 0.5

[ndt_params_indoor]
resolution_X = 0.1
resolution_Y = 0.1
resolution_Z = 0.1
bounding_box_extension = 20.0

[ndt_params_outdoor]
resolution_X = 0.3
resolution_Y = 0.3
resolution_Z = 0.3
bounding_box_extension = 20.0

[filters]
threshold_xy_inner = 0.3   # Remove points inside circle (m)
threshold_xy_outer = 70.0  # Remove points outside circle (m)
output_filter = 0.3        # Output filtering threshold (m)

[processing]
decimation = 0.01                    # Point decimation factor
max_distance_lidar = 30.0           # Maximum processing distance (m)
nr_iter = 1000                      # Number of iterations
use_multithread = true              # Enable multithreading
real_time_threshold_seconds = 0.1   # Real-time processing threshold

[fusion]
convention_nwu = true     # Use NWU coordinate convention
convention_enu = false    # Use ENU coordinate convention
convention_ned = false    # Use NED coordinate convention
ahrs_gain = 0.5          # AHRS fusion gain parameter

[optimization]
use_robust_accurate_lidar_odometry = false
distance_bucket = 0.2
polar_angle_deg = 10.0
azimutal_angle_deg = 10.0
robust_iterations = 20

[quality]
sliding_window_trajectory_length_threshold = 10000
threshold_initial_points = 1000
use_motion_from_previous_step = true
save_calibration_validation = false
```

### Environment-Specific Configurations

#### High-Speed Vehicle (up to 30 km/h)
```toml
[ndt_params_indoor]
resolution_X = 0.3
resolution_Y = 0.3
resolution_Z = 0.3

[ndt_params_outdoor]
resolution_X = 0.5
resolution_Y = 0.5
resolution_Z = 0.5

[filters]
threshold_xy_inner = 3.0
threshold_xy_outer = 70.0
output_filter = 3.0

[processing]
decimation = 0.03
max_distance_lidar = 70.0
nr_iter = 500
sliding_window_trajectory_length_threshold = 200
```

#### Precision Forestry
```toml
[filters]
threshold_xy_inner = 1.5
threshold_xy_outer = 70.0
output_filter = 1.5

[processing]
decimation = 0.01
max_distance_lidar = 30.0
nr_iter = 500
sliding_window_trajectory_length_threshold = 10000
```

## Integration Examples

### Integration with Python

```python
#!/usr/bin/env python3
import subprocess
import os
import sys
from pathlib import Path

def process_mandeye_session(session_path, params_file, output_path):
    """Process a single Mandeye session using HDMapping CLI."""
    cmd = [
        'lidar_odometry_step_1',
        str(session_path),
        str(params_file),
        str(output_path)
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        print(f"✓ Processed {session_path}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Failed to process {session_path}: {e.stderr}")
        return False

def batch_process_sessions(input_root, output_root, params_file):
    """Batch process multiple Mandeye sessions."""
    input_path = Path(input_root)
    output_path = Path(output_root)
    params = Path(params_file)
    
    # Find all session directories
    sessions = [d for d in input_path.iterdir() if d.is_dir()]
    
    successful = 0
    failed = 0
    
    for session in sessions:
        session_output = output_path / session.name
        session_output.mkdir(parents=True, exist_ok=True)
        
        if process_mandeye_session(session, params, session_output):
            successful += 1
        else:
            failed += 1
    
    print(f"\nBatch processing completed:")
    print(f"Successful: {successful}")
    print(f"Failed: {failed}")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python batch_process.py <input_root> <output_root> <params.toml>")
        sys.exit(1)
    
    batch_process_sessions(sys.argv[1], sys.argv[2], sys.argv[3])
```

### Integration with Docker

```dockerfile
# Dockerfile for HDMapping batch processing
FROM ubuntu:22.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    libeigen3-dev \
    libglew-dev \
    freeglut3-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy HDMapping binaries
COPY build/apps/lidar_odometry_step_1/lidar_odometry_step_1 /usr/local/bin/
COPY build/apps/split_multi_livox/split_multi_livox /usr/local/bin/
COPY build/apps/matrix_mul/matrix_mul /usr/local/bin/

# Create working directories
WORKDIR /data
VOLUME ["/data/input", "/data/output", "/data/config"]

# Default command
CMD ["lidar_odometry_step_1"]
```

Usage:
```bash
docker run -v ./input:/data/input -v ./output:/data/output -v ./config:/data/config \
    hdmapping:latest lidar_odometry_step_1 /data/input /data/config/params.toml /data/output
```

### Kubernetes Job

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: hdmapping-batch-processing
spec:
  parallelism: 4
  completions: 10
  template:
    spec:
      containers:
      - name: hdmapping
        image: hdmapping:latest
        command: ["python3", "/scripts/batch_process.py"]
        args: ["/data/input", "/data/output", "/data/config/params.toml"]
        volumeMounts:
        - name: input-data
          mountPath: /data/input
        - name: output-data
          mountPath: /data/output
        - name: config
          mountPath: /data/config
        resources:
          requests:
            memory: "4Gi"
            cpu: "2"
          limits:
            memory: "8Gi"
            cpu: "4"
      volumes:
      - name: input-data
        persistentVolumeClaim:
          claimName: mandeye-input-pvc
      - name: output-data
        persistentVolumeClaim:
          claimName: processed-output-pvc
      - name: config
        configMap:
          name: hdmapping-config
      restartPolicy: OnFailure
```

## Performance Optimization

### CPU Optimization

HDMapping automatically detects CPU architecture and optimizes accordingly:

```bash
# Check CPU optimization status
grep -i "cpu optimization" /var/log/hdmapping.log

# Example output:
# CPU Optimization: AUTO mode detected Intel CPU, using AVX2 optimizations
# Performance boost: ~40% faster processing with SIMD instructions
```

### Memory Management

```bash
# Monitor memory usage during batch processing
#!/bin/bash
while true; do
    ps aux | grep lidar_odometry_step_1 | grep -v grep | \
    awk '{printf "Process: %s, Memory: %s MB, CPU: %s%%\n", $11, $6/1024, $3}'
    sleep 10
done
```

### Threading Configuration

```toml
# Optimal threading for batch processing
[processing]
use_multithread = true
# Number of threads auto-detected based on CPU cores
# Override with environment variable: export OMP_NUM_THREADS=8
```

## Error Handling

### Common Issues and Solutions

#### 1. Insufficient Memory
```bash
# Error: System is out of memory
# Solution: Reduce decimation factor or process smaller chunks
[processing]
decimation = 0.05  # Increase from 0.01 to reduce memory usage
```

#### 2. Missing Input Files
```bash
# Error: No input file found
# Check file permissions and paths
ls -la /path/to/session/
chmod -R 755 /path/to/session/
```

#### 3. Parameter Loading Errors
```bash
# Error: Failed to load TOML parameters
# Validate TOML syntax
python3 -c "import toml; toml.load('params.toml')"
```

### Robust Batch Processing Script

```bash
#!/bin/bash
# Robust batch processing with error handling

set -euo pipefail

LOG_FILE="/var/log/hdmapping_batch.log"
ERROR_DIR="/data/errors"
SUCCESS_COUNT=0
ERROR_COUNT=0

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

process_session() {
    local session_path="$1"
    local session_name=$(basename "$session_path")
    local output_dir="$2/$session_name"
    local params_file="$3"
    
    log "Starting processing: $session_name"
    
    # Create output directory
    mkdir -p "$output_dir"
    
    # Check input files
    if [[ ! -d "$session_path" ]]; then
        log "ERROR: Session directory not found: $session_path"
        return 1
    fi
    
    # Check for required files
    if ! ls "$session_path"/*.csv >/dev/null 2>&1; then
        log "ERROR: No CSV files found in $session_path"
        return 1
    fi
    
    if ! ls "$session_path"/*.laz >/dev/null 2>&1 && ! ls "$session_path"/*.las >/dev/null 2>&1; then
        log "ERROR: No LAZ/LAS files found in $session_path"
        return 1
    fi
    
    # Process with timeout
    if timeout 3600 lidar_odometry_step_1 \
        "$session_path" \
        "$params_file" \
        "$output_dir" >> "$LOG_FILE" 2>&1; then
        
        log "SUCCESS: Completed $session_name"
        ((SUCCESS_COUNT++))
        return 0
    else
        log "ERROR: Processing failed for $session_name"
        # Move problematic session to error directory
        mkdir -p "$ERROR_DIR"
        cp -r "$session_path" "$ERROR_DIR/"
        ((ERROR_COUNT++))
        return 1
    fi
}

# Main processing loop
main() {
    local input_root="$1"
    local output_root="$2"
    local params_file="$3"
    
    log "Starting batch processing"
    log "Input: $input_root"
    log "Output: $output_root"
    log "Parameters: $params_file"
    
    # Validate parameters file
    if [[ ! -f "$params_file" ]]; then
        log "ERROR: Parameters file not found: $params_file"
        exit 1
    fi
    
    # Process each session
    for session_path in "$input_root"/*/; do
        if [[ -d "$session_path" ]]; then
            process_session "$session_path" "$output_root" "$params_file" || true
        fi
    done
    
    log "Batch processing completed"
    log "Successful sessions: $SUCCESS_COUNT"
    log "Failed sessions: $ERROR_COUNT"
    
    # Send notification (optional)
    if command -v mail >/dev/null 2>&1; then
        echo "HDMapping batch processing completed. Success: $SUCCESS_COUNT, Failed: $ERROR_COUNT" | \
        mail -s "HDMapping Batch Processing Report" admin@company.com
    fi
}

# Usage check
if [[ $# -ne 3 ]]; then
    echo "Usage: $0 <input_root> <output_root> <params.toml>"
    echo "Example: $0 /data/sessions /data/processed /config/standard.toml"
    exit 1
fi

main "$@"
```

## Best Practices

### 1. Directory Organization
```
project/
├── data/
│   ├── raw/                    # Raw Mandeye sessions
│   │   ├── session_001/
│   │   ├── session_002/
│   │   └── ...
│   └── processed/              # Processed results
│       ├── session_001/
│       ├── session_002/
│       └── ...
├── config/                     # Parameter files
│   ├── standard_params.toml
│   ├── high_speed_params.toml
│   └── forestry_params.toml
├── scripts/                    # Batch processing scripts
│   ├── batch_process.sh
│   ├── parallel_process.py
│   └── monitor.sh
└── logs/                       # Processing logs
    ├── batch_processing.log
    └── errors/
```

### 2. Resource Planning
- **Memory**: 4-8 GB per concurrent session
- **Storage**: 10-50x input size for intermediate files
- **CPU**: Multi-core recommended (4+ cores)
- **Processing Time**: ~1-3 hours per GB of input data

### 3. Quality Assurance
```bash
# Automated quality checks
check_results() {
    local output_dir="$1"
    
    # Check for required output files
    if [[ ! -f "$output_dir/session.json" ]]; then
        echo "ERROR: session.json not found"
        return 1
    fi
    
    # Check trajectory length
    local traj_length=$(grep "total_length_of_calculated_trajectory" "$output_dir/session.json" | \
                       grep -o '[0-9.]*')
    
    if (( $(echo "$traj_length < 10.0" | bc -l) )); then
        echo "WARNING: Trajectory length suspiciously short: $traj_length m"
    fi
    
    echo "Quality check passed"
    return 0
}
```

This guide provides comprehensive documentation for CLI batch processing capabilities in HDMapping, enabling efficient large-scale data processing workflows for enterprise and research applications.
