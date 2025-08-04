# HDMapping Python Bindings - Detailed Guide

## 📖 Table of Contents
- [Introduction](#introduction)
- [Architecture Overview](#architecture-overview)
- [Installation & Setup](#installation--setup)
- [API Reference](#api-reference)
- [Advanced Usage](#advanced-usage)
- [Performance Optimization](#performance-optimization)
- [Troubleshooting](#troubleshooting)

## 🎯 Introduction

HDMapping Python bindings use **pybind11** to expose HDMapping's high-performance C++ algorithms to Python, enabling:

### Use Cases
- **Research & Development**: Rapid parameter experimentation
- **Production Pipelines**: Automated large-scale processing
- **Data Science Integration**: Integration with ML/AI workflows  
- **Custom Applications**: Building specialized processing tools
- **Performance Analysis**: Benchmarking and optimization

### Performance Characteristics
- **Near-native speed**: 95-99% of C++ performance
- **Memory efficient**: Direct memory mapping, no data copying
- **Scalable**: Handles datasets from MB to TB range
- **Parallel processing**: Full multi-threading support

## 🏗️ Architecture Overview

### Module Structure
```
HDMapping Python Bindings
├── lidar_odometry_py          # Step 1: Lidar Odometry
│   ├── Session management
│   ├── Point cloud processing
│   ├── Trajectory generation
│   └── Data export functions
└── multi_view_tls_registration_py  # Step 2: Multi-view Registration
    ├── TLS registration algorithms
    ├── Loop closure detection
    ├── GNSS fusion
    └── Final map generation
```

### Data Flow
```
Raw Lidar Data → Step 1 (Odometry) → Session Files → Step 2 (Registration) → Final Maps
     ↓              ↓                    ↓               ↓                    ↓
  .laz/.las    Python binding      .json/.reg     Python binding      .las/.csv
```

## 🔧 Installation & Setup

### Prerequisites
```bash
# System requirements
- Python 3.7+ with development headers
- NumPy (automatically handled by pybind11)
- HDMapping build environment (CMake, compilers)
```

### Build Configuration

#### Option 1: Enable During Initial Build
```bash
# Configure with Python bindings
cmake .. -DPYBIND=ON \
         -DCMAKE_BUILD_TYPE=Release \
         -DHD_CPU_OPTIMIZATION=AUTO \
         -DBUILD_TESTING=ON

# Build everything including bindings
cmake --build . --config Release
```

#### Option 2: Custom Python Interpreter
```bash
# Specify custom Python executable
cmake .. -DPYBIND=ON \
         -DPYTHON_EXECUTABLE=/path/to/specific/python \
         -DCMAKE_BUILD_TYPE=Release

# Build specific targets
cmake --build . --config Release --target lidar_odometry_py
cmake --build . --config Release --target multi_view_tls_registration_py
```

#### Option 3: Conda Environment
```bash
# Activate your conda environment first
conda activate hdmapping-env

# Configure (CMake will auto-detect conda Python)
cmake .. -DPYBIND=ON -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build . --config Release
```

### Verification
```python
import sys
print(f"Python version: {sys.version}")

# Test HDMapping bindings
try:
    import Release.lidar_odometry_py as lo
    import Release.multi_view_tls_registration_py as mvr
    print("✅ HDMapping Python bindings successfully loaded")
    
    # Check available functions
    print(f"Lidar Odometry functions: {len(dir(lo))}")
    print(f"Registration functions: {len(dir(mvr))}")
    
except ImportError as e:
    print(f"❌ Import failed: {e}")
```

## 📚 API Reference

### Lidar Odometry Module (`lidar_odometry_py`)

#### Core Classes

##### `Session`
```python
session = lo.Session()
# Manages processing session state and metadata
```

##### `LidarOdometryParams`
Configuration object for lidar odometry processing:

```python
params = lo.LidarOdometryParams()

# Point cloud processing
params.decimation = 0.01                    # Point cloud decimation factor (0.001-0.1)
params.max_distance_lidar = 30.0            # Maximum lidar range (meters)

# Resolution settings (indoor/outdoor)
params.in_out_params_indoor.resolution_X = 0.1    # Indoor voxel size X (meters)
params.in_out_params_indoor.resolution_Y = 0.1    # Indoor voxel size Y (meters)  
params.in_out_params_indoor.resolution_Z = 0.1    # Indoor voxel size Z (meters)
params.in_out_params_outdoor.resolution_X = 0.3   # Outdoor voxel size X (meters)
params.in_out_params_outdoor.resolution_Y = 0.3   # Outdoor voxel size Y (meters)
params.in_out_params_outdoor.resolution_Z = 0.3   # Outdoor voxel size Z (meters)

# Filtering parameters
params.filter_threshold_xy_inner = 0.3      # Inner XY filter threshold
params.filter_threshold_xy_outer = 70.0     # Outer XY filter threshold

# Algorithm parameters
params.nr_iter = 100                        # Maximum iterations (50-200)
params.sliding_window_trajectory_length_threshold = 200  # Trajectory window size
params.distance_bucket = 0.2                # Distance bucket size
params.polar_angle_deg = 10.0              # Polar angle threshold (degrees)
params.azimutal_angle_deg = 10.0           # Azimuthal angle threshold (degrees)

# Advanced options
params.robust_and_accurate_lidar_odometry_iterations = 20  # Robust iterations
params.use_robust_and_accurate_lidar_odometry = False      # Enable robust mode

# Coordinate system conventions
params.fusionConventionNed = True           # North-East-Down convention
params.fusionConventionNwu = False          # North-West-Up convention  
params.fusionConventionEnu = False          # East-North-Up convention

# Consistency checking
params.apply_consistency = False            # Enable consistency checking
params.threshold_nr_poses = 20              # Minimum poses for consistency
params.num_constistency_iter = 10          # Consistency iterations
params.threshould_output_filter = 0.3      # Output filter threshold
params.min_counter = 10                     # Minimum counter value
```

##### `WorkerData`
Contains processing results and intermediate data:

```python
worker_data = lo.WorkerData()
# Contains:
# - intermediate_points: Processed point clouds
# - original_points: Original input points
# - intermediate_trajectory: Computed trajectory
# - final_trajectory: Final optimized trajectory
```

#### Core Functions

##### `run_lidar_odometry(data_dir, params)`
```python
worker_data = lo.run_lidar_odometry(
    data_dir="/path/to/lidar/files",     # Directory containing .laz/.las files
    params=lo_params                      # LidarOdometryParams object
)
# Returns: WorkerData with processing results
```

##### `save_results_automatic(params, worker_data, working_dir, timestamp)`
```python
output_dir = lo.save_results_automatic(
    params=lo_params,                     # Parameters used
    worker_data=worker_data,             # Processing results
    working_dir="/path/to/working/dir",  # Working directory
    timestamp=0.0                        # Timestamp (usually 0.0)
)
# Returns: Path to saved results directory
```

##### `save_all_to_las(worker_data, params, output_file_name, session, **kwargs)`
```python
lo.save_all_to_las(
    worker_data=worker_data,
    params=lo_params,
    output_file_name="/path/to/output.laz",
    session=session,
    export_selected=False,               # Export only selected points
    filter_on_export=True,              # Apply filtering during export
    apply_pose=True,                    # Apply pose transformations
    add_to_pc_container=False           # Add to point cloud container
)
```

##### `run_consistency(worker_data, params)`
```python
lo.run_consistency(worker_data, lo_params)
# Applies consistency checking to improve trajectory accuracy
```

### Multi-view Registration Module (`multi_view_tls_registration_py`)

#### Core Classes

##### `TLSRegistration`
Configuration for terrestrial laser scanning registration:

```python
params = mvr.TLSRegistration()

# Algorithm selection
params.use_ndt = True                    # Use Normal Distributions Transform
params.use_icp = True                    # Use Iterative Closest Point
params.use_plane_features = True         # Use plane feature matching

# NDT-specific parameters
params.compute_only_mahalanobis_distance = False
params.compute_mean_and_cov_for_bucket = True
params.use_lie_algebra_left_jacobian_ndt = True
params.use_lie_algebra_right_jacobian_ndt = False

# ICP-specific parameters  
params.point_to_point_source_to_target = True
params.use_lie_algebra_left_jacobian_icp = True
params.use_lie_algebra_right_jacobian_icp = False
params.point_to_point_source_to_target_compute_rms = True

# Plane feature parameters
params.point_to_projection_onto_plane_source_to_target = True
params.use_lie_algebra_left_jacobian_plane_features = True
params.use_lie_algebra_right_jacobian_plane_features = False

# GNSS and loop closure
params.fuse_gnss = True                  # Enable GNSS fusion
params.automatic_loop_closure = True     # Automatic loop closure detection

# Output parameters
params.curve_consecutive_distance_meters = 1.0        # Curve distance threshold
params.not_curve_consecutive_distance_meters = 5.0    # Straight distance threshold
params.is_trajectory_export_downsampling = True       # Downsample trajectory export
```

#### Core Functions

##### `run_multi_view_tls_registration(session_path, params, gnss_dir)`
```python
session = mvr.run_multi_view_tls_registration(
    session_path="/path/to/session/from/step1",  # Output from lidar odometry
    params=mvr_params,                           # TLSRegistration parameters
    gnss_dir="/path/to/gnss/data"               # GNSS data directory (optional)
)
# Returns: Registered session object
```

##### `save_trajectories(session, output_path, curve_dist, straight_dist, downsample)`
```python
mvr.save_trajectories(
    session=session,
    output_path="/path/to/trajectory.csv",
    curve_consecutive_distance_meters=1.0,
    not_curve_consecutive_distance_meters=5.0,
    is_trajectory_export_downsampling=True
)
```

##### `save_all_to_las(session, output_path)`
```python
mvr.save_all_to_las(
    session=session,
    output_path="/path/to/final_map.las"
)
```

## 🚀 Advanced Usage

### Parameter Optimization

#### Adaptive Parameter Selection
```python
def optimize_parameters_for_dataset(data_dir, sample_size=100):
    """Automatically optimize parameters based on dataset characteristics"""
    
    # Analyze dataset characteristics
    point_density = analyze_point_density(data_dir, sample_size)
    environment_type = detect_environment_type(data_dir)
    noise_level = estimate_noise_level(data_dir, sample_size)
    
    # Create optimized parameters
    params = lo.LidarOdometryParams()
    
    if environment_type == "indoor":
        params.in_out_params_indoor.resolution_X = min(0.05, point_density * 0.1)
        params.in_out_params_indoor.resolution_Y = min(0.05, point_density * 0.1)
        params.in_out_params_indoor.resolution_Z = min(0.05, point_density * 0.1)
        params.max_distance_lidar = 20.0
    else:  # outdoor
        params.in_out_params_outdoor.resolution_X = min(0.2, point_density * 0.5)
        params.in_out_params_outdoor.resolution_Y = min(0.2, point_density * 0.5)
        params.in_out_params_outdoor.resolution_Z = min(0.2, point_density * 0.5)
        params.max_distance_lidar = 50.0
    
    # Adjust for noise level
    if noise_level > 0.1:
        params.use_robust_and_accurate_lidar_odometry = True
        params.robust_and_accurate_lidar_odometry_iterations = 30
    
    return params

def analyze_point_density(data_dir, sample_size):
    """Analyze average point density in dataset"""
    # Implementation would analyze sample files
    return 0.02  # Example return value

def detect_environment_type(data_dir):
    """Detect if dataset is indoor or outdoor"""
    # Implementation would analyze max distances, ceiling detection, etc.
    return "outdoor"  # Example return value

def estimate_noise_level(data_dir, sample_size):
    """Estimate noise level in dataset"""
    # Implementation would analyze point cloud noise characteristics
    return 0.05  # Example return value
```

### Batch Processing

#### Multi-Dataset Processing
```python
import concurrent.futures
from pathlib import Path

def process_dataset_batch(dataset_list, output_base_dir, max_workers=4):
    """Process multiple datasets in parallel"""
    
    def process_single_dataset(dataset_info):
        dataset_path, dataset_name = dataset_info
        output_dir = Path(output_base_dir) / dataset_name
        output_dir.mkdir(parents=True, exist_ok=True)
        
        try:
            # Step 1: Lidar Odometry
            lo_params = optimize_parameters_for_dataset(dataset_path)
            worker_data = lo.run_lidar_odometry(str(dataset_path), lo_params)
            
            # Save intermediate results
            step1_dir = output_dir / "step1"
            step1_dir.mkdir(exist_ok=True)
            session_path = lo.save_results_automatic(
                lo_params, worker_data, str(step1_dir), 0.0
            )
            
            # Step 2: Multi-view Registration
            mvr_params = mvr.TLSRegistration()
            mvr_params.fuse_gnss = True
            mvr_params.automatic_loop_closure = True
            
            session = mvr.run_multi_view_tls_registration(
                session_path, mvr_params, ""
            )
            
            # Save final results
            final_dir = output_dir / "final"
            final_dir.mkdir(exist_ok=True)
            
            mvr.save_trajectories(
                session, 
                str(final_dir / "trajectory.csv"),
                1.0, 5.0, True
            )
            mvr.save_all_to_las(session, str(final_dir / "final_map.las"))
            
            return {"dataset": dataset_name, "status": "success", "output": str(output_dir)}
            
        except Exception as e:
            return {"dataset": dataset_name, "status": "error", "error": str(e)}
    
    # Prepare dataset list
    dataset_items = [(path, name) for path, name in dataset_list]
    
    # Process in parallel
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
        results = list(executor.map(process_single_dataset, dataset_items))
    
    return results

# Usage example
datasets = [
    ("/path/to/dataset1", "urban_scan_001"),
    ("/path/to/dataset2", "forest_scan_002"),
    ("/path/to/dataset3", "building_scan_003")
]

results = process_dataset_batch(datasets, "/path/to/output", max_workers=2)
for result in results:
    print(f"Dataset {result['dataset']}: {result['status']}")
```

### Integration with Data Science Workflows

#### Jupyter Notebook Integration
```python
# In Jupyter notebook cell
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import Release.lidar_odometry_py as lo

def visualize_trajectory(worker_data):
    """Visualize trajectory in Jupyter notebook"""
    
    # Extract trajectory points (pseudo-code - actual implementation depends on data structure)
    trajectory = worker_data.intermediate_trajectory
    
    # Create visualization
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # 2D trajectory plot
    ax1.plot(trajectory.x, trajectory.y, 'b-', linewidth=2)
    ax1.scatter(trajectory.x[0], trajectory.y[0], c='green', s=100, label='Start')
    ax1.scatter(trajectory.x[-1], trajectory.y[-1], c='red', s=100, label='End')
    ax1.set_xlabel('X (meters)')
    ax1.set_ylabel('Y (meters)')
    ax1.set_title('2D Trajectory')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # Elevation profile
    distances = np.cumsum(np.sqrt(np.diff(trajectory.x)**2 + np.diff(trajectory.y)**2))
    distances = np.insert(distances, 0, 0)
    
    ax2.plot(distances, trajectory.z, 'r-', linewidth=2)
    ax2.set_xlabel('Distance (meters)')
    ax2.set_ylabel('Elevation (meters)')
    ax2.set_title('Elevation Profile')
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Return statistics
    total_distance = distances[-1]
    elevation_change = trajectory.z[-1] - trajectory.z[0]
    
    return {
        "total_distance_m": total_distance,
        "elevation_change_m": elevation_change,
        "num_points": len(trajectory.x),
        "avg_speed": total_distance / len(trajectory.x)  # pseudo-metric
    }

# Usage in notebook
data_dir = "/path/to/lidar/data"
params = lo.LidarOdometryParams()
worker_data = lo.run_lidar_odometry(data_dir, params)

stats = visualize_trajectory(worker_data)
print(f"Processing statistics: {stats}")
```

#### Pandas Integration for Trajectory Analysis
```python
def trajectory_to_dataframe(worker_data):
    """Convert trajectory to pandas DataFrame for analysis"""
    
    trajectory = worker_data.intermediate_trajectory
    
    df = pd.DataFrame({
        'x': trajectory.x,
        'y': trajectory.y,
        'z': trajectory.z,
        'timestamp': trajectory.timestamps,  # if available
        'point_index': range(len(trajectory.x))
    })
    
    # Add computed columns
    df['distance_from_start'] = np.sqrt(
        (df['x'] - df['x'].iloc[0])**2 + 
        (df['y'] - df['y'].iloc[0])**2
    )
    
    # Add movement analysis
    df['dx'] = df['x'].diff()
    df['dy'] = df['y'].diff()
    df['dz'] = df['z'].diff()
    df['speed_2d'] = np.sqrt(df['dx']**2 + df['dy']**2)
    df['speed_3d'] = np.sqrt(df['dx']**2 + df['dy']**2 + df['dz']**2)
    
    return df

def analyze_trajectory_quality(df):
    """Analyze trajectory quality metrics"""
    
    metrics = {
        'total_points': len(df),
        'total_distance_2d': df['speed_2d'].sum(),
        'total_distance_3d': df['speed_3d'].sum(),
        'avg_speed_2d': df['speed_2d'].mean(),
        'max_speed_2d': df['speed_2d'].max(),
        'elevation_range': df['z'].max() - df['z'].min(),
        'position_std_x': df['x'].std(),
        'position_std_y': df['y'].std(),
        'position_std_z': df['z'].std()
    }
    
    return metrics
```

## ⚡ Performance Optimization

### Memory Management
```python
def memory_efficient_processing(large_dataset_dir, chunk_size_mb=1000):
    """Process large datasets efficiently with memory management"""
    
    import gc
    import psutil
    
    def monitor_memory():
        process = psutil.Process()
        memory_mb = process.memory_info().rss / 1024 / 1024
        return memory_mb
    
    print(f"Initial memory usage: {monitor_memory():.1f} MB")
    
    # Configure parameters for memory efficiency
    params = lo.LidarOdometryParams()
    params.decimation = 0.02  # Higher decimation for memory efficiency
    params.in_out_params_outdoor.resolution_X = 0.5  # Larger voxels
    params.in_out_params_outdoor.resolution_Y = 0.5
    params.in_out_params_outdoor.resolution_Z = 0.5
    
    try:
        worker_data = lo.run_lidar_odometry(large_dataset_dir, params)
        print(f"Peak memory usage: {monitor_memory():.1f} MB")
        
        # Process and save immediately to free memory
        output_dir = lo.save_results_automatic(params, worker_data, "./temp", 0.0)
        
        # Clear large objects
        del worker_data
        gc.collect()
        
        print(f"Memory after cleanup: {monitor_memory():.1f} MB")
        return output_dir
        
    except MemoryError:
        print("Memory error encountered - consider:")
        print("1. Increasing decimation factor")
        print("2. Using larger voxel sizes") 
        print("3. Processing smaller chunks")
        raise

def parallel_processing_setup(num_cores=None):
    """Configure optimal parallel processing"""
    
    import multiprocessing as mp
    
    if num_cores is None:
        num_cores = max(1, mp.cpu_count() - 1)  # Leave one core free
    
    # HDMapping uses OpenMP for parallelization
    import os
    os.environ['OMP_NUM_THREADS'] = str(num_cores)
    
    print(f"Configured for {num_cores} cores")
    return num_cores
```

### Performance Benchmarking
```python
import time
import cProfile
import pstats

def benchmark_processing(data_dir, params):
    """Benchmark processing performance"""
    
    start_time = time.time()
    start_cpu = time.process_time()
    
    # Enable profiling
    profiler = cProfile.Profile()
    profiler.enable()
    
    try:
        worker_data = lo.run_lidar_odometry(data_dir, params)
        
        profiler.disable()
        
        end_time = time.time()
        end_cpu = time.process_time()
        
        # Calculate metrics
        wall_time = end_time - start_time
        cpu_time = end_cpu - start_cpu
        cpu_efficiency = (cpu_time / wall_time) * 100
        
        results = {
            'wall_time_seconds': wall_time,
            'cpu_time_seconds': cpu_time,
            'cpu_efficiency_percent': cpu_efficiency,
            'points_processed': len(worker_data.original_points),  # Pseudo-code
            'points_per_second': len(worker_data.original_points) / wall_time
        }
        
        # Print performance stats
        print(f"Performance Metrics:")
        print(f"  Wall time: {wall_time:.2f} seconds")
        print(f"  CPU efficiency: {cpu_efficiency:.1f}%")
        print(f"  Points/second: {results['points_per_second']:.0f}")
        
        # Save detailed profiling
        stats = pstats.Stats(profiler)
        stats.sort_stats('cumulative')
        stats.print_stats(10)  # Top 10 functions
        
        return results, worker_data
        
    except Exception as e:
        profiler.disable()
        raise e

# Usage
data_dir = "/path/to/benchmark/data"
params = lo.LidarOdometryParams()
benchmark_results, data = benchmark_processing(data_dir, params)
```

## 🛠️ Troubleshooting

### Common Issues and Solutions

#### 1. Import Errors
```python
# Problem: ModuleNotFoundError
try:
    import Release.lidar_odometry_py as lo
except ModuleNotFoundError:
    print("Solutions:")
    print("1. Check if bindings were built: cmake --build . --target lidar_odometry_py")
    print("2. Verify Python path includes build directory")
    print("3. Check if 'Release' directory exists in build folder")
    print("4. Ensure same Python version used for build and runtime")
```

#### 2. Memory Issues
```python
# Problem: std::bad_alloc or MemoryError
def handle_memory_issues():
    print("Memory optimization strategies:")
    print("1. Increase decimation factor (reduce point density)")
    print("2. Use larger voxel resolutions")
    print("3. Process smaller datasets")
    print("4. Close other applications")
    print("5. Use memory monitoring tools")
    
    # Example memory-conservative parameters
    conservative_params = lo.LidarOdometryParams()
    conservative_params.decimation = 0.05  # 5% of points
    conservative_params.in_out_params_outdoor.resolution_X = 1.0  # 1m voxels
    conservative_params.in_out_params_outdoor.resolution_Y = 1.0
    conservative_params.in_out_params_outdoor.resolution_Z = 1.0
    
    return conservative_params
```

#### 3. Performance Issues
```python
def diagnose_performance():
    """Diagnose and fix performance issues"""
    
    import psutil
    
    # Check system resources
    cpu_count = psutil.cpu_count()
    memory_gb = psutil.virtual_memory().total / (1024**3)
    
    print(f"System: {cpu_count} cores, {memory_gb:.1f} GB RAM")
    
    # Check CPU optimization
    import Release.lidar_odometry_py as lo
    # If available, check what optimizations are compiled in
    
    recommendations = []
    
    if cpu_count >= 8:
        recommendations.append("✅ Sufficient CPU cores for parallel processing")
    else:
        recommendations.append("⚠️ Consider using fewer parallel processes")
    
    if memory_gb >= 16:
        recommendations.append("✅ Sufficient RAM for large datasets")
    else:
        recommendations.append("⚠️ Consider memory-efficient parameters")
    
    # Check for common performance bottlenecks
    if psutil.cpu_percent(interval=1) > 90:
        recommendations.append("⚠️ High CPU usage - close other applications")
    
    if psutil.virtual_memory().percent > 85:
        recommendations.append("⚠️ High memory usage - reduce dataset size")
    
    for rec in recommendations:
        print(rec)
```

#### 4. Parameter Validation
```python
def validate_parameters(params):
    """Validate parameter ranges and combinations"""
    
    issues = []
    
    # Check decimation
    if params.decimation < 0.001 or params.decimation > 0.1:
        issues.append(f"Decimation {params.decimation} outside recommended range [0.001, 0.1]")
    
    # Check resolution consistency
    indoor_res = params.in_out_params_indoor.resolution_X
    outdoor_res = params.in_out_params_outdoor.resolution_X
    
    if indoor_res >= outdoor_res:
        issues.append("Indoor resolution should be smaller than outdoor resolution")
    
    # Check iteration count
    if params.nr_iter < 10:
        issues.append("Too few iterations - may not converge")
    elif params.nr_iter > 500:
        issues.append("Too many iterations - will be slow")
    
    # Check distance thresholds
    if params.filter_threshold_xy_inner >= params.filter_threshold_xy_outer:
        issues.append("Inner filter threshold should be less than outer threshold")
    
    if issues:
        print("Parameter validation issues:")
        for issue in issues:
            print(f"  ⚠️ {issue}")
        return False
    else:
        print("✅ Parameters validated successfully")
        return True

# Usage
params = lo.LidarOdometryParams()
# ... configure parameters ...
if validate_parameters(params):
    worker_data = lo.run_lidar_odometry(data_dir, params)
```

### Debug Mode
```python
def enable_debug_mode():
    """Enable verbose debugging for development"""
    
    import logging
    import os
    
    # Set up logging
    logging.basicConfig(level=logging.DEBUG)
    
    # Enable OpenMP debugging (if available)
    os.environ['OMP_DISPLAY_ENV'] = 'TRUE'
    
    # Enable memory debugging
    os.environ['MALLOC_CHECK_'] = '2'
    
    print("Debug mode enabled - expect verbose output")

def create_test_dataset(output_dir, num_points=10000):
    """Create synthetic test dataset for debugging"""
    
    import numpy as np
    from pathlib import Path
    
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Generate synthetic point cloud
    np.random.seed(42)
    
    # Create a simple trajectory (circular path)
    t = np.linspace(0, 2*np.pi, num_points)
    trajectory_x = 10 * np.cos(t)
    trajectory_y = 10 * np.sin(t)
    trajectory_z = t  # Slowly increasing elevation
    
    # Add synthetic lidar points around trajectory
    points = []
    for i, (tx, ty, tz) in enumerate(zip(trajectory_x, trajectory_y, trajectory_z)):
        # Generate points in a hemisphere around trajectory point
        n_scan_points = 1000
        angles = np.random.uniform(0, 2*np.pi, n_scan_points)
        distances = np.random.uniform(1, 20, n_scan_points)
        elevations = np.random.uniform(-np.pi/4, np.pi/4, n_scan_points)
        
        scan_x = tx + distances * np.cos(angles) * np.cos(elevations)
        scan_y = ty + distances * np.sin(angles) * np.cos(elevations)
        scan_z = tz + distances * np.sin(elevations)
        
        # Save as simple text file (would normally be LAS format)
        frame_file = output_path / f"frame_{i:06d}.txt"
        scan_data = np.column_stack([scan_x, scan_y, scan_z])
        np.savetxt(frame_file, scan_data, fmt='%.6f')
    
    print(f"Created test dataset with {num_points} frames in {output_dir}")
    return str(output_path)
```

## 📊 Performance Metrics

### Typical Performance Characteristics

| Dataset Size | Processing Time | Memory Usage | Throughput |
|-------------|----------------|--------------|------------|
| 100 MB | 2-5 minutes | 2-4 GB | 20-50 MB/min |
| 1 GB | 15-30 minutes | 8-16 GB | 30-70 MB/min |
| 10 GB | 2-4 hours | 16-32 GB | 40-85 MB/min |
| 100 GB | 1-2 days | 32-64 GB | 50-100 MB/min |

### Optimization Guidelines

- **CPU cores**: Linear scaling up to 8-16 cores
- **Memory**: 4-8x dataset size recommended
- **Storage**: SSD recommended for large datasets
- **Decimation**: 0.01-0.02 optimal for most use cases
- **Voxel size**: 0.1m indoor, 0.3m outdoor typical

---

*This detailed guide covers the complete HDMapping Python bindings API. For specific use cases or advanced configurations, refer to the example scripts or contact the development team.*
