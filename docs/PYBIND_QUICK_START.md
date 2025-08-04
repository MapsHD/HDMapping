# HDMapping Python Bindings - Quick Start Guide

## 🐍 Overview

HDMapping Python bindings provide high-performance access to HDMapping's core algorithms from Python, enabling:
- **Batch processing** without GUI overhead
- **Integration** with Python data science workflows
- **Rapid prototyping** with known parameters
- **Automated pipeline** development

## ⚡ Quick Setup

### 1. Enable in CMake Build
```bash
# Enable Python bindings during build
cmake .. -DPYBIND=ON -DCMAKE_BUILD_TYPE=Release

# Build the bindings
cmake --build . --config Release --target lidar_odometry_py
cmake --build . --config Release --target multi_view_tls_registration_py
```

### 2. Verify Installation
```python
# Test imports
import Release.lidar_odometry_py as lo
import Release.multi_view_tls_registration_py as mvr

print("HDMapping Python bindings loaded successfully!")
```

## 🚀 Basic Usage

### Step 1: Lidar Odometry
```python
import os
import Release.lidar_odometry_py as lo

# Configuration
DATA_DIR = "path/to/lidar/data"
WORKING_DIR = "path/to/working/directory"
OUTPUT_DIR = "path/to/output"

# Create session and parameters
dummy_session = lo.Session()
lo_params = lo.LidarOdometryParams()

# Basic configuration
lo_params.decimation = 0.01
lo_params.in_out_params_indoor.resolution_X = 0.1
lo_params.nr_iter = 100
lo_params.max_distance_lidar = 30.0

# Run processing
worker_data = lo.run_lidar_odometry(DATA_DIR, lo_params)

# Save results
output_dir = lo.save_results_automatic(lo_params, worker_data, WORKING_DIR, 0.0)
lo.save_all_to_las(
    worker_data=worker_data,
    params=lo_params,
    output_file_name=os.path.join(OUTPUT_DIR, "result.laz"),
    session=dummy_session,
    export_selected=False,
    filter_on_export=True,
    apply_pose=True
)
```

### Step 2: Multi-view Registration
```python
import Release.multi_view_tls_registration_py as mvr

# Configuration
SESSION_PATH = "path/to/session/from/step1"
OUTPUT_DIR = "path/to/output"

# Create parameters
mvr_params = mvr.TLSRegistration()
mvr_params.fuse_gnss = True
mvr_params.automatic_loop_closure = True

# Run registration
session = mvr.run_multi_view_tls_registration(SESSION_PATH, mvr_params, "")

# Save results
mvr.save_trajectories(session, os.path.join(OUTPUT_DIR, "trajectory.csv"))
mvr.save_all_to_las(session, os.path.join(OUTPUT_DIR, "final_map.las"))
```

## 📊 Typical Workflow

```python
# Complete pipeline example
def process_lidar_dataset(data_dir, output_dir):
    """Process complete lidar dataset through both steps"""
    
    # Step 1: Lidar Odometry
    lo_params = configure_lidar_odometry()
    worker_data = lo.run_lidar_odometry(data_dir, lo_params)
    session_path = save_step1_results(worker_data, lo_params)
    
    # Step 2: Multi-view Registration  
    mvr_params = configure_registration()
    final_session = mvr.run_multi_view_tls_registration(session_path, mvr_params)
    save_final_results(final_session, output_dir)
    
    return final_session
```

## 🎯 Key Benefits

- **10-100x faster** than GUI for batch processing
- **Native C++ performance** with Python convenience  
- **Seamless integration** with existing Python workflows
- **Parameter optimization** through scripting
- **Automated quality control** and validation

## 📋 Requirements

- **Python 3.7+** with development headers
- **NumPy** for array operations
- **HDMapping** built with `PYBIND=ON`
- **Same Python version** used for build and runtime

## 🔗 Next Steps

- See [Detailed Guide](PYBIND_DETAILED_GUIDE.md) for complete API reference
- Check [example.py](../pybind/example.py) for working examples
- Review parameter optimization strategies in detailed documentation

---

*For complete API documentation and advanced usage patterns, see the detailed guide.*
