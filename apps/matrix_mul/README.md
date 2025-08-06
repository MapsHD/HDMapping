# matrix_mul - 4x4 Matrix Multiplication Utility

## Overview

The `matrix_mul` application is a command-line utility for multiplying two 4x4 transformation matrices. It's designed for composing geometric transformations in HDMapping workflows and performance benchmarking of matrix operations.

**Post-PR #160 Status**: ✅ No changes - Pure CLI utility unaffected by export centralization.

## Purpose

- **Matrix composition**: Combine multiple transformation matrices
- **Coordinate system conversions**: Convert between different reference frames
- **Performance testing**: Benchmark matrix multiplication operations
- **Workflow integration**: Part of automated transformation pipelines

## Usage

### Command Line Interface

```bash
matrix_mul <matrix1_file> <matrix2_file>
```

### Parameters

- `matrix1_file`: Path to first 4x4 transformation matrix file
- `matrix2_file`: Path to second 4x4 transformation matrix file

### Matrix File Format

Each matrix file contains exactly 3 lines with 4 space-separated values each (representing the first 3 rows of a 4x4 homogeneous transformation matrix):

```
m00 m01 m02 m03
m10 m11 m12 m13
m20 m21 m22 m23
```

The fourth row [0 0 0 1] is implied for homogeneous transformations.

## Examples

### Basic Usage

1. **Create matrix files**:

   `pose1.txt`:
   ```
   1.0 0.0 0.0 10.5
   0.0 1.0 0.0 5.2
   0.0 0.0 1.0 2.1
   ```

   `pose2.txt`:
   ```
   0.866 -0.5 0.0 2.0
   0.5 0.866 0.0 1.0
   0.0 0.0 1.0 0.0
   ```

2. **Execute multiplication**:
   ```bash
   matrix_mul pose1.txt pose2.txt
   ```

3. **Result** (printed to stdout):
   ```
   0.866 -0.5 0.0 12.5
   0.5 0.866 0.0 6.2
   0.0 0.0 1.0 2.1
   0.0 0.0 0.0 1.0
   ```

## CLI Batch Processing Features

### Automated Transformation Chains

```bash
#!/bin/bash
# Compose multiple transformations
echo "Processing transformation chain..."

# Chain multiple transformations: T_final = T3 * T2 * T1
matrix_mul transform1.txt transform2.txt > temp_result.txt
matrix_mul temp_result.txt transform3.txt > final_transform.txt

echo "Final transformation saved to final_transform.txt"
rm temp_result.txt
```

### Integration with HDMapping Pipeline

```bash
#!/bin/bash
# Convert coordinate systems in batch
for session in sessions/*/; do
    session_name=$(basename "$session")
    
    # Extract transformation from session
    echo "Processing $session_name"
    
    # Apply coordinate system transformation
    matrix_mul base_transform.txt "$session/pose.txt" > "transforms/$session_name.txt"
done
```

## Performance & Benchmarking

### CPU Architecture Benefits
- **AUTO mode**: Automatically detects and uses optimal SIMD instructions
- **Intel CPUs**: AVX2/AVX optimizations for faster matrix operations  
- **AMD CPUs**: Optimized instruction scheduling
- **ARM CPUs**: NEON vectorization for embedded/mobile platforms

## Quick Start
```bash
# Basic matrix multiplication
./matrix_mul --matrix1 A.txt --matrix2 B.txt --output result.txt

# Benchmark performance
./matrix_mul --benchmark --size 1000 --iterations 100
```

## Input/Output
- **Input**: Matrix files (various formats), benchmark parameters
- **Output**: Result matrices, performance metrics

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--matrix1` | - | First input matrix |
| `--matrix2` | - | Second input matrix |
| `--output` | "result.txt" | Output file |
| `--benchmark` | false | Benchmark mode |
| `--cpu_opt` | "auto" | CPU optimization |

## Usage Example
```bash
# Optimized multiplication
./matrix_mul \
  --matrix1 /data/transform_A.txt \
  --matrix2 /data/transform_B.txt \
  --output /results/combined_transform.txt \
  --cpu_opt AVX2
```

## Notes
- Utilizes CPU-specific optimizations
- Supports various matrix formats
- Useful for testing optimization performance
- Includes comprehensive benchmarking tools

---
*Documentation status: 📝 TODO - needs detailed documentation*
