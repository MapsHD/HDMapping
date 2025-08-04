# Matrix Multiplication

## Purpose
High-performance matrix multiplication utility with CPU optimization for mathematical operations in mapping algorithms.

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
