# Quick Start Demo

## Purpose
Demonstration application showcasing HDMapping capabilities with sample data and simplified workflow for new users.

## Quick Start
```bash
# Run demo with included sample data
./quick_start_demo

# Run with custom data
./quick_start_demo --data /path/to/sample/data
```

## Input/Output
- **Input**: Sample point cloud data (included), or custom small dataset
- **Output**: Demo results, tutorial output files, performance metrics

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--data` | "./sample_data" | Path to demo data |
| `--output` | "./demo_output" | Output directory |
| `--tutorial` | true | Show tutorial messages |
| `--benchmark` | false | Run performance benchmarks |

## Usage Example
```bash
# Interactive demo
./quick_start_demo --tutorial

# Performance testing
./quick_start_demo --benchmark --data /path/to/test/data
```

## Notes
- Perfect for new users learning HDMapping workflow
- Includes sample data and step-by-step guidance
- Minimal resource requirements for demonstration purposes
- Shows end-to-end pipeline in simplified form

---
*Documentation status: ⚠️ Basic template - needs detailed documentation*

<!-- 
TODO for detailed documentation:
- [ ] Add tutorial workflow details
- [ ] Add sample data descriptions
- [ ] Add benchmark interpretation
- [ ] Add next steps guidance
-->
