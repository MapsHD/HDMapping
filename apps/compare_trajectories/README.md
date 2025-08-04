# Compare Trajectories

## Purpose
Analysis tool for comparing multiple trajectory files, computing differences, and generating comparison reports with statistical metrics.

## Quick Start
```bash
# Compare two trajectories
./compare_trajectories --ref reference.csv --test trajectory.csv --output comparison/

# Batch comparison
./compare_trajectories --ref ref.csv --test-dir trajectories/ --output batch_results/
```

## Input/Output
- **Input**: Reference trajectory (.csv), test trajectory files, comparison parameters
- **Output**: Comparison reports, difference plots, statistical analysis, aligned trajectories

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--ref` | - | Reference trajectory file |
| `--test` | - | Test trajectory file |
| `--output` | "./comparison" | Output directory |
| `--alignment` | true | Align trajectories before comparison |
| `--metrics` | "rmse,mae" | Metrics to compute |

## Usage Example
```bash
# Detailed comparison with alignment
./compare_trajectories \
  --ref /ground_truth/trajectory.csv \
  --test /results/estimated_trajectory.csv \
  --output /analysis/comparison/ \
  --alignment \
  --metrics "rmse,mae,max_error"
```

## Notes
- Supports automatic trajectory alignment
- Provides comprehensive statistical analysis
- Useful for algorithm validation and benchmarking
- Generates publication-ready plots and reports

---
*Documentation status: ⚠️ Basic template - needs detailed documentation*

<!-- 
TODO for detailed documentation:
- [ ] Add supported trajectory formats
- [ ] Add alignment algorithm details
- [ ] Add available metrics reference
- [ ] Add plot customization options
-->
