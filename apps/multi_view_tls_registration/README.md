# Multi-View TLS Registration

## Purpose
Second step of HDMapping pipeline: performs multi-view terrestrial laser scanning registration with loop closure detection and GNSS fusion.

## Quick Start
```bash
# Process Step 1 results
./multi_view_tls_registration --session step1_session.json --output final_results/

# With GNSS data
./multi_view_tls_registration --session step1_session.json --gnss gnss_data/ --output results/
```

## Input/Output
- **Input**: Session file from Step 1 (.json), GNSS data (optional), configuration
- **Output**: Final registered map (.las), optimized trajectory (.csv), final session

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--session` | - | Input session file from Step 1 |
| `--output` | "./output" | Output directory |
| `--gnss` | - | GNSS data directory (optional) |
| `--loop-closure` | true | Enable automatic loop closure |
| `--use-ndt` | true | Use Normal Distributions Transform |
| `--use-icp` | true | Use Iterative Closest Point |

## Usage Example
```bash
# Full registration with GNSS
./multi_view_tls_registration \
  --session /results/step1/session.json \
  --gnss /data/gnss/ \
  --output /results/final/ \
  --loop-closure
```

## Notes
- **Step 2 Integration**: Requires Step 1 session.json as input
- **Export Centralization**: Uses unified export functions from core library (PR #160)  
- **GNSS Fusion**: Optional but significantly improves accuracy
- **Loop Closure**: Automatic drift correction and trajectory optimization
- **Final Pipeline Stage**: Produces final registered point cloud maps

## Related Applications
- Requires: `lidar_odometry_step_1` output session
- Outputs compatible with: Various export and analysis tools

---
*Documentation status: ✅ Updated for HDMapping v0.85.0 with PR #160 export centralization*
- [ ] Add GNSS integration workflow
- [ ] Add loop closure configuration
- [ ] Add quality assessment metrics
- [ ] Add troubleshooting guide
-->
