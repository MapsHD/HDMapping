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
- Requires Step 1 results as input
- GNSS integration significantly improves accuracy
- Loop closure detection helps with drift correction
- Final step in the HDMapping pipeline

---
*Documentation status: ⚠️ Basic template - needs detailed documentation*

<!-- 
TODO for detailed documentation:
- [ ] Add registration algorithm details
- [ ] Add GNSS integration workflow
- [ ] Add loop closure configuration
- [ ] Add quality assessment metrics
- [ ] Add troubleshooting guide
-->
