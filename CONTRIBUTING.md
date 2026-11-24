# Contributing to HDMapping

Project rules for code, builds and PR scope are in [AGENTS.md](AGENTS.md). They apply to people and coding agents
alike. This file adds the rule that protects the processing results.

## The LIO trajectory must not change

HDMapping's lidar-inertial odometry (`lidar_odometry_step_1`) is state of the art. A PR must not change the trajectory
it computes. CI checks this on every PR, on Linux, Windows and macOS: it runs step 1 with default parameters on
`calibration-room/trial-1` of the [HUMANOID-LIO-DATASET](https://zenodo.org/records/21318128) (380 s, about 300 m, so
it crosses the 200 m sliding-window reset) and fails when the ATE against that platform's golden trajectory,
`tests/golden/calibration-room-trial-1-<Linux|Windows|macOS>.tum`, is above 1 mm.

There is one golden trajectory per platform. Each platform repeats its own result exactly, but the compilers and
standard libraries differ in rounding and in how they order equal elements, so the platforms differ from each other by
about 1-1.5 cm on this recording.

This covers every change, not only changes to the odometry code:

- **Bug fixes.** A fix that changes the trajectory changes behaviour. Making dead code take effect (using a result
  that was discarded, enabling a branch that never ran, correcting the units of a weight) is a change to the algorithm,
  not a cleanup.
- **Default parameters.** They are part of the algorithm. Changing a default changes the trajectory.
- **Refactors, performance work, dependency updates.** They must keep the trajectory as it is.

When the check fails, find what in your change moved the trajectory and keep it as it was. Don't change the tolerance
or the golden trajectory to make the check pass.

## Improving the odometry

A change that is meant to improve the trajectory is welcome, but it goes in its own PR and it must show:

1. **Results on datasets.** Accuracy against ground truth (for example TLS) before and after the change, on several
   recordings from different environments, including recordings longer than
   `sliding_window_trajectory_length_threshold` (200 m). Use the datasets listed in AGENTS.md and the
   [HDMapping datasets](README.md#our-datasets). Report the numbers in the PR description.
2. **The theory.** Why the change is correct and why it improves the result: the math, the model, or a reference.
   A change that only makes the numbers better on one recording is not enough.

Only when both hold up are the golden trajectories updated, in the same PR: each
`tests/golden/calibration-room-trial-1-<platform>.tum` is taken from that PR's CI artifact for the platform
(`lio-calibration-room-trial-1-Linux`, `-Windows`, `-macOS`).

**The decision to update the golden trajectories belongs to Janusz Będkowski and Michał Pełka.** `CODEOWNERS` makes
their approval required for any change under `tests/golden/`, to the CI check and to its tolerance.

## PR checklist

Copy this into the PR description:

```
### Does this PR change the LIO trajectory?
- [ ] No. The LIO trajectory regression check passes on all platforms.
- [ ] Yes, on purpose:
  - datasets and accuracy against ground truth, before -> after:
  - theory (why it is correct and why it is better):
  - golden trajectory updated in this PR: yes / no
  - line for the release notes:
```
