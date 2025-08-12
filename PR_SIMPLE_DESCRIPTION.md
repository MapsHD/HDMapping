# Fix TOML structure: separate motion_model_correction section

## Description
This PR resolves **Issue #155** by separating the `motion_model_correction` parameters into their own dedicated TOML section, improving configuration file organization and readability.

## Problem
In the current implementation, `motion_model_correction_*` parameters are incorrectly grouped with uncertainty parameters in the `[motion_model_uncertainty]` section.

## Solution
- Moved `motion_model_correction_om`, `motion_model_correction_fi`, and `motion_model_correction_ka` to a separate `[motion_model_correction]` section
- Updated the parameter grouping in `apps/lidar_odometry_step_1/toml_io.h`

## Before/After

**Before:**
```toml
[motion_model_uncertainty]
lidar_odometry_motion_model_fi_1_sigma_deg = 0.01
# ... other sigma parameters
motion_model_correction_fi = 0.0  # ❌ Wrong section
motion_model_correction_ka = 0.0
motion_model_correction_om = 0.0
```

**After:**
```toml
[motion_model_uncertainty]
lidar_odometry_motion_model_fi_1_sigma_deg = 0.01
# ... only sigma parameters

[motion_model_correction]  # ✅ Dedicated section
motion_model_correction_fi = 0.0
motion_model_correction_ka = 0.0
motion_model_correction_om = 0.0
```

## Impact
- ✅ Improved logical organization of TOML configuration files
- ✅ No breaking changes - all parameter names remain identical
- ✅ Enhanced readability and maintainability
- ✅ Follows TOML best practices for parameter grouping

## Testing
- Code compiles successfully
- TOML loading/saving functionality verified
- No impact on existing functionality

Closes #155
