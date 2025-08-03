# 🔧 Fix TOML Structure: Separate motion_model_correction Section

## 📋 Summary
This PR resolves **Issue #155** by separating the `motion_model_correction` parameters into their own dedicated TOML section, improving configuration file organization and readability.

## 🎯 Issue Reference
- **Fixes:** #155 
- **Requested by:** @JanuszBedkowski

## 🔄 Changes Made

### Modified File
- `apps/lidar_odometry_step_1/toml_io.h` (Line 94)

### Technical Change
**Before:**
```cpp
{"motion_model_uncertainty", {"lidar_odometry_motion_model_x_1_sigma_m", "lidar_odometry_motion_model_y_1_sigma_m", "lidar_odometry_motion_model_z_1_sigma_m", "lidar_odometry_motion_model_om_1_sigma_deg", "lidar_odometry_motion_model_fi_1_sigma_deg", "lidar_odometry_motion_model_ka_1_sigma_deg", "motion_model_correction_om", "motion_model_correction_fi", "motion_model_correction_ka"}},
```

**After:**
```cpp
{"motion_model_uncertainty", {"lidar_odometry_motion_model_x_1_sigma_m", "lidar_odometry_motion_model_y_1_sigma_m", "lidar_odometry_motion_model_z_1_sigma_m", "lidar_odometry_motion_model_om_1_sigma_deg", "lidar_odometry_motion_model_fi_1_sigma_deg", "lidar_odometry_motion_model_ka_1_sigma_deg"}},
{"motion_model_correction", {"motion_model_correction_om", "motion_model_correction_fi", "motion_model_correction_ka"}},
```

## 📁 TOML File Structure Impact

### Before (Mixed Parameters):
```toml
[motion_model_uncertainty]
lidar_odometry_motion_model_fi_1_sigma_deg = 0.01
lidar_odometry_motion_model_ka_1_sigma_deg = 0.01
lidar_odometry_motion_model_om_1_sigma_deg = 0.01
lidar_odometry_motion_model_x_1_sigma_m = 5e-04
lidar_odometry_motion_model_y_1_sigma_m = 5e-04
lidar_odometry_motion_model_z_1_sigma_m = 5e-04
motion_model_correction_fi = 0.0        # ❌ Conceptually wrong section
motion_model_correction_ka = 0.0
motion_model_correction_om = 0.0
```

### After (Logically Separated):
```toml
[motion_model_uncertainty]
lidar_odometry_motion_model_fi_1_sigma_deg = 0.01
lidar_odometry_motion_model_ka_1_sigma_deg = 0.01
lidar_odometry_motion_model_om_1_sigma_deg = 0.01
lidar_odometry_motion_model_x_1_sigma_m = 5e-04
lidar_odometry_motion_model_y_1_sigma_m = 5e-04
lidar_odometry_motion_model_z_1_sigma_m = 5e-04

[motion_model_correction]               # ✅ Dedicated section
motion_model_correction_fi = 0.0
motion_model_correction_ka = 0.0
motion_model_correction_om = 0.0
```

## ✅ Benefits

1. **Improved Organization**: Logical separation between uncertainty parameters (sigma values) and correction parameters (offset values)
2. **Better Readability**: Clearer TOML configuration files for users
3. **Semantic Correctness**: Parameters are grouped by their conceptual purpose
4. **TOML Best Practices**: Follows standard TOML organization patterns

## 🧪 Testing

- ✅ **Compilation**: Code compiles successfully without errors
- ✅ **Backward Compatibility**: All existing parameter names unchanged
- ✅ **Functionality**: TOML loading/saving works correctly
- ✅ **Structure**: Generated TOML files now have proper section separation

## 🔍 Verification Steps

1. Build the project:
   ```bash
   cmake --build . --config Release --target lidar_odometry_step_1
   ```

2. Generate a TOML configuration file from the application

3. Verify the file structure matches the "After" example above

4. Load the TOML file back into the application to confirm all parameters are read correctly

## 📝 Commit Details

- **Commit:** `a2a1168`
- **Message:** "Fix TOML structure: separate motion_model_correction section (issue #155)"
- **Files Changed:** 1 file, 2 insertions(+), 1 deletion(-)

## 🔧 Implementation Notes

- **Minimal Change**: Only modified the parameter grouping in `toml_io.h`
- **No API Changes**: All public interfaces remain identical
- **Zero Breaking Changes**: Existing code continues to work unchanged
- **Documentation Ready**: Change is self-documenting through improved structure

## 🎯 Related Work

This PR is part of the HDMapping optimization and maintenance effort, which includes:
- CPU architecture optimization system
- Cross-platform compatibility improvements  
- Enhanced configuration management

---

**Ready for Review and Merge** ✅

This change directly addresses the specific request in Issue #155 and improves the overall code organization without any breaking changes.
