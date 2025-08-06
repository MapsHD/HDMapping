# AHRS Gain Parameter Guide

## Overview

The `ahrs_gain` parameter is a **crucial setting** in the AHRS (Attitude and Heading Reference System) filter used for **IMU data fusion** in HDMapping's lidar odometry algorithm.

## Technical Background

### What is AHRS Gain?

`ahrs_gain` is the gain parameter for the **complementary filter AHRS** from the **Fusion library** (developed by Seb Madgwick), which combines:
- **Gyroscope** - for angular velocity measurements
- **Accelerometer** - for orientation estimation relative to gravity
- **Magnetometer** (optional) - for magnetic reference

### Algorithm Function

The filter works as a complementary filter:
```
Orientation = Gyroscope_Integration + Feedback_Term
Feedback_Term = Orientation_Error × ahrs_gain
```

From HDMapping code:
```cpp
ahrs.settings.gain = ahrs_gain;  // Default value: 0.5
```

## Impact of Different Gain Values

### Low Gain Values (< 0.5)

**Advantages:**
- More trust in gyroscope data
- Fast response to real movements
- **Robust against false accelerations from dynamic motion**

**Disadvantages:**
- Susceptible to gyroscope drift over time
- Slower convergence at startup

### High Gain Values (> 0.5)

**Advantages:**
- Aggressive drift correction
- Long-term stability

**Disadvantages:**
- More sensitive to external accelerations and magnetic disturbances
- Can cause oscillations during dynamic movements

### Optimal Value (≈ 0.5)

- Balance between **fast response** and **long-term stability**
- Recommended for most applications

## Implementation in HDMapping

### Core Function

```cpp
void calculate_trajectory(
    Trajectory &trajectory, Imu &imu_data, 
    bool fusionConventionNwu, bool fusionConventionEnu, bool fusionConventionNed, 
    double ahrs_gain)  // Key parameter
{
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    
    // Set custom gain
    ahrs.settings.gain = ahrs_gain;
    
    // For each IMU measurement
    for (const auto &[timestamp_pair, gyr, acc] : imu_data) {
        // Convert to Fusion format
        const FusionVector gyroscope = {gyr.axis.x * 180.0/M_PI, ...};
        const FusionVector accelerometer = {acc.axis.x, acc.axis.y, acc.axis.z};
        
        // Update filter with set gain
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);
        
        // Extract orientation
        FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
    }
}
```

### Configuration

```cpp
// In GUI interface
ImGui::InputDouble("ahrs_gain", &params.ahrs_gain);

// In TOML file
[madgwick_filter]
ahrs_gain = 0.5
```

## For Dynamic Motion Applications

### **YES, for abrupt movements use LOWER `ahrs_gain`!**

#### Physical Principle

**Abrupt movements** generate **large accelerations** that **disturb** accelerometer measurements. When the accelerometer detects additional forces (not just gravity), orientation estimation becomes incorrect.

#### Why Lower Gain?

**With lower gain (e.g., 0.1-0.3):**
- **More trust in gyroscope** = less influenced by false accelerations
- **Smoother response** to disturbances
- **Avoids false corrections** caused by motion accelerations

**With higher gain (e.g., 0.7-1.0):**
- **Aggressive correction** based on accelerometer
- **Oscillations** when accelerations are high
- **Instability** in dynamic movements

### Built-in Protection Mechanisms

The Fusion library has **acceleration rejection** integrated:

```cpp
// From documentation
accelerationRejection = 90.0f  // degrees - threshold for rejection
```

**How it works:**
1. **Compares** accelerometer orientation with algorithm output
2. **If difference > 90°** → **ignores accelerometer** for that update
3. **Functions as dynamic gain** that decreases with accelerations

## Practical Recommendations

### For Different Motion Types

```cpp
// Slow/moderate motion (normal vehicles)
double ahrs_gain = 0.5;  // default

// Abrupt motion (off-road, extreme sports, aggressive UAV)
double ahrs_gain = 0.2;  // more trust in gyroscope

// Extreme motion (crash testing, impacts)
double ahrs_gain = 0.1;  // almost gyroscope-only
```

### HDMapping Specific Configuration

For **dynamic environments** common in lidar odometry (off-road vehicles, UAVs, mobile robots):

```toml
[madgwick_filter]
ahrs_gain = 0.3  # Reduced from 0.5 for more dynamic motion
accelerationRejection = 45.0  # More strict than default (90°)
rejectionTimeout = 1000       # 5 seconds at 200Hz
```

## Trade-offs

### Lower Gain

- ✅ **Robust** against false accelerations
- ✅ **Smooth** in dynamic movements
- ❌ **Long-term drift** (gyroscope drifts)
- ❌ **Slower convergence** at startup

### Optimal Solution

**Adaptive gain** or per-application tuning:

```cpp
// Pseudo-code for adaptive gain
if (acceleration_magnitude > THRESHOLD_HIGH) {
    current_gain = 0.1;  // abrupt motion
} else if (acceleration_magnitude > THRESHOLD_MEDIUM) {
    current_gain = 0.3;  // moderate motion
} else {
    current_gain = 0.5;  // smooth motion
}
```

## Initialization Process

The algorithm uses an intelligent **gain ramp**:
- **Initially**: `gain = 10.0` (very aggressive for fast convergence)
- **Gradually decreases** over 3 seconds to the set value (`ahrs_gain`)
- **Purpose**: Fast convergence from arbitrary orientation to real orientation

## Impact on Lidar Odometry

**Precise orientation from AHRS** is essential for:
- **Sensor motion compensation** between consecutive frames
- **Initial prediction** for registration algorithms (ICP/NDT)
- **Motion estimation stabilization** in high-acceleration scenarios
- **Fusion with lidar odometry** for robust trajectory

## Summary

For **dynamic motion scenarios**, use **lower `ahrs_gain`** (0.1-0.3 instead of 0.5) to:
- **Avoid false corrections** from motion accelerations
- **Maintain orientation stability**
- **Reduce oscillations** in pose estimation

But **monitor long-term drift** and consider implementing **periodic recalibration** or **fusion with other sources** (GPS, visual odometry).

---

*Generated for HDMapping v0.85.0 - Lidar Odometry Documentation*
