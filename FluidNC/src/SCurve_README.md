# S-Curve Acceleration Implementation in FluidNC

## Overview
This implementation adds S-curve (jerk-limited) acceleration support to FluidNC as an alternative to traditional trapezoidal acceleration profiles. S-curve acceleration provides smoother motion by limiting the rate of acceleration change (jerk).

## What Was Implemented

### 1. Configuration Extension
- Added `max_jerk_mm_per_sec3` parameter to axis configuration
- When set to 0.0, S-curve is disabled (traditional trapezoidal acceleration)
- When > 0.0, S-curve acceleration is enabled

### 2. Core Data Structures
- Extended `plan_block_t` with S-curve profile data:
  - `use_s_curve`: Flag indicating if block uses S-curve
  - `s_curve_phases[7]`: Duration of each S-curve phase
  - `s_curve_distances[7]`: Distance covered in each phase

### 3. S-Curve Profile Calculation
- Created `SCurve.h/cpp` with profile computation functions
- Implements 7-phase S-curve profile:
  1. Acceleration jerk-up
  2. Constant acceleration  
  3. Acceleration jerk-down
  4. Constant velocity (cruise)
  5. Deceleration jerk-up
  6. Constant deceleration
  7. Deceleration jerk-down

### 4. Planner Integration
- Added `limit_jerk_by_axis_maximum()` function
- S-curve profile calculation in `plan_buffer_line()`
- Automatic fallback to trapezoidal if S-curve calculation fails

### 5. Stepper Execution
- Added new ramp states for S-curve phases
- Extended stepper preparation structure with S-curve data
- Implemented S-curve ramp state handling in segment preparation

## Usage Example

```yaml
axes:
  x:
    steps_per_mm: 80.000
    max_rate_mm_per_min: 5000.000
    acceleration_mm_per_sec2: 100.000
    max_jerk_mm_per_sec3: 1000.000  # Enable S-curve
    max_travel_mm: 300.000
```

## Benefits
- Reduced mechanical stress and vibration
- Smoother motion and better surface finish  
- Reduced wear on mechanical components
- Better path tracking at high speeds

## Implementation Status
- ✅ Configuration framework
- ✅ Data structures
- ✅ Basic profile calculation
- ✅ Planner integration
- ✅ Stepper framework
- ⚠️  Profile calculation needs mathematical refinement
- ❌ Real-time performance optimization needed
- ❌ Extensive testing required

## Future Improvements
1. Optimize S-curve calculations for real-time performance
2. Add more sophisticated profile case handling
3. Implement S-curve-aware junction velocity planning
4. Add configuration validation and warnings
5. Performance benchmarking and tuning