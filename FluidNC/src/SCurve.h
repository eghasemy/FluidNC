// Copyright (c) 2024 - FluidNC Contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
  SCurve.h - S-curve acceleration profile utilities
  
  Provides functions for computing S-curve (jerk-limited) acceleration profiles
  as an alternative to trapezoidal acceleration profiles.
*/

#include <cmath>
#include <cstdint>

// S-curve profile phase identifiers
enum SCurvePhase {
    SC_ACCEL_JERK_UP = 0,    // Phase 1: Acceleration ramp up (jerk limited)
    SC_ACCEL_CONST = 1,      // Phase 2: Constant acceleration
    SC_ACCEL_JERK_DOWN = 2,  // Phase 3: Acceleration ramp down (jerk limited)
    SC_CRUISE = 3,           // Phase 4: Constant velocity (cruise)
    SC_DECEL_JERK_UP = 4,    // Phase 5: Deceleration ramp up (jerk limited)
    SC_DECEL_CONST = 5,      // Phase 6: Constant deceleration
    SC_DECEL_JERK_DOWN = 6   // Phase 7: Deceleration ramp down (jerk limited)
};

// S-curve profile data structure - optimized for real-time performance
struct SCurveProfile {
    float total_distance;       // Total distance of the move
    float max_velocity;         // Maximum velocity reached
    float max_acceleration;     // Maximum acceleration used
    float max_jerk;            // Maximum jerk used
    
    // Phase durations (in seconds)
    float T[7];                // Duration of each phase
    
    // Phase distances (in mm)
    float S[7];                // Distance covered in each phase
    
    // Phase end velocities (in mm/min)
    float V[7];                // Velocity at end of each phase
    
    // Precomputed values for real-time execution
    float total_time;          // Total profile execution time
    float cruise_velocity;     // Velocity during cruise phase
    float accel_time;         // Total acceleration time
    float decel_time;         // Total deceleration time
    
    // Profile type flags for optimization
    uint8_t profile_type;      // Profile case: 0=full, 1=no_cruise, 2=triangular, 3=reduced
    
    // Valid profile flag
    bool valid;
};

// Profile type constants for optimization
#define SC_PROFILE_FULL      0    // Full 7-phase profile with cruise
#define SC_PROFILE_NO_CRUISE 1    // 6-phase profile without cruise
#define SC_PROFILE_TRIANGULAR 2   // 4-phase triangular profile
#define SC_PROFILE_REDUCED   3    // Reduced acceleration/deceleration

// Calculate S-curve profile for a move - optimized version
SCurveProfile calculate_s_curve_profile(float distance, 
                                      float entry_speed, 
                                      float exit_speed,
                                      float max_velocity,
                                      float max_acceleration,
                                      float max_jerk);

// Fast S-curve profile calculation for specific cases
SCurveProfile calculate_s_curve_fast(float distance, 
                                   float entry_speed, 
                                   float exit_speed,
                                   float max_velocity,
                                   float max_acceleration,
                                   float max_jerk);

// Junction velocity calculation for S-curve planning
float calculate_s_curve_junction_velocity(float distance1, float distance2,
                                         float max_acceleration,
                                         float max_jerk,
                                         float angle_factor);

// Validate S-curve configuration parameters
bool validate_s_curve_config(float max_jerk, float max_acceleration, 
                           float max_velocity, const char** error_msg);

// Get acceleration at time t within an S-curve profile
float s_curve_acceleration_at_time(const SCurveProfile& profile, float time);

// Get velocity at time t within an S-curve profile  
float s_curve_velocity_at_time(const SCurveProfile& profile, float time, float entry_speed);

// Get position at time t within an S-curve profile
float s_curve_position_at_time(const SCurveProfile& profile, float time, float entry_speed);

// Check if S-curve acceleration is beneficial for this move
bool should_use_s_curve(float distance, float max_jerk, float max_acceleration);