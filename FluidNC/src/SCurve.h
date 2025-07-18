// Copyright (c) 2024 - FluidNC Contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
  SCurve.h - S-curve acceleration profile utilities
  
  Provides functions for computing S-curve (jerk-limited) acceleration profiles
  as an alternative to trapezoidal acceleration profiles.
*/

#include <cmath>

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

// S-curve profile data structure
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
    
    // Valid profile flag
    bool valid;
};

// Calculate S-curve profile for a move
SCurveProfile calculate_s_curve_profile(float distance, 
                                      float entry_speed, 
                                      float exit_speed,
                                      float max_velocity,
                                      float max_acceleration,
                                      float max_jerk);

// Get acceleration at time t within an S-curve profile
float s_curve_acceleration_at_time(const SCurveProfile& profile, float time);

// Get velocity at time t within an S-curve profile  
float s_curve_velocity_at_time(const SCurveProfile& profile, float time, float entry_speed);

// Get position at time t within an S-curve profile
float s_curve_position_at_time(const SCurveProfile& profile, float time, float entry_speed);

// Check if S-curve acceleration is beneficial for this move
bool should_use_s_curve(float distance, float max_jerk, float max_acceleration);