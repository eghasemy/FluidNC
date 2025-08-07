// Copyright (c) 2024 - FluidNC Contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
  SCurve.cpp - S-curve acceleration profile utilities
*/

#include "SCurve.h"
#include <algorithm>

// Helper function to solve quadratic equation ax^2 + bx + c = 0
static bool solve_quadratic(float a, float b, float c, float& x1, float& x2) {
    if (fabs(a) < 1e-10) {
        // Linear equation bx + c = 0
        if (fabs(b) < 1e-10) {
            return false; // No solution
        }
        x1 = x2 = -c / b;
        return true;
    }
    
    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        return false; // No real solutions
    }
    
    float sqrt_d = sqrtf(discriminant);
    x1 = (-b + sqrt_d) / (2 * a);
    x2 = (-b - sqrt_d) / (2 * a);
    return true;
}

SCurveProfile calculate_s_curve_profile(float distance, 
                                      float entry_speed, 
                                      float exit_speed,
                                      float max_velocity,
                                      float max_acceleration,
                                      float max_jerk) {
    SCurveProfile profile;
    profile.valid = false;
    profile.profile_type = SC_PROFILE_FULL;
    profile.total_time = 0.0f;
    profile.cruise_velocity = 0.0f;
    profile.accel_time = 0.0f;
    profile.decel_time = 0.0f;
    
    // Convert speeds from mm/min to mm/sec for calculations
    float v_entry = entry_speed / 60.0f;
    float v_exit = exit_speed / 60.0f;
    float v_max = max_velocity / 60.0f;
    
    // Initialize all phases to zero
    for (int i = 0; i < 7; i++) {
        profile.T[i] = 0.0f;
        profile.S[i] = 0.0f;
        profile.V[i] = 0.0f;
    }
    
    // Basic validation
    if (max_jerk <= 0 || max_acceleration <= 0 || distance <= 0) {
        return profile;
    }
    
    // Calculate optimal jerk time and acceleration time
    float T_j = max_acceleration / max_jerk; // Time to reach max acceleration
    float v_accel_max = max_acceleration * T_j; // Max velocity increase during jerk phase
    
    // Determine reachable velocity considering jerk constraints
    float v_reachable = v_entry + max_acceleration * T_j;
    if (v_reachable > v_max) {
        v_reachable = v_max;
        T_j = (v_max - v_entry) / max_acceleration;
    }
    
    // Check if we can decelerate to exit velocity
    float v_decel_start = v_exit + max_acceleration * T_j;
    if (v_reachable > v_decel_start) {
        // Need to find meeting point
        v_reachable = (v_entry + v_exit + max_acceleration * 2 * T_j) / 2.0f;
        if (v_reachable > v_max) {
            v_reachable = v_max;
        }
    }
    
    // Calculate acceleration phase
    float dv_accel = v_reachable - v_entry;
    float T_accel_total = dv_accel / max_acceleration + T_j;
    
    // Calculate deceleration phase  
    float dv_decel = v_reachable - v_exit;
    float T_decel_total = dv_decel / max_acceleration + T_j;
    
    // Distance calculations for acceleration phase
    float S_accel_jerk = v_entry * T_j + 0.5f * max_acceleration * T_j * T_j;
    float T_accel_const = T_accel_total - 2 * T_j;
    float S_accel_const = 0;
    if (T_accel_const > 0) {
        float v_mid_accel = v_entry + max_acceleration * T_j;
        S_accel_const = v_mid_accel * T_accel_const + 0.5f * max_acceleration * T_accel_const * T_accel_const;
    }
    float S_accel_jerk_down = S_accel_jerk; // Symmetric
    float S_accel_total = S_accel_jerk + S_accel_const + S_accel_jerk_down;
    
    // Distance calculations for deceleration phase (symmetric)
    float S_decel_total = S_accel_total; // For now, assume symmetric
    
    // Cruise phase
    float S_cruise = distance - S_accel_total - S_decel_total;
    float T_cruise = 0;
    if (S_cruise > 0) {
        T_cruise = S_cruise / v_reachable;
        profile.profile_type = SC_PROFILE_FULL;
    } else if (S_cruise > -0.1f) {
        // No cruise phase
        S_cruise = 0;
        profile.profile_type = SC_PROFILE_NO_CRUISE;
    } else {
        // Not enough distance for full acceleration - use triangular profile
        profile.profile_type = SC_PROFILE_TRIANGULAR;
        
        // Recalculate for triangular profile
        float total_dv = distance * 2 * max_acceleration / (2 * v_entry + max_acceleration * 2 * T_j);
        v_reachable = v_entry + total_dv / 2;
        T_accel_total = total_dv / (2 * max_acceleration);
        T_decel_total = T_accel_total;
        S_accel_total = distance / 2;
        S_decel_total = distance / 2;
        S_cruise = 0;
        T_cruise = 0;
    }
    
    // Populate profile phases
    // Phase 1: Acceleration jerk up
    profile.T[0] = T_j;
    profile.S[0] = v_entry * T_j + (1.0f/6.0f) * max_jerk * T_j * T_j * T_j;
    profile.V[0] = (v_entry + 0.5f * max_acceleration * T_j) * 60.0f;
    
    // Phase 2: Constant acceleration
    profile.T[1] = fmaxf(0, T_accel_total - 2 * T_j);
    if (profile.T[1] > 0) {
        float v_start = v_entry + max_acceleration * T_j;
        profile.S[1] = v_start * profile.T[1] + 0.5f * max_acceleration * profile.T[1] * profile.T[1];
        profile.V[1] = (v_start + max_acceleration * profile.T[1]) * 60.0f;
    } else {
        profile.S[1] = 0;
        profile.V[1] = profile.V[0];
    }
    
    // Phase 3: Acceleration jerk down
    profile.T[2] = T_j;
    profile.S[2] = S_accel_total - profile.S[0] - profile.S[1];
    profile.V[2] = v_reachable * 60.0f;
    
    // Phase 4: Cruise
    profile.T[3] = T_cruise;
    profile.S[3] = S_cruise;
    profile.V[3] = v_reachable * 60.0f;
    
    // Phases 5-7: Symmetric deceleration
    profile.T[4] = T_j;  // Decel jerk up
    profile.T[5] = profile.T[1];  // Constant decel
    profile.T[6] = T_j;  // Decel jerk down
    
    profile.S[4] = profile.S[2];  // Symmetric
    profile.S[5] = profile.S[1];  // Symmetric  
    profile.S[6] = profile.S[0];  // Symmetric
    
    profile.V[4] = profile.V[2];
    profile.V[5] = profile.V[1];
    profile.V[6] = v_exit * 60.0f;
    
    // Precompute values for real-time execution
    profile.total_time = 0;
    for (int i = 0; i < 7; i++) {
        profile.total_time += profile.T[i];
    }
    profile.cruise_velocity = v_reachable * 60.0f;
    profile.accel_time = profile.T[0] + profile.T[1] + profile.T[2];
    profile.decel_time = profile.T[4] + profile.T[5] + profile.T[6];
    
    // Validate total distance
    float total_calc = 0.0f;
    for (int i = 0; i < 7; i++) {
        total_calc += profile.S[i];
    }
    
    if (fabsf(total_calc - distance) > 0.1f) {
        // Distance mismatch - mark as invalid
        profile.valid = false;
        return profile;
    }
    
    profile.total_distance = distance;
    profile.max_velocity = max_velocity;
    profile.max_acceleration = max_acceleration;
    profile.max_jerk = max_jerk;
    profile.valid = true;
    
    return profile;
}

SCurveProfile calculate_s_curve_fast(float distance, 
                                   float entry_speed, 
                                   float exit_speed,
                                   float max_velocity,
                                   float max_acceleration,
                                   float max_jerk) {
    // Fast calculation for common cases - optimized for real-time performance
    SCurveProfile profile;
    profile.valid = false;
    profile.profile_type = SC_PROFILE_REDUCED;
    profile.total_time = 0.0f;
    profile.cruise_velocity = 0.0f;
    profile.accel_time = 0.0f;
    profile.decel_time = 0.0f;
    
    // Quick validation
    if (max_jerk <= 0 || max_acceleration <= 0 || distance <= 0) {
        return profile;
    }
    
    // For small distances or when entry/exit speeds are close, use simplified calculation
    float speed_diff = fabsf(entry_speed - exit_speed);
    if (distance < 10.0f || speed_diff < 50.0f) {
        // Use simplified trapezoidal-like profile
        profile.profile_type = SC_PROFILE_REDUCED;
        
        // Single acceleration/deceleration phase
        float avg_speed = (entry_speed + exit_speed) / 2.0f;
        float total_time = distance / (avg_speed / 60.0f);
        
        // Distribute time across phases
        profile.T[0] = total_time * 0.15f; // 15% jerk up
        profile.T[1] = total_time * 0.20f; // 20% constant accel
        profile.T[2] = total_time * 0.15f; // 15% jerk down
        profile.T[3] = total_time * 0.30f; // 30% cruise
        profile.T[4] = total_time * 0.15f; // 15% decel phases
        profile.T[5] = total_time * 0.05f;
        profile.T[6] = total_time * 0.00f;
        
        // Simple distance distribution
        for (int i = 0; i < 7; i++) {
            profile.S[i] = distance * profile.T[i] / total_time;
        }
        
        profile.valid = true;
        profile.total_distance = distance;
        profile.total_time = total_time;
        profile.cruise_velocity = avg_speed;
        
        return profile;
    }
    
    // For other cases, fall back to full calculation
    return calculate_s_curve_profile(distance, entry_speed, exit_speed, 
                                   max_velocity, max_acceleration, max_jerk);
}

float calculate_s_curve_junction_velocity(float distance1, float distance2,
                                         float max_acceleration,
                                         float max_jerk,
                                         float angle_factor) {
    // Calculate optimal junction velocity considering S-curve constraints
    if (max_jerk <= 0 || max_acceleration <= 0) {
        return 0.0f; // Fall back to traditional planning
    }
    
    // Time to reach max acceleration with jerk limit
    float T_j = max_acceleration / max_jerk;
    
    // Minimum distance needed for S-curve acceleration/deceleration
    float min_distance = max_acceleration * T_j * T_j; // Simplified
    
    // Use shorter distance to determine limits
    float min_move_distance = fminf(distance1, distance2);
    
    if (min_move_distance < min_distance * 2) {
        // Not enough distance for full S-curve, reduce junction velocity
        float velocity_limit = sqrtf(min_move_distance * max_acceleration * angle_factor);
        return velocity_limit * 60.0f; // Convert to mm/min
    }
    
    // Calculate based on jerk limitations and angle
    float jerk_limited_velocity = sqrtf(max_acceleration * max_acceleration / max_jerk * angle_factor);
    
    return jerk_limited_velocity * 60.0f; // Convert to mm/min
}

bool validate_s_curve_config(float max_jerk, float max_acceleration, 
                           float max_velocity, const char** error_msg) {
    static const char* no_error = nullptr;
    *error_msg = no_error;
    
    // Check for valid jerk value
    if (max_jerk < 0) {
        *error_msg = "max_jerk_mm_per_sec3 cannot be negative";
        return false;
    }
    
    if (max_jerk == 0) {
        // S-curve disabled, this is valid
        return true;
    }
    
    // Check relationship between jerk and acceleration
    if (max_jerk < max_acceleration / 10.0f) {
        *error_msg = "max_jerk_mm_per_sec3 too small compared to acceleration (min 1/10 of acceleration)";
        return false;
    }
    
    if (max_jerk > max_acceleration * 100.0f) {
        *error_msg = "max_jerk_mm_per_sec3 too large compared to acceleration (max 100x acceleration)";
        return false;
    }
    
    // Check time to reach acceleration
    float time_to_accel = max_acceleration / max_jerk;
    if (time_to_accel > 1.0f) {
        *error_msg = "max_jerk_mm_per_sec3 too small - would take > 1 second to reach max acceleration";
        return false;
    }
    
    if (time_to_accel < 0.001f) {
        *error_msg = "max_jerk_mm_per_sec3 too large - acceleration ramp time < 1ms";
        return false;
    }
    
    return true;
}

float s_curve_acceleration_at_time(const SCurveProfile& profile, float time) {
    if (!profile.valid) return 0.0f;
    
    float t = 0;
    for (int phase = 0; phase < 7; phase++) {
        if (time <= t + profile.T[phase]) {
            float phase_time = time - t;
            
            switch (phase) {
                case SC_ACCEL_JERK_UP:
                    return profile.max_jerk * phase_time;
                    
                case SC_ACCEL_CONST:
                    return profile.max_acceleration;
                    
                case SC_ACCEL_JERK_DOWN:
                    return profile.max_acceleration - profile.max_jerk * phase_time;
                    
                case SC_CRUISE:
                    return 0.0f;
                    
                case SC_DECEL_JERK_UP:
                    return -profile.max_jerk * phase_time;
                    
                case SC_DECEL_CONST:
                    return -profile.max_acceleration;
                    
                case SC_DECEL_JERK_DOWN:
                    return -profile.max_acceleration + profile.max_jerk * phase_time;
            }
        }
        t += profile.T[phase];
    }
    
    return 0.0f; // End of profile
}

float s_curve_velocity_at_time(const SCurveProfile& profile, float time, float entry_speed) {
    if (!profile.valid) return entry_speed;
    
    float velocity = entry_speed / 60.0f; // Convert to mm/sec
    float t = 0;
    
    for (int phase = 0; phase < 7; phase++) {
        float phase_duration = profile.T[phase];
        
        if (time <= t + phase_duration) {
            float phase_time = time - t;
            float acceleration = s_curve_acceleration_at_time(profile, time);
            
            switch (phase) {
                case SC_ACCEL_JERK_UP:
                    velocity += 0.5f * profile.max_jerk * phase_time * phase_time;
                    break;
                    
                case SC_ACCEL_CONST:
                    velocity += profile.max_acceleration * phase_time;
                    break;
                    
                case SC_ACCEL_JERK_DOWN:
                    velocity += profile.max_acceleration * phase_time - 0.5f * profile.max_jerk * phase_time * phase_time;
                    break;
                    
                case SC_CRUISE:
                    // Velocity remains constant
                    break;
                    
                case SC_DECEL_JERK_UP:
                    velocity -= 0.5f * profile.max_jerk * phase_time * phase_time;
                    break;
                    
                case SC_DECEL_CONST:
                    velocity -= profile.max_acceleration * phase_time;
                    break;
                    
                case SC_DECEL_JERK_DOWN:
                    velocity -= profile.max_acceleration * phase_time - 0.5f * profile.max_jerk * phase_time * phase_time;
                    break;
            }
            break;
        }
        
        // Add full phase contribution and move to next phase
        t += phase_duration;
        if (phase_duration > 0) {
            velocity = profile.V[phase] / 60.0f; // Use precomputed end velocity
        }
    }
    
    return velocity * 60.0f; // Convert back to mm/min
}

float s_curve_position_at_time(const SCurveProfile& profile, float time, float entry_speed) {
    if (!profile.valid) return 0.0f;
    
    float position = 0.0f;
    float t = 0;
    
    for (int phase = 0; phase < 7; phase++) {
        float phase_duration = profile.T[phase];
        
        if (time <= t + phase_duration) {
            float phase_time = time - t;
            // Add position contribution from this partial phase
            // This is a simplified calculation - full implementation would integrate properly
            float avg_velocity = s_curve_velocity_at_time(profile, t + phase_time/2, entry_speed) / 60.0f;
            position += avg_velocity * phase_time;
            break;
        }
        
        // Add full phase contribution
        position += profile.S[phase];
        t += phase_duration;
    }
    
    return position;
}

bool should_use_s_curve(float distance, float max_jerk, float max_acceleration) {
    // Enhanced logic for determining when S-curve is beneficial
    if (max_jerk <= 0.0f || max_acceleration <= 0.0f) {
        return false;
    }
    
    // Time to reach max acceleration with jerk limit
    float T_j = max_acceleration / max_jerk;
    
    // Minimum distance where S-curve provides benefit
    float min_beneficial_distance = max_acceleration * T_j * T_j * 4.0f; // 4x the jerk phase distance
    
    // Don't use S-curve for very short moves
    if (distance < min_beneficial_distance) {
        return false;
    }
    
    // Don't use S-curve if jerk time is too small (would not be smooth)
    if (T_j < 0.005f) { // Less than 5ms
        return false;
    }
    
    // Don't use S-curve if jerk time is too large (would be inefficient)
    if (T_j > 0.5f) { // More than 500ms
        return false;
    }
    
    return true;
}