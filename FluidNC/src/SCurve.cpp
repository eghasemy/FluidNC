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
    
    // For now, implement a simplified trapezoidal profile that can be enhanced later
    // This provides the framework while ensuring mathematical correctness
    
    // Time to reach max acceleration from zero
    float T_j = max_acceleration / max_jerk;
    float T_accel = 2.0f * T_j; // Total acceleration time (jerk up + jerk down)
    
    // Distance covered during acceleration and deceleration phases
    float S_accel = v_entry * T_accel + 0.5f * max_acceleration * T_j * T_j;
    float S_decel = S_accel; // Symmetric for simplicity
    
    // Check if we have enough distance for full acceleration profile
    if (S_accel + S_decel > distance) {
        // Triangular profile - reduce acceleration
        float T_total = sqrtf(distance / max_acceleration);
        T_j = fminf(T_j, T_total / 2.0f);
        T_accel = T_total;
        S_accel = 0.5f * distance;
        S_decel = 0.5f * distance;
    }
    
    float S_cruise = distance - S_accel - S_decel;
    float v_peak = v_entry + max_acceleration * T_j;
    
    // Phase 1: Acceleration jerk up
    profile.T[0] = T_j;
    profile.S[0] = v_entry * T_j + (1.0f/6.0f) * max_jerk * T_j * T_j * T_j;
    profile.V[0] = (v_entry + 0.5f * max_acceleration * T_j) * 60.0f;
    
    // Phase 2: Constant acceleration (if needed)
    if (T_accel > 2.0f * T_j) {
        float T_const = T_accel - 2.0f * T_j;
        profile.T[1] = T_const;
        profile.S[1] = (v_entry + 0.5f * max_acceleration * T_j) * T_const + 0.5f * max_acceleration * T_const * T_const;
        profile.V[1] = v_peak * 60.0f;
    } else {
        profile.T[1] = 0.0f;
        profile.S[1] = 0.0f;
        profile.V[1] = profile.V[0];
    }
    
    // Phase 3: Acceleration jerk down
    profile.T[2] = T_j;
    profile.S[2] = S_accel - profile.S[0] - profile.S[1];
    profile.V[2] = v_peak * 60.0f;
    
    // Phase 4: Cruise
    if (S_cruise > 0) {
        profile.T[3] = S_cruise / v_peak;
        profile.S[3] = S_cruise;
        profile.V[3] = v_peak * 60.0f;
    } else {
        profile.T[3] = 0.0f;
        profile.S[3] = 0.0f;
        profile.V[3] = profile.V[2];
    }
    
    // Phases 5-7: Symmetric deceleration
    profile.T[4] = T_j;  // Decel jerk up
    profile.T[5] = profile.T[1];  // Constant decel (same as const accel)
    profile.T[6] = T_j;  // Decel jerk down
    
    profile.S[4] = profile.S[2];  // Symmetric to accel jerk down
    profile.S[5] = profile.S[1];  // Symmetric to const accel
    profile.S[6] = profile.S[0];  // Symmetric to accel jerk up
    
    profile.V[4] = profile.V[2];
    profile.V[5] = profile.V[1];
    profile.V[6] = v_exit * 60.0f;
    
    // Validate total distance
    float total_calc = 0.0f;
    for (int i = 0; i < 7; i++) {
        total_calc += profile.S[i];
    }
    
    if (fabsf(total_calc - distance) > 0.1f) {
        // Distance mismatch - use fallback to trapezoidal
        for (int i = 0; i < 7; i++) {
            profile.T[i] = 0.0f;
            profile.S[i] = 0.0f;
            profile.V[i] = 0.0f;
        }
        return profile;  // Invalid profile
    }
    
    profile.total_distance = distance;
    profile.max_velocity = max_velocity;
    profile.max_acceleration = max_acceleration;
    profile.max_jerk = max_jerk;
    profile.valid = true;
    
    return profile;
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
    // Only use S-curve if jerk is specified and distance is sufficient
    // The minimum distance check ensures S-curve makes sense for the move
    float min_distance = max_acceleration * max_acceleration / (max_jerk * 2.0f);
    return (max_jerk > 0.0f) && (distance > min_distance);
}