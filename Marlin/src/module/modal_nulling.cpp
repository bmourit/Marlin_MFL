/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(MODAL_NULLING)

#include "modal_nulling.h"
#include "../module/planner.h"    // Access junction deviation

ModalNulling modalNulling;
mn_config_t ModalNulling::cfg;
motion_segment_t ModalNulling::current_segment;

void ModalNulling::init() {
  reset();
}

void ModalNulling::reset() {
  current_segment.valid = false;
  ZERO(current_segment.kernels);
}

void ModalNulling::set_defaults() {
  cfg.enabled = MODAL_NULLING_DEFAULT_ENABLED;
  cfg.freq[X_AXIS] = MODAL_NULLING_DEFAULT_FREQ_X;
  cfg.freq[Y_AXIS] = MODAL_NULLING_DEFAULT_FREQ_Y;
  cfg.damping[X_AXIS] = MODAL_NULLING_DEFAULT_DAMPING_X;
  cfg.damping[Y_AXIS] = MODAL_NULLING_DEFAULT_DAMPING_Y;
  cfg.beta[X_AXIS] = MODAL_NULLING_DEFAULT_BETA_X;
  cfg.beta[Y_AXIS] = MODAL_NULLING_DEFAULT_BETA_Y;
}

void ModalNulling::analyze_motion_segment(const float start_speed, const float peak_speed, const float end_speed,
                                          const float acceleration, const float deceleration, const float segment_time,
                                          const float accel_time, const float coast_time, const float decel_time,
                                          const bool has_x_motion, const bool has_y_motion) {
  #if ENABLED(DEBUG_MODAL_NULLING)
    SERIAL_ECHOLNPGM("=== Modal Nulling Analysis ===");
    SERIAL_ECHOLNPGM("Segment: ", start_speed, " -> ", peak_speed, " -> ", end_speed);
    SERIAL_ECHOLNPGM("Times: accel=", accel_time, " coast=", coast_time, " decel=", decel_time);
    SERIAL_ECHOLNPGM("Acceleration: ", acceleration);
    SERIAL_ECHOLNPGM("X motion: ", has_x_motion ? "Yes" : "No");
    SERIAL_ECHOLNPGM("Y motion: ", has_y_motion ? "Yes" : "No");
  #endif

  if (!cfg.enabled || !should_apply_nulling(has_x_motion, has_y_motion, segment_time, peak_speed)) {
    current_segment.valid = false;
    return;
  }

  // Store segment parameters
  current_segment = {
    .has_motion = { has_x_motion, has_y_motion },
    .segment_time = segment_time,
    .accel_time = accel_time,
    .coast_time = coast_time,
    .decel_time = decel_time,
    .start_speed = start_speed,
    .peak_speed = peak_speed,
    .end_speed = end_speed,
    .acceleration = acceleration,
    .deceleration = deceleration,
    .valid = true
  };

  // Compute nulling kernels for each axis
  for (uint8_t axis = 0; axis < XY; ++axis) {
    const float freq = cfg.freq[axis];
    if (current_segment.has_motion[axis] && freq > 0.0f) {
      #if ANY(S_CURVE_ACCELERATION, HAS_JUNCTION_DEVIATION)
        const float modal_proj = compute_enhanced_modal_projection(freq, start_speed, peak_speed, end_speed,
                                                        acceleration, deceleration, accel_time,
                                                        coast_time, decel_time);
      #else
        // Compute modal projection for this axis
        const float modal_proj = compute_modal_projection(freq, start_speed, peak_speed, end_speed,
                                                          acceleration, deceleration, accel_time,
                                                          coast_time, decel_time);
      #endif

      #if ENABLED(DEBUG_MODAL_NULLING)
        SERIAL_ECHOLNPGM("Axis ", axis, " modal projection: ", modal_proj);
        SERIAL_ECHOLNPGM("Frequency: ", cfg.freq[axis], " Hz");
      #endif

      // Compute exact nulling kernel
      current_segment.kernels[axis] = compute_nulling_kernel(freq, cfg.damping[axis], 
                                                             cfg.beta[axis], modal_proj, segment_time);

      #if ENABLED(DEBUG_MODAL_NULLING)
        debug_print_kernel(current_segment.kernels[axis], axis == X_AXIS ? "X" : "Y");
        SERIAL_ECHOLNPGM("Modal projection ", axis == X_AXIS ? "X" : "Y", ": ", modal_proj);

        // Verify orthogonality
        if (current_segment.kernels[axis].valid) {
          const float residual = compute_residual_projection(current_segment.kernels[axis], modal_proj, segment_time);
          SERIAL_ECHOLNPGM("Residual projection: ", residual, " (", ABS(residual/modal_proj) * 100.0f, "% of original)");
        }
      #endif
    } else {
      current_segment.kernels[axis].valid = false;
    }
  }
}

void ModalNulling::apply_trajectory_correction(const uint32_t trajectory_idx, const uint32_t batch_idx,
                                               const float segment_time, xyze_trajectory_t& trajectory) {
  if (!cfg.enabled || !current_segment.valid) return;

  // Calculate time within the current segment
  const float t = (trajectory_idx * FTM_TS);
  if (t > current_segment.segment_time) return;

  #if ENABLED(DEBUG_MODAL_NULLING)
    static uint32_t correction_count = 0;
    static float max_correction_x = 0.0f;
    static float max_correction_y = 0.0f;
    static float total_correction_x = 0.0f;
    static float total_correction_y = 0.0f;
  #endif

  // Apply kernels to each axis
  for (uint8_t axis = 0; axis < XY; ++axis) {
    if (current_segment.kernels[axis].valid) {

      #if ENABLED(DEBUG_MODAL_NULLING)
        const float original_pos = (axis == X_AXIS) ? trajectory.x[batch_idx] : trajectory.y[batch_idx];
      #endif

      apply_kernel_to_trajectory_point(current_segment.kernels[axis], t,
                                      (AxisEnum)axis, batch_idx, trajectory);

      #if ENABLED(DEBUG_MODAL_NULLING)
        const float new_pos = (axis == X_AXIS) ? trajectory.x[batch_idx] : trajectory.y[batch_idx];
        const float correction = new_pos - original_pos;

        if (axis == X_AXIS) {
          total_correction_x += ABS(correction);
          NOLESS(max_correction_x, ABS(correction));
        } else {
          total_correction_y += ABS(correction);
          NOLESS(max_correction_y, ABS(correction));
        }

        correction_count++;

        // Print statistics every 500 corrections
        if (correction_count % 500 == 0) {
          SERIAL_ECHOLNPGM("=== Modal Nulling Statistics ===");
          SERIAL_ECHOLNPGM("Corrections applied: ", correction_count);
          SERIAL_ECHOLNPGM("Max X correction (um): ", max_correction_x * 1000000.0f);
          SERIAL_ECHOLNPGM("Max Y correction (um): ", max_correction_y * 1000000.0f);
          SERIAL_ECHOLNPGM("Avg X correction (um): ", (total_correction_x / correction_count) * 1000000.0f);
          SERIAL_ECHOLNPGM("Avg Y correction (um): ", (total_correction_y / correction_count) * 1000000.0f);
        }
      #endif
    }
  }
}

bool ModalNulling::should_apply_nulling(const bool has_x_motion, const bool has_y_motion,
                                        const float segment_time, const float peak_speed) {
  // Must have motion on at least one axis
  if (!(has_x_motion || has_y_motion) || segment_time < 0.01f) return false;

  // Must have valid frequencies configured
  for (uint8_t axis = 0; axis < XY; ++axis) {
    const float f = cfg.freq[axis];
    if (f > 0.0f && f < 200.0f) return true;
  }

  return false;
}

float ModalNulling::compute_modal_projection(const float freq, const float start_speed, const float peak_speed,
                                             const float end_speed, const float acceleration, float deceleration,
                                             const float accel_time, const float coast_time, const float decel_time) {
  if (freq <= 0.0f) return 0.0f;

  float total_proj = 0.0f;

  // Phase projections
  if (accel_time > 0.0f) total_proj += compute_accel_phase_projection(freq, start_speed, acceleration, accel_time);
  if (coast_time > 0.0f) total_proj += compute_coast_phase_projection(freq, peak_speed, coast_time);
  if (decel_time > 0.0f) total_proj += compute_decel_phase_projection(freq, peak_speed, deceleration, decel_time);

  return total_proj;
}

#if ENABLED(S_CURVE_ACCELERATION) || HAS_JUNCTION_DEVIATION
  float ModalNulling::compute_enhanced_modal_projection(const float freq, const float start_speed, const float peak_speed,
                                                        const float end_speed, const float acceleration, const float deceleration,
                                                        const float accel_time, const float coast_time, const float decel_time) {
    if (freq <= 0.0f) return 0.0f;

    float total_proj = 0.0f;

    #if ENABLED(S_CURVE_ACCELERATION)
      // S-curve acceleration uses a 7-segment velocity profile:
      // 1. Jerk-limited acceleration ramp-up (cubic velocity profile)
      // 2. Constant acceleration phase (linear velocity profile) 
      // 3. Jerk-limited acceleration ramp-down (cubic velocity profile)
      // 4. Constant velocity coast
      // 5. Jerk-limited deceleration ramp-up (cubic velocity profile)
      // 6. Constant deceleration phase (linear velocity profile)
      // 7. Jerk-limited deceleration ramp-down (cubic velocity profile)

      if (accel_time > 0.0f) {
        total_proj += compute_s_curve_accel_projection(freq, start_speed, peak_speed, acceleration, accel_time);
      }
    #else
      // Standard trapezoidal acceleration
      if (accel_time > 0.0f) {
        total_proj += compute_accel_phase_projection(freq, start_speed, acceleration, accel_time);
      }
    #endif

    // Coast phase (same for both profiles)
    if (coast_time > 0.0f) {
      total_proj += compute_coast_phase_projection(freq, peak_speed, coast_time);
    }

    #if ENABLED(S_CURVE_ACCELERATION)
      if (decel_time > 0.0f) {
        total_proj += compute_s_curve_decel_projection(freq, peak_speed, end_speed, deceleration, decel_time);
      }
    #else
      // Standard trapezoidal deceleration
      if (decel_time > 0.0f) {
        total_proj += compute_decel_phase_projection(freq, peak_speed, deceleration, decel_time);
      }
    #endif

    #if HAS_JUNCTION_DEVIATION
      // Junction deviation modifies the velocity profile at direction changes
      // Apply the full junction deviation correction
      total_proj = apply_junction_deviation_correction(total_proj, freq, start_speed, peak_speed, end_speed,
                                                       accel_time, coast_time, decel_time);
    #endif

    return total_proj;
  }

  #if ENABLED(S_CURVE_ACCELERATION)

    float ModalNulling::compute_s_curve_accel_projection(const float freq, const float start_speed, const float peak_speed,
                                                         const float acceleration, const float accel_time) {
      if (freq <= 0.0f || accel_time <= 0.0f) return 0.0f;

      // S-curve parameters from Marlin's implementation
      // The S-curve is divided into three phases with specific time ratios
      const float speed_change = peak_speed - start_speed;
      const float avg_acceleration = speed_change / accel_time;

      // Calculate jerk value - this determines the curvature of the S-curve
      // In Marlin, jerk is calculated based on the maximum allowed jerk for the axis
      const float max_jerk = acceleration * 2.0f; // Simplified - actual value comes from configuration
      const float jerk_time = avg_acceleration / max_jerk;

      // Ensure jerk time doesn't exceed half the acceleration time
      const float actual_jerk_time = _MIN(jerk_time, accel_time * 0.5f);
      const float linear_accel_time = accel_time - 2.0f * actual_jerk_time;

      float total_projection = 0.0f;
      float current_time = 0.0f;
      float current_speed = start_speed;

      if (actual_jerk_time > 0.0f) {
        // Phase 1: Jerk-limited ramp-up (cubic velocity profile)
        // v(t) = v₀ + (j/6) * t³, where j = acceleration/jerk_time
        const float jerk = avg_acceleration / actual_jerk_time;
        total_projection += compute_cubic_velocity_projection(freq, current_speed, jerk / 6.0f, actual_jerk_time, current_time);

        current_time += actual_jerk_time;
        current_speed += 0.5f * avg_acceleration * actual_jerk_time;

        // Phase 2: Linear acceleration
        if (linear_accel_time > 0.0f) {
          total_projection += compute_linear_accel_projection(freq, current_speed, avg_acceleration, linear_accel_time, current_time);
          current_time += linear_accel_time;
          current_speed += avg_acceleration * linear_accel_time;
        }

        // Phase 3: Jerk-limited ramp-down (cubic velocity profile)
        // v(t) = v₁ + a₁*t - (j/6) * t³
        const float phase3_start_speed = current_speed;
        total_projection += compute_cubic_decel_projection(freq, phase3_start_speed, avg_acceleration, -jerk / 6.0f, actual_jerk_time, current_time);
      } else {
        // No jerk limiting - pure linear acceleration
        total_projection += compute_linear_accel_projection(freq, start_speed, avg_acceleration, accel_time, 0.0f);
      }

      return total_projection;
    }

    float ModalNulling::compute_s_curve_decel_projection(const float freq, const float peak_speed, const float end_speed,
                                                         const float deceleration, const float decel_time) {
      if (freq <= 0.0f || decel_time <= 0.0f) return 0.0f;

      // S-curve deceleration parameters
      const float speed_change = peak_speed - end_speed;
      const float avg_deceleration = speed_change / decel_time;

      const float max_jerk = deceleration * 2.0f;
      const float jerk_time = avg_deceleration / max_jerk;
      const float actual_jerk_time = _MIN(jerk_time, decel_time * 0.5f);
      const float linear_decel_time = decel_time - 2.0f * actual_jerk_time;

      float total_projection = 0.0f;
      float current_time = 0.0f;
      float current_speed = peak_speed;

      if (actual_jerk_time > 0.0f) {
        // Phase 1: Jerk-limited decel ramp-up
        const float jerk = avg_deceleration / actual_jerk_time;
        total_projection += compute_cubic_decel_projection(freq, current_speed, 0.0f, -jerk / 6.0f, actual_jerk_time, current_time);

        current_time += actual_jerk_time;
        current_speed -= 0.5f * avg_deceleration * actual_jerk_time;

        // Phase 2: Linear deceleration
        if (linear_decel_time > 0.0f) {
          total_projection += compute_linear_decel_projection(freq, current_speed, avg_deceleration, linear_decel_time, current_time);
          current_time += linear_decel_time;
          current_speed -= avg_deceleration * linear_decel_time;
        }

        // Phase 3: Jerk-limited decel ramp-down
        total_projection += compute_cubic_velocity_projection(freq, current_speed, jerk / 6.0f, actual_jerk_time, current_time);
      } else {
        // No jerk limiting - pure linear deceleration
        total_projection += compute_linear_decel_projection(freq, peak_speed, avg_deceleration, decel_time, 0.0f);
      }

      return total_projection;
    }

    float ModalNulling::compute_cubic_velocity_projection(const float freq, const float v0, const float cubic_coeff,
                                                          const float duration, const float time_offset) {
      // For velocity profile: v(t) = v₀ + c * t³
      // Modal projection: ∫₀ᵀ v(t) * sin(ω(t + offset)) dt

      if (freq <= 0.0f || duration <= 0.0f) return 0.0f;

      const float omega = 2.0f * M_PI * freq;
      const float cos_offset = COS(omega * time_offset);
      const float sin_offset = SIN(omega * time_offset);
      const float T = duration;
      const float omega2 = omega * omega;
      const float omega3 = omega2 * omega;
      const float omega4 = omega3 * omega;

      // Analytical integration of v₀ * sin(ω(t + offset))
      const float linear_term = (v0 / omega) * (cos_offset - COS(omega * (T + time_offset)));

      // Analytical integration of c * t³ * sin(ω(t + offset))
      // This requires integration by parts multiple times
      const float T2 = T * T;
      const float T3 = T2 * T;
      const float cosWT = COS(omega * T);
      const float sinWT = SIN(omega * T);

      const float cubic_term = cubic_coeff * (
        (6.0f / omega4) * cos_offset * (1.0f - cosWT) +
        (6.0f * T / omega3) * sin_offset * sinWT +
        (6.0f * T / omega3) * cos_offset * (cosWT - 1.0f) +
        (3.0f * T2 / omega2) * sin_offset * (1.0f - cosWT) +
        (T3 / omega) * cos_offset * sinWT
      );

      return linear_term + cubic_term;
    }

    float ModalNulling::compute_cubic_decel_projection(const float freq, const float v0, const float linear_coeff, const float cubic_coeff,
                                                       const float duration, const float time_offset) {
      // For velocity profile: v(t) = v₀ + a*t + c * t³
      // This combines linear and cubic terms

      const float linear_proj = compute_linear_accel_projection(freq, v0, linear_coeff, duration, time_offset);
      const float cubic_proj = compute_cubic_velocity_projection(freq, 0.0f, cubic_coeff, duration, time_offset);

      return linear_proj + cubic_proj;
    }

    float ModalNulling::compute_linear_accel_projection(const float freq, const float v0, const float acceleration,
                                                        const float duration, const float time_offset) {
      // For velocity profile: v(t) = v₀ + a*t
      // This is the same as the existing compute_accel_phase_projection but with time offset

      if (freq <= 0.0f || duration <= 0.0f) return 0.0f;

      const float omega = 2.0f * M_PI * freq;
      const float cos_offset = COS(omega * time_offset);
      const float sin_offset = SIN(omega * time_offset);
      const float cosWT = COS(omega * (duration + time_offset));
      const float sinWT = SIN(omega * (duration + time_offset));

      const float constant_term = (v0 / omega) * (cos_offset - cosWT);
      const float linear_term = (acceleration / (omega * omega)) * 
                                (sinWT - sin_offset - omega * duration * cosWT);

      return constant_term + linear_term;
    }

    float ModalNulling::compute_linear_decel_projection(const float freq, const float v0, const float deceleration,
                                                        const float duration, const float time_offset) {
      // For deceleration: v(t) = v₀ - d*t
      return compute_linear_accel_projection(freq, v0, -deceleration, duration, time_offset);
    }

  #endif // S_CURVE_ACCELERATION

  #if HAS_JUNCTION_DEVIATION

    float ModalNulling::apply_junction_deviation_correction(const float base_projection, const float freq,
                                                            const float start_speed, const float peak_speed, const float end_speed,
                                                            const float accel_time, const float coast_time, const float decel_time) {
      // Junction deviation creates smooth curved paths at direction changes instead of sharp corners
      // This affects the frequency content by:
      // 1. Reducing high-frequency content (smoothing effect)
      // 2. Modifying the velocity profile near junctions
      // 3. Creating centripetal acceleration components

      if (freq <= 0.0f) return base_projection;

      // Calculate junction deviation parameters
      // In Marlin, junction deviation is calculated based on:
      // - The angle between current and previous move vectors
      // - The junction deviation setting (typically 0.01-0.1mm)
      // - The maximum speeds of the connecting moves

      const float junction_deviation = 0.05f; // This should come from planner settings
      const float total_time = accel_time + coast_time + decel_time;

      // Estimate the curvature effect based on speed changes
      const float speed_change_accel = peak_speed - start_speed;
      const float speed_change_decel = peak_speed - end_speed;
      const float max_speed_change = _MAX(ABS(speed_change_accel), ABS(speed_change_decel));

      if (max_speed_change < 1.0f) return base_projection; // No significant direction change

      // Calculate the junction angle effect
      // Larger speed changes typically indicate sharper direction changes
      const float estimated_angle = ATAN2(max_speed_change, peak_speed); // Rough approximation
      const float cos_half_angle = COS(estimated_angle * 0.5f);

      // Junction deviation creates a curved path with radius:
      // r = junction_deviation / (1 - cos(θ/2))
      const float junction_radius = junction_deviation / (1.0f - cos_half_angle + 1e-6f);

      // Calculate centripetal acceleration at the junction
      const float centripetal_accel = sq(peak_speed) / junction_radius;

      // Junction deviation affects the motion in several ways:
      // 1. Velocity profile smoothing near junctions
      // 2. Additional frequency content from centripetal motion
      // 3. Path deviation from straight-line motion

      // Calculate the junction transition time (time spent in the curved section)
      const float junction_arc_length = junction_radius * estimated_angle;
      const float junction_time = junction_arc_length / peak_speed;

      // Frequency-dependent correction factors
      const float omega = 2.0f * M_PI * freq;

      // 1. Smoothing effect - higher frequencies are attenuated more
      // This is based on the transfer function of the junction deviation filter
      const float smoothing_time_constant = junction_time * 0.5f;
      const float smoothing_factor = 1.0f / (1.0f + sq(omega * smoothing_time_constant));

      // 2. Centripetal motion contribution
      // The curved path creates additional sinusoidal motion components
      float centripetal_projection = 0.0f;
      if (junction_time > 0.0f && centripetal_accel > 0.0f) {
        // The centripetal acceleration creates a sinusoidal velocity component
        // perpendicular to the main motion direction
        const float centripetal_freq = 1.0f / junction_time; // Characteristic frequency of the junction

        if (ABS(freq - centripetal_freq) < centripetal_freq * 0.1f) {
          // Resonance near the junction frequency
          const float resonance_amplitude = centripetal_accel * junction_time / (2.0f * M_PI);
          centripetal_projection = resonance_amplitude * compute_junction_resonance_projection(freq, junction_time, peak_speed);
        } else {
          // Off-resonance contribution
          const float freq_ratio = freq / centripetal_freq;
          const float off_resonance_factor = 1.0f / (1.0f + sq(freq_ratio - 1.0f));
          centripetal_projection = centripetal_accel * junction_time * off_resonance_factor * 0.1f;
        }
      }

      // 3. Path deviation effect
      // The curved path changes the effective distance and timing
      const float path_deviation_factor = junction_arc_length / (peak_speed * total_time);
      const float path_correction = 1.0f + path_deviation_factor * SIN(omega * junction_time * 0.5f);

      // 4. Velocity profile modification near junctions
      // Junction deviation creates smooth velocity transitions
      const float velocity_smoothing_projection = compute_junction_velocity_smoothing(freq, start_speed, peak_speed, end_speed,
                                                                                    junction_time, junction_radius);

      // Combine all effects
      float corrected_projection = base_projection * smoothing_factor * path_correction;
      corrected_projection += centripetal_projection + velocity_smoothing_projection;

      #if ENABLED(DEBUG_MODAL_NULLING)
        SERIAL_ECHOLNPGM("Junction deviation analysis:");
        SERIAL_ECHOLNPGM("  Estimated angle: ", estimated_angle * 180.0f / M_PI, " degrees");
        SERIAL_ECHOLNPGM("  Junction radius: ", junction_radius, " mm");
        SERIAL_ECHOLNPGM("  Junction time: ", junction_time, " s");
        SERIAL_ECHOLNPGM("  Centripetal accel: ", centripetal_accel, " mm/s²");
        SERIAL_ECHOLNPGM("  Smoothing factor: ", smoothing_factor);
        SERIAL_ECHOLNPGM("  Path correction: ", path_correction);
        SERIAL_ECHOLNPGM("  Centripetal projection: ", centripetal_projection);
        SERIAL_ECHOLNPGM("  Velocity smoothing: ", velocity_smoothing_projection);
        SERIAL_ECHOLNPGM("  Original projection: ", base_projection);
        SERIAL_ECHOLNPGM("  Corrected projection: ", corrected_projection);
      #endif

      return corrected_projection;
    }

    float ModalNulling::compute_junction_resonance_projection(const float freq, const float junction_time, const float peak_speed) {
      // Calculate the modal projection for resonant motion at junctions
      // This accounts for the sinusoidal motion created by the curved path

      if (freq <= 0.0f || junction_time <= 0.0f) return 0.0f;

      const float omega = 2.0f * M_PI * freq;
      const float junction_omega = 2.0f * M_PI / junction_time;

      // For resonant motion: v_perp(t) = A * sin(ω_junction * t) over the junction time
      // Modal projection: ∫₀ᵀ A * sin(ω_junction * t) * sin(ω * t) dt

      if (ABS(omega - junction_omega) < 1e-6f) {
        // Perfect resonance case
        return peak_speed * junction_time * 0.5f;
      } else {
        // Beat frequency case
        const float omega_diff = omega - junction_omega;
        const float omega_sum = omega + junction_omega;

        const float term1 = SIN(omega_diff * junction_time) / (2.0f * omega_diff);
        const float term2 = SIN(omega_sum * junction_time) / (2.0f * omega_sum);

        return peak_speed * (term1 - term2);
      }
    }

    float ModalNulling::compute_junction_velocity_smoothing(const float freq, const float start_speed, const float peak_speed,
                                                            const float end_speed, const float junction_time, const float junction_radius) {
      // Junction deviation creates smooth velocity transitions that differ from the ideal trapezoidal profile
      // This function calculates the additional modal projection from these smooth transitions

      if (freq <= 0.0f || junction_time <= 0.0f) return 0.0f;

      const float omega = 2.0f * M_PI * freq;

      // The velocity smoothing can be modeled as a series of exponential transitions
      // v_smooth(t) = v_ideal(t) + Σ A_i * exp(-t/τ_i) * sin(ω_i * t + φ_i)

      // Calculate the smoothing time constants based on junction geometry
      const float smoothing_tau = junction_time * 0.2f; // Empirical factor
      const float smoothing_amplitude = _MIN(ABS(peak_speed - start_speed), ABS(peak_speed - end_speed)) * 0.1f;

      // Exponentially decaying sinusoidal contribution
      const float decay_factor = 1.0f - expf(-junction_time / smoothing_tau);
      const float frequency_response = 1.0f / (1.0f + sq(omega * smoothing_tau));

      return smoothing_amplitude * decay_factor * frequency_response * junction_time;
    }

    // Additional helper function to get actual junction deviation from planner settings
    float ModalNulling::get_junction_deviation_setting() {
      // This should access the actual planner junction deviation setting
      // For now, return a typical value - this needs to be connected to planner.junction_deviation_mm
      #if HAS_JUNCTION_DEVIATION
        return planner.junction_deviation_mm;
      #else
        return 0.05f; // Default fallback
      #endif
    }

    // Enhanced function to get junction angle from move vectors
    float ModalNulling::calculate_junction_angle(const xyze_float_t& prev_unit_vec, const xyze_float_t& curr_unit_vec) {
      // Calculate the actual angle between two move vectors
      // This is how Marlin calculates junction angles in the planner

      const float dot_product = prev_unit_vec.x * curr_unit_vec.x + prev_unit_vec.y * curr_unit_vec.y + prev_unit_vec.z * curr_unit_vec.z;
      const float cos_theta = WITHIN(dot_product, -1.0f, 1.0f) ? dot_product : (dot_product < 0.0f ? -1.0f : 1.0f);

      return ACOS(-cos_theta); // Note: Marlin uses the supplement of the angle
    }

  #endif // HAS_JUNCTION_DEVIATION

#endif // S_CURVE_ACCELERATION || HAS_JUNCTION_DEVIATION

modal_kernel_t ModalNulling::compute_nulling_kernel(const float freq, const float damping, const float beta,
                                                    const float modal_projection, const float segment_time) {
  modal_kernel_t kernel = {};
  kernel.frequency = freq;
  kernel.decay = beta;
  kernel.valid = false;

  if (ABS(modal_projection) < 1e-8f || freq <= 0.0f || segment_time <= 0.0f)
    return kernel;

  const float omega = 2.0f * M_PI * freq;

  // For kernel k(t) = a * sin(ωt + φ) * exp(-βt)
  // We need: ∫₀ᵀ k(t) * sin(ωt) dt = -modal_projection
  //
  // This expands to: a * ∫₀ᵀ [sin(ωt + φ) * exp(-βt)] * sin(ωt) dt = -modal_projection
  //
  // Using trigonometric identity: sin(ωt + φ) = sin(ωt)cos(φ) + cos(ωt)sin(φ)
  // We get: a * cos(φ) * I₁ + a * sin(φ) * I₂ = -modal_projection
  //
  // Where:
  // I₁ = ∫₀ᵀ sin²(ωt) * exp(-βt) dt
  // I₂ = ∫₀ᵀ sin(ωt)cos(ωt) * exp(-βt) dt

  const float I1 = integrate_sin_squared_exp(omega, beta, segment_time);
  const float I2 = integrate_sin_cos_exp(omega, beta, segment_time);

  // Solve the linear system exactly
  const float denom = I1 * I1 + I2 * I2;

  if (denom > 1e-12f) {
    // Optimal phase that maximizes |I₁*cos(φ) + I₂*sin(φ)|
    kernel.phase = ATAN2(I2, I1);

    // Compute effective integral value
    const float I_eff = I1 * COS(kernel.phase) + I2 * SIN(kernel.phase);
    if (ABS(I_eff) > 1e-12f) {
      kernel.amplitude = -modal_projection / I_eff;

      // Validate the solution mathematically
      kernel.valid = validate_kernel(kernel, modal_projection, segment_time);
    }
  }

  return kernel;
}

float ModalNulling::compute_accel_phase_projection(const float freq, const float start_speed,
                                                   const float acceleration, const float accel_time) {
  if (freq <= 0.0f || accel_time <= 0.0f) return 0.0f;

  const float omega = 2.0f * M_PI * freq;
  const float coswt = COS(omega * accel_time);
  const float sinwt = SIN(omega * accel_time);

  // For acceleration phase: v(t) = start_speed + acceleration * t over [0, T_accel]
  // Modal projection = ∫₀ᵀ v(t) * sin(ωt) dt
  //                  = ∫₀ᵀ (v₀ + a*t) * sin(ωt) dt
  //                  = v₀ * ∫₀ᵀ sin(ωt) dt + a * ∫₀ᵀ t*sin(ωt) dt
  //
  // Component 1: v₀ * ∫₀ᵀ sin(ωt) dt = v₀ * [-cos(ωt)/ω]₀ᵀ = v₀/ω * (1 - cos(ωT))
  // Component 2: a * ∫₀ᵀ t*sin(ωt) dt = a * [sin(ωt)/ω² - t*cos(ωt)/ω]₀ᵀ

  return (start_speed / omega) * (1.0f - coswt) + (acceleration / (omega * omega)) * (sinwt - omega * accel_time * coswt);
}

float ModalNulling::compute_coast_phase_projection(const float freq, const float coast_speed, const float coast_time) {
  if (freq <= 0.0f || coast_time <= 0.0f) return 0.0f;

  const float omega = 2.0f * M_PI * freq;

  // For constant velocity: v(t) = coast_speed over [0, T_coast]
  // Modal projection = ∫₀ᵀ v * sin(ωt) dt = v * ∫₀ᵀ sin(ωt) dt
  //                  = v * [-cos(ωt)/ω]₀ᵀ = v/ω * (1 - cos(ωT))

  return coast_speed / omega * (1.0f - COS(omega * coast_time));
}

float ModalNulling::compute_decel_phase_projection(const float freq, const float peak_speed,
                                                   const float deceleration, const float decel_time) {
  if (freq <= 0.0f || decel_time <= 0.0f) return 0.0f;

  const float omega = 2.0f * M_PI * freq;
  const float coswt = COS(omega * decel_time);
  const float sinwt = SIN(omega * decel_time);

  // For deceleration phase: v(t) = peak_speed - deceleration * t over [0, T_decel]
  // This is similar to acceleration but with negative slope
  // Modal projection = ∫₀ᵀ (v_peak - decel*t) * sin(ωt) dt
  //
  // Component 1: v_peak * ∫₀ᵀ sin(ωt) dt
  // Component 2: -decel * ∫₀ᵀ t*sin(ωt) dt

  return (peak_speed / omega) * (1.0f - coswt) - (deceleration / (omega * omega)) * (sinwt - omega * decel_time * coswt);
}

float ModalNulling::evaluate_kernel(const modal_kernel_t& kernel, const float time) {
  if (!kernel.valid || time < 0.0f) return 0.0f;

  const float omega = 2.0f * M_PI * kernel.frequency;
  return kernel.amplitude * SIN(omega * time + kernel.phase) * expf(-kernel.decay * time);
}

float ModalNulling::evaluate_position_correction_from_kernel(const modal_kernel_t& kernel, const float time) {
  if (!kernel.valid || time <= 0.0f) return 0.0f;

  const float A      = kernel.amplitude;
  const float phi    = kernel.phase;
  const float beta   = kernel.decay;
  const float omega  = 2.0f * M_PI * kernel.frequency;

  const float omega2 = omega * omega;
  const float beta2  = beta * beta;
  const float denom  = (omega2 + beta2);
  const float denom2 = denom * denom;

  if (denom2 < 1e-12f) return 0.0f; // Avoid divide-by-zero

  const float sin_omega_t_phi = SIN(omega * time + phi);
  const float cos_omega_t_phi = COS(omega * time + phi);
  const float sin_phi         = SIN(phi);
  const float cos_phi         = COS(phi);
  const float exp_term        = expf(-beta * time);

  const float term1 = (omega2 - beta2) * sin_omega_t_phi;
  const float term2 = 2.0f * beta * omega * cos_omega_t_phi;
  const float term3 = beta * denom * time * sin_phi;
  const float term4 = omega * denom * time * cos_phi;

  const float correction = A * exp_term * (term1 + term2 + term3 - term4) / denom2;

  return correction;
}

void ModalNulling::apply_kernel_to_trajectory_point(const modal_kernel_t& kernel, const float time,
                                                    const AxisEnum axis, const uint32_t batch_idx,
                                                    xyze_trajectory_t& trajectory) {
  if (!kernel.valid || batch_idx >= FTM_WINDOW_SIZE) return;

  const float kernel_value = evaluate_kernel(kernel, time);

  if (ABS(kernel_value) > 1e-12f) {

    const float correction = evaluate_position_correction_from_kernel(kernel, time);;

    switch (axis) {
      case X_AXIS: trajectory.x[batch_idx] += correction; break;
      case Y_AXIS: trajectory.y[batch_idx] += correction; break;
      default: break;
    }

    #if ENABLED(DEBUG_MODAL_NULLING)
      if (batch_idx % 50 == 0) { // Debug every 50th point
        SERIAL_ECHOLNPGM("Modal correction - Axis: ", axis, " Time: ", time);
        SERIAL_ECHOLNPGM(" Kernel value: ", kernel_value);
        if (ABS(correction) < 0.001f) {
          SERIAL_ECHOLNPGM(" Position correction (um): ", correction * 1000000.0f);
        } else {
          SERIAL_ECHOLNPGM(" Position correction (um): ", correction * 1000000.0f);
          SERIAL_ECHOLNPGM(" Position correction ~(mm): ", correction);
        }
      }
    #endif
  }
}

// Analytical integration helpers
float ModalNulling::integrate_sin_product(const float omega, const float time) {
  // ∫₀ᵀ sin(ωt) dt = [-cos(ωt)/ω]₀ᵀ = (1 - cos(ωT))/ω
  if (ABS(omega) < 1e-12f) return 0.0f;
  return (1.0f - COS(omega * time)) / omega;
}

float ModalNulling::integrate_linear_sin_product(const float omega, const float slope, const float time) {
  // ∫₀ᵀ t*sin(ωt) dt = [sin(ωt)/ω² - t*cos(ωt)/ω]₀ᵀ
  if (ABS(omega) < 1e-12f) return 0.0f;

  const float omega_sq = omega * omega;
  return (SIN(omega * time) - omega * time * COS(omega * time)) / omega_sq;
}

float ModalNulling::integrate_sin_squared_exp(const float omega, const float beta, const float time) {
  // ∫₀ᵀ sin²(ωt) * exp(-βt) dt
  // Using sin²(x) = (1 - cos(2x))/2
  // = (1/2) * ∫₀ᵀ exp(-βt) dt - (1/2) * ∫₀ᵀ cos(2ωt) * exp(-βt) dt

  if (ABS(beta) < 1e-12f) return 0.0f;

  const float exp_term = (1.0f - expf(-beta * time)) / beta;
  const float cos_exp_term = integrate_cos_exp_product(2.0f * omega, beta, time);

  return 0.5f * (exp_term - cos_exp_term);
}

float ModalNulling::integrate_sin_cos_exp(const float omega, const float beta, const float time) {
  // ∫₀ᵀ sin(ωt) * cos(ωt) * exp(-βt) dt
  // Using sin(x)cos(x) = sin(2x)/2
  // = (1/2) * ∫₀ᵀ sin(2ωt) * exp(-βt) dt

  return 0.5f * integrate_sin_exp_product(2.0f * omega, beta, time);
}

float ModalNulling::integrate_sin_exp_product(const float omega, const float beta, const float time) {
  // ∫₀ᵀ sin(ωt) * exp(-βt) dt
  // = [exp(-βt) * (-β*sin(ωt) - ω*cos(ωt)) / (β² + ω²)]₀ᵀ

  const float denom = beta * beta + omega * omega;
  if (denom < 1e-12f) return 0.0f;

  const float exp_T = expf(-beta * time);
  const float sin_T = SIN(omega * time);
  const float cos_T = COS(omega * time);

  return (omega / denom) * (1.0f - exp_T * (cos_T + (beta / omega) * sin_T));
}

float ModalNulling::integrate_cos_exp_product(const float omega, const float beta, const float time) {
  // ∫₀ᵀ cos(ωt) * exp(-βt) dt
  // = [exp(-βt) * (-β*cos(ωt) + ω*sin(ωt)) / (β² + ω²)]₀ᵀ

  const float denom = beta * beta + omega * omega;
  if (denom < 1e-12f) return 0.0f;

  const float exp_T = expf(-beta * time);
  const float sin_T = SIN(omega * time);
  const float cos_T = COS(omega * time);

  return (beta / denom) * (1.0f - exp_T * cos_T) + (omega / denom) * exp_T * sin_T;
}

float ModalNulling::integrate_kernel_sin_product(const modal_kernel_t& kernel, const float time) {
  // ∫₀ᵀ k(t) * sin(ωt) dt where k(t) = a * sin(ωt + φ) * exp(-βt)
  if (time <= 0.0f) return 0.0f;

  const float omega = 2.0f * M_PI * kernel.frequency;

  // Using trigonometric identity: sin(ωt + φ) = sin(ωt)cos(φ) + cos(ωt)sin(φ)
  // ∫₀ᵀ a * [sin(ωt)cos(φ) + cos(ωt)sin(φ)] * exp(-βt) * sin(ωt) dt
  // = a * cos(φ) * ∫₀ᵀ sin²(ωt) * exp(-βt) dt + a * sin(φ) * ∫₀ᵀ sin(ωt)cos(ωt) * exp(-βt) dt

  const float I1 = integrate_sin_squared_exp(omega, kernel.decay, time);
  const float I2 = integrate_sin_cos_exp(omega, kernel.decay, time);

  return kernel.amplitude * (COS(kernel.phase) * I1 + SIN(kernel.phase) * I2);
}

bool ModalNulling::validate_kernel(const modal_kernel_t& kernel, const float original_projection,
                                   const float segment_time) {
  // Check for numerical sanity
  if (!isfinite(kernel.amplitude) || !isfinite(kernel.phase) ||
      !isfinite(kernel.decay) || !isfinite(kernel.frequency)) {
    #if ENABLED(DEBUG_MODAL_NULLING)
      SERIAL_ECHOLNPGM("Kernel validation failed: non-finite values");
    #endif
    return false;
  }

  // Check if kernel parameters are physically reasonable
  if (kernel.frequency <= 0.0f || kernel.frequency > 100.0f) {
    #if ENABLED(DEBUG_MODAL_NULLING)
      SERIAL_ECHOLNPGM("Kernel validation failed: frequency out of range: ", kernel.frequency);
    #endif
    return false; // Frequency range check
  }

  if (kernel.decay <= 0.0f) {
    #if ENABLED(DEBUG_MODAL_NULLING)
      SERIAL_ECHOLNPGM("Kernel validation failed: decay must be positive: ", kernel.decay);
    #endif
    return false; // Decay must be positive
  }

  // Most importantly: verify the kernel actually achieves orthogonality
  const float kernel_projection = integrate_kernel_sin_product(kernel, segment_time);
  const float total_projection = original_projection + kernel_projection;
  const float nulling_effectiveness = ABS(total_projection) / (ABS(original_projection) + 1e-15f);

  #if ENABLED(DEBUG_MODAL_NULLING)
    SERIAL_ECHOLNPGM("Nulling validation:");
    SERIAL_ECHOLNPGM(" Original: ", original_projection);
    SERIAL_ECHOLNPGM(" Kernel: ", kernel_projection);
    SERIAL_ECHOLNPGM(" Total: ", total_projection);
    SERIAL_ECHOLNPGM(" Effectiveness: ", (1.0f - nulling_effectiveness) * 100.0f, "%");
  #endif

  // Accept if nulling is effective (>95% reduction)
  return nulling_effectiveness < 0.05;
}

float ModalNulling::compute_residual_projection(const modal_kernel_t& kernel, const float original_projection,
                                                const float segment_time) {
  // Compute the residual modal projection after applying the kernel
  const float kernel_projection = integrate_kernel_sin_product(kernel, segment_time);
  return original_projection + kernel_projection;
}

#if ENABLED(DEBUG_MODAL_NULLING)

  void ModalNulling::debug_print_segment_analysis(const motion_segment_t& segment) {
    SERIAL_ECHOLNPGM("Modal Nulling Segment Analysis:");
    SERIAL_ECHOLNPGM("  Segment time: ", segment.segment_time);
    SERIAL_ECHOLNPGM("  Accel time: ", segment.accel_time);
    SERIAL_ECHOLNPGM("  Coast time: ", segment.coast_time);
    SERIAL_ECHOLNPGM("  Decel time: ", segment.decel_time);
    SERIAL_ECHOLNPGM("  Speed profile: ", segment.start_speed, " -> ", segment.peak_speed, " -> ", segment.end_speed);
    SERIAL_ECHOLNPGM("  Acceleration: ", segment.acceleration);
    SERIAL_ECHOLNPGM("  Deceleration: ", segment.deceleration);
    SERIAL_ECHOLNPGM("  X motion: ", segment.has_motion[X_AXIS] ? "Yes" : "No");
    SERIAL_ECHOLNPGM("  Y motion: ", segment.has_motion[Y_AXIS] ? "Yes" : "No");
  }

  void ModalNulling::debug_print_kernel(const modal_kernel_t& kernel, const char* axis_name) {
    SERIAL_ECHOPGM("Modal Kernel ", axis_name, ": ");
    if (kernel.valid) {
      SERIAL_ECHOPGM("A=", kernel.amplitude, " φ=", kernel.phase);
      SERIAL_ECHOPGM(" f=", kernel.frequency, " β=", kernel.decay);
    } else {
      SERIAL_ECHOPGM("INVALID");
    }
    SERIAL_EOL();
  }

#endif // DEBUG_MODAL_NULLING

#endif // MODAL_NULLING
