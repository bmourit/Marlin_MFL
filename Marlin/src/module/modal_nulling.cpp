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
#include "planner.h"

ModalNulling modalNulling;
mn_config_t ModalNulling::cfg;

void ModalNulling::init() {
  set_defaults();
}

void ModalNulling::reset() {
  set_defaults();
}

void ModalNulling::set_defaults() {
  cfg.enabled = MODAL_NULLING_DEFAULT_ENABLED;
  cfg.freq[X_AXIS] = MODAL_NULLING_DEFAULT_FREQ_X;
  cfg.freq[Y_AXIS] = MODAL_NULLING_DEFAULT_FREQ_Y;
  cfg.damping[X_AXIS] = MODAL_NULLING_DEFAULT_DAMPING_X;
  cfg.damping[Y_AXIS] = MODAL_NULLING_DEFAULT_DAMPING_Y;
}

bool ModalNulling::should_modify_block(const block_t* const block) {
  if (block->steps.x == 0 && block->steps.y == 0) return false;
  if (block->step_event_count < 100) return false;
  if (block->accelerate_before < 10 || (block->step_event_count - block->decelerate_start) < 10)
    return false;
  return true;
}

void ModalNulling::modify_trapezoid_parameters(block_t* const block, 
                                                 uint32_t& initial_rate, uint32_t& final_rate,
                                                 int32_t& accelerate_steps, int32_t& decelerate_steps,
                                                 int32_t& plateau_steps, const float inverse_accel) {
  if (!cfg.enabled) return;
  if (!should_modify_block(block)) return;

  // Convert rates back to speeds for calculations
  const float spmm = block->steps_per_mm;
  const float initial_speed = float(initial_rate) / spmm;
  const float final_speed = float(final_rate) / spmm;
  const float nominal_speed = float(block->nominal_rate) / spmm;

  // Calculate acceleration phase parameters
  if (accelerate_steps > 0) {
    const float accel_time = (nominal_speed - initial_speed) / block->acceleration;
    float accel_adj_factor = compute_acceleration_adjustment(block, initial_speed, nominal_speed, accel_time);

    if (accel_adj_factor != 1.0f) {
      // Modify acceleration
      block->acceleration *= accel_adj_factor;
      block->acceleration_steps_per_s2 = uint32_t(block->acceleration * spmm);

      // Recalculate acceleration steps with new acceleration
      const float new_accel_steps_float = 0.5f * (FLOAT_SQ(block->nominal_rate) - FLOAT_SQ(initial_rate)) / block->acceleration_steps_per_s2;
      accelerate_steps = CEIL(new_accel_steps_float);
    }
  }

  // Calculate deceleration phase parameters
  if (decelerate_steps > 0) {
    const float decel_time = (nominal_speed - final_speed) / block->acceleration;
    float decel_adj_factor = compute_deceleration_adjustment(block, nominal_speed, final_speed, decel_time);

    if (decel_adj_factor != 1.0f) {
      // For deceleration, we might need a different approach since we're using the same acceleration value
      // This is a design decision - you might want separate accel/decel values
      const float new_decel_steps_float = 0.5f * (FLOAT_SQ(block->nominal_rate) - FLOAT_SQ(final_rate)) / (block->acceleration_steps_per_s2 * decel_adj_factor);
      decelerate_steps = CEIL(new_decel_steps_float);
    }
  }

  // Recalculate plateau steps
  plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // Handle case where acceleration + deceleration exceeds total steps
  if (plateau_steps < 0) {
    // Redistribute steps proportionally
    const float total_accel_decel = accelerate_steps + decelerate_steps;
    accelerate_steps = LROUND(accelerate_steps * block->step_event_count / total_accel_decel);
    decelerate_steps = block->step_event_count - accelerate_steps;
    plateau_steps = 0;
  }
}

float ModalNulling::compute_acceleration_adjustment(block_t* const block, const float initial_speed,
                                                      const float final_speed, const float time) {
  float x_factor = 1.0f, y_factor = 1.0f;

  if (block->steps.x > 0 && cfg.freq[X_AXIS] > 0) {
    x_factor = compute_modal_nulling_factor(cfg.freq[X_AXIS], cfg.damping[X_AXIS], 
                                           block->acceleration, initial_speed, time);
  }

  if (block->steps.y > 0 && cfg.freq[Y_AXIS] > 0) {
    y_factor = compute_modal_nulling_factor(cfg.freq[Y_AXIS], cfg.damping[Y_AXIS], 
                                           block->acceleration, initial_speed, time);
  }

  // Use the more conservative factor
  float adj_factor = fminf(x_factor, y_factor);

  // Constrain to reasonable bounds
  return constrain(adj_factor, 0.5f, 2.0f);
}

float ModalNulling::compute_deceleration_adjustment(block_t* const block, const float initial_speed,
                                                    const float final_speed, const float time) {
  float x_factor = 1.0f, y_factor = 1.0f;

  if (block->steps.x > 0 && cfg.freq[X_AXIS] > 0) {
    x_factor = compute_modal_nulling_factor(cfg.freq[X_AXIS], cfg.damping[X_AXIS], 
                                           block->acceleration, initial_speed, time);
  }

  if (block->steps.y > 0 && cfg.freq[Y_AXIS] > 0) {
    y_factor = compute_modal_nulling_factor(cfg.freq[Y_AXIS], cfg.damping[Y_AXIS], 
                                            block->acceleration, initial_speed, time);
  }

  return fminf(x_factor, y_factor);
}

// Analytical computation of modal projection for trapezoidal velocity profile
float ModalNulling::compute_modal_projection(const float freq, const float damping, 
                                             const float accel, const float initial_rate,
                                             const float time) {
  if (time <= 0.0f || freq <= 0.0f) return 0.0f;

  const float omega_n = 2.0f * M_PI * freq;                   // Natural frequency
  const float zeta = constrain(damping, 0.01f, 0.99f);        // Damping ratio
  const float omega_d = omega_n * sqrtf(1.0f - zeta * zeta);  // Damped frequency
  const float sigma = zeta * omega_n;                         // Decay rate

  // For velocity profile v(t) = v0 + accel*t over [0,T]
  // Modal projection = ∫₀ᵀ v(t) * φ(t) dt where φ(t) = sin(ω_d*t) * exp(-σ*t)

  const float exp_T = expf(-sigma * time);
  const float cos_T = cosf(omega_d * time);
  const float sin_T = sinf(omega_d * time);

  // Component 1: ∫₀ᵀ v0 * sin(ω_d*t) * exp(-σ*t) dt
  const float denom1 = sigma * sigma + omega_d * omega_d;
  float proj1 = 0.0f;
  if (denom1 > 1e-12f) {
    proj1 = initial_rate * omega_d / denom1 * (1.0f - exp_T * (cos_T + (sigma / omega_d) * sin_T));
  }

  // Component 2: ∫₀ᵀ accel*t * sin(ω_d*t) * exp(-σ*t) dt
  const float denom2 = denom1 * denom1;
  float proj2 = 0.0f;
  if (denom2 > 1e-12f) {
    const float A = 2.0f * sigma * omega_d;
    const float B = omega_d * omega_d - sigma * sigma;
    const float C = 2.0f * sigma * sigma * omega_d;

    proj2 = accel / denom2 * (A - exp_T * (A * cos_T + (C + B * omega_d * time) * sin_T));
  }

  return proj1 + proj2;
}

// Derivative of modal projection with respect to acceleration scaling factor
float ModalNulling::compute_modal_projection_derivative(const float freq, const float damping,
                                                        const float accel, const float initial_rate,
                                                        const float time) {
  if (time <= 0.0f || freq <= 0.0f) return 0.0f;

  const float omega_n = 2.0f * M_PI * freq;
  const float zeta = constrain(damping, 0.01f, 0.99f);
  const float omega_d = omega_n * sqrtf(1.0f - zeta * zeta);
  const float sigma = zeta * omega_n;

  // For scaled acceleration α*accel, the derivative d/dα of the projection
  // This is just the acceleration component of the projection

  const float exp_T = expf(-sigma * time);
  const float cos_T = cosf(omega_d * time);
  const float sin_T = sinf(omega_d * time);

  const float denom = (sigma * sigma + omega_d * omega_d) * (sigma * sigma + omega_d * omega_d);

  if (denom < 1e-12f) return 0.0f;

  const float A = 2.0f * sigma * omega_d;
  const float B = omega_d * omega_d - sigma * sigma;
  const float C = 2.0f * sigma * sigma * omega_d;

  return A / denom * (1.0f - exp_T * (cos_T + ((C / A) + (B * omega_d * time) / A) * sin_T));
}

// Compute the acceleration adjustment needed to nullify modal excitation
float ModalNulling::compute_modal_nulling_factor(const float freq, const float damping,
                                                 const float accel, const float initial_rate,
                                                 const float time) {
  // Current modal projection with unit acceleration
  float current_projection = compute_modal_projection(freq, damping, accel, initial_rate, time);

  if (fabsf(current_projection) < 1e-8f) return 1.0f; // Already nullified

  // Compute how projection changes with acceleration scaling
  float proj_derivative = compute_modal_projection_derivative(freq, damping, accel, initial_rate, time);

  if (fabsf(proj_derivative) < 1e-8f) return 1.0f; // Cannot nullify

  // TODO: Get rid of this and use a better, more accurate nulling approach
  //
  // Newton's method: find scaling factor α such that projection(α*accel) ≈ 0
  // projection(α) ≈ projection(1) + (α-1) * derivative
  // Set to zero: α = 1 - projection(1) / derivative
  float alpha = 1.0f - current_projection / proj_derivative;

  // Constrain to reasonable bounds to avoid instability
  return constrain(alpha, 0.3f, 3.0f);
}

#if ENABLED(S_CURVE_ACCELERATION)
  // TODO: Avoid duplicating this function
  FORCE_INLINE static uint32_t calculate_inverse_time(const uint32_t d) {
    return d ? 0xFFFFFFFF / d : 0xFFFFFFFF;
  }

  void ModalNulling::modify_scurve_parameters(block_t* const block,
                                              uint32_t& initial_rate, uint32_t& final_rate, uint32_t& cruise_rate,
                                              uint32_t& acceleration_time, uint32_t& deceleration_time,
                                              uint32_t& acceleration_time_inverse, uint32_t& deceleration_time_inverse) {
    if (!cfg.enabled) return;
    if (!should_modify_block(block)) return;

    const float spmm = block->steps_per_mm;
    const float initial_speed = float(initial_rate) / spmm;
    const float final_speed = float(final_rate) / spmm;
    const float cruise_speed = float(cruise_rate) / spmm;

    // Calculate S-curve acceleration phase adjustment
    if (acceleration_time > 0) {
      const float accel_time_sec = float(acceleration_time) / STEPPER_TIMER_RATE;
      float accel_adj_factor = compute_scurve_acceleration_adjustment(block, initial_speed, cruise_speed, accel_time_sec);

      if (accel_adj_factor != 1.0f) {
        // Modify acceleration timing for S-curve
        acceleration_time = uint32_t(acceleration_time / accel_adj_factor);
        acceleration_time_inverse = calculate_inverse_time(acceleration_time);
      }
    }

    // Calculate S-curve deceleration phase adjustment
    if (deceleration_time > 0) {
      const float decel_time_sec = float(deceleration_time) / STEPPER_TIMER_RATE;
      float decel_adj_factor = compute_scurve_deceleration_adjustment(block, cruise_speed, final_speed, decel_time_sec);

      if (decel_adj_factor != 1.0f) {
        deceleration_time = uint32_t(deceleration_time / decel_adj_factor);
        deceleration_time_inverse = calculate_inverse_time(deceleration_time);
      }
    }
  }

  float ModalNulling::compute_scurve_acceleration_adjustment(block_t* const block, const float initial_speed,
                                                             const float final_speed, const float time) {
    float x_factor = 1.0f, y_factor = 1.0f;

    // For S-curve, we need to account for the smooth acceleration profile
    // The velocity profile is approximately: v(t) = v0 + (vf-v0) * smooth_step(t/T)
    // where smooth_step(x) = 3x² - 2x³

    if (block->steps.x > 0 && cfg.freq[X_AXIS] > 0) {
      x_factor = compute_modal_nulling_factor_scurve(cfg.freq[X_AXIS], cfg.damping[X_AXIS], 
                                                     initial_speed, final_speed, time);
    }

    if (block->steps.y > 0 && cfg.freq[Y_AXIS] > 0) {
      y_factor = compute_modal_nulling_factor_scurve(cfg.freq[Y_AXIS], cfg.damping[Y_AXIS], 
                                                     initial_speed, final_speed, time);
    }

    float adj_factor = fminf(x_factor, y_factor);

    return constrain(adj_factor, 0.5f, 2.0f);
  }

  float ModalNulling::compute_scurve_deceleration_adjustment(block_t* const block, const float initial_speed,
                                                             const float final_speed, const float time) {
    // Similar to acceleration but for deceleration phase
    return compute_scurve_acceleration_adjustment(block, initial_speed, final_speed, time);
  }

  // S-curve specific modal nulling calculation
  float ModalNulling::compute_modal_nulling_factor_scurve(const float freq, const float damping,
                                                          const float initial_speed, const float final_speed,
                                                          const float time) {
    if (time <= 0.0f || freq <= 0.0f) return 1.0f;

    // For S-curve velocity profile: v(t) = v0 + (v1-v0) * smooth_step(t/T)
    // where smooth_step(x) = 3x² - 2x³
    // This requires analytical integration of the S-curve profile with the modal basis

    float current_projection = compute_modal_projection_scurve(freq, damping, initial_speed, final_speed, time);

    if (fabsf(current_projection) < 1e-8f) return 1.0f;

    // For S-curve, the adjustment is more complex since we're modifying timing rather than acceleration
    // We use a simplified approach (for now): find the time scaling that minimizes modal excitation

    const float test_factors[] = { 0.8f, 0.9f, 1.1f, 1.25f };
    float best_factor = 1.0f;
    float min_projection = fabsf(current_projection);

    for (uint8_t i = 0; i < 4; i++) {
      const float test_factor = test_factors[i];
      const float test_time = time * test_factor;
      const float test_projection = compute_modal_projection_scurve(freq, damping, initial_speed, final_speed, test_time);

      if (fabsf(test_projection) < min_projection) {
        min_projection = fabsf(test_projection);
        best_factor = test_factor;
      }
    }

    return best_factor;
  }

  // Analytical computation of modal projection for S-curve velocity profile
  float ModalNulling::compute_modal_projection_scurve(const float freq, const float damping,
                                                      const float initial_speed, const float final_speed,
                                                      const float time) {
    if (time <= 0.0f || freq <= 0.0f) return 0.0f;

    const float omega_n = 2.0f * M_PI * freq;
    const float zeta = constrain(damping, 0.01f, 0.99f);
    const float omega_d = omega_n * sqrtf(1.0f - zeta * zeta);
    const float sigma = zeta * omega_n;

    // For S-curve: v(t) = v0 + (v1-v0) * (3(t/T)² - 2(t/T)³)
    // Let s = t/T, then v(t) = v0 + (v1-v0) * (3s² - 2s³)
    // Modal projection = ∫₀ᵀ v(t) * sin(ω_d*t) * exp(-σ*t) dt

    // This is a complex analytical integration. For practical implementation,
    // we can use numerical integration with sufficient accuracy

    float projection = 0.0f;
    const uint8_t num_samples = 32; // Higher resolution for S-curve
    const float dt = time / num_samples;

    for (uint8_t i = 0; i < num_samples; i++) {
      const float t = (i + 0.5f) * dt; // Midpoint rule
      const float s = t / time;
      const float smooth_step = 3.0f * s * s - 2.0f * s * s * s;
      const float velocity = initial_speed + (final_speed - initial_speed) * smooth_step;
      const float modal_basis = sinf(omega_d * t) * expf(-sigma * t);

      projection += velocity * modal_basis * dt;
    }

    return projection;
  }
#endif // S_CURVE_ACCELERATION

// Enhanced modal excitation simulation for validation/debugging
float ModalNulling::simulate_modal_response_analytical(const float freq, const float damping, 
                                                       const float accel, const float initial_rate,
                                                       const float final_rate, const float time) {
  // This function provides an alternative analytical approach for validation
  const float omega_n = 2.0f * M_PI * freq;
  const float zeta = constrain(damping, 0.01f, 0.99f);

  // Transfer function magnitude at resonant frequency
  // |H(jω)| = ωₙ² / √[(ωₙ² - ω²)² + (2ζωₙω)²]
  // At resonance (ω = ωₙ): |H(jωₙ)| = 1/(2ζ)
  const float resonance_gain = 1.0f / (2.0f * zeta);

  // Compute RMS of velocity profile over the time interval
  const float v_avg = (initial_rate + final_rate) / 2.0f;
  const float v_rms = sqrtf((initial_rate * initial_rate + initial_rate * final_rate + final_rate * final_rate) / 3.0f);

  // Approximate modal response as RMS velocity × resonance gain
  return v_rms * resonance_gain;
}

// Compute optimal phase shift for modal nulling (advanced feature)
float ModalNulling::compute_optimal_phase_shift(const float freq, const float damping,
                                                const float accel, const float initial_rate,
                                                const float time) {
  const float omega_d = 2.0f * M_PI * freq * sqrtf(1.0f - damping * damping);

  // For a trapezoidal profile, the optimal phase shift minimizes
  // the projection onto both sin(ωt) and cos(ωt) components

  // Compute projections onto sin and cos components
  float sin_proj = compute_modal_projection(freq, damping, accel, initial_rate, time);
  float cos_proj = compute_modal_projection_cosine(freq, damping, accel, initial_rate, time);

  // Optimal phase: φ = atan2(-sin_proj, -cos_proj)
  return atan2f(-sin_proj, -cos_proj);
}

// Helper function for cosine projection (needed for phase optimization)
float ModalNulling::compute_modal_projection_cosine(const float freq, const float damping,
                                                    const float accel, const float initial_rate,
                                                    const float time) {
  if (time <= 0.0f || freq <= 0.0f) return 0.0f;

  const float omega_n = 2.0f * M_PI * freq;
  const float zeta = constrain(damping, 0.01f, 0.99f);
  const float omega_d = omega_n * sqrtf(1.0f - zeta * zeta);
  const float sigma = zeta * omega_n;

  // Similar to sine projection but with cos(ω_d*t) instead of sin(ω_d*t)
  const float exp_T = expf(-sigma * time);
  const float cos_T = cosf(omega_d * time);
  const float sin_T = sinf(omega_d * time);

  // Component 1: ∫₀ᵀ v0 * cos(ω_d*t) * exp(-σ*t) dt
  const float denom1 = sigma * sigma + omega_d * omega_d;
  float proj1 = 0.0f;
  if (denom1 > 1e-12f) {
    proj1 = initial_rate * sigma / denom1 * (1.0f - exp_T * (cos_T - (omega_d / sigma) * sin_T));
  }

  // Component 2: ∫₀ᵀ accel*t * cos(ω_d*t) * exp(-σ*t) dt
  const float denom2 = denom1 * denom1;
  float proj2 = 0.0f;
  if (denom2 > 1e-12f) {
    const float A = sigma * sigma - omega_d * omega_d;
    const float B = 2.0f * sigma * omega_d;

    proj2 = accel * sigma / denom2 * (A - exp_T * (A * cos_T - (B + A * sigma * time) * sin_T));
  }

  return proj1 + proj2;
}

// Validate modal nulling effectiveness (for debugging/tuning)
bool ModalNulling::validate_modal_nulling(const float freq, const float damping,
                                          const float original_accel, const float modified_accel,
                                          const float initial_rate, const float time) {
  float original_projection = compute_modal_projection(freq, damping, original_accel, initial_rate, time);
  float modified_projection = compute_modal_projection(freq, damping, modified_accel, initial_rate, time);

  // Check if modification reduced modal excitation by at least 50%
  return fabsf(modified_projection) < 0.5f * fabsf(original_projection);
}

#endif // MODAL_NULLING
