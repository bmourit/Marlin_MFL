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

ModalNulling modalNulling;
mn_config_t ModalNulling::cfg;
motion_segment_t ModalNulling::current_segment;

void ModalNulling::init() {
  set_defaults();
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
                                          const float acceleration, const float segment_time,
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

  if (!cfg.enabled) {
    current_segment.valid = false;
    return;
  }

  // Check if nulling should be applied
  if (!should_apply_nulling(has_x_motion, has_y_motion, segment_time, peak_speed)) {
    current_segment.valid = false;
    return;
  }

  // Store segment parameters
  current_segment.has_motion[X_AXIS] = has_x_motion;
  current_segment.has_motion[Y_AXIS] = has_y_motion;
  current_segment.segment_time = segment_time;
  current_segment.accel_time = accel_time;
  current_segment.coast_time = coast_time;
  current_segment.decel_time = decel_time;
  current_segment.start_speed = start_speed;
  current_segment.peak_speed = peak_speed;
  current_segment.end_speed = end_speed;
  current_segment.acceleration = acceleration;

  // Compute nulling kernels for each axis
  for (uint8_t axis = 0; axis < XY; axis++) {
    if (current_segment.has_motion[axis] && cfg.freq[axis] > 0.0f) {
      // Compute modal projection for this axis
      const float modal_proj = compute_modal_projection(cfg.freq[axis], start_speed, peak_speed, end_speed,
                                                        acceleration, accel_time, coast_time, decel_time);

      #if ENABLED(DEBUG_MODAL_NULLING)
        SERIAL_ECHOLNPGM("Axis ", axis, " modal projection: ", modal_proj);
        SERIAL_ECHOLNPGM("Frequency: ", cfg.freq[axis], " Hz");
      #endif

      // Compute exact nulling kernel
      current_segment.kernels[axis] = compute_nulling_kernel(cfg.freq[axis], cfg.damping[axis],
                                                             cfg.beta[axis], modal_proj, segment_time);

      #if ENABLED(DEBUG_MODAL_NULLING)
        debug_print_kernel(current_segment.kernels[axis], axis == X_AXIS ? "X" : "Y");
        SERIAL_ECHOLNPGM("Modal projection ", axis == X_AXIS ? "X" : "Y", ": ", modal_proj);

        // Verify orthogonality
        if (current_segment.kernels[axis].valid) {
          const float residual = compute_residual_projection(current_segment.kernels[axis], modal_proj, segment_time);
          SERIAL_ECHOLNPGM("Residual projection: ", residual, " (", fabsf(residual/modal_proj) * 100.0f, "% of original)");
        }
      #endif
    } else {
      current_segment.kernels[axis].valid = false;
    }
  }

  current_segment.valid = true;
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
  for (uint8_t axis = 0; axis < XY; axis++) {
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
          total_correction_x += fabsf(correction);
          NOLESS(max_correction_x, fabsf(correction));
        } else {
          total_correction_y += fabsf(correction);
          NOLESS(max_correction_y, fabsf(correction));
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
  if (!has_x_motion && !has_y_motion) return false;

  // Segment must be long enough for meaningful nulling
  if (segment_time < 0.01f) return false;

  // Don't apply to very fast moves (likely travel moves)
  //if (peak_speed > 500.0f) return false;

  // Must have valid frequencies configured
  bool has_valid_freq = false;
  for (uint8_t axis = 0; axis < XY; axis++) {
    if (cfg.freq[axis] > 0.0f && cfg.freq[axis] < 100.0f) {
      has_valid_freq = true;
      break;
    }
  }

  return has_valid_freq;
}

float ModalNulling::compute_modal_projection(const float freq, const float start_speed, const float peak_speed,
                                            const float end_speed, const float acceleration,
                                            const float accel_time, const float coast_time, const float decel_time) {
  if (freq <= 0.0f) return 0.0f;

  float total_projection = 0.0f;

  // Acceleration phase projection
  if (accel_time > 0.0f) {
    total_projection += compute_accel_phase_projection(freq, start_speed, acceleration, accel_time);
  }

  // Coast phase projection
  if (coast_time > 0.0f) {
    total_projection += compute_coast_phase_projection(freq, peak_speed, coast_time);
  }

  // Deceleration phase projection
  if (decel_time > 0.0f) {
    const float deceleration = (peak_speed - end_speed) / decel_time;
    total_projection += compute_decel_phase_projection(freq, peak_speed, deceleration, decel_time);
  }

  return total_projection;
}

modal_kernel_t ModalNulling::compute_nulling_kernel(const float freq, const float damping, const float beta,
                                                   const float modal_projection, const float segment_time) {
  modal_kernel_t kernel = {0};
  kernel.frequency = freq;
  kernel.decay = beta;
  kernel.valid = false;

  if (fabsf(modal_projection) < 1e-8f || freq <= 0.0f || segment_time <= 0.0f) {
    return kernel;
  }

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
    kernel.phase = atan2f(I2, I1);

    // Compute effective integral value
    const float I_eff = I1 * cosf(kernel.phase) + I2 * sinf(kernel.phase);

    if (fabsf(I_eff) > 1e-12f) {
      kernel.amplitude = -modal_projection / I_eff;

      // Validate the solution mathematically rather than using arbitrary bounds
      kernel.valid = validate_kernel(kernel, modal_projection, segment_time);
    }
  }

  return kernel;
}

float ModalNulling::compute_accel_phase_projection(const float freq, const float start_speed,
                                                  const float acceleration, const float accel_time) {
  if (freq <= 0.0f || accel_time <= 0.0f) return 0.0f;

  const float omega = 2.0f * M_PI * freq;

  // For acceleration phase: v(t) = start_speed + acceleration * t over [0, T_accel]
  // Modal projection = ∫₀ᵀ v(t) * sin(ωt) dt
  //                  = ∫₀ᵀ (v₀ + a*t) * sin(ωt) dt
  //                  = v₀ * ∫₀ᵀ sin(ωt) dt + a * ∫₀ᵀ t*sin(ωt) dt

  // Component 1: v₀ * ∫₀ᵀ sin(ωt) dt = v₀ * [-cos(ωt)/ω]₀ᵀ = v₀/ω * (1 - cos(ωT))
  const float comp1 = start_speed / omega * (1.0f - cosf(omega * accel_time));

  // Component 2: a * ∫₀ᵀ t*sin(ωt) dt = a * [sin(ωt)/ω² - t*cos(ωt)/ω]₀ᵀ
  const float comp2 = acceleration / (omega * omega) *
                     (sinf(omega * accel_time) - omega * accel_time * cosf(omega * accel_time));

  return comp1 + comp2;
}

float ModalNulling::compute_coast_phase_projection(const float freq, const float coast_speed, const float coast_time) {
  if (freq <= 0.0f || coast_time <= 0.0f) return 0.0f;

  const float omega = 2.0f * M_PI * freq;

  // For constant velocity: v(t) = coast_speed over [0, T_coast]
  // Modal projection = ∫₀ᵀ v * sin(ωt) dt = v * ∫₀ᵀ sin(ωt) dt
  //                  = v * [-cos(ωt)/ω]₀ᵀ = v/ω * (1 - cos(ωT))

  return coast_speed / omega * (1.0f - cosf(omega * coast_time));
}

float ModalNulling::compute_decel_phase_projection(const float freq, const float peak_speed,
                                                  const float deceleration, const float decel_time) {
  if (freq <= 0.0f || decel_time <= 0.0f) return 0.0f;

  const float omega = 2.0f * M_PI * freq;

  // For deceleration phase: v(t) = peak_speed - deceleration * t over [0, T_decel]
  // This is similar to acceleration but with negative slope
  // Modal projection = ∫₀ᵀ (v_peak - decel*t) * sin(ωt) dt

  // Component 1: v_peak * ∫₀ᵀ sin(ωt) dt
  const float comp1 = peak_speed / omega * (1.0f - cosf(omega * decel_time));

  // Component 2: -decel * ∫₀ᵀ t*sin(ωt) dt
  const float comp2 = -deceleration / (omega * omega) *
                     (sinf(omega * decel_time) - omega * decel_time * cosf(omega * decel_time));

  return comp1 + comp2;
}

float ModalNulling::evaluate_kernel(const modal_kernel_t& kernel, const float time) {
  if (!kernel.valid || time < 0.0f) return 0.0f;

  const float omega = 2.0f * M_PI * kernel.frequency;
  return kernel.amplitude * sinf(omega * time + kernel.phase) * expf(-kernel.decay * time);
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

  const float sin_omega_t_phi = sinf(omega * time + phi);
  const float cos_omega_t_phi = cosf(omega * time + phi);
  const float sin_phi         = sinf(phi);
  const float cos_phi         = cosf(phi);
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

  if (fabsf(kernel_value) > 1e-12f) {

    #if ENABLED(DEBUG_MODAL_NULLING)
      const float original_pos = (axis == X_AXIS) ? trajectory.x[batch_idx] : trajectory.y[batch_idx];
    #endif

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
        SERIAL_ECHOLNPGM(" Position correction: ", correction);
        SERIAL_ECHOLNPGM(" Original pos: ", original_pos);
        SERIAL_ECHOLNPGM(" Corrected pos: ", (axis == X_AXIS) ? trajectory.x[batch_idx] : trajectory.y[batch_idx]);

        // For very small values, multiply by 1000000 to show micrometers
        if (fabsf(correction) < 0.001f) {
          SERIAL_ECHOLNPGM(" Correction (micrometers): ", correction * 1000000.0f);
        }
      }
    #endif
  }
}

// Analytical integration helpers
float ModalNulling::integrate_sin_product(const float omega, const float time) {
  // ∫₀ᵀ sin(ωt) dt = [-cos(ωt)/ω]₀ᵀ = (1 - cos(ωT))/ω
  if (fabsf(omega) < 1e-12f) return 0.0f;
  return (1.0f - cosf(omega * time)) / omega;
}

float ModalNulling::integrate_linear_sin_product(const float omega, const float slope, const float time) {
  // ∫₀ᵀ t*sin(ωt) dt = [sin(ωt)/ω² - t*cos(ωt)/ω]₀ᵀ
  if (fabsf(omega) < 1e-12f) return 0.0f;
  
  const float omega_sq = omega * omega;
  return (sinf(omega * time) - omega * time * cosf(omega * time)) / omega_sq;
}

float ModalNulling::integrate_sin_squared_exp(const float omega, const float beta, const float time) {
  // ∫₀ᵀ sin²(ωt) * exp(-βt) dt
  // Using sin²(x) = (1 - cos(2x))/2
  // = (1/2) * ∫₀ᵀ exp(-βt) dt - (1/2) * ∫₀ᵀ cos(2ωt) * exp(-βt) dt

  if (fabsf(beta) < 1e-12f) return 0.0f;

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
  const float sin_T = sinf(omega * time);
  const float cos_T = cosf(omega * time);

  return (omega / denom) * (1.0f - exp_T * (cos_T + (beta / omega) * sin_T));
}

float ModalNulling::integrate_cos_exp_product(const float omega, const float beta, const float time) {
  // ∫₀ᵀ cos(ωt) * exp(-βt) dt
  // = [exp(-βt) * (-β*cos(ωt) + ω*sin(ωt)) / (β² + ω²)]₀ᵀ

  const float denom = beta * beta + omega * omega;
  if (denom < 1e-12f) return 0.0f;

  const float exp_T = expf(-beta * time);
  const float sin_T = sinf(omega * time);
  const float cos_T = cosf(omega * time);

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

  return kernel.amplitude * (cosf(kernel.phase) * I1 + sinf(kernel.phase) * I2);
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
  const float nulling_effectiveness = fabsf(total_projection) / (fabsf(original_projection) + 1e-15f);

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
