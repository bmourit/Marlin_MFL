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

#pragma once

/**
 * modal_nulling.h - Modal-Orthogonal Nulling for FT_Motion
 *
 * This feature implements real-time modal nulling that works exclusively with FT_Motion
 * to cancel resonance excitation by ensuring trajectory orthogonality to dominant modes.
 *
 * Designed to be an alternative improvement to the patented filtered B-Splines approach.
 *
 * Theory:
 * For a second-order system H(s) = ωₙ²/(s² + 2ζωₙs + ωₙ²)
 * We minimize modal projection: E_mode = ∫ x(t) * sin(ω₀t) dt = 0
 *
 * The correction kernel: k(t) = a * sin(ω₀t + φ) * exp(-βt)
 * Results in: x'(t) = x(t) + k(t) with zero modal excitation
 */

#include "../inc/MarlinConfigPre.h"

#if ENABLED(MODAL_NULLING)

#include "../core/types.h"
#include "ft_types.h"

#if DISABLED(HIGH_PRECISION_KERNELS)

  // x in radians, expected in range [0, PI]
  // Very fast and surprisingly accurate
  static float fast_sin_pi_half(float x) {
    return (16.0f * x * (M_PI - x)) / (5.0f * M_PI * M_PI - 4.0f * x * (M_PI - x));
  }

  static float sin_approx(float x) {
    // Wrap x to [0, 2π]
    while (x < 0.0f)      x += 2.0f * M_PI;
    while (x > 2.0f * M_PI) x -= 2.0f * M_PI;

    // Reflect into [0, PI]
    bool negate = false;
    if (x > M_PI) {
      x -= M_PI;
      negate = true;
    }

    float s = fast_sin_pi_half(x);
    return negate ? -s : s;
  }

  static float cos_approx(float x) {
    return sin_approx(x + M_PI_2);
  }

#endif // DISABLED(HIGH_PRECISION_KERNELS)

#if ENABLED(HIGH_PRECISION_KERNELS)
  #define SIN(x)  sinf(x)
  #define COS(x)  cosf(x)
#else
  #define SIN(x)  sin_approx(x)
  #define COS(x)  cos_approx(x)
#endif

typedef struct {
  bool enabled = MODAL_NULLING_DEFAULT_ENABLED;
  float freq[XY] = { MODAL_NULLING_DEFAULT_FREQ_X, MODAL_NULLING_DEFAULT_FREQ_Y };          // Resonant frequencies ω₀ (Hz)
  float damping[XY] = { MODAL_NULLING_DEFAULT_DAMPING_X, MODAL_NULLING_DEFAULT_DAMPING_Y }; // Damping ratios ζ
  float beta[XY] = { MODAL_NULLING_DEFAULT_BETA_X, MODAL_NULLING_DEFAULT_BETA_Y };          // Kernel decay rates β
} mn_config_t;

// Modal nulling kernel parameters
typedef struct {
  float amplitude;    // Kernel amplitude 'a'
  float phase;        // Kernel phase 'φ'
  float decay;        // Kernel decay 'β'
  float frequency;    // Target frequency ω₀
  bool valid;         // Kernel validity flag
} modal_kernel_t;

// Motion segment analysis for modal nulling
typedef struct {
  bool has_motion[XY];           // Motion flags for each axis
  float segment_time;            // Total segment duration
  float accel_time;              // Acceleration phase duration
  float coast_time;              // Coasting phase duration  
  float decel_time;              // Deceleration phase duration
  float start_speed;             // Initial speed
  float peak_speed;              // Peak/nominal speed
  float end_speed;               // Final speed
  float acceleration;            // Acceleration magnitude
  float deceleration;            // Deceleration magnitude
  modal_kernel_t kernels[XY];    // Pre-computed kernels for each axis
  bool valid;                    // Analysis validity flag
} motion_segment_t;

class ModalNulling {
public:
  // Public variables
  static mn_config_t cfg;

  // Public methods
  static void init();
  static void reset();
  static void set_defaults();

  // FT_Motion integration - trajectory modification
  static void analyze_motion_segment(const float start_speed, const float peak_speed, const float end_speed,
                                     const float acceleration, const float deceleration, const float segment_time,
                                     const float accel_time, const float coast_time, const float decel_time,
                                     const bool has_x_motion, const bool has_y_motion);

  static void apply_trajectory_correction(const uint32_t trajectory_idx, const uint32_t batch_idx,
                                          const float segment_time, xyze_trajectory_t& trajectory);

private:
  // Current motion segment analysis
  static motion_segment_t current_segment;

  // Core modal nulling functions
  static bool should_apply_nulling(const bool has_x_motion, const bool has_y_motion,
                                   const float segment_time, const float peak_speed);

  // Modal projection calculations
  static float compute_modal_projection(const float freq, const float start_speed, const float peak_speed,
                                        const float end_speed, const float acceleration, float deceleration,
                                        const float accel_time, const float coast_time, const float decel_time);

  #if ANY(S_CURVE_ACCELERATION, HAS_JUNCTION_DEVIATION)

    static float compute_enhanced_modal_projection(const float freq, const float start_speed, const float peak_speed,
                                                   const float end_speed, const float acceleration, const float deceleration,
                                                   const float accel_time, const float coast_time, const float decel_time);

    #if ENABLED(S_CURVE_ACCELERATION)
      static float compute_s_curve_accel_projection(const float freq, const float start_speed, const float peak_speed,
                                                    const float acceleration, const float accel_time);
      static float compute_s_curve_decel_projection(const float freq, const float peak_speed, const float end_speed,
                                                    const float deceleration, const float decel_time);
      static float compute_cubic_velocity_projection(const float freq, const float v0, const float cubic_coeff,
                                                     const float duration, const float time_offset);
      static float compute_cubic_decel_projection(const float freq, const float v0, const float linear_coeff, const float cubic_coeff,
                                                  const float duration, const float time_offset);
      static float compute_linear_accel_projection(const float freq, const float v0, const float acceleration,
                                                   const float duration, const float time_offset);
      static float compute_linear_decel_projection(const float freq, const float v0, const float deceleration,
                                                   const float duration, const float time_offset);
    #endif

    #if HAS_JUNCTION_DEVIATION

      static float apply_junction_deviation_correction(const float base_projection, const float freq,
                                                       const float start_speed, const float peak_speed, const float end_speed,
                                                       const float accel_time, const float coast_time, const float decel_time);
      static float compute_junction_resonance_projection(const float freq, const float junction_time, const float peak_speed);
      static float compute_junction_velocity_smoothing(const float freq, const float start_speed, const float peak_speed,
                                                       const float end_speed, const float junction_time, const float junction_radius);
      static float get_junction_deviation_setting();
      static float calculate_junction_angle(const xyze_float_t& prev_unit_vec, const xyze_float_t& curr_unit_vec);

    #endif

  #endif // S_CURVE_ACCELERATION || HAS_JUNCTION_DEVIATION

  // Kernel computation and optimization
  static modal_kernel_t compute_nulling_kernel(const float freq, const float damping, const float beta,
                                               const float modal_projection, const float segment_time);

  // Analytical modal projection for different motion phases
  static float compute_accel_phase_projection(const float freq, const float start_speed,
                                              const float acceleration, const float accel_time);
  static float compute_coast_phase_projection(const float freq, const float coast_speed, const float coast_time);
  static float compute_decel_phase_projection(const float freq, const float peak_speed,
                                              const float deceleration, const float decel_time);

  // Kernel application to trajectory points
  static float evaluate_kernel(const modal_kernel_t& kernel, const float time);
  static float evaluate_position_correction_from_kernel(const modal_kernel_t& kernel, const float time);
  static void apply_kernel_to_trajectory_point(const modal_kernel_t& kernel, const float time,
                                               const AxisEnum axis, const uint32_t batch_idx,
                                               xyze_trajectory_t& trajectory);

  // Analytical integration helpers
  static float integrate_sin_product(const float omega, const float time);
  static float integrate_linear_sin_product(const float omega, const float slope, const float time);
  static float integrate_kernel_sin_product(const modal_kernel_t& kernel, const float time);
  static float integrate_sin_squared_exp(const float omega, const float beta, const float time);
  static float integrate_sin_cos_exp(const float omega, const float beta, const float time);
  static float integrate_sin_exp_product(const float omega, const float beta, const float time);
  static float integrate_cos_exp_product(const float omega, const float beta, const float time);

  // Validation and debugging
  static bool validate_kernel(const modal_kernel_t& kernel, const float original_projection,
                              const float segment_time);
  static float compute_residual_projection(const modal_kernel_t& kernel, const float original_projection,
                                           const float segment_time);

  #if ENABLED(DEBUG_MODAL_NULLING)
    static void debug_print_segment_analysis(const motion_segment_t& segment);
    static void debug_print_kernel(const modal_kernel_t& kernel, const char* axis_name);
  #endif
};

extern ModalNulling modalNulling;

#endif // MODAL_NULLING && FT_MOTION
