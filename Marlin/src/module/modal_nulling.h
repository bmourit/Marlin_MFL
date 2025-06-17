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
 * modal_nulling.h - Modal-Orthogonal Nulling Kernel
 *
 * This feature implements a real-time, orthogonal model-based modifier
 * that predicts and cancels modal excitations before they occur -
 * using closed-form modal nulling kernels that adapt to move shapes
 *
 * Designed as an alternative to, and an improvement over, filtered B-Splines.
 */

#include "../inc/MarlinConfigPre.h"
#include "../module/planner.h"
#include "../core/types.h"

typedef struct {
  bool enabled = MODAL_NULLING_DEFAULT_ENABLED;
  float freq[XY] = { MODAL_NULLING_DEFAULT_FREQ_X, MODAL_NULLING_DEFAULT_FREQ_Y };          // Resonant frequencies for X and Y axes (Hz)
  float damping[XY] = { MODAL_NULLING_DEFAULT_DAMPING_X, MODAL_NULLING_DEFAULT_DAMPING_Y }; // Damping factors for X and Y axes (dimensionless)
} mn_config_t;

class ModalNulling {
public:
  // Public variables
  static omn_config_t cfg;

  // Public methods
  static void init();
  static void reset();
  static void set_defaults();

  // Apply modal nulling to a block
  static void modify_trapezoid_parameters(block_t* const block, uint32_t& initial_rate,
                                          uint32_t& final_rate, int32_t& accelerate_steps,
                                          int32_t& decelerate_steps, int32_t& plateau_steps,
                                          const float inverse_accel);

  #if ENABLED(S_CURVE_ACCELERATION)
    static void modify_scurve_parameters(block_t* const block, uint32_t& initial_rate,
                                        uint32_t& final_rate, uint32_t& cruise_rate,
                                        uint32_t& acceleration_time, uint32_t& deceleration_time,
                                        uint32_t& acceleration_time_inverse, uint32_t& deceleration_time_inverse);
  #endif

private:
  // Internal methods
  static bool should_modify_block(const block_t* const block);
  static float compute_acceleration_adjustment(block_t* const block, const float initial_speed,
                                               const float final_speed, const float time);
  static float compute_deceleration_adjustment(block_t* const block, const float initial_speed,
                                               const float final_speed, const float time);

  // Analytical computation of modal projection
  static float compute_modal_projection(const float freq, const float damping,
                                        const float accel, const float initial_rate,
                                        const float time);

  // Derivative of modal projection w.r.t. acceleration scaling
  static float compute_modal_projection_derivative(const float freq, const float damping,
                                                   const float accel, const float initial_rate,
                                                   const float time);

  // Compute nulling factor using analytical orthogonality
  static float compute_modal_nulling_factor(const float freq, const float damping,
                                            const float accel, const float initial_rate,
                                            const float time);

  #if ENABLED(S_CURVE_ACCELERATION)
    static float compute_scurve_acceleration_adjustment(block_t* const block, const float initial_speed,
                                                        const float final_speed, const float time);
    static float compute_scurve_deceleration_adjustment(block_t* const block, const float initial_speed,
                                                        const float final_speed, const float time);
    static float compute_modal_nulling_factor_scurve(const float freq, const float damping,
                                                     const float initial_speed, const float final_speed,
                                                     const float time);
    static float compute_modal_projection_scurve(const float freq, const float damping,
                                                const float initial_speed, const float final_speed,
                                                const float time);
  #endif // S_CURVE_ACCELERATION

  // Advanced modal analysis functions
  static float simulate_modal_response_analytical(const float freq, const float damping, 
                                                  const float accel, const float initial_rate,
                                                  const float final_rate, const float time);

  static float compute_optimal_phase_shift(const float freq, const float damping,
                                           const float accel, const float initial_rate,
                                           const float time);

  static float compute_modal_projection_cosine(const float freq, const float damping,
                                               const float accel, const float initial_rate,
                                               const float time);

  // Validation and debugging
  static bool validate_modal_nulling(const float freq, const float damping,
                                     const float original_accel, const float modified_accel,
                                     const float initial_rate, const float time);
};

extern ModalNulling modalNulling;
