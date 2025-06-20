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

#include "../../../inc/MarlinConfig.h"

#if ENABLED(MODAL_NULLING)

#include "../../gcode.h"
#include "../../../module/modal_nulling.h"

void GcodeSuite::M957_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading_etc(forReplay, F("Modal Nulling"));
  SERIAL_ECHOLNPGM(" M957 S", modalNulling.cfg.enabled ? 1 : 0);
  SERIAL_ECHOLNPGM(" M957 X F", modalNulling.cfg.freq[X_AXIS],
                   " D", modalNulling.cfg.damping[X_AXIS],
                   " B", modalNulling.cfg.beta[X_AXIS]);
  SERIAL_ECHOLNPGM(" M957 Y F", modalNulling.cfg.freq[Y_AXIS],
                   " D", modalNulling.cfg.damping[Y_AXIS],
                   " B", modalNulling.cfg.beta[Y_AXIS]);
}

/**
 * M957: Get or Set Modal Nulling Parameters
 *  S<0|1>   Enable (1) or Disable (0) Modal Nulling
 *  F<freq>  Set the frequency. If axes (X, Y) are not specified, set for all axes.
 *  D<ratio> Set the damping ratio. If axes (X, Y) are not specified, set for all axes.
 *  B<ratio> Set the beta ratio. If axes (X, Y) are not specified, set for all axes.
 *  X        Set the given parameters only for the X axis.
 *  Y        Set the given parameters only for the Y axis.
 */
void GcodeSuite::M957() {
  if (!parser.seen_any()) return M957_report();

  // Parse enable/disable parameter
  if (parser.seen('S')) {
    const bool enable = parser.value_bool();
    modalNulling.cfg.enabled = enable;
  }

  const bool seen_X = parser.seen_test('X'),
             seen_Y = parser.seen_test('Y'),
             for_X = seen_X || (!seen_X && !seen_Y),
             for_Y = seen_Y || (!seen_X && !seen_Y);

  if (parser.seen('F')) {
    const float freq = parser.value_float();
    if (WITHIN(freq, 1.0f, 100.0f)) {
      if (for_X) modalNulling.cfg.freq[X_AXIS] = freq;
      if (for_Y) modalNulling.cfg.freq[Y_AXIS] = freq;
    }
    else {
      SERIAL_ECHO_MSG("?Frequency (F) value out of range (1-100 Hz)");
    }
  }

  if (parser.seen('D')) {
    const float damping = parser.value_float();
    if (WITHIN(damping, 0.01f, 0.99f)) {
      if (for_X) modalNulling.cfg.damping[X_AXIS] = damping;
      if (for_Y) modalNulling.cfg.damping[Y_AXIS] = damping;
    }
    else {
      SERIAL_ECHO_MSG("?Damping (D) value out of range (0.01-0.99)");
    }
  }

  if (parser.seen('B')) {
    const float beta = parser.value_float();
    if (WITHIN(beta, 0.1f, 10.0f)) {
      if (for_X) modalNulling.cfg.beta[X_AXIS] = beta;
      if (for_Y) modalNulling.cfg.beta[Y_AXIS] = beta;
    }
    else {
      SERIAL_ECHO_MSG("?Beta (B) value out of range (0.1-10.0)");
    }
  }
}

#endif // MODAL_NULLING