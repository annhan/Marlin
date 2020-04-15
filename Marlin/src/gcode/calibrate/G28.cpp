/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/stepper.h"
#include "../../module/endstops.h"

#if HOTENDS > 1
  #include "../../module/tool_change.h"
#endif

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(SENSORLESS_HOMING)
  #include "../../feature/tmc_util.h"
#endif

#include "../../module/probe.h"

#if ENABLED(BLTOUCH)
  #include "../../feature/bltouch.h"
#endif

#include "../../lcd/ultralcd.h"

#if HAS_L64XX                         // set L6470 absolute position registers to counts
  #include "../../libs/L64XX/L64XX_Marlin.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"

#if ENABLED(QUICK_HOME)
  #if IS_SCARA
    extern Planner planner;
    static void mWork_Home_EndStop(double _theta,double _psi,feedRate_t feedRate){
      float e_tam = 0;
      #if ENABLED(mWorkDebugGoHome)
        SERIAL_ECHOPAIR("feedRate Go Home :", feedRate );
        SERIAL_CHAR("\n");
      #endif
      uint8_t extruder = 0;
      float mm = 360;
      planner.buffer_segment(_theta, _psi, delta.c, e_tam, feedRate, extruder, mm);
      millis_t time_end = millis() + 1000;
      while (!endstops.checkEndStop()) {
          const millis_t ms = millis();
          if (ELAPSED(ms, time_end)) { // check
            time_end = ms + 1000;
            const float x_tam = planner.get_axis_position_degrees(A_AXIS),
                        y_tam = planner.get_axis_position_degrees(B_AXIS);
            if (NEAR(x_tam, _theta) && NEAR(y_tam, _psi)) break;
          }
          idle();
      }
     #if ENABLED(mWorkDEBUGProtocol)
        SERIAL_CHAR("THOAT KHOI WIhLE LOOP \n");
      #endif
      endstops.validate_homing_move();
    }
    static void mWork_Set_Pos_Frome_angles(double A, double B){
      forward_kinematics_SCARA(A,B);
      current_position.set(cartes.x, cartes.y);
      sync_plan_position();
    }
    static void quick_home_xy() {
      mWork_Set_Pos_Frome_angles(0,0);
      mWork_Home_EndStop(360.0 * X_HOME_DIR,360.0* X_HOME_DIR,homing_feedrate(X_AXIS)); //Move Y 360 angles and wait endstop
      mWork_Set_Pos_Frome_angles(0,0);
      mWork_Home_EndStop( 0 , 360.0* Y_HOME_DIR , homing_feedrate(Y_AXIS)); //Move Y 360 angles and wait endstop
      mWork_Set_Pos_Frome_angles(X_POS_HOME_DEGREE,Y_POS_HOME_DEGREE);
    }
  #else
    static void quick_home_xy() {
      current_position.set(0.0, 0.0);
      #if ENABLED(mWorkDebugGoHome)
        SERIAL_CHAR("QUICK_HOME SET HOME XY\n");
      #endif
      sync_plan_position();
      const int x_axis_home_dir = x_home_dir(active_extruder);
      const float mlx = max_length(X_AXIS),
                  mly = max_length(Y_AXIS),
                  mlratio = mlx > mly ? mly / mlx : mlx / mly,
                  fr_mm_s = _MIN(homing_feedrate(X_AXIS), homing_feedrate(Y_AXIS)) * SQRT(sq(mlratio) + 1.0);
      #if ENABLED(mWorkDebugGoHome)
        SERIAL_ECHOPAIR("max LENGH x :", mlx );
        SERIAL_ECHOPAIR(" Y :", mly );
        SERIAL_CHAR("\n");
      #endif
      do_blocking_move_to_xy(1.5 * mlx * x_axis_home_dir, 1.5 * mly * home_dir(Y_AXIS), fr_mm_s);
      endstops.validate_homing_move();
      current_position.set(0.0, 0.0);
    }
  
  #endif
  #endif // QUICK_HOME



/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 *  O   Home only if position is unknown
 *
 *  Rn  Raise by n mm/inches before homing
 *
 * Cartesian/SCARA parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 */
void GcodeSuite::G28() {
  if (DEBUGGING(LEVELING)) {
    DEBUG_ECHOLNPGM(">>> G28");
    log_machine_info();
  }
  #if ENABLED(MARLIN_DEV_MODE)
    if (parser.seen('S')) {
      LOOP_XYZ(a) set_axis_is_at_home((AxisEnum)a);
      sync_plan_position();
      SERIAL_ECHOLNPGM("Simulated Homing");
      report_current_position();
      if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("<<< G28");
      return;
    }
  #endif

  // Home (O)nly if position is unknown
  if (!homing_needed() && parser.boolval('O')) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("> homing not needed, skip\n<<< G28");
    return;
  }
  // Wait for planner moves to finish!
  planner.synchronize();
  #define HAS_CURRENT_HOME(N) (defined(N##_CURRENT_HOME) && N##_CURRENT_HOME != N##_CURRENT)
  #define HAS_HOMING_CURRENT (HAS_CURRENT_HOME(X) || HAS_CURRENT_HOME(X2) || HAS_CURRENT_HOME(Y) || HAS_CURRENT_HOME(Y2))

  #if HAS_HOMING_CURRENT
    auto debug_current = [](PGM_P const s, const int16_t a, const int16_t b){
      serialprintPGM(s); DEBUG_ECHOLNPAIR(" current: ", a, " -> ", b);
    };
    #if HAS_CURRENT_HOME(X)
      const int16_t tmc_save_current_X = stepperX.getMilliamps();
      stepperX.rms_current(X_CURRENT_HOME);
      if (DEBUGGING(LEVELING)) debug_current(PSTR("X"), tmc_save_current_X, X_CURRENT_HOME);
    #endif
    #if HAS_CURRENT_HOME(X2)
      const int16_t tmc_save_current_X2 = stepperX2.getMilliamps();
      stepperX2.rms_current(X2_CURRENT_HOME);
      if (DEBUGGING(LEVELING)) debug_current(PSTR("X2"), tmc_save_current_X2, X2_CURRENT_HOME);
    #endif
    #if HAS_CURRENT_HOME(Y)
      const int16_t tmc_save_current_Y = stepperY.getMilliamps();
      stepperY.rms_current(Y_CURRENT_HOME);
      if (DEBUGGING(LEVELING)) debug_current(PSTR("Y"), tmc_save_current_Y, Y_CURRENT_HOME);
    #endif
    #if HAS_CURRENT_HOME(Y2)
      const int16_t tmc_save_current_Y2 = stepperY2.getMilliamps();
      stepperY2.rms_current(Y2_CURRENT_HOME);
      if (DEBUGGING(LEVELING)) debug_current(PSTR("Y2"), tmc_save_current_Y2, Y2_CURRENT_HOME);
    #endif
  #endif

  #if ENABLED(IMPROVE_HOMING_RELIABILITY)
    slow_homing_t slow_homing = begin_slow_homing();
  #endif

  // Always home with tool 0 active
  #if HOTENDS > 1
    #if DISABLED(DELTA) || ENABLED(DELTA_HOME_TO_SAFE_ZONE)
      const uint8_t old_tool_index = active_extruder;
    #endif
    tool_change(0, true);
  #endif

  #if HAS_DUPLICATION_MODE
    extruder_duplication_enabled = false;
  #endif

  remember_feedrate_scaling_off();

  endstops.enable(true); // Enable endstops for next homing move

    const bool homeX = parser.seen('X'), homeY = parser.seen('Y'), homeZ = parser.seen('Z'),
               home_all = homeX == homeY && homeX == homeZ, // All or None
               doX = home_all || homeX, doY = home_all || homeY, doZ = home_all || homeZ;

    destination = current_position;

    #if Z_HOME_DIR > 0  // If homing away from BED do Z first

      if (doZ) homeaxis(Z_AXIS);

    #endif

    const float z_homing_height =
      (DISABLED(UNKNOWN_Z_NO_RAISE) || TEST(axis_known_position, Z_AXIS))
        ? (parser.seenval('R') ? parser.value_linear_units() : Z_HOMING_HEIGHT)
        : 0;

    if (z_homing_height && (doX || doY || ENABLED(Z_SAFE_HOMING))) {
      // Raise Z before homing any other axes and z is not already high enough (never lower z)
      destination.z = z_homing_height + (TEST(axis_known_position, Z_AXIS) ? 0.0f : current_position.z);
      if (destination.z > current_position.z) {
        //if (DEBUGGING(LEVELING))
        DEBUG_ECHOLNPAIR("Raise Z (before homing) to ", destination.z);
        do_blocking_move_to_z(destination.z);
      }
    }

    #if ENABLED(QUICK_HOME)

      if (doX && doY) quick_home_xy();

    #endif

    // Home Y (before X)
    if (ENABLED(HOME_Y_BEFORE_X) && (doY || (ENABLED(CODEPENDENT_XY_HOMING) && doX)))
      homeaxis(Y_AXIS);

    // Home X
    if (doX || (doY && ENABLED(CODEPENDENT_XY_HOMING) && DISABLED(HOME_Y_BEFORE_X))) { homeaxis(X_AXIS);}
    // Home Y (after X)
    if (DISABLED(HOME_Y_BEFORE_X) && doY) homeaxis(Y_AXIS);
    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0
      if (doZ) { homeaxis(Z_AXIS);} // doZ
    #endif // Z_HOME_DIR < 0

  sync_plan_position();
  endstops.not_homing();
  restore_feedrate_and_scaling();
  ui.refresh();
  report_current_position();
  if (ENABLED(NANODLP_Z_SYNC) && (doZ || ENABLED(NANODLP_ALL_AXIS)))SERIAL_ECHOLNPGM(STR_Z_MOVE_COMP);
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("<<< G28");
}
