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

/**
 * scara.cpp
 */

#include "../inc/MarlinConfig.h"

#if IS_SCARA

#include "scara.h"
#include "motion.h"
#include "planner.h"

float delta_segments_per_second = SCARA_SEGMENTS_PER_SECOND;

void scara_set_axis_is_at_home(const AxisEnum axis) {
  if (axis == Z_AXIS)
    current_position.z = Z_HOME_POS;
  else {

    /**
     * SCARA homes XY at the same time
     */
    xyz_pos_t homeposition;
    LOOP_XYZ(i) homeposition[i] = base_home_pos((AxisEnum)i);

    #if ENABLED(MORGAN_SCARA)
      // MORGAN_SCARA uses arm angles for AB home position
      // SERIAL_ECHOLNPAIR("homeposition A:", homeposition.a, " B:", homeposition.b);
      inverse_kinematics(homeposition);
      forward_kinematics_SCARA(delta.a, delta.b);
      current_position[axis] = cartes[axis];
    #else
      // MP_SCARA uses a Cartesian XY home position
      // SERIAL_ECHOPGM("homeposition");
      // SERIAL_ECHOLNPAIR_P(SP_X_LBL, homeposition.x, SP_Y_LBL, homeposition.y);
      current_position[axis] = homeposition[axis];
    #endif

    // SERIAL_ECHOPGM("Cartesian");
    // SERIAL_ECHOLNPAIR_P(SP_X_LBL, current_position.x, SP_Y_LBL, current_position.y);
    update_software_endstops(axis);
  }
}

static constexpr xy_pos_t scara_offset = { SCARA_OFFSET_X, SCARA_OFFSET_Y };

/**
 * Morgan SCARA Forward Kinematics. Results in 'cartes'.
 * Maths and first version by QHARLEY.
 * Integrated into Marlin and slightly restructured by Joachim Cerny.
 */
void forward_kinematics_SCARA(const float &a, const float &b) {

  const float a_sin = sin(RADIANS(a)) * L1,
              a_cos = cos(RADIANS(a)) * L1,
              b_sin = sin(RADIANS(b)) * L2,
              b_cos = cos(RADIANS(b)) * L2;

  cartes.set(a_cos + b_cos + scara_offset.x,  // theta
             a_sin + b_sin + scara_offset.y); // theta+phi

  /*
    SERIAL_ECHOLNPAIR(
      "SCARA FK Angle a=", a,
      " b=", b,
      " a_sin=", a_sin,
      " a_cos=", a_cos,
      " b_sin=", b_sin,
      " b_cos=", b_cos
    );
    SERIAL_ECHOLNPAIR(" cartes (X,Y) = "(cartes.x, ", ", cartes.y, ")");
  //*/
}
#if ENABLED(mWorkProtocol)
/**
 * Job step theta and psi scara
*/
void jogStepScara(const xyz_pos_t &raw){
  float x_tam =planner.get_axis_position_degrees(A_AXIS), y_tam=planner.get_axis_position_degrees(B_AXIS);
  delta.set(x_tam + raw.x, y_tam + raw.y, raw.z);
  forward_kinematics_SCARA(x_tam + raw.x,y_tam + raw.y);
  destination.x=cartes.x;
  destination.y=cartes.y;
  #if ENABLED(mWorkDEBUGProtocol)
    SERIAL_ECHOPAIR("XPOS:", x_tam );
    SERIAL_ECHOPAIR(" XSTEP:", raw.x);
    SERIAL_ECHOPAIR(" XNEW:", x_tam + raw.x);
    SERIAL_ECHOPAIR(" YPOS:", y_tam );
    SERIAL_ECHOPAIR(" YSTEP:", raw.y);
    SERIAL_ECHOPAIR(" YNEW:", y_tam + raw.y);
    SERIAL_CHAR("\n");
 #endif
}
#endif
void inverse_kinematics(const xyz_pos_t &raw) {

  #if ENABLED(MORGAN_SCARA)
    /**
     * Morgan SCARA Inverse Kinematics. Results in 'delta'.
     *
     * See http://forums.reprap.org/read.php?185,283327
     *
     * Maths and first version by QHARLEY.
     * Integrated into Marlin and slightly restructured by Joachim Cerny.
     */
		/*
    */
	float C2, S2, SK1, SK2, THETA, PSI;
	const xy_pos_t spos = raw - scara_offset;
  #if ENABLED(mWorkDEBUGProtocol)
    SERIAL_ECHOPAIR("SCARA ", spos.x);
    SERIAL_ECHOPAIR(":", spos.y);
    SERIAL_CHAR("\n");
  #endif //mWorkDEBUGProtocol
  if ((raw.x >0)&&(raw.y<0)){
    const float H2 = HYPOT2(spos.x, spos.y);
    float E = -1 *acos((H2 - L1_2 - L2_2) / (2*L1*L2));
    float Q= -1 *(acos((H2 + L1_2 - L2_2) / (2*L1*sqrt(H2))));
    float S = atan2(spos.y,spos.x) - Q;
    THETA = S;
    PSI = E;
  }
  else{
    const float H2 = HYPOT2(spos.x, spos.y);
    float E = acos((H2 - L1_2 - L2_2) / (2*L1*L2));
    float Q= (acos((H2 + L1_2 - L2_2) / (2*L1*sqrt(H2))));
    float S = atan2(spos.y,spos.x) - Q;
    THETA = S;
    PSI = E;
  }
  double doX=DEGREES(THETA);
  double doY=DEGREES(PSI);
  
	delta.set(doX, doY + doX, raw.z);
  #if ENABLED(mWorkDEBUGProtocol)
    SERIAL_ECHOPAIR("SCARA ", doX);
    SERIAL_ECHOPAIR(":", doY);
    SERIAL_CHAR("\n");
  #endif //mWorkDEBUGProtocol
  #else // MP_SCARA

    const float x = raw.x, y = raw.y, c = HYPOT(x, y),
                THETA3 = ATAN2(y, x),
                THETA1 = THETA3 + ACOS((sq(c) + sq(L1) - sq(L2)) / (2.0f * c * L1)),
                THETA2 = THETA3 - ACOS((sq(c) + sq(L2) - sq(L1)) / (2.0f * c * L2));

    delta.set(DEGREES(THETA1), DEGREES(THETA2), raw.z);

    /*
      DEBUG_POS("SCARA IK", raw);
      DEBUG_POS("SCARA IK", delta);
      SERIAL_ECHOLNPAIR("  SCARA (x,y) ", x, ",", y," Theta1=", THETA1, " Theta2=", THETA2);
    //*/

  #endif // MP_SCARA
}

void scara_report_positions() {
  SERIAL_ECHOLNPAIR("SCARA Theta:", planner.get_axis_position_degrees(A_AXIS), "  Psi+Theta:", planner.get_axis_position_degrees(B_AXIS));
  SERIAL_EOL();
}

#endif // IS_SCARA
