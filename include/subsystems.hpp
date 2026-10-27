#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

//--LIFT--
inline pros::Motor l_lift(7);
inline pros::Motor r_lift(-8);

inline ez::PID liftPID{0.45, 0, 0, 0, "Lift"}; //startin guess, tune
inline bool lift_manual = false; //true while a button is driving the lift

//TODO: measure these in encoder degrees (step 8)
inline const double LIFT_FLOOR = 0;
inline const double LIFT_UP_SMALL = 600;
inline const double LIFT_CEILING = 2480; //temporary, measure real value
inline const double LIFT_CEILING_CLAW_OPEN = 360; //claw open AND vert in blocks at about 380

inline double lift_pos() {return (l_lift.get_position() + r_lift.get_position()) / 2.0;}
inline void lift_move(int v) {l_lift.move(v); r_lift.move(v); } //v is -127..127

inline void lift_task() {
  while (true) {
    if (!lift_manual) lift_move(liftPID.compute(lift_pos()));
    pros::delay(ez::util::DELAY_TIME);
  }
}

inline void lift_set(double target) { lift_manual = false; liftPID.target_set(target); }
inline void lift_drop() {          // let the lift fall on coast
    lift_manual = true;              // stop the PID from driving it
    l_lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    r_lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    lift_move(0);
}
inline const double LIFT_PICKUP_CLEAR = 150;   // TODO: "a bit" so the pin doesn't drag
inline void lift_wait(double tolerance = 15, int timeout_ms = 2000) {
  uint32_t start = pros::millis();
  while (fabs(liftPID.target_get() - lift_pos()) > tolerance && pros::millis() - start < timeout_ms)
    pros::delay(10);
}

// ---------- Pistons ----------
inline ez::Piston claw('B');
inline ez::Piston claw_vert('A');

inline const bool CLAW_OPEN = false;           // flip if L1 closes the claw instead
inline const bool CLAW_CLOSED = !CLAW_OPEN;
inline const bool VERT_OUT = true;            // flip if Y pulls the piston in instead
inline const bool VERT_IN = !VERT_OUT;