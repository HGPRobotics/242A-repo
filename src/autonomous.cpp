#include "main.h"

// Motor ports
#define DRIVE_LEFT_PORT 1
#define DRIVE_RIGHT_PORT 2

// Other defines
#define PI 3.14159265

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

 void move (int goalInches, int power, pros::Motor driveLeft, pros::Motor driveRight) {
   const double kp = 0.25;
   const double circumference = 4*PI;
   double goalRotations = (goalInches/circumference);

   // Rotate the motors for x rotations at y power.
   driveLeft.move_relative(goalRotations, power);
   driveRight.move_relative(goalRotations, power);

   while(!((driveLeft.get_position() < goalRotations+5) && (driveLeft.get_position() > goalRotations-5))) {
     pros::delay(2);
   }
 }

void moveWithDriveStraight (int goalInches, int power, pros::Motor driveLeft, pros::Motor driveRight) {
  const double kS = 0.25;
  const double circumference = 4*PI;
  double goalRotations = (goalInches/circumference);

  // Rotate the motors for x rotations at y power.
  driveLeft.move_relative(goalRotations, power);
  driveRight.move_relative(goalRotations, power);

  while(!((driveLeft.get_position() < goalRotations+5) && (driveLeft.get_position() > goalRotations-5))) {
    double master = driveLeft.get_position();
    double slave = driveRight.get_position();

    if(master != slave) {
      double error = master - slave;

      driveRight.modify_profiled_velocity((driveRight.get_actual_velocity()+(error*kS)));
    }

    pros::delay(2);
  }
}

void moveWithDriveStraightAndPControl (int goalInches, int power, pros::Motor driveLeft, pros::Motor driveRight) {
  const double kS = 0.25; // Constant for stright driving
  const double kP = 0.25; // Constant for P Control
  const double circumference = 4*PI;
  double goalRotations = (goalInches/circumference); // Setpoint

  // Rotate the motors for x rotations at y power.
  driveLeft.move_relative(goalRotations, power);
  driveRight.move_relative(goalRotations, power);

  while(!((driveLeft.get_position() < goalRotations+5) && (driveLeft.get_position() > goalRotations-5))) {
    double master = driveLeft.get_position();
    double slave = driveRight.get_position();

    if(master != slave) {
      double error = master - slave;

      driveRight.modify_profiled_velocity((driveRight.get_actual_velocity()+(error*kS)));
    }

    if(master != goalRotations) {
      double error = goalRotations - master;
      double newPower = error*kP;

      driveLeft.modify_profiled_velocity(newPower);
      driveRight.modify_profiled_velocity(newPower);
    }

    pros::delay(2);
  }
}

void moveWithDriveStraightAndPDControl (int goalInches, int power, pros::Motor driveLeft, pros::Motor driveRight) {
  const double kS = 0.25; // Constant for stright driving
  const double kP = 0.25; // Constant for P Control
  const double kD = 0.25; // Constant for D Control
  const double circumference = 4*PI;
  double preError;
  double goalRotations = (goalInches/circumference); // Setpoint

  // Rotate the motors for x rotations at y power.
  driveLeft.move_relative(goalRotations, power);
  driveRight.move_relative(goalRotations, power);

  while(!((driveLeft.get_position() < goalRotations+5) && (driveLeft.get_position() > goalRotations-5))) {
    double master = driveLeft.get_position();
    double slave = driveRight.get_position();

    if(master != slave) {
      double error = master - slave;

      driveRight.modify_profiled_velocity((driveRight.get_actual_velocity()+(error*kS)));
    }

    if(master != goalRotations) {
      double error = goalRotations - master;
      double derivative = error - preError;
      preError = error;

      double newPower = (error*kP) + (derivative*kD);

      driveLeft.modify_profiled_velocity(newPower);
      driveRight.modify_profiled_velocity(newPower);
    }

    pros::delay(2);
  }
}

void autonomous() {
  // Setup motors
  pros::Motor drive_left (DRIVE_LEFT_PORT);
  pros::Motor drive_right (DRIVE_RIGHT_PORT);

  
}
