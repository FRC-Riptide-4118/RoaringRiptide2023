// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BalanceDrive.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
BalanceDrive::BalanceDrive(Drive* drive)
    : CommandHelper(
          frc2::PIDController(DriveConstants::balance_coefficients.kP, DriveConstants::balance_coefficients.kI, DriveConstants::balance_coefficients.kD),
          // This should return the measurement
          [drive] { 
          
            double angles[3];
            drive->GetTiltAngles(angles);
            return angles[DriveConstants::Axis::y]; 
          
          },
          // This should return the setpoint (can also be a constant)
          [] { return DriveConstants::balanced_angle; },
          // This uses the output
          [drive] (double output) {
          
            drive->ArcadeDrive(-output, 0);

          }) {

            AddRequirements({drive});

          }

// Returns true when the command should end.
bool BalanceDrive::IsFinished() {

  // this command will remain scheduled until manually stopped
  // this should help ensure that position is maintained without needing to re-click or hold a button
  return false;

}