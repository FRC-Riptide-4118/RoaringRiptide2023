// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveToRamp.h"

DriveToRamp::DriveToRamp(Drive* drive) {
  // Use addRequirements() here to declare subsystem dependencies.

  this->drive = drive;
  AddRequirements( {drive} );

}

// Called when the command is initially scheduled.
void DriveToRamp::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveToRamp::Execute() {

  this->drive->ArcadeDrive(0.5, 0);

}

// Called once the command ends or is interrupted.
void DriveToRamp::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveToRamp::IsFinished() {

  double angles[3];
  this->drive->GetTiltAngles(angles);
  return angles[DriveConstants::Axis::y] < DriveConstants::exceed_angle;

}
