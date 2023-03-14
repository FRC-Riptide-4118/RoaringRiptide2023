// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RequireArm.h"

RequireArm::RequireArm(Arm* arm) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
}

// Called when the command is initially scheduled.
void RequireArm::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RequireArm::Execute() {}

// Called once the command ends or is interrupted.
void RequireArm::End(bool interrupted) {}

// Returns true when the command should end.
bool RequireArm::IsFinished() {
  return false;
}
