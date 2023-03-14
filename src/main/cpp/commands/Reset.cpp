// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Reset.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Reset::Reset(Drive* drive, Arm* arm, Gripper* gripper) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->m_drive = drive;
  this->m_arm = arm;
  this->m_gripper = gripper;
  AddRequirements({drive, arm, gripper});
}

// Called when the command is initially scheduled.
void Reset::Initialize() {

  this->m_drive->Reset();
  this->m_arm->ResetEncoders();
  this->m_gripper->Reset();

}
