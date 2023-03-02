// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DefaultArm.h"

DefaultArm::DefaultArm(Arm* arm) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->m_arm = arm;
  AddRequirements( {arm} );

}

// Called when the command is initially scheduled.
void DefaultArm::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DefaultArm::Execute() {

  this->m_arm->RunJointToPower(ArmConstants::ArmJoint::shoulder, 0);
  this->m_arm->RunJointToPower(ArmConstants::ArmJoint::elbow, 0);
  this->m_arm->RunJointToPower(ArmConstants::ArmJoint::wrist, 0);

}

// Called once the command ends or is interrupted.
void DefaultArm::End(bool interrupted) {}

// Returns true when the command should end.
bool DefaultArm::IsFinished() {
  return false;
}
