// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveArm.h"
#include <stdio.h>

DriveArm::DriveArm(Arm* arm, std::function<double()> tool_right_y_velocity, std::function<double()> tool_left_y_velocity, std::function<bool()> go_forward, std::function<bool()> go_backwards) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->m_arm = arm;
  this->tool_right_y_velocity = tool_right_y_velocity;
  this->tool_left_y_velocity = tool_left_y_velocity;
  this->go_forward = go_forward;
  this->go_backwards = go_backwards; 
  AddRequirements( {arm} );
}

// Called when the command is initially scheduled.
void DriveArm::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveArm::Execute() {

  /*
  // double tool_velocity[2] = { this->tool_left_y_velocity(), this->tool_right_y_velocity() };
  double tool_velocity[2] = { this->tool_left_y_velocity(), this->tool_right_y_velocity() };
  double angles_rad[3] = { this->m_arm->CalcAngleRadFromEncoder(ArmConstants::ArmJoint::shoulder),
                           this->m_arm->CalcAngleRadFromEncoder(ArmConstants::ArmJoint::elbow),
                           this->m_arm->CalcAngleRadFromEncoder(ArmConstants::ArmJoint::wrist) };
  double omega[3];
  this->m_arm->CalcJointVelocities(omega, angles_rad, tool_velocity, 0);
  printf("%f %f %f\n", ArmConstants::SHOULDER_TICKS_PER_RADIAN*omega[ArmConstants::ArmJoint::shoulder], ArmConstants::ELBOW_TICKS_PER_RADIAN*omega[ArmConstants::ArmJoint::elbow], ArmConstants::WRIST_TICKS_PER_RADIAN*omega[ArmConstants::ArmJoint::wrist]);
  this->m_arm->RunJointToPowerAndHold(ArmConstants::ArmJoint::shoulder, 50*omega[ArmConstants::ArmJoint::shoulder]);
  this->m_arm->RunJointToPowerAndHold(ArmConstants::ArmJoint::elbow, 50*omega[ArmConstants::ArmJoint::elbow]);
  this->m_arm->RunJointToPowerAndHold(ArmConstants::ArmJoint::wrist, 50*omega[ArmConstants::ArmJoint::wrist]);
  */

  this->m_arm->RunJointToPowerAndHold_UNSAFE(ArmConstants::ArmJoint::elbow, this->tool_right_y_velocity());
  // this->m_arm->RunJointToPowerAndHold_UNSAFE(ArmConstants::ArmJoint::wrist, this->tool_left_y_velocity());
  if (this->go_forward())
    this->m_arm->RunJointToPowerAndHold_UNSAFE(ArmConstants::ArmJoint::shoulder, 0.5);
  else if (this->go_backwards())
    this->m_arm->RunJointToPowerAndHold_UNSAFE(ArmConstants::ArmJoint::shoulder, -0.5);
  else
    this->m_arm->RunJointToPowerAndHold_UNSAFE(ArmConstants::ArmJoint::shoulder, 0);

}

// Called once the command ends or is interrupted.
void DriveArm::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveArm::IsFinished() {
  return false;
}
