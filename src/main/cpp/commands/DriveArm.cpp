// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveArm.h"

DriveArm::DriveArm(Arm* arm, std::function<double()> tool_x_velocity, std::function<double()> tool_y_velocity) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->m_arm = arm;
  this->tool_x_velocity = this->tool_x_velocity;
  this->tool_y_velocity = this->tool_y_velocity; 
  AddRequirements( {arm} );
}

// Called when the command is initially scheduled.
void DriveArm::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveArm::Execute() {

  double tool_velocity[2] = { this->tool_x_velocity(), this->tool_y_velocity() };
  double angles_rad[3] = { this->m_arm->CalcAngleRadFromEncoder(ArmConstants::ArmJoint::shoulder),
                           this->m_arm->CalcAngleRadFromEncoder(ArmConstants::ArmJoint::elbow),
                           this->m_arm->CalcAngleRadFromEncoder(ArmConstants::ArmJoint::wrist), };
  double omega[3];
  this->m_arm->CalcJointVelocities(omega, angles_rad, tool_velocity, 0);
  this->m_arm->RunJointToSpeed(ArmConstants::ArmJoint::shoulder, omega[ArmConstants::ArmJoint::shoulder]);
  this->m_arm->RunJointToSpeed(ArmConstants::ArmJoint::elbow, omega[ArmConstants::ArmJoint::elbow]);
  this->m_arm->RunJointToSpeed(ArmConstants::ArmJoint::wrist, omega[ArmConstants::ArmJoint::wrist]);

}

// Called once the command ends or is interrupted.
void DriveArm::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveArm::IsFinished() {
  return false;
}
