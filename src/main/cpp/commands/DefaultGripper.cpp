// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DefaultGripper.h"

DefaultGripper::DefaultGripper(Gripper* m_gripper, std::function<double()> setpoint) {
  
  this->setpoint = setpoint;
  this->m_gripper = m_gripper;
  AddRequirements({m_gripper});

}


// Called repeatedly when this Command is scheduled to run
void DefaultGripper::Execute() {

  this->m_gripper->Stop();
  this->m_gripper->RunJointToPowerAndHold_UNSAFE(setpoint);

}