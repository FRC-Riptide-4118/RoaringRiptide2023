// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <subsystems/Gripper.h>

class DefaultGripper
    : public frc2::CommandHelper<frc2::CommandBase, DefaultGripper> {
 public:
  DefaultGripper(Gripper* m_gripper, std::function<double()> setpoint);

  void Execute() override;

 private:
  std::function<double()> setpoint;
  Gripper* m_gripper;
  
};
