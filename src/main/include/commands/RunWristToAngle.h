// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>
#include <subsystems/Gripper.h>

class RunWristToAngle
    : public frc2::CommandHelper<frc2::PIDCommand, RunWristToAngle> {
 public:
  RunWristToAngle(Gripper* gripper, double setpoint_radian, PID_Coefficients pid_controller);

  bool IsFinished() override;

 private:
  Gripper* m_gripper;
  double setpoint_radian;

};
