// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>
#include <subsystems/Arm.h>

class RunJointToAngle
    : public frc2::CommandHelper<frc2::PIDCommand, RunJointToAngle> {
 public:
  RunJointToAngle(Arm* arm, ArmConstants::ArmJoint arm_joint, double setpoint_radian, PID_Coefficients pid_controller);

  bool IsFinished() override;

 private:
  Arm* m_arm;
  ArmConstants::ArmJoint arm_joint;
  double setpoint_radian;

};
