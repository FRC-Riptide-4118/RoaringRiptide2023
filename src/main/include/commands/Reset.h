// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include <subsystems/Drive.h>
#include <subsystems/Arm.h>
#include <subsystems/Gripper.h>

class Reset
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 Reset> {
 public:
  Reset(Drive* drive, Arm* arm, Gripper* gripper);

  void Initialize() override;

 private:
  Drive* m_drive;
  Arm* m_arm;
  Gripper* m_gripper;

};
