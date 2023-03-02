// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Arm.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveArm
    : public frc2::CommandHelper<frc2::CommandBase, DriveArm> {
 public:
  DriveArm(Arm* arm, std::function<double()> tool_x_velocity, std::function<double()> tool_y_velocity);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Arm* m_arm;
  std::function<double()> tool_x_velocity;
  std::function<double()> tool_y_velocity;
  
};
