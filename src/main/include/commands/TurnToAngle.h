// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>
#include <subsystems/Drive.h>

class TurnToAngle
    : public frc2::CommandHelper<frc2::PIDCommand, TurnToAngle> {
 public:
  TurnToAngle(Drive* drive, double setpoint);

  bool IsFinished() override;

 private:
  Drive* m_drive;
  double setpoint;

};
