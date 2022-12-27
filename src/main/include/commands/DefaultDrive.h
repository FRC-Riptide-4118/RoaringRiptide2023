// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// include header guard
#pragma once

// all frc/frc2 includes
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/filter/LinearFilter.h>
// all subsystem includes
#include <subsystems/Drive.h>

// DefaultDrive will run automatically when Tele-Op mode is running
class DefaultDrive : public frc2::CommandHelper<frc2::CommandBase, DefaultDrive> {
 
 public:
  // DefaultDrive constructor takes a pointer to a Drive subsystem and pointers to two functions that will return doubles representing a forward/rotate power
  DefaultDrive(Drive* subsystem, std::function<double()> forward, std::function<double()> rotate);
  // Execute will be called when the command is called
  void Execute() override;

 private:
  // The pointer to the Drive subsystem
  Drive* m_drive;
  // The pointer to the function returning the forward power
  std::function<double()> m_forward;
  // The pointer to the function returning the rotate power
  std::function<double()> m_rotate;

};