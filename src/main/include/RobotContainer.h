// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// --- GENERAL FRC --- //
#include <frc2/command/Command.h>
#include <frc/XboxController.h>

// --- SUBSYSTEMS --- //
#include <subsystems/Drive.h>

// --- COMMANDS -- //
#include <commands/DefaultDrive.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  
  frc::XboxController driver_controller{ControllerConstants::driver_controller_port};

  frc2::Command* m_autonomousCommand;

  Drive m_drive;

  void ConfigureButtonBindings();
};
