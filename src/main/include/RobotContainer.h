// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// --- GENERAL FRC --- //
#include <frc2/command/Command.h>
#include <frc/XboxController.h>
#include <frc2/command/button/Button.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/InstantCommand.h>

// --- SUBSYSTEMS --- //
#include <subsystems/Drive.h>

// --- COMMANDS -- //
#include <commands/DefaultDrive.h>
#include <commands/BalanceDrive.h>

// --- OTHER --- //

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
  
  /* ----- SUBSYSTEM DECALARATIONS ----- */

  Drive m_drive;


  /* ----- SIMPLE COMMAND DECALARATIONS ---- */

  // [DRIVE]
  frc2::InstantCommand toggle_balance{ [this] { m_drive.ToggleBalance(); }, {&m_drive} };


  /* ----- AUTO DECLARATIONS ---- */

  frc2::Command* m_autonomousCommand;


  /* ------ CONTROLLER/BUTTON DECLARATIONS ---- */

  frc::XboxController driver_controller{ControllerConstants::driver_controller_port};

  frc2::Button a_button{[&] { return driver_controller.GetAButton(); } };
  frc2::Button b_button{[&] { return driver_controller.GetBButton(); } };
  frc2::Button x_button{[&] { return driver_controller.GetXButton(); } };
  frc2::Button y_button{[&] { return driver_controller.GetYButton(); } };

  void ConfigureButtonBindings();

};
