// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// --- GENERAL FRC --- //
#include <frc2/command/Command.h>
<<<<<<< Updated upstream
#include <frc/XboxController.h>
=======
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/Button.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/InstantCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <wpi/fs.h>
>>>>>>> Stashed changes

// --- SUBSYSTEMS --- //
#include <subsystems/Drive.h>

// --- COMMANDS -- //
#include <commands/DefaultDrive.h>
<<<<<<< Updated upstream
=======
#include <commands/BalanceDrive.h>
#include <commands/DriveToRamp.h>

// --- OTHER --- //
>>>>>>> Stashed changes

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
  
<<<<<<< Updated upstream
  frc::XboxController driver_controller{ControllerConstants::driver_controller_port};

  frc2::Command* m_autonomousCommand;

  Drive m_drive;
=======
  frc::SendableChooser<frc2::Command*> m_auto_chooser;

  frc::Trajectory test_trajectory;
  fs::path deployDirectory; 


  /* ----- SUBSYSTEM DECALARATIONS ----- */

  Drive m_drive;


  /* ----- SIMPLE COMMAND DECALARATIONS ---- */

  frc2::InstantCommand toggle_balance{ [this] { m_drive.ToggleBalance(); }, {&m_drive} };

  frc2::RamseteCommand test_path{

    // trajectory
    test_trajectory,
    // pose
    [this] { return m_drive.GetPose(); },
    // controller
    frc::RamseteController{DriveConstants::kRamseteB, DriveConstants::kRamseteZeta},
    // kinematics
    m_drive.GetKinematics(),
    // output
    [this](auto left_speed, auto right_speed) { m_drive.VelocityDrive(left_speed, DriveConstants::Side::left); m_drive.VelocityDrive(right_speed, DriveConstants::Side::right); },
    // requirements
    { &m_drive }

  };

  /* ----- AUTO DECLARATIONS ---- */

  frc2::Command* m_autonomousCommand;

  frc2::SequentialCommandGroup balance_auto{

    DriveToRamp(&m_drive),
    BalanceDrive(&m_drive)

  };

  /* ------ CONTROLLER/BUTTON DECLARATIONS ---- */

  frc2::CommandXboxController driver_controller{ControllerConstants::driver_controller_port};

  frc2::Trigger x_button = driver_controller.X();
>>>>>>> Stashed changes

  void ConfigureButtonBindings();
};
