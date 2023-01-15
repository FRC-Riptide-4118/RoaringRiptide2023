// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// --- GENERAL FRC --- //
#include <frc2/command/Command.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/Button.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/fs.h>

// --- SUBSYSTEMS --- //
#include <subsystems/Drive.h>

// --- COMMANDS -- //
#include <commands/DefaultDrive.h>
#include <commands/BalanceDrive.h>
#include <commands/DriveToRamp.h>

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

  void Reset(void);

 private:

  frc::SendableChooser<frc2::Command*> m_auto_chooser;

  fs::path deployDirectory{frc::filesystem::GetDeployDirectory() + "/paths" + "/test.wpilib.json"}; 
  frc::Trajectory test_trajectory{frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string())};

  fs::path deployDirectory2{frc::filesystem::GetDeployDirectory() + "/paths" + "/straight.wpilib.json"}; 
  frc::Trajectory test_trajectory2{frc::TrajectoryUtil::FromPathweaverJson(deployDirectory2.string())};

  frc::Field2d m_field;

  /* ----- SUBSYSTEM DECALARATIONS ----- */

  Drive m_drive;

  /* ----- SIMPLE COMMAND DECALARATIONS ---- */

  frc2::InstantCommand toggle_balance{ [this] { m_drive.ToggleBalance(); }, {&m_drive} };
  frc2::InstantCommand reset_drive{ [this] { m_drive.Reset(); }, {&m_drive} };

  frc2::RunCommand run_to_speed{ [this] { m_drive.VelocityDrive(10000_mps, DriveConstants::Side::left); m_drive.VelocityDrive(10000_mps, DriveConstants::Side::right); } };

  /* ----- AUTO DECLARATIONS ---- */

  frc2::Command* m_autonomousCommand;

  frc2::SequentialCommandGroup balance_auto{

    DriveToRamp(&m_drive),
    BalanceDrive(&m_drive)

  };

  frc2::RamseteCommand ramsete_path_track{

    // trajectory [x]
    test_trajectory2,
    // pose [x]
    [this] { return this->m_drive.GetPose(); },
    // controller
    frc::RamseteController{DriveConstants::kRamseteB, DriveConstants::kRamseteZeta},
    // kinematics
    frc::DifferentialDriveKinematics{DriveConstants::track_width},
    // output [x]
    [this](units::meters_per_second_t left_speed, units::meters_per_second_t right_speed) { this->m_drive.VelocityDrive(left_speed, DriveConstants::Side::left); this->m_drive.VelocityDrive(right_speed, DriveConstants::Side::right); },
    // requirements [x]
    { &this->m_drive }

  };

  /* ------ CONTROLLER/BUTTON DECLARATIONS ---- */

  frc2::CommandXboxController driver_controller{ControllerConstants::driver_controller_port};

  frc2::Trigger x_button = driver_controller.X();
  frc2::Trigger y_button = driver_controller.Y();


  void ConfigureButtonBindings();

};
