// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {

  frc::Shuffleboard::GetTab("Game").Add(this->m_auto_chooser);
  this->m_auto_chooser.SetDefaultOption("Balance", &balance_auto);

  this->deployDirectory = frc::filesystem::GetDeployDirectory();
  this->deployDirectory = deployDirectory / "paths" / "test.wpilib.json";
  this->test_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand( DefaultDrive{ &m_drive, [this] {return driver_controller.GetLeftY(); }, [this] {return -driver_controller.GetRightX(); } } );

  frc::Shuffleboard::GetTab("Game").AddNumber("X", [this] { double angles[3]; m_drive.GetTiltAngles(angles); return angles[DriveConstants::Axis::x]; } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Y", [this] { double angles[3]; m_drive.GetTiltAngles(angles); return angles[DriveConstants::Axis::y]; } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Z", [this] { double angles[3]; m_drive.GetTiltAngles(angles); return angles[DriveConstants::Axis::z]; } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Heading", [this] { return m_drive.GetAngle(); } );
  frc::Shuffleboard::GetTab("Game").AddBoolean("Balance", [this] { return m_drive.GetBalanceActive(); } );

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
<<<<<<< Updated upstream
  // Configure your button bindings here
=======

  x_button.OnTrue( &toggle_balance );
  x_button.ToggleOnTrue( BalanceDrive(&m_drive).ToPtr() );

>>>>>>> Stashed changes
}

frc2::Command* RobotContainer::GetAutonomousCommand() {

  // An example command will be run in autonomous
  return this->m_auto_chooser.GetSelected();

}
