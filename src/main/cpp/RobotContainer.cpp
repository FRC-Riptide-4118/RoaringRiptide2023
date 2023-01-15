// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <RobotContainer.h>

RobotContainer::RobotContainer() {

  this->m_drive.CreateOdomoetry( 0_m, 0_m, this->test_trajectory2.InitialPose() );

  frc::SmartDashboard::PutData("Field", &this->m_field);
  this->m_field.GetObject("traj")->SetTrajectory(this->test_trajectory2);
  this->m_drive.SetField(&this->m_field);

  frc::Shuffleboard::GetTab("Game").Add(this->m_auto_chooser);
  this->m_auto_chooser.SetDefaultOption("Balance", &balance_auto);
  this->m_auto_chooser.AddOption("Trajectory", &ramsete_path_track);

  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand( DefaultDrive{ &m_drive, [this] {return driver_controller.GetLeftY(); }, [this] {return -driver_controller.GetRightX(); } } );

  frc::Shuffleboard::GetTab("Game").AddNumber("X", [this] { double angles[3]; m_drive.GetTiltAngles(angles); return angles[DriveConstants::Axis::x]; } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Y", [this] { double angles[3]; m_drive.GetTiltAngles(angles); return angles[DriveConstants::Axis::y]; } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Z", [this] { double angles[3]; m_drive.GetTiltAngles(angles); return angles[DriveConstants::Axis::z]; } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Heading", [this] { return m_drive.GetAngle(); } );
  frc::Shuffleboard::GetTab("Game").AddBoolean("Balance", [this] { return m_drive.GetBalanceActive(); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Left Speed", [this] { return m_drive.GetVelocity(DriveConstants::Side::left).value(); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Right Speed", [this] { return m_drive.GetVelocity(DriveConstants::Side::right).value(); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Xpos", [this] { return m_drive.GetPose().X().value(); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Ypos", [this] { return m_drive.GetPose().Y().value(); } );
  frc::Shuffleboard::GetTab("Game").AddString("Path Path", [this] { return this->deployDirectory.string(); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Xinitial", [this] { return this->test_trajectory.InitialPose().X().value(); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Yinitial", [this] { return this->test_trajectory.InitialPose().Y().value(); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Left Position", [this] { return this->m_drive.GetPosition(DriveConstants::Side::left).value(); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("Right Position", [this] { return this->m_drive.GetPosition(DriveConstants::Side::right).value(); } );

  // Configure the button bindings
  ConfigureButtonBindings();

}

void RobotContainer::ConfigureButtonBindings() {

  // x_button.ToggleOnTrue( BalanceDrive(&m_drive).ToPtr() );

}

frc2::Command* RobotContainer::GetAutonomousCommand() {

  // An example command will be run in autonomous
  return this->m_auto_chooser.GetSelected();

}

void RobotContainer::Reset(void) {

  this->m_drive.CreateOdomoetry( 0_m, 0_m, this->test_trajectory2.InitialPose() );

}