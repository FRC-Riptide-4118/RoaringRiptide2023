// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <RobotContainer.h>

RobotContainer::RobotContainer() {

  m_drive.CreateOdometry(0_m, 0_m, test_trajectory.InitialPose());

  m_drive.SetField(&field);
  frc::SmartDashboard::PutData("Field", &field);
  field.GetObject("traj")->SetTrajectory(test_trajectory);

  frc::Shuffleboard::GetTab("Game").AddNumber("left velocity", [this] { return m_drive.GetRawEncoderVelocity(DriveConstants::Side::left); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("right velocity", [this] { return m_drive.GetRawEncoderVelocity(DriveConstants::Side::right); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("left position", [this] { return m_drive.GetRawEncoderPosition(DriveConstants::Side::left); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("right position", [this] { return m_drive.GetRawEncoderPosition(DriveConstants::Side::right); } );

  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand( DefaultDrive{ &m_drive, [this] {return driver_controller.GetRightX(); }, [this] {return -driver_controller.GetLeftY(); } } );

  // Configure the button bindings
  ConfigureButtonBindings();

}

void RobotContainer::ConfigureButtonBindings() {

  // x_button.ToggleOnTrue( BalanceDrive(&m_drive).ToPtr() );
  right_bumper.ToggleOnTrue(&cone_toggle);
  right_trigger.WhileTrue(&cube_in);
  b_button.WhileTrue(&cube_out);

}

frc2::Command* RobotContainer::GetAutonomousCommand() {

  // An example command will be run in autonomous
  return this->m_auto_chooser.GetSelected();

}