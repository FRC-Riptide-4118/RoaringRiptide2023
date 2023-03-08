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
  
  frc::Shuffleboard::GetTab("Game").AddNumber("shoulder encoder", [this] { return m_arm.GetRawEncoderPosition(ArmConstants::ArmJoint::shoulder); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("elbow encoder", [this] { return m_arm.GetRawEncoderPosition(ArmConstants::ArmJoint::elbow); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("wrist encoder", [this] { return m_arm.GetRawEncoderPosition(ArmConstants::ArmJoint::wrist); } );
  
  frc::Shuffleboard::GetTab("Game").AddNumber("shoulder limit", [this] { return m_arm.GetLimitSwitch(ArmConstants::ArmJoint::shoulder); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("elbow limit", [this] { return m_arm.GetLimitSwitch(ArmConstants::ArmJoint::elbow); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("wrist limit", [this] { return m_arm.GetLimitSwitch(ArmConstants::ArmJoint::wrist); } );

  frc::Shuffleboard::GetTab("Game").AddNumber("shoulder angle", [this] { return m_arm.CalcAngleRadFromEncoder(ArmConstants::ArmJoint::shoulder); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("elbow angle", [this] { return m_arm.CalcAngleRadFromEncoder(ArmConstants::ArmJoint::elbow); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("wrist angle", [this] { return m_arm.CalcAngleRadFromEncoder(ArmConstants::ArmJoint::wrist); } );

  frc::Shuffleboard::GetTab("Game").AddNumber("arm x", [this] { return arm_controller.GetRightX(); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("arm y", [this] { return arm_controller.GetLeftY(); } );


  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand( DefaultDrive{ &m_drive, [this] {return driver_controller.GetRightX(); }, [this] {return driver_controller.GetLeftY(); } } );
  m_arm.SetDefaultCommand( DriveArm{ &m_arm, [this] {return arm_controller.GetRightX();}, [this] {return arm_controller.GetLeftY();} } );
  m_gripper.SetDefaultCommand( DefaultGripper(&m_gripper) );

  // Configure the button bindings
  ConfigureButtonBindings();
\
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