// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <RobotContainer.h>

RobotContainer::RobotContainer() {

  frc::Shuffleboard::GetTab("Game").Add(this->m_auto_chooser);
  m_auto_chooser.SetDefaultOption("High Drive-Spin", &place_cone_high_drive_spin);
  m_auto_chooser.SetDefaultOption("Mid Drive-Spin", &place_cone_mid_drive_spin);
  m_auto_chooser.SetDefaultOption("High Drive", &place_cone_high_drive);
  m_auto_chooser.SetDefaultOption("Mid Drive", &place_cone_mid_drive);
  m_auto_chooser.SetDefaultOption("Nothing", &cone_grab);

  m_drive.CreateOdometry(0_m, 0_m, test_trajectory.InitialPose());

  m_drive.SetField(&field);
  frc::SmartDashboard::PutData("Field", &field);
  field.GetObject("traj")->SetTrajectory(test_trajectory);
  
  frc::Shuffleboard::GetTab("Game").AddNumber("shoulder angle", [this] { return RAD_2_DEG(m_arm.CalcAngleRadFromEncoder(ArmConstants::ArmJoint::shoulder)); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("elbow angle", [this] { return RAD_2_DEG(m_arm.CalcAngleRadFromEncoder(ArmConstants::ArmJoint::elbow)); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("wrist angle", [this] { return RAD_2_DEG(m_gripper.CalcAngleRadFromEncoder()); } );

  frc::Shuffleboard::GetTab("Game").AddNumber("left pos", [this] { return m_drive.GetRawEncoderPosition(DriveConstants::Side::left); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("right pos", [this] { return m_drive.GetRawEncoderPosition(DriveConstants::Side::right); } );
  frc::Shuffleboard::GetTab("Game").AddNumber("heading", [this] { return m_drive.GetHeading(); } );

  frc::SmartDashboard::PutData("RESET", new Reset(&m_drive, &m_arm, &m_gripper));

  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand( DefaultDrive{ &m_drive, [this] {return driver_controller.GetRightX()*ControllerConstants::X_SCALE; }, [this] {return driver_controller.GetLeftY()*ControllerConstants::Y_SCALE; } } );
  m_arm.SetDefaultCommand( DriveArm{ &m_arm, [this] {return arm_controller.GetRightY()/2;}, [this] {return arm_controller.GetLeftY();}, [this] {return (arm_controller.GetPOV() == 0);}, [this] {return (arm_controller.GetPOV() == 180);} } );
  m_gripper.SetDefaultCommand( DefaultGripper(&m_gripper, [this] {return arm_controller.GetLeftY();}) );

  // Configure the button bindings
  ConfigureButtonBindings();

}

void RobotContainer::ConfigureButtonBindings() {

  // x_button.ToggleOnTrue( BalanceDrive(&m_drive).ToPtr() );
  right_bumper.ToggleOnTrue(&cone_toggle);
  right_trigger.WhileTrue(&cube_in);
  left_trigger.WhileTrue(&cube_out);
  a_button.WhileTrue(&run_to_start);
  y_button.WhileTrue(&run_to_high);
  x_button.WhileTrue(&run_to_low);
  b_button.WhileTrue(&run_to_pickup_floor);
  left_bumper.WhileTrue(&run_to_sideways_pickup);
  driver_a_button.ToggleOnTrue(&toggle_limelight_led);

}

frc2::Command* RobotContainer::GetAutonomousCommand() {

  // An example command will be run in autonomous
  return this->m_auto_chooser.GetSelected();

}