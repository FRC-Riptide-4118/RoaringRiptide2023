// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>
#include <Constants.h>

class Arm : public frc2::SubsystemBase {
 public:
  Arm();
  void Periodic() override;
  
  double GetRawEncoderPosition(ArmConstants::ArmJoint arm_joint);
  double GetEncoderRawVelocity(ArmConstants::ArmJoint arm_joint);
  void RunJointToSpeed(ArmConstants::ArmJoint arm_joint, double setpoint);
  void RunJointToPower(ArmConstants::ArmJoint arm_joint, double setpoint);
  void CalcJointVelocities( double omega[3], double angles_rad[3], double tool_velocity[2], double omega_tool );
  double CalcAngleRadFromEncoder(ArmConstants::ArmJoint arm_joint);
  bool GetLimitSwitch(ArmConstants::ArmJoint arm_joint);
  void SetLimitSwitchLatches(ArmConstants::ArmJoint arm_joint, bool* start_latches, bool* max_latches);

 private:
  WPI_TalonSRX shoulder_left_motor = {ArmConstants::shoulder_left_motor_id};
  WPI_VictorSPX shoulder_right_motor = {ArmConstants::shoulder_right_motor_id};
  TalonFX elbow_left_motor = {ArmConstants::elbow_left_motor_id};
  TalonFX elbow_right_motor = {ArmConstants::elbow_right_motor_id};
  WPI_TalonSRX wrist_motor = {ArmConstants::wrist_motor_id};

  frc::DigitalInput shoulder_limit{ArmConstants::shoulder_limit_port};
  frc::DigitalInput elbow_limit{ArmConstants::elbow_limit_port};
  frc::DigitalInput wrist_limit{ArmConstants::wrist_limit_port};

  bool limit_switch_start_latches[3];
  bool limit_switch_max_latches[3];

};