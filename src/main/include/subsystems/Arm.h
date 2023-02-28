// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>

#include <frc2/command/SubsystemBase.h>

class Arm : public frc2::SubsystemBase {
 public:
  Arm();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  WPI_TalonSRX shoulder_left_motor = {ArmConstants::shoulder_left_motor_id};
  WPI_VictorSPX shoulder_right_motor = {ArmConstants::shoulder_right_motor_id};
  WPI_TalonFX elbow_left_motor = {ArmConstants::elbow_left_motor_id};
  WPI_TalonFX elbow_right_motor = {ArmConstants::elbow_right_motor_id};
  WPI_TalonSRX wrist_motor = {ArmConstants::wrist_motor_id};

  frc::DigitalInput shoulder_limit{ArmConstants::shoulder_limit_port};
  frc::DigitalInput elbow_limit{ArmConstants::elbow_limit_port};
  frc::DigitalInput wrist_limit{ArmConstants::wrist_limit_port};

};

/*
    const int shoulder_left_motor_id = 7;
    const int shoulder_right_motor_id = 8;
    const int elbow_left_motor_id = 9;
    const int elbow_right_motor_id = 10;
    const int wrist_motor_id = 11;

    const int shoulder_limit_port = 0;
    const int elbow_limit_port = 1;
    const int wrist_limit_port = 2;
*/