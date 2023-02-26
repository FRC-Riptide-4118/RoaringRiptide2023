// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>
#include <Constants.h>

class Gripper : public frc2::SubsystemBase {
 public:
  Gripper();
  void Periodic() override;
  void CubeIn(void);
  void CubeOut(void);
  void Stop(void);
  void ConeGrab(void);
  void ConeRelease(void); 
  void ConeToggle(void);

 private:
  bool gripper_position;
  WPI_VictorSPX gripper_motor = {GripperConstants::gripper_motor_id};
  frc::DoubleSolenoid gripper_grab{frc::PneumaticsModuleType::CTREPCM, GripperConstants::PCM_grip, GripperConstants::PCM_release};

};
