// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// header include guard
#pragma once

// all frc/frc2 includes
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/filter/LinearFilter.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/shuffleboard/Shuffleboard.h>
// CTRE/Phoenix include
#include "ctre/Phoenix.h"
// all other includes
#include "Constants.h"

// The Drive subsystem will represent an abstraction of the drivebase to allow for general robot mobility
class Drive : public frc2::SubsystemBase {
 
 public:
  // Drive constructor takes no arguments
  Drive();
  // Periodic method will not run anything
  void Periodic() override;
  // Curvature drive implementation
  void CurvatureDrive(double forward, double rotate);
  // Arcade drive implementation
  void ArcadeDrive(double forward, double rotate);
  // ResetEncoder will set both left and right encoders to 0
  void ResetEncoder();
  // Get the current (filtered) velocity
  double GetVelocity();
  // Get the current left talon encoder count
  double GetPosition();
  // Reset the gyroscope angle
  void ResetAngle();
  // Get the current gyroscope angle
  double GetAngle();
  // Get the current (unfiltered) velocity
  double GetUnfilteredVelocity();
  // Get wheel speeds from drive kinematics definition (convert linear + angular velocity of chassis into left + right velocity)
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds(frc::ChassisSpeeds chs_spd);
  // DriveToDistance will use PID control and encoders to drive a specific distance in a straight line
  void DriveToDistance(double setpoint);

 private:
  // All of the left motor controllers are defined here
  WPI_TalonSRX left_talon = {DriveConstants::left_talon_id};
  WPI_VictorSPX left_victor1 = {DriveConstants::left_victor1_id};
  WPI_VictorSPX left_victor2 = {DriveConstants::left_victor2_id};
  // All of the right motor controllers are declared here
  WPI_TalonSRX right_talon = {DriveConstants::right_talon_id};
  WPI_VictorSPX right_victor1 = {DriveConstants::right_victor1_id};
  WPI_VictorSPX right_victor2 = {DriveConstants::right_victor2_id};
  // The left and right motor controllers are turned into SpeedControllerGroups
  frc::MotorControllerGroup left{left_talon, left_victor1, left_victor2};
  frc::MotorControllerGroup right{right_talon, right_victor1, right_victor2};
  // The MotorControllerGroups form a DifferentialDrive
  frc::DifferentialDrive drive{left, right};
  // DifferentialDrive kinematics object
  frc::DifferentialDriveKinematics drive_kinematics{DriveConstants::track_width};
  // Filter for retrieving encoder information
  frc::LinearFilter<double> encoder_filter = frc::LinearFilter<double>::SinglePoleIIR(DriveConstants::encoder_filter_cutoff_frequency, ROBORIO_LOOP_PERIOD);
  // gyroscope sensor
  frc::ADXRS450_Gyro gyroscope;
  // network tables pointer
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("drive");


};