// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// header include guard
#pragma once

// all frc/frc2 includes
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
// CTRE/Phoenix include
#include <ctre/Phoenix.h>
// all other includes
#include <Constants.h>

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
  units::meters_per_second_t GetVelocity(DriveConstants::Side side);
  // Get the current left talon encoder count
  units::meter_t GetPosition(DriveConstants::Side side);
  // Reset the gyroscope angle
  void ResetAngle();
  // Get the current gyroscope angle
  double GetAngle();
  void VelocityDrive(units::meters_per_second_t speed, DriveConstants::Side side);
  // Get the current (unfiltered) velocity
  double GetUnfilteredVelocity();
  // Get wheel speeds from drive kinematics definition (convert linear + angular velocity of chassis into left + right velocity)
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds(void);
  // DriveToDistance will use PID control and encoders to drive a specific distance in a straight line
  void DriveToDistance(double setpoint);
  // Get tilt angles from Pigeon IMU accelerometer
  void GetTiltAngles(double* tiltAngles);
  // Toggle balance variable
  void ToggleBalance();
  // Get balance variable
  bool GetBalanceActive();
  frc::Pose2d GetPose(void);
  void Reset(void);
  void SetField(frc::Field2d* field);
  void UpdateField(void);
  void CreateOdomoetry(units::meter_t xpos, units::meter_t ypos, frc::Pose2d pose); 

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
  
  // network tables pointer
  // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("drive");
  // IMU
  ctre::phoenix::sensors::PigeonIMU pigeon_imu = {DriveConstants::pigeon_id};
  // variable to keep track of whether or not to balance
  bool balance_active;
  frc::DifferentialDriveOdometry* m_odometry;
  frc::Field2d* field;

};