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
//REV Include
#include <rev/CANSparkMax.h>
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
  double GetRawEncoderVelocity(DriveConstants::Side side);
  double GetRawEncoderPosition(DriveConstants::Side side);
  units::meter_t GetEncoderPositionMeters(DriveConstants::Side side);
  double GetHeading();
  void CreateOdometry(units::meter_t xpos, units::meter_t ypos, frc::Pose2d pose);
  void SetField(frc::Field2d* field);

 private:
  // All of the left motor controllers are defined here
  rev::CANSparkMax left_spark0{DriveConstants::left_spark0, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax left_spark1{DriveConstants::left_spark1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax left_spark2{DriveConstants::left_spark2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax right_spark0{DriveConstants::right_spark0, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax right_spark1{DriveConstants::right_spark1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax right_spark2{DriveConstants::right_spark2, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxRelativeEncoder left_encoder{left_spark0.GetEncoder()};
  rev::SparkMaxRelativeEncoder right_encoder{right_spark0.GetEncoder()};

  ctre::phoenix::sensors::PigeonIMU pigeon_imu = {DriveConstants::pigeon_id};

  // The MotorControllerGroups form a DifferentialDrive
  frc::DifferentialDrive drive{left_spark0, right_spark0};

  frc::DifferentialDriveOdometry* odometry;
  frc::Field2d* field;

};