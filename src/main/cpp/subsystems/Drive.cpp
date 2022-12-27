// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// include the Drive subsystem
#include "subsystems/Drive.h"

// Drive constructor
Drive::Drive() {

    this->table->PutNumber("kP", DriveConstants::drive_PID_coefficients.kP);
    this->table->PutNumber("kI", DriveConstants::drive_PID_coefficients.kI);
    this->table->PutNumber("kD", DriveConstants::drive_PID_coefficients.kD);

    // Reset left/right talon information
    this->left_talon.ConfigFactoryDefault();
    this->right_talon.ConfigFactoryDefault();

    // As of 2022, DifferentialDrive no longer automatically inverts direction
    left.SetInverted(true);

    // left motor controllers always follow this->left_talon
    this->left_victor1.Follow(this->left_talon);
    this->left_victor2.Follow(this->left_talon);

    // right motor controllers always follow this->right_talon
    this->right_victor1.Follow(this->right_talon);
    this->right_victor2.Follow(this->right_talon);

    // disable safety to avoid weird errors
    drive.SetSafetyEnabled(false);
    this->left_talon.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder);
    this->right_talon.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder);

    // set the current encoder value to 0
    this->left_talon.SetSelectedSensorPosition(0);
    this->right_talon.SetSelectedSensorPosition(0);

    // limit acceleration to avoid brownouts
    // this->left_talon.ConfigOpenloopRamp(2.8);
    // this->right_talon.ConfigOpenloopRamp(2.8);

    // set left PID coefficients for motor controllers
    this->left_talon.Config_kF(0, DriveConstants::drive_PID_coefficients.kF);
    this->left_talon.Config_kP(0, DriveConstants::drive_PID_coefficients.kP);
    this->left_talon.Config_kI(0, DriveConstants::drive_PID_coefficients.kI);
    this->left_talon.Config_kD(0, DriveConstants::drive_PID_coefficients.kD);

    // set right PID coefficients for motor controllers
    this->right_talon.Config_kF(0, DriveConstants::drive_PID_coefficients.kF);
    this->right_talon.Config_kP(0, DriveConstants::drive_PID_coefficients.kP);
    this->right_talon.Config_kI(0, DriveConstants::drive_PID_coefficients.kI);
    this->right_talon.Config_kD(0, DriveConstants::drive_PID_coefficients.kD);

    // disable sensor phase sensing
    this->left_talon.SetSensorPhase(false);
    this->right_talon.SetSensorPhase(false);

}

// This method will be called once per scheduler run
// No implementation necessary
void Drive::Periodic() {

    encoder_filter.Calculate( this->left_talon.GetSelectedSensorVelocity() );

}

void Drive::ResetEncoder() {

    // reset encoders to position 0
    this->left_talon.SetSelectedSensorPosition(0);
    this->right_talon.SetSelectedSensorPosition(0);

}

double Drive::GetVelocity() {

    // get the filtered velocity of just the left side
    return encoder_filter.Calculate( this->left_talon.GetSelectedSensorVelocity() );

}

double Drive::GetPosition() {

    return this->left_talon.GetSelectedSensorPosition();

}

void Drive::ResetAngle() {

    this->gyroscope.Reset();

}

double Drive::GetAngle() {

    return this->gyroscope.GetAngle();

}

double Drive::GetUnfilteredVelocity() {

    return this->left_talon.GetSelectedSensorVelocity();

}

frc::DifferentialDriveWheelSpeeds Drive::GetWheelSpeeds(frc::ChassisSpeeds chs_spd) {

    // return the result of converting the chassis speed info into individual wheel speed info
    return drive_kinematics.ToWheelSpeeds(chs_spd);

}

void Drive::DriveToDistance(double setpoint) {

    // set left PID coefficients for motor controllers
    this->left_talon.Config_kF(0, table->GetNumber("kF", DriveConstants::drive_PID_coefficients.kF));
    this->left_talon.Config_kP(0, table->GetNumber("kP", DriveConstants::drive_PID_coefficients.kP));
    this->left_talon.Config_kI(0, table->GetNumber("kI", DriveConstants::drive_PID_coefficients.kI));
    this->left_talon.Config_kD(0, table->GetNumber("kD", DriveConstants::drive_PID_coefficients.kD));

    // set right PID coefficients for motor controllers
    this->right_talon.Config_kF(0, table->GetNumber("kF", DriveConstants::drive_PID_coefficients.kF));
    this->right_talon.Config_kP(0, table->GetNumber("kP", DriveConstants::drive_PID_coefficients.kP));
    this->right_talon.Config_kI(0, table->GetNumber("kI", DriveConstants::drive_PID_coefficients.kI));
    this->right_talon.Config_kD(0, table->GetNumber("kD", DriveConstants::drive_PID_coefficients.kD));

    this->left_talon.Set(ControlMode::Position, setpoint);
    this->right_talon.Set(ControlMode::Position, -setpoint);
}

void Drive::CurvatureDrive(double forward, double rotate) {

    // call curvature drive on the DifferentialDrive object (with quick turn set to false)
    drive.CurvatureDrive(forward, rotate, false);

}

void Drive::ArcadeDrive(double forward, double rotate) {

    // call arcade drive on DifferentialDrive object
    drive.ArcadeDrive(forward, rotate);

}