// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// include the Drive subsystem
#include "subsystems/Drive.h"

// Drive constructor
Drive::Drive() : 
m_odometry{ frc::Rotation2d(units::degree_t(this->GetAngle())), units::meter_t(0.0), units::meter_t(0.0) }
{

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

    frc::Rotation2d rotation{ units::degree_t(this->GetAngle()) };
    m_odometry.Update(rotation, this->GetEncoderPosition(DriveConstants::Side::left), this->GetEncoderPosition(DriveConstants::Side::right));
    
}

void Drive::ResetEncoder() {

    // reset encoders to position 0
    this->left_talon.SetSelectedSensorPosition(0);
    this->right_talon.SetSelectedSensorPosition(0);

}

units::meters_per_second_t Drive::GetVelocity(DriveConstants::Side side) {

    if (side == DriveConstants::Side::left) {

        return units::meters_per_second_t( this->left_talon.GetSelectedSensorPosition() );

    }
    else {

        return units::meters_per_second_t{ this->right_talon.GetSelectedSensorPosition() };

    }

}

double Drive::GetPosition() {

    return this->left_talon.GetSelectedSensorPosition();

}

void Drive::ResetAngle() {

    return;

}

double Drive::GetAngle() {

    return this->pigeon_imu.GetFusedHeading();

}

double Drive::GetUnfilteredVelocity() {

    return this->left_talon.GetSelectedSensorVelocity();

}

void Drive::VelocityDrive(units::meters_per_second_t speed, DriveConstants::Side side) {

    if (side == DriveConstants::Side::left) {

        this->left_talon.Set(ControlMode::Velocity, speed.value());

    }
    else {

        this->right_talon.Set(ControlMode::Velocity, speed.value());

    }

}

frc::DifferentialDriveWheelSpeeds Drive::GetWheelSpeeds(void) {

    frc::DifferentialDriveWheelSpeeds wheel_speeds;

    wheel_speeds.left = this->GetVelocity(DriveConstants::Side::left);
    wheel_speeds.right = this->GetVelocity(DriveConstants::Side::right);

    return wheel_speeds;

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

<<<<<<< Updated upstream
=======
}

void Drive::GetTiltAngles(double* tiltAngles) {

    this->pigeon_imu.GetAccelerometerAngles(tiltAngles);

}

void Drive::ToggleBalance() {

    this->balance_active = !this->balance_active;

}

bool Drive::GetBalanceActive() {

    return this->balance_active;

}

units::meter_t Drive::GetEncoderPosition(DriveConstants::Side side) {

    if (side == DriveConstants::Side::left) {

        return units::meter_t( this->left_talon.GetSelectedSensorPosition() );

    }
    else {

        return units::meter_t{ this->right_talon.GetSelectedSensorPosition() };

    }

}

frc::Pose2d Drive::GetPose(void) {

    return this->m_odometry.GetPose();

}

frc::DifferentialDriveKinematics Drive::GetKinematics(void) {

    return this->drive_kinematics;

>>>>>>> Stashed changes
}