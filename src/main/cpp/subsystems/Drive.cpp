// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// include the Drive subsystem
#include <subsystems/Drive.h>

// Drive constructor
Drive::Drive() {

    drive.SetSafetyEnabled(false);

    left_spark1.Follow(left_spark0);
    left_spark2.Follow(left_spark0);

    right_spark1.Follow(right_spark0);
    right_spark2.Follow(right_spark0);

}

// This method will be called once per scheduler run
void Drive::Periodic() {

    frc::Rotation2d rotation{ units::radian_t(this->GetHeading()*DriveConstants::deg_to_radian) };
    this->odometry->Update(rotation, this->GetEncoderPositionMeters(DriveConstants::Side::left), this->GetEncoderPositionMeters(DriveConstants::Side::right) );
    this->field->SetRobotPose(this->odometry->GetPose());

}

void Drive::ArcadeDrive(double forward, double rotate) {

    // call arcade drive on DifferentialDrive object
    drive.ArcadeDrive(forward, rotate);

}

void Drive::CurvatureDrive(double forward, double rotate) {

    // call curvature drive on the DifferentialDrive object (with quick turn set to false)
    drive.CurvatureDrive(forward, rotate, false);

}

double Drive::GetRawEncoderVelocity(DriveConstants::Side side) {

    switch(side) {

        case DriveConstants::Side::left:
            return this->left_encoder.GetVelocity();
            break;

        case DriveConstants::Side::right:
            return this->right_encoder.GetVelocity();
            break;

        default:
            return 0;
            break;

    }

}

double Drive::GetRawEncoderPosition(DriveConstants::Side side) {

    switch(side) {

        case DriveConstants::Side::left:
            return this->left_encoder.GetPosition();
            break;

        case DriveConstants::Side::right:
            return this->right_encoder.GetPosition();
            break;

        default:
            return 0;
            break;

    }

}

units::meter_t Drive::GetEncoderPositionMeters(DriveConstants::Side side) {

    switch(side) {

        case DriveConstants::Side::left:
            return this->left_encoder.GetPosition()*DriveConstants::meters_per_tick;
            break;

        case DriveConstants::Side::right:
            return this->right_encoder.GetPosition()*DriveConstants::meters_per_tick;
            break;

        default:
            return 0_m;
            break;

    }

}

double Drive::GetHeading() {

    return this->pigeon_imu.GetFusedHeading();

}

void Drive::CreateOdometry(units::meter_t xpos, units::meter_t ypos, frc::Pose2d pose) {

    if (this->odometry != nullptr)
        delete this->odometry;
    this->odometry = new frc::DifferentialDriveOdometry{ units::radian_t(this->GetHeading()*DriveConstants::deg_to_radian), xpos, ypos, pose };

}

void Drive::SetField(frc::Field2d* field) {

    this->field = field;

}