// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// include the Drive subsystem
#include <subsystems/Drive.h>

// Drive constructor
Drive::Drive() {

    this->Reset();

    drive.SetSafetyEnabled(false);

    left_spark0.SetClosedLoopRampRate(1);
    right_spark0.SetClosedLoopRampRate(1);

    left_spark1.Follow(left_spark0);
    left_spark2.Follow(left_spark0);

    right_spark1.Follow(right_spark0);
    right_spark2.Follow(right_spark0);

    left_pid_controller.SetP(DriveConstants::drive_PID_coefficients.kP);
    left_pid_controller.SetI(DriveConstants::drive_PID_coefficients.kI);
    left_pid_controller.SetD(DriveConstants::drive_PID_coefficients.kD);

    right_pid_controller.SetP(DriveConstants::drive_PID_coefficients.kP);
    right_pid_controller.SetI(DriveConstants::drive_PID_coefficients.kI);
    right_pid_controller.SetD(DriveConstants::drive_PID_coefficients.kD);

    this->limelight_led_status = LimelightConstants::FORCE_LED_OFF;

}

// This method will be called once per scheduler run
void Drive::Periodic() {

    frc::Rotation2d rotation{ units::radian_t(this->GetHeading()*DriveConstants::deg_to_radian) };
    this->odometry->Update(rotation, this->GetEncoderPositionMeters(DriveConstants::Side::left), -this->GetEncoderPositionMeters(DriveConstants::Side::right) );
    this->field->SetRobotPose(this->odometry->GetPose());

    this->table->PutNumber("ledMode", this->limelight_led_status);
    this->table->PutNumber("camMode", LimelightConstants::NORMAL_MODE);

}

void Drive::ArcadeDrive(double forward, double rotate) {

    // call arcade drive on DifferentialDrive object
    drive.ArcadeDrive(forward, rotate);

}

void Drive::CurvatureDrive(double forward, double rotate) {

    // call curvature drive on the DifferentialDrive object (with quick turn set to false)
    drive.CurvatureDrive(forward, rotate, false);

}

void Drive::VelocityDrive(units::meters_per_second_t left_velocity, units::meters_per_second_t right_velocity) {

    double left_rpm = 60*(left_velocity.value()/DriveConstants::wheel_circumference.value());
    double right_rpm = -60*(left_velocity.value()/DriveConstants::wheel_circumference.value());

    left_pid_controller.SetReference(left_rpm, rev::CANSparkMax::ControlType::kVelocity);
    right_pid_controller.SetReference(right_rpm, rev::CANSparkMax::ControlType::kVelocity);
    
}

void Drive::PositionDrive(double setpoint) {

    left_pid_controller.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);
    right_pid_controller.SetReference(-setpoint, rev::CANSparkMax::ControlType::kPosition);

}

void Drive::BreakPID(void) {

    this->left_spark0.Set(0);
    this->right_spark0.Set(0);

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

frc::DifferentialDriveOdometry* Drive::GetOdometry(void) {

    return this->odometry;
    
}

void Drive::SetField(frc::Field2d* field) {

    this->field = field;

}

void Drive::Reset(void) {

    this->left_encoder.SetPosition(0);
    this->right_encoder.SetPosition(0);
    this->pigeon_imu.SetFusedHeading(0);

}

void Drive::ToggleLimelight(void) {

    if (this->limelight_led_status == LimelightConstants::FORCE_LED_OFF)
        this->limelight_led_status = LimelightConstants::FORCE_LED_ON;
    else
        this->limelight_led_status = LimelightConstants::FORCE_LED_OFF;

}