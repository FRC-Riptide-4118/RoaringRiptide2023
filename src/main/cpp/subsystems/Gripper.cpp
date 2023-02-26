// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Gripper.h"

Gripper::Gripper() {

    this->gripper_position = false;
    this->ConeRelease();

    this->gripper_motor.SetInverted(false);

}

// This method will be called once per scheduler run
void Gripper::Periodic() {}

void Gripper::CubeIn(void) {

    this->gripper_motor.Set(ControlMode::PercentOutput, GripperConstants::gripper_speed);

}

void Gripper::CubeOut(void) {

    this->gripper_motor.Set(ControlMode::PercentOutput, -GripperConstants::gripper_speed);

}

void Gripper::Stop(void) {

    this->gripper_motor.Set(ControlMode::PercentOutput, 0);

}

void Gripper::ConeGrab(void) {

    this->gripper_grab.Set(frc::DoubleSolenoid::kReverse);

}

void Gripper::ConeRelease(void) {

    this->gripper_grab.Set(frc::DoubleSolenoid::kForward);

}

void Gripper::ConeToggle(void) {

    switch (gripper_position) {

        case false:
            this->ConeGrab();
            this->gripper_position = true;
            break;
        default:
            this->ConeRelease();
            this->gripper_position = false;
            break;
    }

}