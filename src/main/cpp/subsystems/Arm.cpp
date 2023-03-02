// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

Arm::Arm() {

    this->elbow_left_motor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    this->elbow_right_motor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);

    this->shoulder_left_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder);
    this->wrist_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder);

}

void Arm::Periodic() {}

double Arm::GetEncoderRawPosition(ArmConstants::ArmJoint arm_joint) {
    
    switch(arm_joint) {

        case ArmConstants::ArmJoint::shoulder:
            return this->shoulder_left_motor.GetSelectedSensorPosition();

        case ArmConstants::ArmJoint::elbow:
            return this->elbow_left_motor.GetSelectedSensorPosition();

        default:
            return this->wrist_motor.GetSelectedSensorPosition();

    }

}

double Arm::GetEncoderRawVelocity(ArmConstants::ArmJoint arm_joint) {
    
    switch(arm_joint) {

        case ArmConstants::ArmJoint::shoulder:
            return this->shoulder_left_motor.GetSelectedSensorVelocity();

        case ArmConstants::ArmJoint::elbow:
            return this->elbow_left_motor.GetSelectedSensorVelocity();

        default:
            return this->wrist_motor.GetSelectedSensorVelocity();

    }

}

void Arm::RunJointToSpeed(ArmConstants::ArmJoint arm_joint, double setpoint) {
    
    switch(arm_joint) {

        case ArmConstants::ArmJoint::shoulder:
            this->shoulder_left_motor.Set(ControlMode::Velocity, setpoint);
            break;

        case ArmConstants::ArmJoint::elbow:
            this->elbow_left_motor.Set(ControlMode::Velocity, setpoint);
            break;

        default:
            this->wrist_motor.Set(ControlMode::Velocity, setpoint);
            break;

    }

}

