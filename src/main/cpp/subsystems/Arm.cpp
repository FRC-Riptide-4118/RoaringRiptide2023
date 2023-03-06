// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

Arm::Arm() {

    this->shoulder_left_motor.ConfigFactoryDefault();
    this->shoulder_right_motor.ConfigFactoryDefault();

    this->elbow_left_motor.ConfigFactoryDefault();
    this->elbow_right_motor.ConfigFactoryDefault();

    this->wrist_motor.ConfigFactoryDefault();

    this->shoulder_left_motor.Config_kF(0, ArmConstants::shoulder_pid.kF );
    this->shoulder_left_motor.Config_kP(0, ArmConstants::shoulder_pid.kP );
    this->shoulder_left_motor.Config_kI(0, ArmConstants::shoulder_pid.kI );
    this->shoulder_left_motor.Config_kD(0, ArmConstants::shoulder_pid.kD );

    this->elbow_left_motor.Config_kF(0, ArmConstants::elbow_pid.kF );
    this->elbow_left_motor.Config_kP(0, ArmConstants::elbow_pid.kP );
    this->elbow_left_motor.Config_kI(0, ArmConstants::elbow_pid.kI );
    this->elbow_left_motor.Config_kD(0, ArmConstants::elbow_pid.kD );

    this->wrist_motor.Config_kF(0, ArmConstants::wrist_pid.kF );
    this->wrist_motor.Config_kP(0, ArmConstants::wrist_pid.kP );
    this->wrist_motor.Config_kI(0, ArmConstants::wrist_pid.kI );
    this->wrist_motor.Config_kD(0, ArmConstants::wrist_pid.kD );

    this->elbow_left_motor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    // this->elbow_right_motor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    this->elbow_right_motor.SetInverted(true);
    this->elbow_right_motor.Follow(this->elbow_left_motor);

    this->shoulder_left_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder);
    this->shoulder_right_motor.SetInverted(true);
    this->shoulder_right_motor.Follow(this->shoulder_left_motor);

    this->wrist_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder);

    this->wrist_motor.SetSelectedSensorPosition(0);
    this->elbow_left_motor.SetSelectedSensorPosition(0);
    this->shoulder_left_motor.SetSelectedSensorPosition(0);

}

void Arm::Periodic() {}

double Arm::GetRawEncoderPosition(ArmConstants::ArmJoint arm_joint) {
    
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

void Arm::RunJointToPower(ArmConstants::ArmJoint arm_joint, double setpoint) {
    
    switch(arm_joint) {

        case ArmConstants::ArmJoint::shoulder:
            this->shoulder_left_motor.Set(ControlMode::PercentOutput, setpoint);
            break;

        case ArmConstants::ArmJoint::elbow:
            this->elbow_left_motor.Set(ControlMode::PercentOutput, setpoint);
            break;

        default:
            this->wrist_motor.Set(ControlMode::PercentOutput, setpoint);
            break;

    }

}

void Arm::CalcJointVelocities( double omega[3], double angles_rad[3], double tool_velocity[2], double omega_tool ) {

    double coord2_orig[2], coord3_orig[2], tool_pt[2], angle_sum12, angle_sum13;
    
    angle_sum12 = angles_rad[0] + angles_rad[1] ;
    angle_sum13 = angle_sum12 + angles_rad[2] ;
    
    coord2_orig[0] = ArmConstants::A1*cos(angles_rad[0]) ;
    coord2_orig[1] = ArmConstants::A1*sin(angles_rad[0]) ;
    coord3_orig[0] = coord2_orig[0] + ArmConstants::A1*cos(angle_sum12) ;
    coord3_orig[1] = coord2_orig[1] + ArmConstants::A1*sin(angle_sum12) ;
    tool_pt[0] = coord3_orig[0] + ArmConstants::TOOL_DIST*cos(angle_sum13) ;
    tool_pt[1] = coord3_orig[1] + ArmConstants::TOOL_DIST*sin(angle_sum13) ;
    
    
    // need to get vO_3toF
    // vP_3toF = vO_3toF + omega_3toF x P_inF
    double vO[2] ;
    vO[0] = tool_velocity[0] + omega_tool*tool_pt[1] ;
    vO[1] = tool_velocity[1] - omega_tool*tool_pt[0] ;

    double mymat[3][3], den ;
    
    den = (coord2_orig[0]*coord3_orig[1] - coord2_orig[1]*coord3_orig[0]) ;
    mymat[0][0] = 1.0 ;
    mymat[0][1] = -(coord2_orig[0] - coord3_orig[0])/den ;
    mymat[0][2] = -(coord2_orig[1] - coord3_orig[1])/den ;
    
    mymat[1][0] = 0.0 ;
    mymat[1][1] = -coord3_orig[0]/den ;
    mymat[1][2] = -coord3_orig[1]/den ;
    
    mymat[2][0] = 0.0 ;
    mymat[2][1] = coord2_orig[0]/den ;
    mymat[2][2] = coord2_orig[1]/den ;
    
    omega[0] = mymat[0][0] * omega_tool + mymat[0][1]*vO[0] + mymat[0][2]*vO[1] ;
    omega[1] = mymat[1][0] * omega_tool + mymat[1][1]*vO[0] + mymat[1][2]*vO[1] ;
    omega[2] = mymat[2][0] * omega_tool + mymat[2][1]*vO[0] + mymat[2][2]*vO[1] ;

}

double Arm::CalcAngleRadFromEncoder(ArmConstants::ArmJoint arm_joint) {

    switch(arm_joint) {

        case ArmConstants::ArmJoint::shoulder:
            return -this->GetRawEncoderPosition(arm_joint)/ArmConstants::SHOULDER_TICKS_PER_RADIAN + ArmConstants::SHOULDER_RAD_OFFSET;

        case ArmConstants::ArmJoint::elbow:
            return this->GetRawEncoderPosition(arm_joint)/ArmConstants::ELBOW_TICKS_PER_RADIAN + ArmConstants::ELBOW_RAD_OFFSET;

        default:
            return -this->GetRawEncoderPosition(arm_joint)/ArmConstants::WRIST_TICKS_PER_RADIAN + ArmConstants::WRIST_RAD_OFFSET;

    }
    
}

bool Arm::GetLimitSwitch(ArmConstants::ArmJoint arm_joint) {

    switch(arm_joint) {

        case ArmConstants::ArmJoint::shoulder:
            return !this->shoulder_limit.Get();

        case ArmConstants::ArmJoint::elbow:
            return !this->elbow_limit.Get();

        default:
            return !this->wrist_limit.Get();

    }

}