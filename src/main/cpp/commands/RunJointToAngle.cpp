// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunJointToAngle.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
RunJointToAngle::RunJointToAngle(Arm* arm, ArmConstants::ArmJoint arm_joint, double setpoint_radian, PID_Coefficients pid_controller)
    : CommandHelper{frc2::PIDController{pid_controller.kP, pid_controller.kI, pid_controller.kD},
                    // This should return the measurement
                    [this] { return this->m_arm->CalcAngleRadFromEncoder(this->arm_joint); },
                    // This should return the setpoint (can also be a constant)
                    [this] { return this->setpoint_radian; },
                    // This uses the output
                    [this] (double output) {
                      
                      this->m_arm->RunJointToPowerAndHold_UNSAFE(this->arm_joint, -output);

                    }} {

                      this->m_arm = arm;
                      this->arm_joint = arm_joint;
                      this->setpoint_radian = setpoint_radian;

                    }

// Returns true when the command should end.
bool RunJointToAngle::IsFinished() {
  
  // double diff = abs( this->setpoint_radian - this->m_arm->CalcAngleRadFromEncoder(this->arm_joint) );
  // return ( diff <= DEG_2_RAD(2) );
  return false; 

}
