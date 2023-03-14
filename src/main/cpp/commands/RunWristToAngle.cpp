// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunWristToAngle.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
RunWristToAngle::RunWristToAngle(Gripper* gripper, double setpoint_radian, PID_Coefficients pid_controller)
    : CommandHelper{frc2::PIDController{pid_controller.kP, pid_controller.kI, pid_controller.kD},
                    // This should return the measurement
                    [this] { return this->m_gripper->CalcAngleRadFromEncoder(); },
                    // This should return the setpoint (can also be a constant)
                    [this] { return this->setpoint_radian; },
                    // This uses the output
                    [this] (double output) {
                      
                      
                      this->m_gripper->RunJointToPowerAndHold_UNSAFE([output] {return -output;});

                    }} {

                      this->m_gripper = gripper;
                      this->setpoint_radian = setpoint_radian;
                      AddRequirements({this->m_gripper});

                    }

// Returns true when the command should end.
bool RunWristToAngle::IsFinished() {
  
  // double diff = abs( this->setpoint_radian - this->m_arm->CalcAngleRadFromEncoder(this->arm_joint) );
  // return ( diff <= DEG_2_RAD(2) );
  return false; 

}
