// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurnToAngle.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TurnToAngle::TurnToAngle(Drive* drive, double setpoint)
    : CommandHelper{frc2::PIDController{0.8/180, 0.001, 0},
                    // This should return the measurement
                    [this] { return this->m_drive->GetHeading(); },
                    // This should return the setpoint (can also be a constant)
                    [this] { return this->setpoint; },
                    // This uses the output
                    [this] (double output) {
                      
                      this->m_drive->ArcadeDrive(-output, 0);

                    }} {

                      this->m_drive = drive;
                      this->setpoint = setpoint;
                      AddRequirements({drive});

                    }

// Returns true when the command should end.
bool TurnToAngle::IsFinished() {
  return false;
}
