// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// include command header file
#include <commands/DefaultDrive.h>

DefaultDrive::DefaultDrive(Drive* drive, std::function<double()> forward, std::function<double()> rotate) {
  
  // initialize all local variables based on the provided arguments
  m_drive = drive;
  m_forward = forward;
  m_rotate = rotate;

  // the Drive subsystem is a requirement for this command
  AddRequirements({m_drive});

}

void DefaultDrive::Execute() {

  if (DriveConstants::drive_mode == CURVATURE_DRIVE_MODE) {

    m_drive->CurvatureDrive(m_forward(), m_rotate());

  }
  else {

    m_drive->ArcadeDrive(m_forward(), m_rotate());

  }
  
}