// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand( DefaultDrive{ &m_drive, [this] {return driver_controller.GetLeftY(); }, [this] {return -driver_controller.GetRightX(); } } );

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {

  x_button.WhenPressed(&toggle_balance);
  frc2::Trigger balance_trigger( [this] { return m_drive.GetBalanceActive(); } );
  balance_trigger.WhenActive( BalanceDrive(&m_drive) );

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return this->m_autonomousCommand;
}
