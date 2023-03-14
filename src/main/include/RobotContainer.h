// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// --- GENERAL FRC --- //
#include <frc2/command/Command.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/Button.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

// --- SUBSYSTEMS --- //
#include <subsystems/Drive.h>
#include <subsystems/Gripper.h>
#include <subsystems/Arm.h>

// --- COMMANDS -- //
#include <commands/DefaultDrive.h>
#include <commands/DefaultGripper.h>
#include <commands/DefaultArm.h>
#include <commands/DriveArm.h>
#include <commands/RunJointToAngle.h>
#include <commands/RequireArm.h>
#include <commands/RunWristToAngle.h>
#include <commands/Reset.h>
#include <commands/TurnToAngle.h>

// --- OTHER --- //
#include <Constants.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:

  frc::SendableChooser<frc2::Command*> m_auto_chooser;

  std::string deployDirectory = frc::filesystem::GetDeployDirectory() + "/paths/output" + "/straight_back.wpilib.json"; 
  frc::Trajectory test_trajectory{frc::TrajectoryUtil::FromPathweaverJson(deployDirectory)};

  /* ----- SUBSYSTEM DECALARATIONS ----- */

  frc::Field2d field;

  Drive m_drive;
  Arm m_arm;
  Gripper m_gripper{[this] {return m_arm.CalcAngleRadFromEncoder(ArmConstants::ArmJoint::elbow) + m_arm.CalcAngleRadFromEncoder(ArmConstants::ArmJoint::shoulder);}};

  /* ----- SIMPLE COMMAND DECALARATIONS ---- */

  TurnToAngle spin{&m_drive, 180};

  // [GRIPPER]
  frc2::InstantCommand cone_toggle{ [this] {m_gripper.ConeToggle();}, {&m_gripper}};
  frc2::InstantCommand cone_grab{[this] {m_gripper.ConeGrab();}, {&m_gripper}};
  frc2::InstantCommand cone_release{[this] {m_gripper.ConeRelease();}, {&m_gripper}};
  frc2::RunCommand cube_in{[this] {m_gripper.CubeIn();}, {&m_gripper}};
  frc2::RunCommand cube_out{[this] {m_gripper.CubeOut();}, {&m_gripper}};

  frc2::InstantCommand reset{ [this] { m_arm.ResetEncoders(); m_gripper.Reset(); m_drive.Reset(); }, {&m_arm} };
  frc2::InstantCommand toggle_limelight_led{ [this] { m_drive.ToggleLimelight(); }, {&m_drive} };

  // [ARM]
  RunJointToAngle run_shoulder_to_start{ &m_arm, ArmConstants::ArmJoint::shoulder, DEG_2_RAD( ArmConstants::SHOULDER_START ), PID_Coefficients(0, 0.8, 0, 0) };
  RunJointToAngle run_shoulder_to_pickup_floor{ &m_arm, ArmConstants::ArmJoint::shoulder, DEG_2_RAD( ArmConstants::SHOULDER_PICKUP_FLOOR ), PID_Coefficients(0, 0.8, 0, 0) };
  RunJointToAngle run_shoulder_to_pickup_sideways{ &m_arm, ArmConstants::ArmJoint::shoulder, DEG_2_RAD( ArmConstants::SHOULDER_FLOOR_SIDEWAYS ), PID_Coefficients(0, 0.8, 0, 0) };
  RunJointToAngle run_shoulder_to_low{ &m_arm, ArmConstants::ArmJoint::shoulder, DEG_2_RAD( ArmConstants::SHOULDER_LOW ), PID_Coefficients(0, 0.8, 0, 0) };
  RunJointToAngle run_shoulder_to_high{ &m_arm, ArmConstants::ArmJoint::shoulder, DEG_2_RAD( ArmConstants::SHOULDER_HIGH ), PID_Coefficients(0, 0.8, 0, 0) };

  RunJointToAngle run_elbow_to_start{ &m_arm, ArmConstants::ArmJoint::elbow, DEG_2_RAD( ArmConstants::ELBOW_START ), PID_Coefficients(0, 1.0/2.0, 0, 0) };
  RunJointToAngle run_elbow_to_pickup_floor{ &m_arm, ArmConstants::ArmJoint::elbow, DEG_2_RAD( ArmConstants::ELBOW_PICKUP_FLOOR ), PID_Coefficients(0, 1.0/2.0, 0, 0) };
  RunJointToAngle run_elbow_to_pickup_sideways{ &m_arm, ArmConstants::ArmJoint::elbow, DEG_2_RAD( ArmConstants::ELBOW_FLOOR_SIDEWAYS ), PID_Coefficients(0, 1.0/2.0, 0, 0) };
  RunJointToAngle run_elbow_to_low{ &m_arm, ArmConstants::ArmJoint::elbow, DEG_2_RAD( ArmConstants::ELBOW_LOW ), PID_Coefficients(0, 1.0/2.0, 0, 0) };
  RunJointToAngle run_elbow_to_high{ &m_arm, ArmConstants::ArmJoint::elbow, DEG_2_RAD( ArmConstants::ELBOW_HIGH ), PID_Coefficients(0, 1.0/2.0, 0, 0) };

  RunWristToAngle run_wrist_to_start{ &m_gripper, DEG_2_RAD( ArmConstants::WRIST_START ), PID_Coefficients(0, 1.0/1.5, 0, 0) };
  RunWristToAngle run_wrist_to_pickup_floor{ &m_gripper, DEG_2_RAD( ArmConstants::WRIST_PICKUP_FLOOR ), PID_Coefficients(0, 1.0/1.5, 0, 0) };
  RunWristToAngle run_wrist_to_pickup_sideways{ &m_gripper, DEG_2_RAD( ArmConstants::WRIST_FLOOR_SIDEWAYS ), PID_Coefficients(0, 1.0/1.5, 0, 0) };
  RunWristToAngle run_wrist_to_low{ &m_gripper, DEG_2_RAD( ArmConstants::WRIST_LOW ), PID_Coefficients(0, 1.0/1.5, 0, 0) };
  RunWristToAngle run_wrist_to_high{ &m_gripper, DEG_2_RAD( ArmConstants::WRIST_HIGH ), PID_Coefficients(0, 1.0/1.5, 0, 0) };

  frc2::ParallelCommandGroup run_to_start{

    RequireArm(&m_arm),
    run_shoulder_to_start,
    run_elbow_to_start,
    run_wrist_to_start

  };

  frc2::ParallelCommandGroup run_to_high{

    RequireArm(&m_arm),
    run_shoulder_to_high,
    run_elbow_to_high,

  };

  frc2::ParallelCommandGroup run_to_low{

    RequireArm(&m_arm),
    run_shoulder_to_low,
    run_elbow_to_low

  };
  
  frc2::ParallelCommandGroup run_to_pickup_floor{

    RequireArm(&m_arm),
    run_shoulder_to_pickup_floor,
    run_elbow_to_pickup_floor,
    run_wrist_to_pickup_floor

  };

  frc2::ParallelCommandGroup run_to_sideways_pickup{

    RequireArm(&m_arm),
    run_shoulder_to_pickup_sideways,
    run_elbow_to_pickup_sideways

  };

  frc2::RamseteCommand drive_path{

    test_trajectory,
    [this] { return m_drive.GetOdometry()->GetPose(); },
    frc::RamseteController{DriveConstants::kRamseteB, DriveConstants::kRamseteZeta},
    DriveConstants::kDriveKinematics,
    [this](auto left, auto right) { m_drive.VelocityDrive(left, right); },
    {&m_drive}

  };

  frc2::SequentialCommandGroup place_cone_high_drive{

    reset,

    cone_grab,

    frc2::ParallelRaceGroup{

      frc2::ParallelCommandGroup{
        RequireArm(&m_arm),
        run_shoulder_to_high,
        run_elbow_to_high
      },

      frc2::WaitCommand(2_s)

    },

    frc2::ParallelRaceGroup{

      frc2::ParallelCommandGroup{
        RequireArm(&m_arm),
        run_wrist_to_high
      },

      frc2::WaitCommand(1_s)

    },

    cone_release,

    frc2::ParallelRaceGroup{

      frc2::RunCommand{

        [this] {m_drive.PositionDrive(DriveConstants::AWAY_FROM_SCORE);},
        {&m_drive}

      },

      frc2::WaitCommand(1_s)

    },

    frc2::ParallelCommandGroup{
      RequireArm(&m_arm),
      run_shoulder_to_start,
      run_elbow_to_start,
      run_wrist_to_start
    }

  };

  frc2::SequentialCommandGroup place_cone_mid_drive{

    reset,

    cone_grab,

    frc2::ParallelRaceGroup{

      frc2::ParallelCommandGroup{
        RequireArm(&m_arm),
        run_shoulder_to_low,
        run_elbow_to_low
      },

      frc2::WaitCommand(2_s)

    },

    frc2::ParallelRaceGroup{

      frc2::ParallelCommandGroup{
        RequireArm(&m_arm),
        run_wrist_to_high
      },

      frc2::WaitCommand(1_s)

    },

    cone_release,

    frc2::ParallelRaceGroup{

      frc2::RunCommand{

        [this] {m_drive.PositionDrive(DriveConstants::AWAY_FROM_SCORE);},
        {&m_drive}

      },

      frc2::WaitCommand(1_s)

    },

    frc2::ParallelCommandGroup{
      RequireArm(&m_arm),
      run_shoulder_to_start,
      run_elbow_to_start,
      run_wrist_to_start
    }

  };

  frc2::SequentialCommandGroup place_cone_high_drive_spin{

    reset,

    cone_grab,

    frc2::ParallelRaceGroup{

      frc2::ParallelCommandGroup{
        RequireArm(&m_arm),
        run_shoulder_to_high,
        run_elbow_to_high
      },

      frc2::WaitCommand(2_s)

    },

    frc2::ParallelRaceGroup{

      frc2::ParallelCommandGroup{
        RequireArm(&m_arm),
        run_wrist_to_high
      },

      frc2::WaitCommand(1_s)

    },

    cone_release,

    frc2::ParallelRaceGroup{

      frc2::RunCommand{

        [this] {m_drive.PositionDrive(DriveConstants::AWAY_FROM_SCORE);},
        {&m_drive}

      },

      frc2::WaitCommand(1_s)

    },

    frc2::ParallelCommandGroup{

      frc2::ParallelCommandGroup{
        RequireArm(&m_arm),
        run_shoulder_to_start,
        run_elbow_to_start,
        run_wrist_to_start
      },

      frc2::SequentialCommandGroup{

        frc2::ParallelRaceGroup{

          frc2::RunCommand{

            [this] {m_drive.PositionDrive(DriveConstants::OUT_OF_COMMUNITY);},
            {&m_drive}

          },

        frc2::WaitCommand(2_s)

      },

      spin

      }

    }

  };

  frc2::SequentialCommandGroup place_cone_mid_drive_spin{

    reset,

    cone_grab,

    frc2::ParallelRaceGroup{

      frc2::ParallelCommandGroup{
        RequireArm(&m_arm),
        run_shoulder_to_low,
        run_elbow_to_low
      },

      frc2::WaitCommand(2_s)

    },

    frc2::ParallelRaceGroup{

      frc2::ParallelCommandGroup{
        RequireArm(&m_arm),
        run_wrist_to_high
      },

      frc2::WaitCommand(1_s)

    },

    cone_release,

    frc2::ParallelRaceGroup{

      frc2::RunCommand{

        [this] {m_drive.PositionDrive(DriveConstants::AWAY_FROM_SCORE);},
        {&m_drive}

      },

      frc2::WaitCommand(1_s)

    },

    frc2::ParallelCommandGroup{

      frc2::ParallelCommandGroup{
        RequireArm(&m_arm),
        run_shoulder_to_start,
        run_elbow_to_start,
        run_wrist_to_start
      },

      frc2::SequentialCommandGroup{

        frc2::ParallelRaceGroup{

          frc2::RunCommand{

            [this] {m_drive.PositionDrive(DriveConstants::OUT_OF_COMMUNITY);},
            {&m_drive}

          },

        frc2::WaitCommand(2_s)

      },

      spin

      }

    }

  };
  /* ------ CONTROLLER/BUTTON DECLARATIONS ---- */

  frc2::CommandXboxController driver_controller{ControllerConstants::driver_controller_port};
  frc2::CommandXboxController arm_controller{ControllerConstants::arm_controller_port};

  frc2::Trigger right_bumper = arm_controller.RightBumper();
  frc2::Trigger left_bumper = arm_controller.LeftBumper();
  frc2::Trigger right_trigger = arm_controller.RightTrigger();
  frc2::Trigger left_trigger = arm_controller.LeftTrigger();
  frc2::Trigger x_button = arm_controller.X();
  frc2::Trigger y_button = arm_controller.Y();
  frc2::Trigger a_button = arm_controller.A();
  frc2::Trigger b_button = arm_controller.B();
  frc2::Trigger driver_a_button = driver_controller.A();

  void ConfigureButtonBindings();

};
