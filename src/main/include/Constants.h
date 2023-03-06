// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// header include guard
#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
#include <units/length.h>
#include <units/angle.h>
#include "user_defined/PID_Coefficients.h"
#include <frc/kinematics/DifferentialDriveKinematics.h>

#define CURVATURE_DRIVE_MODE 0
#define ARCADE_DRIVE_MODE 1

#define ROBORIO_LOOP_PERIOD 0.02_s // corresponds to a frequency of 50Hz

#define PI 3.14159

// DriveConstants namespace is a location for all constants realted to the Drive subsystem
namespace DriveConstants {

    // left motor controller IDs
    const int left_spark0 = 0x25;
    const int left_spark1 = 0x24;
    const int left_spark2 = 0x23;

    // right motor controller IDs
    const int right_spark0 = 0x20;
    const int right_spark1 = 0x21;
    const int right_spark2 = 0x22;

    // drive PID coefficients
    const PID_Coefficients drive_PID_coefficients(0, 0.001, 0, 0);
    const PID_Coefficients gyro_PID_coefficients(0, 0.008333, 0, 0);
    const PID_Coefficients limelight_PID_coefficients(0, 0.035, 2.0e-5, 0);
    const PID_Coefficients balance_coefficients(0, 0.01, 0, 0);

    // control drive mode (either curvature or arcade)
    const int drive_mode = ARCADE_DRIVE_MODE;

    const double encoder_filter_cutoff_frequency = 0.1;

    const auto track_width = 0.762_m;

    const int gyroscope_cs = 0;

    const int pigeon_id = 0;

    const double balanced_angle = 0.0;
    enum Axis {x=0, y=1, z=2};

    const double exceed_angle = -45.0;

    enum Side {left=0, right=1};

    constexpr auto kRamseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
    constexpr auto kRamseteZeta = 0.7 / 1_rad;

    const frc::DifferentialDriveKinematics kDriveKinematics{track_width};

    constexpr auto wheel_diameter = 0.1524_m;
    constexpr auto wheel_circumference = 3.14*wheel_diameter;

    constexpr double ticks_per_rev = 8;

    constexpr auto meters_per_tick = wheel_circumference / ticks_per_rev;

    const double deg_to_radian = 3.14/180;

    constexpr auto kMaxSpeed = 4_mps;
    constexpr auto kMaxAcceleration = 3_mps / 1_s;

}

// GripperConstants is a location for all constants related to the Gripper
namespace GripperConstants {

    const int gripper_motor_id = 6;
    const int PCM_grip = 6;
    const int PCM_release = 7;

    const double gripper_speed = 0.25;

}

// ArmConstants is a location for all constants related to the Arm
namespace ArmConstants {

    const int shoulder_left_motor_id = 2;
    const int shoulder_right_motor_id = 8;
    const int elbow_left_motor_id = 9;
    const int elbow_right_motor_id = 10;
    const int wrist_motor_id = 1;

    const int shoulder_limit_port = 7;
    const int elbow_limit_port = 8;
    const int wrist_limit_port = 9;

    const double A1 = 34.0;
    const double A2 = 28.5;
    const double TOOL_DIST = 6.5;

    const double SHOULDER_TICKS_PER_RADIAN = (1756.0/1.29);
    const double ELBOW_TICKS_PER_RADIAN = (119682.0/2.81);
    const double WRIST_TICKS_PER_RADIAN = (7594/2.53);

    const double SHOULDER_RAD_OFFSET = (3.0/4.0)*PI;
    const double ELBOW_RAD_OFFSET = (200.0/180.0)*PI;
    const double WRIST_RAD_OFFSET = (130.0/180.0)*PI;

    const PID_Coefficients shoulder_pid(0, 0.01, 0, 0);
    const PID_Coefficients elbow_pid(0, 0.01, 0, 0);
    const PID_Coefficients wrist_pid(0, 0.01, 0, 0);

    enum ArmJoint {shoulder=0, elbow=1, wrist=2};

}

// ControllerConstants is a location for all constants related to the XboxController
namespace ControllerConstants {

    // the primary Xbox controller is connected on port 0
    const int driver_controller_port = 0;
    // the secondary arm driver port
    const int arm_controller_port = 1;

}