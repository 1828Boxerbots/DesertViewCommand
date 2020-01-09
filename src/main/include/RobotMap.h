/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

constexpr int FRONT_LEFT_MOTOR = 4;
constexpr int FRONT_RIGHT_MOTOR = 7;
constexpr int BACK_RIGHT_MOTOR = 6;
constexpr int BACK_LEFT_MOTOR = 3;

constexpr int LEFT_ENCODER_CHANNEL_A = 9;
constexpr int LEFT_ENCODER_CHANNEL_B = 8;
constexpr int RIGHT_ENCODER_CHANNEL_A = 0;
constexpr int RIGHT_ENCODER_CHANNEL_B = 1;


constexpr int XBOXCONTROLLER_USB = 0;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

// For example to map the left and right motors, you could define the
// following variables to use with your drivetrain subsystem.
// constexpr int kLeftMotor = 1;
// constexpr int kRightMotor = 2;

// If you are using multiple modules, make sure to define both the port
// number and the module. For example you with a rangefinder:
// constexpr int kRangeFinderPort = 1;
// constexpr int kRangeFinderModule = 1;
