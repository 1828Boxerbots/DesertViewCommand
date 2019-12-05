/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/Spark.h>
#include <frc/XboxController.h>
#include <frc/ADXRS450_Gyro.h>
#include "RobotMap.h"
#include <frc/AnalogInput.h>

using namespace frc;

class DriveTrain : public frc::Subsystem {
 private:

  Spark m_leftMotor  {LEFTDRIVE };
  Spark m_rightMotor {RIGHTDRIVE};

  ADXRS450_Gyro m_gyro {SPI::Port::kOnboardCS0};
  AnalogInput m_US {ULTRASONIC};

  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
  DriveTrain();
  void InitDefaultCommand() override;
  void TeleopDrive(XboxController* controller);
  void StopDriveMotors();
};
