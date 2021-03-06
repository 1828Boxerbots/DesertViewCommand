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
#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/Counter.h>
#include "RobotMap.h"


using namespace frc;

class DriveTrain : public frc::Subsystem {
 private:
  Counter* counter;
  Spark m_leftMotor  {LEFTDRIVE };
  Spark m_rightMotor {RIGHTDRIVE};
  ADXRS450_Gyro m_gyro {SPI::Port::kOnboardCS0};
  AnalogInput m_US {ULTRASONIC};
  DigitalInput m_Lidar {LIDAR};

  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
  DriveTrain();
  void LidarInit();
  double GetDistance();
  void InitDefaultCommand() override;
  void TeleopDrive(XboxController* controller);
  void StopDriveMotors();
};