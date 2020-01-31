/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Spark.h>
#include "Constants.h"

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  void TurnLeft();
  void TurnRight();
  void StopMotors();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

 private:

 frc::Spark m_leftMotor{LEFT_MOTOR_DRIVE};
  frc::Spark m_rightMotor{RIGHT_MOTOR_DRIVE};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
