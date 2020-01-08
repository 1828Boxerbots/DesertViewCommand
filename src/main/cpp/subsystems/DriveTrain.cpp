/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"
#include <frc/SmartDashboard/SmartDashboard.h>

using namespace frc;

DriveTrain::DriveTrain() : Subsystem("DriveTrain") {}

double DriveTrain::GetDistance()
{
  double centimeters = (counter->GetPeriod() * 100000.0 / 10.0);
  SmartDashboard::PutNumber("Lidar Value", centimeters );
  SmartDashboard::PutNumber("Raw (Get) Lidar Value", counter->Get());;;
  SmartDashboard::PutNumber("Raw (GetPeriod) Lidar Value", counter->GetPeriod());;;
  if (counter->Get() >= 1)
  {
    return centimeters;
  }
}

void DriveTrain::LidarInit()
{
  counter = new Counter(LIDAR);
  counter->SetMaxPeriod(1.0);
  counter->SetSemiPeriodMode(true);
  counter->Reset();
}

void DriveTrain::TeleopDrive(XboxController* controller)
{
  double rightY = controller->GetY(frc::GenericHID::kRightHand);
  SmartDashboard::PutNumber("Right Stick Value", rightY);
  double leftY = controller->GetY(frc::GenericHID::kLeftHand);
  SmartDashboard::PutNumber("Left Stick Value", leftY);

  m_leftMotor.Set(0.5*leftY);
  m_rightMotor.Set(-0.5*rightY);

  double angleOof = m_gyro.GetAngle();
  SmartDashboard::PutNumber("Don't break please", angleOof);

  double ultraValue = m_US.GetVoltage();
  SmartDashboard::PutNumber("Ultrasonic Value", ultraValue);
  const double voltToMeter = (4.885/5);
  const int calibration = 3;
  double distance = ultraValue * voltToMeter * calibration;
  SmartDashboard::PutNumber("Ultrasonic Distance", distance);
}

void DriveTrain::StopDriveMotors()
{
  m_leftMotor.Set(0.0);
  m_rightMotor.Set(0.0);
}

void DriveTrain::InitDefaultCommand() 
{
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
