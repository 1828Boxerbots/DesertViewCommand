/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"
#include "Util.h"
#include <frc/SmartDashboard/SmartDashboard.h>

using namespace frc;
DriveTrain::DriveTrain() : Subsystem("DriveTrain") {}

void DriveTrain::TeleopDrive(XboxController* controller)
{
  double distance = m_leftMotorEncoder->GetDistance();
  //The (distance/25.4) converts mm to in.
  SmartDashboard::PutNumber("m_leftMotorEncoder", (distance/25.4));
  SmartDashboard::PutNumber("m_rightMotorEncoder", (distance/25.4));
  double leftX = controller->GetX(frc::GenericHID::kLeftHand);
  double leftY = controller->GetY(frc::GenericHID::kLeftHand);

  double leftSideDrive = -leftX + leftY;
  double rightSideDrive = -leftX - leftY;

  double limitedLeftOverallValue = Util::Limit(1, -1, leftSideDrive);
  double limiedRightOverallValue = Util::Limit(1, -1, rightSideDrive);

  m_frontLeftMotor.Set(limitedLeftOverallValue);
  m_backLeftMotor.Set(limitedLeftOverallValue);
  m_backRightMotor.Set(limiedRightOverallValue);
  m_frontRightMotor.Set(limiedRightOverallValue);
  
}

void DriveTrain::InitDefaultCommand() 
{
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void DriveTrain::StopDriveMotors()
{
  m_frontLeftMotor.Set(0.0);
  m_backLeftMotor.Set(0.0);
  m_backRightMotor.Set(0.0);
  m_frontRightMotor.Set(0.0);
}
void DriveTrain::InvertMotors()
{
  m_frontLeftMotor.SetInverted(false);
  m_backLeftMotor.SetInverted(false);
  m_backRightMotor.SetInverted(false);
  m_frontRightMotor.SetInverted(false);
  
}