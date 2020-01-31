/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"

DriveTrain::DriveTrain() {}

void DriveTrain::TurnLeft()
{
    m_leftMotor.Set(-0.5);
    m_rightMotor.Set(0.5);

}
void DriveTrain::TurnRight()
{
     m_leftMotor.Set(0.5);
    m_rightMotor.Set(-0.5);

}
void DriveTrain::StopMotors()
{
     m_leftMotor.Set(0.0);
    m_rightMotor.Set(0.0);
}
// This method will be called once per scheduler run
void DriveTrain::Periodic() {}
