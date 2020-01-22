/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/Lidar.h"
#include "Robot.h"

Lidar::Lidar() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::m_lidar.get());
}

// Called just before this Command runs the first time
void Lidar::Initialize() 
{
  Robot::m_lidar->Init();
}

// Called repeatedly when this Command is scheduled to run
void Lidar::Execute() 
{
  SmartDashboard::PutNumber("Loop", isloop++);
  Robot::m_lidar->GetDistance();
  //SmartDashboard::PutNumber("Lidar Distance", Robot::m_lidar->GetDistance());
}

// Make this return true when this Command no longer needs to run execute()
bool Lidar::IsFinished() { return false; }

// Called once after isFinished returns true
void Lidar::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Lidar::Interrupted() {}
