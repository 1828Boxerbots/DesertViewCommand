  /*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

void Robot::RobotInit() 
{
  m_right.SetInverted(true);
  m_leftEncoder.Reset();
  m_rightEncoder.Reset();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  

 
}

void Robot::AutonomousPeriodic() {
  
}

void Robot::TeleopInit() {
  //set the motor to make them stop when it start up
  m_right.Set(0.0);
  m_left.Set(0.0);
}

void Robot::TeleopPeriodic() 
{
  //checking if in telop and the robot is enable
  while(IsOperatorControl() && IsEnabled())
  {
    //taking the X and Y form the left joystick on the xbox controller 
      double X = m_controller.GetX(frc::GenericHID::kLeftHand);
      double Y = m_controller.GetY(frc::GenericHID::kLeftHand);

    //set the motors speeds for the left and right motors
      m_left.Set(Y-X);
      m_right.Set(Y+X);

    //getting the encoders values
      double lefttick = m_leftEncoder.Get();
      double righttick = m_rightEncoder.Get();

    
    //sending the encoders vaules to smartdashboard
      SmartDashboard::PutNumber("left tick", -1 * lefttick);
      SmartDashboard::PutNumber("right tick",righttick);   
  }
  //stoping the motor when diver disabled the robot
  if(IsDisabled())
  {
    m_left.Set(0.0);
    m_right.Set(0.0);
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
