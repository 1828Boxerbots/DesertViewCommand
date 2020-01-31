/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

constexpr int LEFT_MOTOR_DRIVE = 2;
constexpr int RIGHT_MOTOR_DRIVE = 1;

constexpr int USB_XBOXCONTROLLER = 0;
constexpr int USB_CAMERA= 0;

constexpr int TURN_RIGHT = -1;
constexpr int TURN_LEFT = 1;
constexpr int IS_CENTER = 0;

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

 /*m_pCamera->IntakeFrame();
  m_pCamera->FilterFrame();
  m_pCamera->CenterMomment();
  turn = m_pCamera->WhereToTurn();
  switch (turn)
  {
    case TURN_RIGHT:
      m_pDriveTrain->TurnRight();

    break;
    case TURN_LEFT:
      m_pDriveTrain->TurnLeft();
    break;
    case IS_CENTER:
      m_pDriveTrain->StopMotors();
    break;
      
    default:
     m_pDriveTrain->StopMotors();
    break;
  }
  m_pCamera->PrintTurn(turn);*/
