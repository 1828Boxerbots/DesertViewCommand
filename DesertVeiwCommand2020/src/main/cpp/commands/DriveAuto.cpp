/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveAuto.h"

DriveAuto::DriveAuto(CameraSubsystem* m_camera, DriveTrain* m_drivetrain, frc::XboxController* m_xbocontroller) 
: m_pCamera{m_camera}, m_pDriveTrain{m_drivetrain}, m_pXboxController{m_xbocontroller}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_camera);
  AddRequirements(m_drivetrain);
}

// Called when the command is initially scheduled.
void DriveAuto::Initialize() 
{
 m_pCamera->InterlizeCamera(USB_CAMERA);
}

// Called repeatedly when this Command is scheduled to run
void DriveAuto::Execute()
{
  
  /*==GetA = m_pXboxController->GetAButton();
  GetB = m_pXboxController->GetBButton();
  if (GetA = true)
  {
    switch (m_pCamera->WhereToTurn())
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
    if (GetB = true)
    {
      m_pDriveTrain->StopMotors();
    }
  */
  m_pCamera->IntakeFrame();
  m_pCamera->FilterFrame();
  m_pCamera->CenterMomment();

  GetA = m_pXboxController->GetAButton();
  GetB = m_pXboxController->GetBButton();

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
  
  m_pCamera->PrintTurn(turn);
}

// Called once the command ends or is interrupted.
void DriveAuto::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveAuto::IsFinished() { return false; }
