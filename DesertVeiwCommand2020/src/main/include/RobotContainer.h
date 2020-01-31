/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once


#include <frc2/command/Command.h>
#include <frc/XboxController.h>
#include "commands/ExampleCommand.h"
#include "subsystems/ExampleSubsystem.h"
#include "commands/DriveAuto.h"
#include "subsystems/CameraSubsystem.h"
#include "subsystems/DriveTrain.h"
#include "Constants.h"



/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  void Auto();
  frc2::Command* GetAutonomousCommand();

 private:
  // The robot's subsystems and commands are defined here...
  frc::XboxController m_xboxController{USB_XBOXCONTROLLER};
  CameraSubsystem m_camera;
  DriveTrain m_driveTrain;

  DriveAuto m_driveAuto{&m_camera,&m_driveTrain,&m_xboxController};



  void ConfigureButtonBindings();
};
