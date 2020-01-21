/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <frc/commands/Command.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "OI.h"

#include "commands/DriveTrainCMD.h"
#include "commands/Lidar.h"

#include "subsystems/DriveTrain.h"
#include "subsystems/LidarSubsystem.h"

using namespace frc;
using namespace std;

class Robot : public frc::TimedRobot {
 public:

  static OI m_oi;

  static std::shared_ptr <DriveTrain> m_driveTrain;
  static std::shared_ptr <LidarSubsystem> m_lidar;

  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc::SendableChooser<frc::Command*> m_chooser;

  DriveTrainCMD m_driveTrainCMD;
  Lidar m_lidarCMD;
};
