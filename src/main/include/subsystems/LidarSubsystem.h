/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/I2C.h>

class LidarSubsystem : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  frc::I2C m_device {frc::I2C::Port::kOnboard, LIDAR_ADDRESS};
  const int LIDAR_ADDRESS = 0x62;
  const int ACQ_COMMAND = 0x00;
  const int ACQ_CONFIG_REG = 0x04;
  const int STATUS = 0x01;
  const int FULL_DELAY_HIGH = 0x0f;
  const int FULL_DELAY_LOW = 0x10;

 public:
  LidarSubsystem();
  void InitDefaultCommand() override;
  void Init();
  int GetDistance();
};
