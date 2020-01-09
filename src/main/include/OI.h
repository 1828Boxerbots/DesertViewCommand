/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/XboxController.h>
#include "RobotMap.h"
using namespace frc;

class OI {
  private:
  XboxController m_XboxController {XBOXCONTROLLER_USB};
 public:
  XboxController* GetController();
  OI();
};
