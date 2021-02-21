/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "rev/Rev2mDistanceSensor.h"

/**
 * The Rev2mDistanceSensor vendor dependency is required for this 
 * driver to work. 
 * 
 * If you do not have this library, go to https://github.com/REVrobotics/2m-Distance-Sensor/releases and install the .zip file
 * 
 * From there, unzip the library and move it under C:Users/Public/wpilib/2020
 * 
 * Then, move the .jar file into the vendordeps folder, and merge the maven folder with the one already in the filepath.
 * 
 * Open VSCode, right click build.gradle, and select Manage Vendor Libraries. Select "Install offline", and select the 2mDistanceSensor.
 * 
 * Finally, build the code and this driver should work.
 */


class BoxerbotsDistanceSensor {
 public:
  
  BoxerbotsDistanceSensor();
  
  void DistInit();
  
  double GetRange();
  
  void Disabled();

  rev::Rev2mDistanceSensor* m_pDevice;

  int pulse = 0;
  int initPulse = 0;
};
