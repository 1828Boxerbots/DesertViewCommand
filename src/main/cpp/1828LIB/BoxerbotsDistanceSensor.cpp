/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "BoxerbotsDistanceSensor.h"
#include <frc/smartdashboard/SmartDashboard.h>

BoxerbotsDistanceSensor::BoxerbotsDistanceSensor() 
{  
  m_pDevice = new rev::Rev2mDistanceSensor {rev::Rev2mDistanceSensor::Port::kOnboard, 
                                            rev::Rev2mDistanceSensor::DistanceUnit::kInches};
}

void BoxerbotsDistanceSensor::DistInit()
{
  m_pDevice->SetAutomaticMode(true);
  m_pDevice->SetEnabled(true);
}

double BoxerbotsDistanceSensor::GetRange()
{
  /**
  * The current measurement is considered valid if IsRangeValid()
  * returns true.
  * 
  * This is helpful because it returns a special value when
  * nothing is detected. 
  */

  bool isValid = m_pDevice->IsRangeValid();
  frc::SmartDashboard::PutBoolean("M_device valid", isValid);
  if(isValid) 
  {
    double distance = m_pDevice->GetRange();
    frc::SmartDashboard::PutNumber("M_device distance", distance);
    return distance;
  }
  else 
  {
    frc::SmartDashboard::PutNumber("M_device distance", -1);
    return -1;
  }

}

void BoxerbotsDistanceSensor::Disabled()
{
  m_pDevice->SetAutomaticMode(false);
  m_pDevice->SetEnabled(false);
  frc::SmartDashboard::PutString("M_device", "disabled");
}