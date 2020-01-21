/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/LidarSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

LidarSubsystem::LidarSubsystem() : Subsystem("ExampleSubsystem") {}

void LidarSubsystem::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.


void LidarSubsystem::Init() 
{
    m_device.Write(ACQ_COMMAND, ACQ_CONFIG_REG);
}

int LidarSubsystem::GetDistance() 
{
    uint8_t val;
    while(m_device.Read(STATUS, 1, &val) == false) 
    {
        if (val & 0x1 == 0)
        {
            break;
        }
    }

    uint8_t high;
    uint8_t low;
    m_device.Read(FULL_DELAY_HIGH, 1, &high);
    m_device.Read(FULL_DELAY_LOW , 1, &low );
    uint16_t centimeters = high << 8 | low;

    SmartDashboard::PutNumber("Lidar Distance", (int)centimeters);

    return (int)centimeters;
}
