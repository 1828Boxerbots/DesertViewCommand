/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/Spark.h>
#include <frc/XboxController.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/PWM.h>
#include <frc/Counter.h>
#include <frc/Relay.h>
#include "RobotMap.h"
#include "rev/Rev2mDistanceSensor.h"
#include "../../cpp/1828LIB/BoxerbotsDistanceSensor.h"
//#include <adi/ADIS16448_IMU.h>


using namespace frc;
using namespace rev;

class DriveTrain : public frc::Subsystem {
 private:
  Counter*            counter                                                                                            ;
  Spark               m_leftMotor       {LEFTDRIVE                                                                      };
  Spark               m_rightMotor      {RIGHTDRIVE                                                                     };
  ADXRS450_Gyro       m_gyro            {SPI::Port::kOnboardCS0                                                         };
  AnalogInput         m_USL             {ULTRASONIC_LONG                                                                };
  AnalogInput         m_USS             {ULTRASONIC_SHORT                                                               };
  AnalogInput         m_USP             {ULTRASONIC_PWMWIRES                                                            };
  PWM                 m_Lidar           {LIDAR                                                                          };
  Relay               m_spike           {SPIKE                                                                          };
  DigitalInput        m_limitSwitch     {LIMIT                                                                          };
  BoxerbotsDistanceSensor m_distSensor;
  //ADIS16448_IMU m_imu {};
  
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
         DriveTrain()                           ;
  void   DriveTrainInit()                       ;
  void   DriveTrainEnd()                        ;
  double GetDistance()                          ;
  void   InitDefaultCommand() override          ;
  void   TeleopDrive(XboxController* controller);
  void   TurnSpikeOn()                          ;
  void   TurnSpikeOff()                         ;
  void   DistanceSensorInit()                   ;
  void   DistanceSensorDisabled()               ;
  void   DistanceSensorTeleop()                 ;
};