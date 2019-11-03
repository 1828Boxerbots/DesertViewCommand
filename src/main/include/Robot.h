/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <frc/IterativeRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Spark.h>
#include <frc/XboxController.h>
#include <frc/Encoder.h>
#include <frc/Compressor.h>


class Robot : public frc::IterativeRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
    //robotmap
        //setting DIO ports
            //left encoders
                const int LEFTCHANELA = 0;
                const int LEFTCHANELB = 1;
            //right encoder
                const int RIGHTCHANELA = 2;
                const int RIGHTCHANELB = 3;
        //setting pwm ports
            //setting the pwm ports for the spark controllers(each two motor on both side are y split)
                const int PWMLEFTMOTOR = 2;
                const int PWMRIGHTMOTRS = 1;
        //setting USB ports
            //setting the USB ports for the controller
                const int USBXBOXCONTROLLER = 0;
        //setting the PCM ports

                const int PCMCOMPRESSOR = 0;

    //setting the encoder types
        frc::Encoder m_leftEncoder{LEFTCHANELA,LEFTCHANELB,false,frc::CounterBase::k4X};
        frc::Encoder m_rightEncoder{RIGHTCHANELA,RIGHTCHANELB,true,frc::CounterBase::k4X};
    

    /*using the spark constructer to create a spark controller for the y split motors on the left side
    (if your using a motor controller that isn't spark, for example talons, us the coraspoing liberay for that class)*/
        frc::Spark m_left{PWMLEFTMOTOR};

    /*using the spark constructer to create a spark controller for the y split motors on the left side
    (if your using a motor controller that isn't spark, for example talons, us the coraspoing liberay for that class)*/
        frc::Spark m_right{PWMRIGHTMOTRS};

    // Createing a object for the controller( using xbox controller here, but if you going to use a joystick)
        frc::XboxController m_controller{USBXBOXCONTROLLER};

        frc::Compressor m_compressor{PCMCOMPRESSOR};
 
};
