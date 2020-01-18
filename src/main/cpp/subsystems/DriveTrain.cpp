/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

using namespace frc;

DriveTrain::DriveTrain() : Subsystem("DriveTrain") {}

double DriveTrain::GetDistance()
{
  double centimeters = (counter->GetPeriod() * 100000.0 / 10.0);
  SmartDashboard::PutNumber("Lidar Value", centimeters );
  SmartDashboard::PutNumber("Raw (Get) Lidar Value", counter->Get());;;
  SmartDashboard::PutNumber("Raw (GetPeriod) Lidar Value", counter->GetPeriod());;;
  if (counter->Get() >= 1)
  {
    return centimeters;
  }
}

void DriveTrain::LidarInit()
{
  counter = new Counter(LIDAR);
  counter->SetMaxPeriod(1.0);
  counter->SetSemiPeriodMode(true);
  counter->Reset();
}

void DriveTrain::TeleopDrive(XboxController* controller)
{
  double rightY = controller->GetY(frc::GenericHID::kRightHand);
  SmartDashboard::PutNumber("Right Stick Value", rightY);
  double leftY = controller->GetY(frc::GenericHID::kLeftHand);
  SmartDashboard::PutNumber("Left Stick Value", leftY);

  m_leftMotor.Set(0.5*leftY);
  m_rightMotor.Set(-0.5*rightY);

  double angleOof = m_gyro.GetAngle();
  SmartDashboard::PutNumber("Don't break please", angleOof);

  double ultraValue = m_US.GetVoltage();
  SmartDashboard::PutNumber("Ultrasonic Value", ultraValue);
  const double voltToMeter = (4.885/5);
  const int calibration = 3;
  double distance = ultraValue * voltToMeter * calibration;
  SmartDashboard::PutNumber("Ultrasonic Distance", distance);

  bool limitSwitchValue = !(m_limitSwitch.Get());
  SmartDashboard::PutBoolean("Limit Switch Pressed", limitSwitchValue);
  if (limitSwitchValue)
  {
    const wpi::Twine error = "I lost the game";
    DriverStation::ReportError(error);
  }


  double temperature = m_imu.GetTemperature();
  double pressure = m_imu.GetBarometricPressure();
  double xangle = m_imu.GetGyroAngleX();
  double yangle = m_imu.GetGyroAngleY();
  double zangle = m_imu.GetGyroAngleZ();
  double accelx = m_imu.GetAccelInstantX();
  double accely = m_imu.GetAccelInstantY();
  double accelz = m_imu.GetAccelInstantZ();
  double magx = m_imu.GetMagInstantX();
  double magy = m_imu.GetMagInstantY();
  double magz = m_imu.GetMagInstantZ();
 SmartDashboard::PutNumber("Temp CELSIUS", temperature);
 SmartDashboard::PutNumber("Pressure MBAR", pressure);
 SmartDashboard::PutNumber("XAngle Degrees", xangle);
 SmartDashboard::PutNumber("YAngle Degrees", yangle);
 SmartDashboard::PutNumber("ZAngle Degrees", zangle);
 SmartDashboard::PutNumber("Accelerometer X", accelx);
 SmartDashboard::PutNumber("Accelerometer Y", accely);
 SmartDashboard::PutNumber("Accelerometer Z", accelz);
 SmartDashboard::PutNumber("Magnetometer X", magx);
 SmartDashboard::PutNumber("Magnetometer Y", magy);
 SmartDashboard::PutNumber("Magnetometer Z", magz);

}

void DriveTrain::StopDriveMotors()
{
  m_leftMotor.Set(0.0);
  m_rightMotor.Set(0.0);
}

void DriveTrain::InitDefaultCommand() 
{
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void DriveTrain::TurnSpikeOn()
{
  m_spike.Set(Relay::kForward);
}


void DriveTrain::TurnSpikeOff()
{
  m_spike.Set(Relay::kOff);
}


// Put methods for controlling this subsystem
// here. Call these from Commands.
