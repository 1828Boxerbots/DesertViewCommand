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


  const double voltToMeter = (4.885/5);
  // A meter is 39.37 inches: Use this to ensure you are measuring proper distance.
  const double calibrationL = 3.0;
  const double calibrationS = 1.2;
  const double calibrationP = 1.15;


  double ultraValueL = m_USL.GetVoltage();
  SmartDashboard::PutNumber("Ultrasonic Long Voltage", ultraValueL);
  double distanceL = ultraValueL * voltToMeter * calibrationL;
  SmartDashboard::PutNumber("Ultrasonic Long Distance M", distanceL);


  double ultraValueS = m_USS.GetVoltage();
  SmartDashboard::PutNumber("Ultrasonic Short Voltage", ultraValueS);
  double distanceS = ultraValueS * voltToMeter * calibrationS;
  SmartDashboard::PutNumber("Ultrasonic Short Distance M", distanceS);


  double ultraValueP = m_USP.GetVoltage();
  SmartDashboard::PutNumber("Ultrasonic PWM Wired Voltage", ultraValueP);
  double distanceP = ultraValueP * voltToMeter * calibrationP;
  SmartDashboard::PutNumber("Ultrasonic PWM Wired Distance M", distanceP);


  bool limitSwitchValue = !(m_limitSwitch.Get());
  SmartDashboard::PutBoolean("Limit Switch Pressed", limitSwitchValue);

  if (limitSwitchValue)
  {
    const wpi::Twine error = "I lost the game";
    DriverStation::ReportError(error);
    TurnSpikeOn();
  }
  else
  {
    TurnSpikeOff();
  }

  /*
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
 */
}

void DriveTrain::StopDriveMotors()
{
  m_leftMotor.Set(0.0);
  m_rightMotor.Set(0.0);
}


void DriveTrain::InitDefaultCommand() {}


void DriveTrain::TurnSpikeOn()
{
  m_spike.Set(Relay::kForward);
}


void DriveTrain::TurnSpikeOff()
{
  m_spike.Set(Relay::kOff);
}


void DriveTrain::DistanceSensorInit()
{
  m_distSensor.SetAutomaticMode(true);
  m_distSensor.SetEnabled(true);
}


void DriveTrain::DistanceSensorTeleop()
{
  bool isValid = m_distSensor.IsRangeValid();

  frc::SmartDashboard::PutBoolean("Data Valid", isValid);

  if(isValid) 
  {
  /**
   * The current measured range is returned from GetRange(). By default
   * this range is returned in inches.
   */
  frc::SmartDashboard::PutNumber("Distance (in)", m_distSensor.GetRange());

  /**
   * The timestamp of the last valid measurement (measured in seconds since 
   * the program started), is returned by GetTimestamp().
   */
  frc::SmartDashboard::PutNumber("Timestamp", m_distSensor.GetTimestamp());
  }
  else 
  {
  frc::SmartDashboard::PutNumber("Distance (in)", -1);
  }
}


void DriveTrain::DistanceSensorDisabled()
{
  m_distSensor.SetAutomaticMode(false);
  m_distSensor.SetEnabled(false);
}


// Put methods for controlling this subsystem
// here. Call these from Commands.
