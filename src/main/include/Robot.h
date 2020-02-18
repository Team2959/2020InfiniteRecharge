/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>

#include <frc/Joystick.h>
#include <utility/Conditioning.h>

#include <subsystems/Drivetrain.h>
#include <subsystems/Intake.h>
#include <subsystems/Shooter.h>

class Robot : public frc::TimedRobot
{
private:
  // this variables is used to keep track of the times RobotPeriodic is called
  int m_skips = 0;
  int m_powercellsCounted = 0;
  int m_kickerRampIncrements = 0;

  // Joysticks 
  frc::Joystick m_leftDriverJoystick {0};
  frc::Joystick m_rightDriverJoystick {1};
  cwtech::UniformConditioning m_conditioningDriverJoysticks {};

  // Drivetrain controller
  Drivetrain m_drivetrain {};
  Intake m_intake {};
  Shooter m_shooter {};

  enum class States
  {
    Traveling,
    Firing,
    Climbing,
    ColorWheel,
    Loading,
  };

  States m_currentState = States::Traveling;

  void SwitchState(States state);
  void DoCurrentState();

  void TravelingInit();
  void TravelingPeriodic();
  void FiringInit();
  void FiringPeriodic();
  void ClimbingInit();
  void ClimbingPeriodic();
  void ColorWheelInit();
  void ColorWheelPeriodic();
  void LoadingInit();
  void LoadingPeriodic();

public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
};
