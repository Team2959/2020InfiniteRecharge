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
#include <frc/buttons/JoystickButton.h>
#include <utility/Conditioning.h>
#include <subsystems/Drivetrain.h>
#include <subsystems/Intake.h>
#include <subsystems/Shooter.h>
#include <subsystems/ColorWheel.h>

class Robot : public frc::TimedRobot
{
private:
  // this variables is used to keep track of the times RobotPeriodic is called
  int m_skips = 0;
  int m_powercellsCounted = 0;
  int m_kickerPulseCounts = 0;

  // Joysticks 
  frc::Joystick m_driverJoystick {0};
  frc::Joystick m_coPilot {1};
  frc::JoystickButton m_quickTurn {&m_driverJoystick, kQuickTurn};

  cwtech::UniformConditioning m_driverSpeedConditioning {}; // Speed
  cwtech::UniformConditioning m_driverRotationConditioning {}; // Rotation
  
  const double kDefaultDeadband = 0.07;
  const double kDefaultOutputOffset = 0.0;
  const double kDefaultExponent = 3.0;

  bool m_passed2ndStage = false;

  // Drivetrain controller
  Drivetrain m_drivetrain {};
  Intake m_intake {};
  Shooter m_shooter {};
  // ColorWheel m_colorWheel {};

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

  void ClearPressedAndReleasedOperatorButtons();

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

  static double GetDistanceAngle(double distance);
  static std::tuple<double, double> GetMotorOutputForAimAndDrive(double targetY);

public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
};
