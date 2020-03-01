/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <thread>
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/buttons/JoystickButton.h>
#include <utility/Conditioning.h>
#include <subsystems/Drivetrain.h>
#include <subsystems/Intake.h>
#include <subsystems/Shooter.h>
#include <subsystems/ColorWheel.h>
#include <subsystems/Climb.h>
#include <networktables/NetworkTableEntry.h>

static constexpr double PI{ 3.14159265359 };
static constexpr double DegreesToRadiansFactor{ PI / 180.0 };

static constexpr double DegreesToRadians(double degrees) { return degrees * DegreesToRadiansFactor; }
static constexpr double RadiansToDegrees(double radians) { return radians / DegreesToRadiansFactor; }

class Robot : public frc::TimedRobot
{
private:
  // this variables is used to keep track of the times RobotPeriodic is called
  int m_skips = 0;
  int m_powercellsCounted = 0;

  // Joysticks 
  frc::Joystick m_driverJoystick {0};
  frc::Joystick m_coPilot {1};
  frc::Joystick m_throttle {2};
  frc::JoystickButton m_quickTurn {&m_driverJoystick, kQuickTurn};

  cwtech::UniformConditioning m_driverSpeedConditioning {}; // Speed
  cwtech::UniformConditioning m_driverRotationConditioning {}; // Rotation
  
  const double kDefaultDeadband = 0.07;
  const double kDefaultOutputOffset = 0.0;
  const double kDefaultExponent = 3.0;
  const double kDefaultAutoTurnMultiplier = 0.05;
  const double kDefaultAutoTurnDegrees = 30.0;

  bool m_passed2ndStage = false;
  double m_autoTurnMultiplier = kDefaultAutoTurnMultiplier;
  double m_autoTurnDegrees = kDefaultAutoTurnDegrees;

  // Drivetrain controller
  Drivetrain m_drivetrain {};
  Intake m_intake {};
  Shooter m_shooter {};
  // ColorWheel m_colorWheel {};
  Climb m_climb {};

  nt::NetworkTableEntry m_tvEntry;
  nt::NetworkTableEntry m_txEntry;
  nt::NetworkTableEntry m_tyEntry;

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

  void ProcessUnjammingButtonPresses();
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

  static double GetTargetDistanceFromAngle(double angle);
  static double GetTargetAngleFromDistance(double distance);
  // bool IsTargetValid() const { return m_tvEntry.GetDouble(0.0) != 0.0; }
  bool IsTargetValid() const { return true; }
  double GetTargetDistance() const { return GetTargetDistanceFromAngle(GetTargetYAngle()); }
  std::tuple<double, double> GetMotorOutputForAimAndDrive(double targetY);
  double GetTargetXAngle() const;
  double GetTargetYAngle() const;

  void TurnToTarget();

  void UpdateActivePowerCells();

public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
};
