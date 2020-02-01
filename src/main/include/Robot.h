/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>

#include <subsystems/Drivetrain.h>

#include <utility/Callable.h>
#include <utility/Conditioning.h>

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
private:
// this variables is used to keep track of the times RobotPeriodic is called
  int m_skips = 0;

// Joysticks 
  frc::Joystick m_left{0};
  frc::Joystick m_right{1};

  cwtech::UniformConditioning m_conditioning{};

// Drivetrain controller
  Drivetrain m_drivetrain{};
};
