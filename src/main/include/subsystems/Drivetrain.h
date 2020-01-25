/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/geometry/Pose2d.h>

#include <string>

class Drivetrain
{
private:

  rev::CANSparkMax m_leftPrimary{1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollower1{2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollower2{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANPIDController m_leftPID{m_leftPrimary};
  rev::CANEncoder m_leftEncoder{m_leftPrimary};

  rev::CANSparkMax m_rightPrimary{4, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollower1{5, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollower2{6, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANPIDController m_rightPID{m_rightPrimary};
  rev::CANEncoder m_rightEncoder{m_rightPrimary};
public:
  Drivetrain();
  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void SetSpeeds(double left, double right);

  // SmartDashboard
  bool m_smartDashboardEnabled = true;
  void InitalShowToSmartDashboard();
  void UpdateFromSmartDashboard();

  // save pose for later focusing on drive
  void UpdatePose();
  void ResetPose(const frc::Pose2d& pose);
  frc::Pose2d GetPose();
private:
  std::string kName = "Drivetrain";
};
