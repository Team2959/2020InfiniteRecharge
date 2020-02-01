/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <rev/CANSparkMax.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/geometry/Pose2d.h>
#include <frc/SPI.h>
#include <frc/ADXRS450_Gyro.h>

// Using navX from Kauai Labs
#include <AHRS.h>

#include <string>

#include <RobotMap.h>

#include <utility/Conditioning.h>

class Drivetrain
{
private:

  rev::CANSparkMax m_leftPrimary{kDrivetrainLeftPrimary, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollower1{kDrivetrainLeftFollower1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollower2{kDrivetrainLeftFollower2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANPIDController m_leftPID{m_leftPrimary};
  rev::CANEncoder m_leftEncoder{m_leftPrimary};

  rev::CANSparkMax m_rightPrimary{kDrivetrainRightPrimary, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollower1{kDrivetrainRightFollower1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollower2{kDrivetrainRightFollower2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANPIDController m_rightPID{m_rightPrimary};
  rev::CANEncoder m_rightEncoder{m_rightPrimary};

// constructing the navX device using the MXP port
  AHRS m_navX{frc::SPI::kMXP};

  cwtech::UniformConditioning conditioning{};
  std::string kName = "Drivetrain";
public:
  Drivetrain();

  // Driving
  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void SetSpeeds(double left, double right);

  // SmartDashboard

  // bool m_smartDashboardEnabled = true;
  // may not be used --^

  void InitalShowToSmartDashboard();
  void UpdateFromSmartDashboard();

  // save pose for later focusing on drive
  void UpdatePose();
  void ResetPose(const frc::Pose2d& pose);
  frc::Pose2d GetPose();
};
