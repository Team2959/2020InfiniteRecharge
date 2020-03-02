/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <rev/CANSparkMax.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/geometry/Pose2d.h>
#include <frc/SPI.h>
#include <frc/ADXRS450_Gyro.h>

// Using navX from Kauai Labs
#include <AHRS.h>

#include <string>

#include <RobotMap.h>

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

  frc::SpeedControllerGroup m_leftGroup {m_leftPrimary};
  frc::SpeedControllerGroup m_rightGroup {m_rightPrimary};

  frc::DifferentialDrive m_differentialDrive {m_leftGroup, m_rightGroup};

  // constructing the navX device using the MXP port
  AHRS m_navX{frc::SPI::kMXP};

  // Smart Dashboard
  const std::string kName = "Drive/";
  const std::string kDebug = kName + "Debug";
  const std::string kPGain = kName + "P Gain";
  const std::string kIGain = kName + "I Gain";
  const std::string kFF = kName + "Feed Forward";
  const std::string kIZone = kName + "I Zone";
  const std::string kAutoKp = kName + "Auto Turn kP";
  const std::string kAutoLimitAngle = kName + "Auto Turn Limit Angle";
  const std::string kAutoMinSpeed = kName + "Auto Turn Min Speed";

  const double kOpenLoopRampRate = 0.25;
  const double kCurrentLimit = 50;
  const double kDefaultAutoKp = 0.05;
  const double kDefaultLimitAngle = 2.0;
  const double kDefaultMinSpeed = 0.1;

  bool m_debugEnable;

  double m_autoKp = kDefaultAutoKp;
  double m_autoLimitAngle = kDefaultLimitAngle;
  double m_autoMinSpeed = kDefaultMinSpeed;

  void SetupSparkMax(rev::CANSparkMax* controller);

public:
  static constexpr double kMaxVelocity = 4500; // 5676.0;

  Drivetrain();

  // Driving
  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void SetSpeeds(double left, double right);
  void CurvatureDrive(double speed, double rotation, bool quickTurn);
  void Drive(units::meter_t meters);
  double GetAngle();
  double GetPostion();
  bool IsAtPosition(double position);

  void InitalShowToSmartDashboard();
  void UpdateFromSmartDashboard();

  bool TryTurnToTargetAngle(double targetAngle);
};
