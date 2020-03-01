#pragma once

// Constants
const double kCloseToSameValue = 0.0000001;
const double kBasicNEOMaxVelocity = 5676.0;

// Drivetrain SparkMax CAN IDs =================================
// Left
const int kDrivetrainLeftPrimary = 1;
const int kDrivetrainLeftFollower1 = 2;
const int kDrivetrainLeftFollower2 = 3;
// Right
const int kDrivetrainRightPrimary = 4;
const int kDrivetrainRightFollower1 = 5;
const int kDrivetrainRightFollower2 = 6;
// Shooter SparkMax CAN IDs ===================================
const int kShooterPrimary = 7;
const int kShooterFollower = 8;
// Victor SPX CAN IDs
const int kIntakeVictorSpxCanId = 10;
const int kKickerVictorSpxCanId = 11;
const int kColorWheelVictorSpxCanId = 12;
const int kConveyorVictorSpxCanId = 13;
// Talon SRX CAN IDs
const int kClimbLeftTalonSrxCanId = 14;
const int kClimbRightTalonSrxCanId = 15;

// Pnuematics Control Module
const int kShooterAngleAdjusterPcmId = 0;
const int kColorWheelEngagePcmId = 1;

// Intake Sensor Digital IO ports
const int kNewPowercellSensor = 0;
const int kSecuredPowercellSensor = 1;
const int kKickerSensor = 2;

// Joystick Buttons
// driver
const int kFire = 1;
const int kQuickTurn = 2;
const int kIntakeToggle = 9;
// co-pilot
const int kGoToColor = 6;
const int kEngageColorWheel = 5;
const int kSpinColorWheel = 4;
const int kClimbExtend = 2;
const int kSetAngle = 12;
const int kClimbRetract = 1;
const int kReverseConveyor = 11;
const int kReverseIntake = 3;
const int kReverseKicker = 9;
const int kTurnToTarget = 7;
