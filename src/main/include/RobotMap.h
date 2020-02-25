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
const int kGoToColor = 1;
const int kEngageColorWheel = 2;
const int kSpinColorWheel = 3;
const int kClimbExtend = 5;
const int kSetAngle = 6;
const int kClimbRetract = 7;
const int kReverseConveyor = 9;
const int kReverseIntake = 10;
