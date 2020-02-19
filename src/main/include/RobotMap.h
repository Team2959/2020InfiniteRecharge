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
// Intake Picker-upper Victor SPX Motor's port
const int kIntakePrimary = 10;
// Kicker Victor SPX Motor's port
const int kKicker = 11;
// Conveyor Victor SPX Motor's ports
const int kConveyorPrimary = 13;

// Pnuematics Control Module
const int kShooterPnuematicsAngleAdjuster = 0;

// Intake Sensor Digital IO ports
const int kNewPowercellSensor = 0;
const int kSecuredPowercellSensor = 1;
const int kKickerSensor = 2;

const int kColorWheelEngageColorWheel = 1;
const int kColorWheelSpinMotor = 12;
