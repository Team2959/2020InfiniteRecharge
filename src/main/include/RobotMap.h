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
// Shooter TalonSRX CAN IDs ===================================
const int kShooterKicker = 14;

// Pnuematics Control Module
const int kShooterPnuematicsAngleAdjuster = 0;

// Intake Sensor Digital IO ports
const int kEndSensor = 0;
const int kNewPowercellSensor = 1;
const int kSecuredPowercellSensor = 2;

// Intake Picker-uper Motor's ports 
const int kIntakeMotor1 = 10;
const int kIntakeMotor2 = 11;

const int kConveyorMotor1 = 12;
const int kConveyorMotor2 = 13;

