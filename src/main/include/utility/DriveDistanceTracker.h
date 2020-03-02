#pragma once

#include <subsystems/Drivetrain.h>

class DriveDistanceTracker
{
protected:
    Drivetrain& m_drivetrain;
    double m_startingEncoderTicks;
    static constexpr double kInchesPerTick{0.05155}; 
public:
    DriveDistanceTracker(Drivetrain& drivetrain);
    double GetDistanceInInches();
};