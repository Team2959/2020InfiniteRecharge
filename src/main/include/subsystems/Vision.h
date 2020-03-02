#pragma once

#include <networktables/NetworkTableEntry.h>

class Vision
{
private:
    nt::NetworkTableEntry m_tvEntry;
    nt::NetworkTableEntry m_txEntry;
    nt::NetworkTableEntry m_tyEntry;

    static double GetTargetDistanceFromAngle(double angle);
    static double GetTargetAngleFromDistance(double distance);
    // bool IsTargetValid() const { return m_tvEntry.GetDouble(0.0) != 0.0; }
    // bool IsTargetValid() const { return true; }
    double GetTargetDistance() const { return GetTargetDistanceFromAngle(GetTargetYAngle()); }
    std::tuple<double, double> GetMotorOutputForAimAndDrive(double targetY);

    double GetTargetXAngleRadians() const;
    double GetTargetYAngleRadians() const;
    double GetTargetYAngleDegrees() const;

public:
    void OnRopotInit();
    void OnRobotPeriodic();

    double GetTargetXAngleDegrees() const;
};
