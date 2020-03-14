#pragma once

#include <networktables/NetworkTableEntry.h>

enum class CameraMode { VisionProcessing = 0, Driver = 1 };

class Vision
{
private:
    nt::NetworkTableEntry m_camModeEntry;
    nt::NetworkTableEntry m_tvEntry;
    nt::NetworkTableEntry m_txEntry;
    nt::NetworkTableEntry m_tyEntry;

    static double GetTargetDistanceFromAngle(double angle);
    static double GetTargetAngleFromDistance(double distance);
    // bool IsTargetValid() const { return m_tvEntry.GetDouble(0.0) != 0.0; }
    // bool IsTargetValid() const { return true; }
    std::tuple<double, double> GetMotorOutputForAimAndDrive(double targetY);

    double GetTargetXAngleRadians() const;
    double GetTargetYAngleRadians() const;
    double GetTargetYAngleDegrees() const;

public:
    void OnRopotInit();
    void OnRobotPeriodic();
    void OnTeleopPeriodic();

    double GetTargetDistanceInInches() const { return GetTargetDistanceFromAngle(GetTargetYAngleDegrees()); }
    double GetTargetXAngleDegrees() const;
    CameraMode GetCameraMode() const;
    void SetCameraMode(CameraMode mode);
};
