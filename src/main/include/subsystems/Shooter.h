#pragma once

#include <RobotMap.h>
#include <rev/CANSparkMax.h>

class Shooter
{
private:
    rev::CANSparkMax m_primary {kShooterPrimary, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_follower {kShooterFollower, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANEncoder m_encoder {m_primary};
    rev::CANPIDController m_PID {m_primary};

    std::string kName = "Shooter";

public:
    Shooter();

    void OnRobotInit();
    void OnRobotPeriodic();

    void SetSpeed(double speed);
};
