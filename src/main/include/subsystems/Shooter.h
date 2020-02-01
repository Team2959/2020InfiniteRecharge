#pragma once

#include <rev/CANSparkMax.h>

#include <RobotMap.h>

class Shooter
{
private:
    rev::CANSparkMax m_primary{kShooterPrimary, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_follower{kShooterFollower, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANEncoder m_encoder{m_primary};
    rev::CANPIDController m_PID{m_primary};
public:
    void OnRobotInit();
    void SetSpeed(double speed);
};
