#pragma once

#include <rev/CANSparkMax.h>

#include <RobotMap.h>

class Shooter
{
private:
    rev::CANSparkMax m_left{kShooterLeft, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_right{kShooterRight, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
public:
};
