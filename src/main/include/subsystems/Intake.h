#pragma once

#include <frc/DigitalInput.h>
#include <frc/VictorSP.h>

#include <RobotMap.h>

class Intake
{
private:
    frc::DigitalInput m_endSensor{kEndSensor};
    frc::DigitalInput m_newPowercellSensor{kNewPowercellSensor};
    frc::DigitalInput m_securedPowercellSensor{kSecuredPowercellSensor};

    frc::VictorSP m_intakeWheelMotor1{kIntakeMotor1};
    frc::VictorSP m_intakeWheelMotor2{kIntakeMotor2};

    frc::VictorSP m_conveyorMotor1{kConveyorMotor1};
    frc::VictorSP m_conveyorMotor2{kConveyorMotor2};
public:
    enum class SensorLocation
    {
        End,
        NewPowercell,
        SecuredPowercell
    };
    bool GetSensor(SensorLocation location);
};