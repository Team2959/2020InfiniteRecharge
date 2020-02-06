#pragma once

#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <RobotMap.h>

class Intake
{
private:
    frc::DigitalInput m_endSensor{kEndSensor};
    frc::DigitalInput m_newPowercellSensor{kNewPowercellSensor};
    frc::DigitalInput m_securedPowercellSensor{kSecuredPowercellSensor};

    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intakeWheelMotor1{kIntakeMotor1};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intakeWheelMotor2{kIntakeMotor2};

    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyorMotor1{kConveyorMotor1};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyorMotor2{kConveyorMotor2};
public:
    enum class SensorLocation
    {
        End,
        NewPowercell,
        SecuredPowercell
    };
    bool GetSensor(SensorLocation location);
};