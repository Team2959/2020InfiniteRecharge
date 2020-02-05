#pragma once

class Intake
{
public:
    enum class SensorLocation
    {
        End,
        None,
    };
    bool GetSensor(SensorLocation location);
    void SetConveyorSpeed(double speed);
};