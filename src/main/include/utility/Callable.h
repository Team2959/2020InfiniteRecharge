#pragma once

namespace cwtech
{

class Callable
{
public:
    virtual void OnRobotInit() = 0;
    virtual void OnRobotPeriodic() = 0;
    virtual void OnTeleopInit() = 0;
    virtual void OnTeleopPeriodic() = 0;

    // Not always used so not pure virtual
    virtual void OnAutonomousInit() {};
    virtual void OnAutonomousPeriodic() {};
};

}
