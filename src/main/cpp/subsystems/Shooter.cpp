#include <subsystems/Shooter.h>

void Shooter::OnRobotInit()
{
    // Have the follower follow the primary except invert 
    // because they are opposite of one another
    m_follower.Follow(m_primary, true);
}

void Shooter::SetSpeed(double speed)
{
    if(speed >= 0) 
    {
        m_PID.SetReference(speed, rev::ControlType::kVelocity);
    }
}
