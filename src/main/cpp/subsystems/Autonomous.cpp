
#include <subsystems/Autonomous.h>

Autonomous::Autonomous(StateManager& stateManager, Shooter& shooter, Drivetrain& driveTrain) 
    : m_stateManager(stateManager), m_shooter(shooter), m_driveTrain(driveTrain)
{   
}

void Autonomous::OnAutoInit()
{
    m_shooter.SetAngle(false);
    // read starting position from dashboard
    m_shooter.SetSpeed(2500);
    // m_shooter.SetSpeedFromTargetDistance(m_vision.GetTargetDistanceInInches());
}

void Autonomous::Periodic()
{
    if(m_shooter.CloseToSpeed() && m_stateManager.CurrentState() != States::Firing)
    {
        m_stateManager.StartState(States::Firing);
    }
    else if( m_stateManager.CurrentState() == States::Firing && m_stateManager.ArePowerCellsEmpty())
    {
        m_stateManager.StartState(States::Traveling);
        m_shooter.SetSpeed(1000);   // idle shooter speed
        m_autoDriveDistanceTracker.StartingPosition(m_driveTrain.GetPostion());
        m_driveTrain.SetSpeeds(-Drivetrain::kMaxVelocity * 0.5, 
                               -Drivetrain::kMaxVelocity * 0.5);
    }
    else if( m_stateManager.CurrentState() == States::Traveling &&
             m_autoDriveDistanceTracker.GetDistanceInInches(m_driveTrain.GetPostion()) <= (-5.0 * 12.0))
    {
        m_driveTrain.SetSpeeds(0,0);
        m_autoTurnTargetAngle = m_driveTrain.GetAngle() + 90;
    }
    else if( m_stateManager.CurrentState() == States::Traveling && m_autoTurnTargetAngle != 0)
    {
        if(m_driveTrain.TryTurnToTargetAngle(m_autoTurnTargetAngle) == false)
        {
            m_autoTurnTargetAngle = 0;
        }
    }
}
