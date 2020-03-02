
#include <utility/DriveDistanceTracker.h>

DriveDistanceTracker::DriveDistanceTracker(Drivetrain& drivetrain)
    : m_drivetrain(drivetrain), m_startingEncoderTicks(m_drivetrain.GetPostion())
{
}

double DriveDistanceTracker::GetDistanceInInches()
{
    return (m_drivetrain.GetPostion() - m_startingEncoderTicks) * kInchesPerTick;
}