#pragma once

#include <subsystems/Intake.h>
#include <subsystems/Shooter.h>
#include <subsystems/ColorWheel.h>
#include <subsystems/Vision.h>
#include <subsystems/Drivetrain.h>
#include <frc/Joystick.h>

enum class States
{
    Traveling,
    Firing,
    Climbing,
    ColorWheel,
    Loading,
    Ready,
};


class StateManager
{
private:
    Intake& m_intake;
    Shooter& m_shooter;
    Vision& m_vision;
    Drivetrain& m_drivetrain;
    States m_currentState;
    frc::Joystick& m_driverJoystick;
    frc::Joystick& m_coPilotJoystick;

    int m_powercellsCounted;

    void ProcessUnjammingButtonPresses();
    void ClearPressedAndReleasedOperatorButtons();

    void TravelingInit();
    void TravelingPeriodic();
    void FiringInit();
    void FiringPeriodic();
    void ClimbingInit();
    void ClimbingPeriodic();
    void ColorWheelInit();
    void ColorWheelPeriodic();
    void LoadingInit();
    void LoadingPeriodic();

    void UpdateActivePowerCells();

public:
    StateManager(Intake& intake, Shooter& shooter, Vision& vision, Drivetrain& drivetrain, frc::Joystick& driverJoystick, frc::Joystick& coPilotJoystick, int powerCellsCount);

    States CurrentState() const { return m_currentState; }
    bool ArePowerCellsEmpty() const { return m_powercellsCounted == 0; }
    void StartState(States state);
    void Periodic();
    void Reset();
};

