#pragma once

#include <string>
#include <tuple>
#include <frc/util/Color.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/Solenoid.h>
#include <frc/SerialPort.h>

#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <RobotMap.h>

class ColorWheel
{
private:
    static constexpr frc::Color kBlack = frc::Color(0,0,0);
    // Color Sensor - light off
    static constexpr frc::Color kBlueTarget = frc::Color(0.165, 0.467, 0.368);
    static constexpr frc::Color kGreenTarget = frc::Color(0.213, 0.614, 0.176);
    static constexpr frc::Color kRedTarget = frc::Color(0.663, 0.273, 0.063);
    static constexpr frc::Color kYellowTarget = frc::Color(0.423, 0.502, 0.071);
    static constexpr frc::Color kBlueGreenTarget = frc::Color(0.18, 0.506, 0.31);
    static constexpr frc::Color kBlueYellowTarget = frc::Color(0.352, 0.494, 0.155);
    static constexpr frc::Color kGreenRedTarget = frc::Color(0.472, 0.414, 0.115);
    static constexpr frc::Color kRedYellowTarget = frc::Color(0.516, 0.407, 0.067);
    // Color Sensor - light on
    // static constexpr frc::Color kBlueTarget = frc::Color(0.14, 0.42, 0.438);
    // static constexpr frc::Color kGreenTarget = frc::Color(0.183, 0.581, 0.235);
    // static constexpr frc::Color kRedTarget = frc::Color(0.49, 0.155, 0.156);
    // static constexpr frc::Color kYellowTarget = frc::Color(0.32, 0.553, 0.125);
    // static constexpr frc::Color kBlueGreenTarget = frc::Color(0.151, 0.464, 0.384);
    // static constexpr frc::Color kBlueYellowTarget = frc::Color(0.253, 0.506, 0.238);
    // static constexpr frc::Color kGreenRedTarget = frc::Color(0.307, 0.488, 0.203);
    // static constexpr frc::Color kRedYellowTarget = frc::Color(0.393, 0.468, 0.139);

    rev::ColorSensorV3 m_colorSensor {frc::I2C::Port::kOnboard};
    rev::ColorMatch m_colorMatcher;

    frc::Color m_countedColor = kGreenTarget;
    frc::Color m_gameDataTargetColor = kBlack;
    frc::Color m_lastColor = kBlack;
    bool m_countColors = false;
    bool m_logColors = false;
    int m_colorCount = 0;

    std::vector<std::tuple< std::string/* Guessed Color */, double /* Red */, double /* Green */, double /* Blue */ > > m_colorTracking;

    std::string ColorName(frc::Color matchedColor);
    frc::Color GetColorFromName(std::string colorName);
    void SetTargetColorFromGameData();

    frc::Solenoid m_engageColorWheel{kColorWheelEngageColorWheel};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_spinMotor{kColorWheelSpinMotor};

    frc::SerialPort m_bling {115200, frc::SerialPort::kUSB1};
    
public:
    void OnRobotInit();
    void UpdateColorSensorValues(int skips);

    void SetSpinMotorSpeed(double speed);
    void EngageColorWheel(bool engage);

    void ResetCounter();
    void UpdateCount();
    int GetCount();
    frc::Color GetColorToSpinTo();
    frc::Color GetCurrentColor();
};
