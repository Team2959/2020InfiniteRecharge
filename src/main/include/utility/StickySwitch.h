#pragma once

#include <frc/DigitalInput.h>

namespace cwtech
{

class StickySwitch : public frc::DigitalInput
{
private:
    bool m_pressed;
    bool m_lastRead;
public:
    StickySwitch(int port);
    bool GetPressed();
    void ProcessForPressed();
};

}
