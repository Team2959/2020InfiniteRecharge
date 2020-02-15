
#include <utility/StickySwitch.h>

cwtech::StickySwitch::StickySwitch(int port)
    : DigitalInput(port), m_pressed(false)
{
}

bool cwtech::StickySwitch::GetPressed()
{
    if(m_pressed)
    {
        m_pressed = false;
        return true;
    }
    return false;
}

void cwtech::StickySwitch::ProcessForPressed()
{
    auto currentState = Get();
    if(m_pressed == false && currentState == true && m_lastRead == false)
    {
        m_pressed = true;
    }
    m_lastRead = currentState;
}
