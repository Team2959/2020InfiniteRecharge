
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
    if(m_pressed == false && Get())
    {
        m_pressed = true;
    }
}
