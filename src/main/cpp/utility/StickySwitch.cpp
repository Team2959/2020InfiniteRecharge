
#include <utility/StickySwitch.h>

namespace cwtech
{

StickySwitch::StickySwitch(int port)
    : DigitalInput(port), m_pressed(false)
{
}

bool StickySwitch::GetPressed()
{
    if(m_pressed)
    {
        m_pressed = false;
        return true;
    }
    return false;
}

void StickySwitch::ProcessForPressed()
{
    if(m_pressed == false && Get())
    {
        m_pressed = true;
    }
}

}
