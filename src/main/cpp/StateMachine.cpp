#include <StateMachine.h>

StateMachine::StateMachine()
{

}

template<typename F>
void StateMachine::ActivateState(F func)
{
    m_activeState = func;
}

template<typename T, typename F>
void StateMachine::ActivateState(T obj, F func)
{
    m_activeState = std::bind(func, obj);
}

void StateMachine::Update()
{
    // Call current state
    m_activeState();
}
