#pragma once

#include <vector>
#include <functional>

class StateMachine
{
private:
    std::function<void()> m_activeState;
public:
    StateMachine();
    template<typename F>
    void ActivateState(F func);
    template<typename T, typename F>
    void ActivateState(T obj, F func);
    void Update();
};
