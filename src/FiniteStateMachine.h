#ifndef __FINITESTATEMACHINE_H__DEFINED
#define __FINITESTATEMACHINE_H__DEFINED

#include "State.h"
#include <vector>

class FiniteStateMachine{

public:
    States_e currentState;

    FiniteStateMachine();
    ~FiniteStateMachine();
    void act();
    void update();

    //std::vector<State> states;

};

#endif