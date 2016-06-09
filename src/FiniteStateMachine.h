#ifndef __FINITESTATEMACHINE_H__DEFINED
#define __FINITESTATEMACHINE_H__DEFINED

#include "State.h"
#include <vector>
#include "model.h"

class FiniteStateMachine{

public:
    States_e currentState;

    FiniteStateMachine();
    ~FiniteStateMachine();
    void act(model_s model);
    void update(model_s model);

};

#endif