#ifndef __STATE_H__DEFINED
#define __STATE_H__DEFINED

class State{
public:
    bool mayAct;

    State();
    ~State();
};

enum States_e {state1_e, state2_e, state3_e, NUM_STATES, NO_TRANSITION};

class state1 : public State{
public:
    States_e getNext();
    void act();
};

class state2 : public State{
public:
    States_e getNext();
    void act();
};

class state3 : public State{
public:
    States_e getNext();
    void act();
};

#endif