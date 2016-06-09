#ifndef __STATE_H__DEFINED
#define __STATE_H__DEFINED

#include "model.h"

class State{
public:
    bool mayAct;

    State();
    ~State();
};

enum States_e {START_e, CALIBRATE_e, SEARCH_e, NO_MATCH_e, MOVE_e, FORWARD_e, TURN_e, MATCH_e, OLD_AIRFEILD_e, NEW_AIRFEILD_e, NUM_STATES, NO_TRANSITION};

class StartState : public State{
public:
    States_e getNext(model_s model);
    void act();
};

class CalibrateState : public State{
public:
    States_e getNext(model_s model);
    void act();
};

class SearchState : public State{
public:
    States_e getNext(model_s model);
    void act();
};

class NoMatchState : public State{
public:
    States_e getNext(model_s model);
    void act();
};

class MoveState : public State{
public:
    States_e getNext(model_s model);
    void act();
};

class ForwardState : public State{
public:
    States_e getNext(model_s model);
    void act();
};

class TurnState : public State{
public:
    States_e getNext(model_s model);
    void act();
};

class MatchState : public State{
public:
    States_e getNext(model_s model);
    void act();
};

class OldAirfeildState : public State{
public:
    States_e getNext(model_s model);
    void act();
};

class NewAirfeildState : public State{
public:
    States_e getNext(model_s model);
    void act();
};

#endif