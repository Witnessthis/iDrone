#ifndef __STATE_H__DEFINED
#define __STATE_H__DEFINED

#include "model.h"
#include "ControlPanel.h"

class State{
public:
    bool mayAct;
    ControlPanel controlPanel;

    State();
    ~State();
};

enum States_e {START_e, CALIBRATE_e, SEARCH_e, NO_MATCH_e, MOVE_e, ADJUST_FRONT_e, TURN_e, MATCH_e, OLD_AIRFEILD_e, NEW_AIRFEILD_e, NUM_STATES, NO_TRANSITION};

class StartState : public State{
public:
    States_e getNext(model_s model);
    void act(model_s model);
};

class CalibrateState : public State{
public:
    States_e getNext(model_s model);
    void act(model_s model);
};

class SearchState : public State{
public:
    States_e getNext(model_s model);
    void act(model_s model);
};

class NoMatchState : public State{
public:
    States_e getNext(model_s model);
    void act(model_s model);
};

class MoveState : public State{
public:
    States_e getNext(model_s model);
    void act(model_s model);
};

class AdjustFrontState : public State{
public:
    States_e getNext(model_s model);
    void act(model_s model);
};

class TurnState : public State{
public:
    States_e getNext(model_s model);
    void act(model_s model);
};

class MatchState : public State{
public:
    States_e getNext(model_s model);
    void act(model_s model);
};

class OldAirfeildState : public State{
public:
    States_e getNext(model_s model);
    void act(model_s model);
};

class NewAirfeildState : public State{
public:
    States_e getNext(model_s model);
    void act(model_s model);
};

#endif