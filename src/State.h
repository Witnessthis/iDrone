#ifndef __STATE_H__DEFINED
#define __STATE_H__DEFINED

#include "model.h"
#include "ControlPanel.h"
#include <ros/ros.h>
#include <ctime>
#include <chrono>

#define ADJUSTED_BORDER_HEIGHT_P 116
#define ADJUSTED_BORDER_MARGIN_P 10
#define ADJUSTED_ERROR_MARGIN_P 2
#define ADJUSTED_BOTTOM_MARGIN 10
#define ADJUSTED_RIGHT_CENTER_MARGIN 0.3
#define ADJUSTED_LEFT_CENTER_MARGIN -0.3

#define DIAGONAL_MOVEMENT_T 500
#define STRAIGHT_MOVEMENT_T 750
#define FREEZE_TIME_T 1000

bool isFrontAdjusted(int r, int l, int t, int b, int c);
bool isBottomAdjusted(float dx, float dy);

class State{
public:
    bool mayAct;
    ControlPanel controlPanel;

    State();
    ~State();
};

enum States_e {START_e, CALIBRATE_e, SEARCH_e, MOVE_NEW_POS_e, MOVE_e, ADJUST_FRONT_e, ADJUST_BOTTOM_e, MATCH_e, OLD_AIRFEILD_e, NEW_AIRFEILD_e, NUM_STATES, NO_TRANSITION};

enum Search_Pattern_e {MOVEMENT_FREEZE, MOVEMENT1_e, MOVEMENT2_e, MOVEMENT3_e, MOVEMENT4_e, MOVEMENT5_e, MOVEMENT_COMPLETE_e};

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
    //time_t start;
    std::chrono::milliseconds start;
    Search_Pattern_e pattern;
    void reset();
    States_e getNext(model_s model);
    void act(model_s model);
};

class MoveNewPosState : public State{
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

class AdjustBottomState : public State{
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