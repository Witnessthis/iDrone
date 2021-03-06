#ifndef __STATE_H__DEFINED
#define __STATE_H__DEFINED

#include "model.h"
#include "ControlPanel.h"
#include <ros/ros.h>
#include <ctime>
#include <chrono>

#define ADJUSTED_BORDER_HEIGHT_CM 150 //TODO: fix this naming issue!!!  this value NEEDS to be between 40 - 250.
#define ADJUSTED_BORDER_MARGIN_P 5
#define ADJUSTED_ERROR_MARGIN_P 0
#define ADJUSTED_BOTTOM_MARGIN 50
#define ADJUSTED_RIGHT_CENTER_MARGIN 0.2
#define ADJUSTED_LEFT_CENTER_MARGIN -0.2

int cmToPixel(int cm);

const int adjusted_border_height_p = 78;//cmToPixel(ADJUSTED_BORDER_HEIGHT_CM);

#define STRAIGHT_MOVEMENT_T 1000
#define FREEZE_TIME_T 5000

#define BAD_MATCH_TOLERANCE 100
#define REQUIRED_CONSECUTIVE_MATCHES 8
#define STABILIZE_TIMER_T 2000

bool isFrontAdjusted(int r, int l, int t, int b, float c);
bool isBottomAdjusted(float dx, float dy);

class State{
public:
    bool mayAct;
    ControlPanel controlPanel;

    State();
    ~State();
};

enum States_e {START_e, CALIBRATE_e, SEARCH_e, MOVE_NEW_POS_e, MOVE_e, ADJUST_FRONT_e, ADJUST_BOTTOM_e, MATCH_e, OLD_AIRFEILD_e, NEW_AIRFEILD_e, NUM_STATES, NO_TRANSITION};

enum Search_Pattern_e {MOVEMENT_FREEZE, MOVEMENT_UP_e, MOVEMENT1_e, MOVEMENT2_e, MOVEMENT3_e, MOVEMENT4_e, MOVEMENT5_e, MOVEMENT_COMPLETE_e};

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
    time_t start;
    void reset();
    States_e getNext(model_s model);
    void act(model_s model);
};

class AdjustBottomState : public State{
public:
    bool doLand;
    std::chrono::milliseconds start;
    void reset();
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