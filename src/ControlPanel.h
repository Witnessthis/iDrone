#ifndef __CONTROL_PANEL_H__DEFINED
#define __CONTROL_PANEL_H__DEFINED

//collection of drone navigation command functions
class ControlPanel{

public:
    void takeOff();
    void hover();
    void spinLeft();
    void spinRight();
    void goLeft(float speed);
    void goRight(float speed);
    void land();
    void up(float speed);
    void down(float speed);
    void forward(float speed);
    void backward(float speed);
    void reset();
    void flatTrim();
    void frontCam();
    void bottomCam();
    void diagForwardRight();
    void diagBackwardRight();

    //model updates
    void updateSearchState();
    void updateNextAirfield();
    void updateFoundAirfield(int wallID);
    void updateCurrentWallmarking(int wallID);
};

#endif