#ifndef __CONTROL_PANEL_H__DEFINED
#define __CONTROL_PANEL_H__DEFINED

class ControlPanel{

public:
    void takeOff();
    void hover();
    void spinLeft();
    void spinRight();
    void goLeft(float speed);
    void goRight(float speed);
    void land();
    void up();
    void down();
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