#ifndef __CONTROL_PANEL_H__DEFINED
#define __CONTROL_PANEL_H__DEFINED

class ControlPanel{

public:
    void takeOff();
    void hover();
    void spinLeft();
    void spinRight();
    void goLeft();
    void goRight();
    void land();
    void up();
    void down();
    void forward();
    void backward();
    void reset();
    void flatTrim();
    void frontCam();
    void bottomCam();
};

#endif