#include "State.h"
#include <iostream>

State::State() {
    mayAct = true;
}

State::~State() { }

States_e StartState::getNext(model_s model) {
    std::cout << "battery test" << model.navdata.batteryPercent << std::endl;

    if(model.navdata.state == 4){//drone is hovering
        return CALIBRATE_e;
    }

    return NO_TRANSITION;
}

void StartState::act() {
    //TODO set calibration flag false
}

States_e CalibrateState::getNext(model_s model) {
    if(model.hasCalibrated){
        return MOVE_e;
    }

    return NO_TRANSITION;
}

void CalibrateState::act() { }

States_e SearchState::getNext(model_s model) {

    return NO_TRANSITION;
}

void SearchState::act() { }

States_e NoMatchState::getNext(model_s model) {

    return NO_TRANSITION;
}

void NoMatchState::act() { }

States_e MoveState::getNext(model_s model) {


    return NO_TRANSITION;
}

void MoveState::act() {
    //front camera
    //hover
    //spin slowly
    //wait for opencv input

}

States_e ForwardState::getNext(model_s model) {
    return NO_TRANSITION; }

void ForwardState::act() { }

States_e TurnState::getNext(model_s model) {
    return NO_TRANSITION;}

void TurnState::act() { }

States_e MatchState::getNext(model_s model) {
    return NO_TRANSITION; }

void MatchState::act() { }

States_e OldAirfeildState::getNext(model_s model) {
    return NO_TRANSITION; }

void OldAirfeildState::act() { }

States_e NewAirfeildState::getNext(model_s model) {
    return NO_TRANSITION; }

void NewAirfeildState::act() { }