#include "State.h"
#include <iostream>

State::State() {
    mayAct = true;
}

State::~State() { }

States_e StartState::getNext(model_s model) {
    if(model.navdata.state == 4){//drone is hovering
        std::cout << "Calibrate" << std::endl;
        return CALIBRATE_e;
    }

    return NO_TRANSITION;
}

void StartState::act(model_s model) {
    controlPanel.takeOff();
    //controlPanel.hover();
}

States_e CalibrateState::getNext(model_s model) {
    //std::cout << "Calibrated: " << model.hasCalibrated << std::endl;

    if(model.hasCalibrated){
        return MOVE_e;
    }

    return NO_TRANSITION;
}

void CalibrateState::act(model_s model) {
    controlPanel.land();
    //model.hasCalibrated = true;
}

States_e SearchState::getNext(model_s model) {

    return NO_TRANSITION;
}

void SearchState::act(model_s model) { }

States_e NoMatchState::getNext(model_s model) {

    return NO_TRANSITION;
}

void NoMatchState::act(model_s model) { }

States_e MoveState::getNext(model_s model) {
    std::cout << "id: " << model.qrSpotted << std::endl;

    if(model.qrSpotted != "unknown"){
        for(int i=0; i<NUM_WALL_MARKINGS; i++){
            if(model.wallMarkings[i].id == model.qrSpotted && !model.wallMarkings[i].hasBeenVisited){
                //TODO test this later plz
                return ADJUST_FRONT_e;
            }
        }
    }
    else{
        std::cout << "Adjust state transition plz"  << std::endl;
        return ADJUST_FRONT_e;
    }

    std::cout << "no transitions" << std::endl;
    return NO_TRANSITION;
}

void MoveState::act(model_s model) {
    //front camera
    //hover
    //spin slowly
    //wait for opencv input (msg: r_height, i_height, c_position)
    //adjust according to the QR code in view (center)

}

States_e AdjustFrontState::getNext(model_s model) {
    //adjust the drone to center in front of the QR code
    //r_height l_height c_pos qr_id



    return NO_TRANSITION; }

void AdjustFrontState::act(model_s model) { }

bool isFrontAdjusted(int r, int l, int c){
    //TODO do dis
    return false;
}

States_e TurnState::getNext(model_s model) {
    return NO_TRANSITION;}

void TurnState::act(model_s model) { }

States_e MatchState::getNext(model_s model) {
    return NO_TRANSITION; }

void MatchState::act(model_s model) { }

States_e OldAirfeildState::getNext(model_s model) {
    return NO_TRANSITION; }

void OldAirfeildState::act(model_s model) { }

States_e NewAirfeildState::getNext(model_s model) {
    return NO_TRANSITION; }

void NewAirfeildState::act(model_s model) { }
