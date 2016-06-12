#include "State.h"
#include <iostream>
#include <string>

State::State() {
    mayAct = true;
}

State::~State() { }

States_e StartState::getNext(model_s model) {
    if(model.navdata.state == 4){//drone is hovering
        std::cout << "Calibrate" << std::endl;
        return MOVE_e;
    }

    return NO_TRANSITION;
}

void StartState::act(model_s model) {
    //controlPanel.reset();
    controlPanel.takeOff();
    controlPanel.hover();
}

States_e CalibrateState::getNext(model_s model) {
    //std::cout << "Calibrated: " << model.hasCalibrated << std::endl;
    std::cout << "Calibrated: " << model.hasCalibrated << std::endl;
    if(model.hasCalibrated){
        return MOVE_e;
    }

    return NO_TRANSITION;
}

void CalibrateState::act(model_s model) {
    //controlPanel.land();
    std::cout << "Calibrating"<< std::endl;
    controlPanel.hover();
    model.hasCalibrated = true;
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

    if(model.qrSpotted == ""){
        return NO_TRANSITION;
    }

    if(model.qrAdjust.qr_id != "unknown"){
        for(int i=0; i<NUM_WALL_MARKINGS; i++){
            //std::cout << "wallmarking id: " << model.wallMarkings[i].id << std::endl;
            if(model.wallMarkings[i].id.compare(model.qrSpotted) && !model.wallMarkings[i].hasBeenVisited){
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
    //controlPanel.hover();
    //controlPanel.spinLeft();
    //wait for opencv input (msg: r_height, i_height, c_position)
    //adjust according to the QR code in view (center)

}

States_e AdjustFrontState::getNext(model_s model) {
    //adjust the drone to center in front of the QR code
    //r_height l_height c_pos qr_id



    return NO_TRANSITION; }

void AdjustFrontState::act(model_s model) {
    std::cout << "land " << std::endl;
    controlPanel.land();

    /*if(isFrontAdjusted(model.qrAdjust.r_height, model.qrAdjust.l_height, model.qrAdjust.t_length, model.qrAdjust.b_length, model.qrAdjust.c_pos)){
        //is adjusted
        controlPanel.hover();
    }
    else if(true){

    }*/

}


bool isFrontAdjusted(int r, int l, int t, int b, int c){
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
