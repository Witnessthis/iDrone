#include "State.h"
#include <iostream>
#include <string>

State::State() {
    mayAct = true;
}

State::~State() { }

States_e StartState::getNext(model_s model) {
    if(model.navdata.state == 4){//drone is hovering
        return MOVE_e;
    }

    return NO_TRANSITION;
}

void StartState::act(model_s model) {
    //controlPanel.reset();

    if(model.navdata.state == 2){
        controlPanel.flatTrim();
    }

    controlPanel.frontCam();

    //controlPanel.takeOff();
    //controlPanel.hover();
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

States_e MoveNewPosState::getNext(model_s model) {

    return NO_TRANSITION;
}

void MoveNewPosState::act(model_s model) { }

States_e MoveState::getNext(model_s model) {
    std::cout << "id: " << model.qrSpotted << std::endl;

    if(model.qrSpotted == ""){
        return NO_TRANSITION;
    }

    if(model.qrAdjust.qr_id != "unknown"){
        std::cout << "for loop " << model.qrSpotted << std::endl;

        for(int i=0; i<NUM_WALL_MARKINGS; i++){
            std::cout << "wallmarking id: " << model.qrSpotted << std::endl;

            if((model.wallMarkings[i].id == model.qrSpotted) && !model.wallMarkings[i].hasBeenVisited){
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
    controlPanel.spinLeft();
    //wait for opencv input (msg: r_height, i_height, c_position)
    //adjust according to the QR code in view (center)

}

States_e AdjustFrontState::getNext(model_s model) {
    //adjust the drone to center in front of the QR code

    if(isFrontAdjusted(model.qrAdjust.r_height, model.qrAdjust.l_height, model.qrAdjust.t_length, model.qrAdjust.b_length, model.qrAdjust.c_pos)){
        for(int i=0; i<NUM_WALL_MARKINGS; i++){
            //find marking
            if(model.wallMarkings[i].id == model.qrAdjust.qr_id){
                if(model.wallMarkings[i].hasBeenVisited){//old wall marking
                    if(model.wallMarkings[i].id == model.airfields[model.nextAirfield].wallMarking){//has next airfield
                        return SEARCH_e;
                    }
                    else{//does not have next airfield
                        return MOVE_NEW_POS_e;
                    }
                }
                else{//new wall marking
                    return SEARCH_e;
                }

            }
        }
    }

    return NO_TRANSITION;
}

void AdjustFrontState::act(model_s model) {

    if(isFrontAdjusted(model.qrAdjust.r_height, model.qrAdjust.l_height, model.qrAdjust.t_length, model.qrAdjust.b_length, model.qrAdjust.c_pos)){
        //is adjusted
        controlPanel.hover();
    }
    else if(model.qrAdjust.c_pos < 0){
        //qr is far to the left in the image
        controlPanel.spinLeft();
    }
    else if (model.qrAdjust.c_pos > 0){
        //qr is far to the right in the image
        controlPanel.spinRight();
    }
    else if((model.qrAdjust.r_height + ADJUSTED_BORDER_MARGIN_P) < ADJUSTED_BORDER_HEIGHT_P &&
            (model.qrAdjust.l_height + ADJUSTED_BORDER_MARGIN_P) < ADJUSTED_BORDER_HEIGHT_P){
        //qr is too far away
        controlPanel.forward();
    }
    else if((model.qrAdjust.r_height - ADJUSTED_BORDER_MARGIN_P) > ADJUSTED_BORDER_HEIGHT_P &&
            (model.qrAdjust.l_height - ADJUSTED_BORDER_MARGIN_P) > ADJUSTED_BORDER_HEIGHT_P){
        //qr is too close
        controlPanel.backward();
    }
    else if(model.qrAdjust.r_height > (model.qrAdjust.l_height + ADJUSTED_ERROR_MARGIN_P)){
        //qr is to the left
        controlPanel.goLeft();
    }
    else if((model.qrAdjust.r_height + ADJUSTED_ERROR_MARGIN_P) < model.qrAdjust.l_height){
        //qr is to the right
        controlPanel.goRight();
    }
    else if(model.qrAdjust.t_length > (model.qrAdjust.b_length + ADJUSTED_ERROR_MARGIN_P)){
        //qr is to the top
        controlPanel.down();
    }
    else if((model.qrAdjust.t_length + ADJUSTED_ERROR_MARGIN_P) < model.qrAdjust.b_length){
        //qr is to the bottom
        controlPanel.up();
    }
    else{
        //dont know what to do
        controlPanel.hover();
    }

}

bool isFrontAdjusted(int r, int l, int t, int b, int c){
    if (((-ADJUSTED_ERROR_MARGIN_P) < (r - l) < ADJUSTED_ERROR_MARGIN_P) &&
            ((-ADJUSTED_ERROR_MARGIN_P) < (t - b) < ADJUSTED_ERROR_MARGIN_P) &&
            (-1 < c < 1)){

        return true;
    }

    return false;
}

States_e AdjustBottomState::getNext(model_s model) {


    return NO_TRANSITION;}

void AdjustBottomState::act(model_s model) {

    if(true){

    }

    controlPanel.hover();

}

States_e MatchState::getNext(model_s model) {
    return NO_TRANSITION; }

void MatchState::act(model_s model) { }

States_e OldAirfeildState::getNext(model_s model) {
    return NO_TRANSITION; }

void OldAirfeildState::act(model_s model) { }

States_e NewAirfeildState::getNext(model_s model) {
    return NO_TRANSITION; }

void NewAirfeildState::act(model_s model) { }
