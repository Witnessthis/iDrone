#include "State.h"
#include <iostream>
#include <string>
#include <cmath>
#include <chrono>

State::State() {
    mayAct = true;
}

State::~State() { }

States_e StartState::getNext(model_s model) {
    if(model.navdata.state == 4){//drone is hovering
        //return MOVE_e;
        return SEARCH_e;
    }

    return NO_TRANSITION;
}

void StartState::act(model_s model) {
    //controlPanel.reset();

    if(model.navdata.state == 2){
        controlPanel.flatTrim();
    }

    controlPanel.frontCam();

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
    if(model.airfields[model.nextAirfield].airfieldQR == model.qrSpotted){
        return ADJUST_BOTTOM_e;
    }
    else if(pattern == MOVEMENT_COMPLETE_e){
        return MOVE_e;
    }

    return NO_TRANSITION;
}

void SearchState::act(model_s model) {
    std::chrono::milliseconds currentTime = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
    );

    //std::chrono::milliseconds waitTime(1000);
/*
    time_t currentTime;
    time(&currentTime);

    double d = difftime(currentTime, start);*/

    switch (pattern){
        case MOVEMENT_FREEZE:
            if((currentTime.count() - start.count()) < FREEZE_TIME_T) {
                controlPanel.hover();
            } else {
                start = std::chrono::duration_cast< std::chrono::milliseconds >(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                pattern = MOVEMENT1_e;
            }

        case MOVEMENT1_e:

            std::cout << "Difftime: " << (currentTime.count() - start.count()) << std::endl;

            if((currentTime.count() - start.count()) < STRAIGHT_MOVEMENT_T){
                std::cout << "search state: " << pattern << std::endl;
                //controlPanel.diagForwardRight();
                //controlPanel.forward();
                //controlPanel.hover();
                controlPanel.goRight();

            }
            else{
                start = std::chrono::duration_cast< std::chrono::milliseconds >(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                pattern = MOVEMENT2_e;
            }
            break;
        case MOVEMENT2_e:
            std::cout << "Difftime: " << (currentTime.count() - start.count()) << std::endl;

            if((currentTime.count() - start.count()) < STRAIGHT_MOVEMENT_T){
                std::cout << "search state: " << pattern << std::endl;
                //controlPanel.diagForwardRight();
                //controlPanel.forward();
                //controlPanel.hover();
                controlPanel.backward();
            }
            else{
                start = std::chrono::duration_cast< std::chrono::milliseconds >(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                pattern = MOVEMENT3_e;
            }
            break;
        case MOVEMENT3_e:
            std::cout << "Difftime: " << (currentTime.count() - start.count()) << std::endl;

            if((currentTime.count() - start.count()) < STRAIGHT_MOVEMENT_T){
                std::cout << "search state: " << pattern << std::endl;
                //controlPanel.diagForwardRight();
                //controlPanel.forward();
                //controlPanel.hover();
                controlPanel.goLeft();
            }
            else{
                start = std::chrono::duration_cast< std::chrono::milliseconds >(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                pattern = MOVEMENT4_e;
            }
            break;
        case MOVEMENT4_e:
            std::cout << "Difftime: " << (currentTime.count() - start.count()) << std::endl;

            if((currentTime.count() - start.count()) < STRAIGHT_MOVEMENT_T){
                std::cout << "search state: " << pattern << std::endl;
                //controlPanel.diagForwardRight();
                //controlPanel.forward();
                //controlPanel.hover();
                controlPanel.forward();
            }
            else{
                start = std::chrono::duration_cast< std::chrono::milliseconds >(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                pattern = MOVEMENT_COMPLETE_e;
            }
            break;
        case MOVEMENT5_e:

            break;
        case MOVEMENT_COMPLETE_e:
            if((currentTime.count() - start.count()) < FREEZE_TIME_T){
                std::cout << "search state: " << pattern << std::endl;
                //controlPanel.diagForwardRight();
                //controlPanel.forward();
                //controlPanel.hover();
                controlPanel.hover();
            }
            else{
                controlPanel.land();
            }

            break;
    }
}

void SearchState::reset() {
    //time(&start);
    start = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
    );
    pattern = MOVEMENT_FREEZE;
}

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
    controlPanel.frontCam();
    controlPanel.spinLeft();
}

States_e AdjustFrontState::getNext(model_s model) {
    //adjust the drone to center in front of the QR code
/*
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
*/
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
    controlPanel.bottomCam();

    int delta_x = model.afAdjust.c_x - model.afAdjust.imgc_x;
    int delta_y = model.afAdjust.c_y - model.afAdjust.imgc_y;

    if(model.afAdjust.match == NO_MATCH_e || model.afAdjust.match == BAD_MATCH_e || isBottomAdjusted(delta_x, delta_y)){
        controlPanel.hover();
    }
    else {
        if(abs(delta_x) > abs(delta_y)){
            if(delta_x > 0) {
                controlPanel.goLeft();
            } else {
                controlPanel.goRight();
            }
        } else {
            if(delta_y > 0) {
                controlPanel.up();
            } else {
                controlPanel.down();
            }
        }
    }

    /*
    if(model.afAdjust.match != ""){
        if(model.qrSpotted.qr_id == model.nextAirfield){
            controlPanel.land();
            model.hasVisited.push_back(model.qrSpotted.qr_id);
            return START_e;
        } else if(model.qrSpotted.qr_id != model.nextAirfield){
            model.airfields->wallMarking = model.qrSpotted.qr_id; // wallmarking saved
            model.airfields->airfieldQR = model.qrSpotted.qr_id; // airfield marking saved XXX NOT CORRECT XXX
            model.airfields->x = model.afAdjust.imgc_x;
            model.airfields->y = model.afAdjust.imgc_y;
            return SEARCH_e;
        } else if (model.nextAirfield == model.airfields->airfieldQR){
            // go to model.airfields->wallMarking (saved wall marking) through saved x and y coordinates?
            //controlPanel.land();
            return START_e;
        }else{
            for(int k = 0; k < model.hasVisited.size(); k++){
                if(model.qrSpotted.qr_id == model.hasVisited[k]){
                    return SEARCH_e;
                }
            }

        }
    }
     */

}

bool isBottomAdjusted(int dx, int dy){
    return ((abs(dx) < ADJUSTED_ERROR_MARGIN_P) && (abs(dy) < ADJUSTED_ERROR_MARGIN_P));
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
