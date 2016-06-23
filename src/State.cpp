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

        std::cout << "MOVE_e" << std::endl;
        return MOVE_e;
        //return ADJUST_BOTTOM_e;
        //return SEARCH_e;
        //return ADJUST_FRONT_e;
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
    if(model.airfields[model.nextAirfield].airfieldQR == model.qrSpotted || //next airfield spotted try to adjust
            model.afAdjust.match > 1){//good match get closer
        std::cout << "ADJUST_BOTTOM_e" << std::endl;

        return ADJUST_BOTTOM_e;
    }
    else if(pattern == MOVEMENT_COMPLETE_e){//completed search pattern with no result
        std::cout << "MOVE_e" << std::endl;

        return MOVE_e;
    }

    return NO_TRANSITION;
}

void SearchState::act(model_s model) {
    controlPanel.bottomCam();

    std::chrono::milliseconds currentTime = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
    );

    //std::cout << "altitude: " << model.navdata.altd << std::endl;

    //time_t currentTime;
    //time(&currentTime);

    //double d = difftime(currentTime, start);

    switch (pattern){
        case MOVEMENT_FREEZE:
            if((currentTime.count() - start.count()) < FREEZE_TIME_T){
                    controlPanel.hover();
            } else {
                start = std::chrono::duration_cast< std::chrono::milliseconds >(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                pattern = MOVEMENT_UP_e;
            }
            break;
        case MOVEMENT_UP_e:
            if(model.navdata.altd < 1600){
                controlPanel.up(1);
            }else {
                start = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                //pattern = MOVEMENT1_e;
                controlPanel.updateSearchState();
                pattern = MOVEMENT4_e;
            }
            break;
        case MOVEMENT1_e:

            //std::cout << "Difftime: " << (currentTime.count() - start.count()) << std::endl;

            if((currentTime.count() - start.count()) < STRAIGHT_MOVEMENT_T){
                //std::cout << "search state: " << pattern << std::endl;
                controlPanel.goRight(0.1);
            }
            else{
                start = std::chrono::duration_cast< std::chrono::milliseconds >(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                pattern = MOVEMENT2_e;
            }
            break;
        case MOVEMENT2_e:
            //std::cout << "Difftime: " << (currentTime.count() - start.count()) << std::endl;

            if((currentTime.count() - start.count()) < STRAIGHT_MOVEMENT_T){
                //std::cout << "search state: " << pattern << std::endl;
                controlPanel.backward(0.1);
            }
            else{
                start = std::chrono::duration_cast< std::chrono::milliseconds >(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                pattern = MOVEMENT3_e;
            }
            break;
        case MOVEMENT3_e:
            //std::cout << "Difftime: " << (currentTime.count() - start.count()) << std::endl;

            if((currentTime.count() - start.count()) < STRAIGHT_MOVEMENT_T){
                //std::cout << "search state: " << pattern << std::endl;
                controlPanel.goLeft(0.1);
            }
            else{
                start = std::chrono::duration_cast< std::chrono::milliseconds >(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                pattern = MOVEMENT4_e;
            }
            break;
        case MOVEMENT4_e:
            //std::cout << "Difftime: " << (currentTime.count() - start.count()) << std::endl;

            if((currentTime.count() - start.count()) < STRAIGHT_MOVEMENT_T){
                //std::cout << "search state: " << pattern << std::endl;
                //controlPanel.forward(0.1);
            }
            else{
                start = std::chrono::duration_cast< std::chrono::milliseconds >(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                controlPanel.updateSearchState();
                pattern = MOVEMENT_COMPLETE_e;
            }
            break;
        case MOVEMENT5_e:

            break;
        case MOVEMENT_COMPLETE_e:
            if((currentTime.count() - start.count()) < FREEZE_TIME_T){
                //std::cout << "search state: " << pattern << std::endl;
                controlPanel.hover();
            }
            else{
                controlPanel.hover();
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
    if(model.qrSpotted != ""
       && model.qrSpotted != model.wallMarkings[model.currentWallMarking].id
            //ugly hardcoding
           && model.qrSpotted != "W01.00"
           && model.qrSpotted != "W00.04"
           && model.qrSpotted != "W02.00"
            ){//found new QR

        std::cout << "ADJUST_FRONT_e" << std::endl;

        return ADJUST_FRONT_e;
    }

    return NO_TRANSITION;
}

void MoveNewPosState::act(model_s model) {
    if(model.currentWallMarking != W11_e || model.currentWallMarking != W21_e || model.currentWallMarking != W00_e || model.currentWallMarking != W30_e ){
        controlPanel.goLeft(1);
        controlPanel.goLeft(1);
        controlPanel.hover();
    }
    else{
        controlPanel.spinLeft();
    }
}

States_e MoveState::getNext(model_s model) {
    //std::cout << "Searching for QR" << std::endl;

    if(model.qrSpotted == "" || model.navdata.altd < 1400){
        //std::cout << "model.navdata.altd: " << model.navdata.altd << std::endl;
        return NO_TRANSITION;
    }

    if(model.qrAdjust.qr_id != "unknown"
           && model.qrSpotted != "W01.00"
           && model.qrSpotted != "W00.04"
           && model.qrSpotted != "W02.00"){

        for(int i=0; i<NUM_WALL_MARKINGS; i++){
            if(model.wallMarkings[i].id == model.qrSpotted){
                controlPanel.updateCurrentWallmarking(i);
                std::cout << "ADJUST_FRONT_e" << std::endl;

                return ADJUST_FRONT_e;
            }
        }
    }

    return NO_TRANSITION;
}

void MoveState::act(model_s model) {


    controlPanel.frontCam();

    if(model.navdata.altd < 1400){
        controlPanel.up(0.25);
    }
    else if(model.navdata.altd > 1600){
        controlPanel.down(0.25);
    }
    else{
        controlPanel.spinLeft();
        controlPanel.hover();
    }

}

States_e AdjustFrontState::getNext(model_s model) {
    //adjust the drone to center in front of the QR code
    time_t currentTime;
    time(&currentTime);

    if(model.qrAdjust.qr_id == "" && difftime(currentTime, start) > 2){
        std::cout << "MOVE_e" << std::endl;

        return MOVE_e;
    }

    if(model.qrAdjust.qr_id != ""){

        time(&start);
    }

    if(isFrontAdjusted(model.qrAdjust.r_height, model.qrAdjust.l_height, model.qrAdjust.t_length, model.qrAdjust.b_length, model.qrAdjust.c_pos)){
        for(int i=0; i<NUM_WALL_MARKINGS; i++){
            //find marking
            if(model.wallMarkings[i].id == model.qrAdjust.qr_id){
                if(model.wallMarkings[i].hasBeenVisited){//old wall marking
                    if(i == model.airfields[model.nextAirfield].wallMarking){//has next airfield
                        std::cout << "SEARCH_e" << std::endl;

                        return SEARCH_e;
                    }
                    else{//does not have next airfield
                        std::cout << "MOVE_NEW_POS_e" << std::endl;

                        return MOVE_NEW_POS_e;
                    }
                }
                else{//new wall marking
                    std::cout << "SEARCH_e" << std::endl;

                    return SEARCH_e;
                }

            }
        }
    }

    return NO_TRANSITION;
}

void AdjustFrontState::act(model_s model) {
    time_t currentTime;
    time(&currentTime);

    if(isFrontAdjusted(model.qrAdjust.r_height, model.qrAdjust.l_height, model.qrAdjust.t_length, model.qrAdjust.b_length, model.qrAdjust.c_pos)){
        //is adjusted
        controlPanel.hover();
        //std::cout << "is adjusted" << std::endl;
    }
    else if(model.qrAdjust.c_pos < ADJUSTED_LEFT_CENTER_MARGIN){
        //qr is far to the left in the image
//        std::cout << "spin left" << std::endl;
        controlPanel.spinLeft();
        controlPanel.hover();
    }
    else if (model.qrAdjust.c_pos > ADJUSTED_RIGHT_CENTER_MARGIN){
        //qr is far to the right in the image
//        std::cout << "spin right" << std::endl;
        controlPanel.spinRight();
        controlPanel.hover();
    }
    else if(model.qrAdjust.t_length > (model.qrAdjust.b_length + ADJUSTED_ERROR_MARGIN_P)){
        //qr is to the top
//        std::cout << "down" << std::endl;
        controlPanel.down(0.25);
    }
    else if((model.qrAdjust.t_length + ADJUSTED_ERROR_MARGIN_P) < model.qrAdjust.b_length){
        //qr is to the bottom
//        std::cout << "up" << std::endl;
        controlPanel.up(0.25);
    }

    else if((model.qrAdjust.r_height + ADJUSTED_BORDER_MARGIN_P) < adjusted_border_height_p &&
            (model.qrAdjust.l_height + ADJUSTED_BORDER_MARGIN_P) < adjusted_border_height_p){
        //qr is too far away
        //std::cout << "forward" << std::endl;
        //controlPanel.forward(0.5);
        controlPanel.forward(0.5);
        controlPanel.hover();
    }
    else if((model.qrAdjust.r_height - ADJUSTED_BORDER_MARGIN_P) > adjusted_border_height_p &&
            (model.qrAdjust.l_height - ADJUSTED_BORDER_MARGIN_P) > adjusted_border_height_p){
        //qr is too close
        //std::cout << "backward" << std::endl;
        //controlPanel.backward(0.5);
        controlPanel.backward(0.5);
        controlPanel.hover();
    }

    else if(model.qrAdjust.r_height > (model.qrAdjust.l_height + ADJUSTED_ERROR_MARGIN_P)){
        //qr is to the left
//        std::cout << "go left" << std::endl;
        controlPanel.goLeft(1);
        controlPanel.hover();
    }
    else if((model.qrAdjust.r_height + ADJUSTED_ERROR_MARGIN_P) < model.qrAdjust.l_height){
        //qr is to the right
//        std::cout << "go right" << std::endl;
        controlPanel.goRight(1);
        controlPanel.hover();
    }

    else{
        //dont know what to do
        std::cout << "i am confused" << std::endl;
        controlPanel.hover();

        return;
    }

}

void AdjustFrontState::reset(){
    time(&start);
}

bool isFrontAdjusted(int r, int l, int t, int b, float c){
    bool centerLeft = !(c < ADJUSTED_LEFT_CENTER_MARGIN);
    bool centerRight = !(c > ADJUSTED_RIGHT_CENTER_MARGIN);
    bool forward = !((r + ADJUSTED_BORDER_MARGIN_P) < adjusted_border_height_p &&
     (l + ADJUSTED_BORDER_MARGIN_P) < adjusted_border_height_p);
    bool backward = !((r - ADJUSTED_BORDER_MARGIN_P) > adjusted_border_height_p &&
                     (l - ADJUSTED_BORDER_MARGIN_P) > adjusted_border_height_p);
    bool left = !(r > (l + ADJUSTED_ERROR_MARGIN_P));
    bool right = !((r + ADJUSTED_ERROR_MARGIN_P) < l);
    bool down = !(t > (b + ADJUSTED_ERROR_MARGIN_P));
    bool up = !((t + ADJUSTED_ERROR_MARGIN_P) < b);

    return centerLeft & centerRight & forward & backward & left & right & down & up;
}

States_e AdjustBottomState::getNext(model_s model) {

    if(model.navdata.state == 2){//has landed
        return START_e;
    }

    if(doLand){
        return NO_TRANSITION;
    }

    if(model.qrSpotted != ""){
        for(int i = 0; i< NUM_AIRFIELDS; i++){
            if(model.airfields[i].airfieldQR == model.qrSpotted){
                controlPanel.updateFoundAirfield(i);

                if(i != model.nextAirfield){
                    std::cout << "MOVE_e" << std::endl;

                    return MOVE_e;
                }
            }
        }
    }

    if(model.qrSpotted == model.airfields[model.nextAirfield].airfieldQR){//found next airfield and we are ready to land
        //this should be done in a new state, but whatever
        std::cout << "LAND NOW" << std::endl;
        doLand = true;
        start = std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()
        );

        return NO_TRANSITION; //should not take off just yet
    }

    if(model.badMatchCounter > BAD_MATCH_TOLERANCE){
        if(model.consecutiveMatchesCounter < REQUIRED_CONSECUTIVE_MATCHES){
            controlPanel.updateSearchState();
        }


        std::cout << "MOVE_e " << model.consecutiveMatchesCounter <<  std::endl;
        return MOVE_e;
    }

    /*
    if(model.navdata.altd > 1600 &&
            model.afAdjust.match == NO_MATCH_e){
        std::cout << "MOVE_e" << std::endl;
        return MOVE_e;
    }*/

    return NO_TRANSITION;
}

void AdjustBottomState::act(model_s model) {
    controlPanel.bottomCam();

    float delta_x = model.afAdjust.c_x - model.afAdjust.imgc_x;
    float delta_y = model.afAdjust.c_y - model.afAdjust.imgc_y;

    std::chrono::milliseconds currentTime = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
    );

    if(doLand){
        if((currentTime.count() - start.count()) < STABILIZE_TIMER_T){
            std::cout << "countdown: " << currentTime.count() - start.count() << std::endl;
            controlPanel.hover();
            return;
        }
        else{
            controlPanel.land();
            controlPanel.updateNextAirfield();
        }
    }

    if(model.afAdjust.match == NO_MATCH_e || model.afAdjust.match == BAD_MATCH_e){
        controlPanel.hover();

    }
    else if(isBottomAdjusted(delta_x, delta_y)){
       if(model.navdata.altd > 700){
           //std::cout << "is adjusted" << std::endl;
           controlPanel.down(0.25);
       }
       else{
           //std::cout << "no match" << std::endl;
           controlPanel.hover();
       }
    }
    else {
        if(abs(delta_x) > abs(delta_y)){
            if(delta_x < 0) {
                //std::cout << "go left" << std::endl;
                controlPanel.goLeft(0.25);
                controlPanel.hover();
            } else {
                //std::cout << "go right" << std::endl;
                controlPanel.goRight(0.25);
                controlPanel.hover();
            }
        } else {
            if(delta_y < 0) {
                //std::cout << "forward" << std::endl;
                controlPanel.forward(0.25);
                controlPanel.hover();
            } else {
                //std::cout << "backward" << std::endl;
                controlPanel.backward(0.25);
                controlPanel.hover();
            }
        }
    }

}

void AdjustBottomState::reset() {
    start = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
    );

    doLand = false;
}

bool isBottomAdjusted(float dx, float dy){
    //std::cout << "dx, dy" << dx << ", " << dy << std::endl;
    return ((abs(dx) < ADJUSTED_BOTTOM_MARGIN) && (abs(dy) < ADJUSTED_BOTTOM_MARGIN));
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

int cmToPixel(int cm){

    return (-2.1*(10^-13)*pow(cm, 7)) + (2.3*(10^-10)*pow(cm, 6)) - (1.1*(10^-7)*pow(cm, 5)) + (2.9*(10^-5)*pow(cm, 4)) - (0.0045*pow(cm, 3)) + (0.44*pow(cm, 2)) - (26*cm) - (8.3*(10^2));
}