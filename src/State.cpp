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
    if(model.afAdjust.match > 1){//model.airfields[model.nextAirfield].airfieldQR == model.qrSpotted){
        return ADJUST_BOTTOM_e;
    }
        /*
    else if(pattern == MOVEMENT_COMPLETE_e){
        controlPanel.updateSearchState();
        return MOVE_e;
    }*/
/*
    if(model.navdata.altd >= 2000){
        controlPanel.hover();
        return ADJUST_BOTTOM_e;
    }
*/
    return NO_TRANSITION;
}

void SearchState::act(model_s model) {
    controlPanel.bottomCam();

    std::chrono::milliseconds currentTime = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
    );

    std::cout << "altitude: " << model.navdata.altd << std::endl;

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
            if(model.navdata.altd < 1700){
                controlPanel.up();
            }else {
                start = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()
                );
                //pattern = MOVEMENT1_e;
                pattern = MOVEMENT_COMPLETE_e;
            }
            break;
        case MOVEMENT1_e:

            std::cout << "Difftime: " << (currentTime.count() - start.count()) << std::endl;

            if((currentTime.count() - start.count()) < STRAIGHT_MOVEMENT_T){
                std::cout << "search state: " << pattern << std::endl;
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
            std::cout << "Difftime: " << (currentTime.count() - start.count()) << std::endl;

            if((currentTime.count() - start.count()) < STRAIGHT_MOVEMENT_T){
                std::cout << "search state: " << pattern << std::endl;
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
            std::cout << "Difftime: " << (currentTime.count() - start.count()) << std::endl;

            if((currentTime.count() - start.count()) < STRAIGHT_MOVEMENT_T){
                std::cout << "search state: " << pattern << std::endl;
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
            std::cout << "Difftime: " << (currentTime.count() - start.count()) << std::endl;

            if((currentTime.count() - start.count()) < STRAIGHT_MOVEMENT_T){
                std::cout << "search state: " << pattern << std::endl;
                controlPanel.forward(0.1);
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

    return NO_TRANSITION;
}

void MoveNewPosState::act(model_s model) { }

States_e MoveState::getNext(model_s model) {
    std::cout << "id: " << model.qrSpotted << std::endl;

    if(model.qrSpotted == "" || model.navdata.altd < 1400){
        std::cout << "model.navdata.altd: " << model.navdata.altd << std::endl;
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

    if(model.navdata.altd < 1400){
        controlPanel.up();
    }
    else if(model.navdata.altd > 1600){
        controlPanel.down();
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
    time_t currentTime;
    time(&currentTime);

    //std::cout << "time elapsed: " << difftime(currentTime, start) << std::endl;

    if(isFrontAdjusted(model.qrAdjust.r_height, model.qrAdjust.l_height, model.qrAdjust.t_length, model.qrAdjust.b_length, model.qrAdjust.c_pos)){
        //is adjusted
        controlPanel.hover();
        std::cout << "is adjusted" << std::endl;
        //controlPanel.land();
    }
    else if(model.qrAdjust.c_pos < ADJUSTED_LEFT_CENTER_MARGIN){
        //qr is far to the left in the image
        std::cout << "spin left" << std::endl;
        controlPanel.spinLeft();
        controlPanel.hover();
    }
    else if (model.qrAdjust.c_pos > ADJUSTED_RIGHT_CENTER_MARGIN){
        //qr is far to the right in the image
        std::cout << "spin right" << std::endl;
        controlPanel.spinRight();
        controlPanel.hover();
    }
    else if(model.qrAdjust.t_length > (model.qrAdjust.b_length + ADJUSTED_ERROR_MARGIN_P)){
        //qr is to the top
        std::cout << "down" << std::endl;
        controlPanel.down();
    }
    else if((model.qrAdjust.t_length + ADJUSTED_ERROR_MARGIN_P) < model.qrAdjust.b_length){
        //qr is to the bottom
        std::cout << "up" << std::endl;
        controlPanel.up();
    }
    else if((model.qrAdjust.r_height + ADJUSTED_BORDER_MARGIN_P) < adjusted_border_height_p &&
            (model.qrAdjust.l_height + ADJUSTED_BORDER_MARGIN_P) < adjusted_border_height_p){
        //qr is too far away
        //std::cout <<"adjusted_border_height_p: " << adjusted_border_height_p << std::endl;
        std::cout << "forward" << std::endl;
        controlPanel.forward(1);
        controlPanel.forward(1);
        controlPanel.hover();
    }
    else if((model.qrAdjust.r_height - ADJUSTED_BORDER_MARGIN_P) > adjusted_border_height_p &&
            (model.qrAdjust.l_height - ADJUSTED_BORDER_MARGIN_P) > adjusted_border_height_p){
        //qr is too close
        //std::cout <<"adjusted_border_height_p: " << adjusted_border_height_p << std::endl;
        std::cout << "backward" << std::endl;
        controlPanel.backward(1);
        controlPanel.backward(1);
        controlPanel.hover();
    }
    else if(model.qrAdjust.r_height > (model.qrAdjust.l_height + ADJUSTED_ERROR_MARGIN_P)){
        //qr is to the left
        std::cout << "go left" << std::endl;
        controlPanel.goLeft(1);
        controlPanel.hover();
    }
    else if((model.qrAdjust.r_height + ADJUSTED_ERROR_MARGIN_P) < model.qrAdjust.l_height){
        //qr is to the right
        std::cout << "go right" << std::endl;
        controlPanel.goRight(1);
        controlPanel.hover();
    }
    else{
        //dont know what to do
        std::cout << "i am confuse" << std::endl;
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


    return NO_TRANSITION;}

void AdjustBottomState::act(model_s model) {
    controlPanel.bottomCam();

    float delta_x = model.afAdjust.c_x - model.afAdjust.imgc_x;
    float delta_y = model.afAdjust.c_y - model.afAdjust.imgc_y;

    if(model.afAdjust.match == NO_MATCH_e || model.afAdjust.match == BAD_MATCH_e || isBottomAdjusted(delta_x, delta_y)){
        if(isBottomAdjusted(delta_x, delta_y)){
            std::cout << "is adjusted" << std::endl;

            controlPanel.land();
        }
        else{
            std::cout << "no match" << std::endl;
        }

        controlPanel.hover();

    }


    else {
        if(abs(delta_x) > abs(delta_y)){
            if(delta_x < 0) {
                std::cout << "go left" << std::endl;
                controlPanel.goLeft(0.1);
                //controlPanel.hover();
            } else {
                std::cout << "go right" << std::endl;
                controlPanel.goRight(0.1);
                //controlPanel.hover();
            }
        } else {
            if(delta_y < 0) {
                std::cout << "forward" << std::endl;
                controlPanel.forward(0.1);
                //controlPanel.hover();
            } else {
                std::cout << "backward" << std::endl;
                controlPanel.backward(0.1);
                //controlPanel.hover();
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