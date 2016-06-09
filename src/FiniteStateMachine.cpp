#include <iostream>
#include "FiniteStateMachine.h"

StartState startState;
CalibrateState calibrateState;
SearchState searchState;
NoMatchState noMatchState;
MoveState moveState;
AdjustFrontState adjustFrontState;
TurnState turnState;
MatchState matchState;
OldAirfeildState oldAirfeildState;
NewAirfeildState newAirfeildState;

FiniteStateMachine::FiniteStateMachine() {
    currentState = START_e;
}

FiniteStateMachine::~FiniteStateMachine() { }

void FiniteStateMachine::update(model_s model) {
    std::cout << "update" << std::endl;
    std::cout << "currentState: " << currentState << std::endl;
    std::cout << "Num States: " << NUM_STATES << std::endl;

    States_e next = NO_TRANSITION;

    switch (currentState) {
        case START_e:
            next = startState.getNext(model);
            break;

        case CALIBRATE_e:
            next = calibrateState.getNext(model);
            break;

        case SEARCH_e:
            next = searchState.getNext(model);
            break;

        case NO_MATCH_e:
            next = noMatchState.getNext(model);
            break;

        case MOVE_e:
            next = moveState.getNext(model);
            break;

        case ADJUST_FRONT_e:
            next = adjustFrontState.getNext(model);
            break;

        case TURN_e:
            next = turnState.getNext(model);
            break;

        case OLD_AIRFEILD_e:
            next = oldAirfeildState.getNext(model);
            break;

        case NEW_AIRFEILD_e:
            next = newAirfeildState.getNext(model);
            break;

        case MATCH_e:
            next = matchState.getNext(model);
            break;


        default:
            std::cout << "state get next not implemented" << std::endl;
    }

    //std::cout << "next transition: " << next << std::endl;

    if (next != NO_TRANSITION) {
        //TODO interrupt current action

        switch (next) {
            case START_e:
                startState.mayAct = true;
                break;
            case CALIBRATE_e:
                calibrateState.mayAct = true;
                break;
            case SEARCH_e:
                searchState.mayAct = true;
                break;
            case NO_MATCH_e:
                noMatchState.mayAct = true;
                break;
            case MOVE_e:
                moveState.mayAct = true;
                break;
            case ADJUST_FRONT_e:
                adjustFrontState.mayAct = true;
                break;
            case TURN_e:
                turnState.mayAct = true;
                break;
            case OLD_AIRFEILD_e:
                oldAirfeildState.mayAct = true;
                break;
            case NEW_AIRFEILD_e:
                newAirfeildState.mayAct = true;
                break;
            case MATCH_e:
                matchState.mayAct = true;
                break;

            default:
                std::cout << "state get next not implemented" << std::endl;
        }

        currentState = next;
    }
}

void FiniteStateMachine::act(model_s model) {

    //std::cout << "currentState: " << currentState << std::endl;
    //std::cout << "Num States: " << NUM_STATES << std::endl;

    switch (currentState) {
        case START_e:
            if (startState.mayAct) {
                startState.act(model);
            }
            break;
        case CALIBRATE_e:
            if (calibrateState.mayAct) {
                calibrateState.act(model);
            }
            break;
        case SEARCH_e:
            if (searchState.mayAct) {
                searchState.act(model);
            }
            break;
        case NO_MATCH_e:
            if (noMatchState.mayAct) {
                noMatchState.act(model);
            }
            break;
        case MOVE_e:
            if (moveState.mayAct) {
                moveState.act(model);
            }
            break;
        case ADJUST_FRONT_e:
            if (adjustFrontState.mayAct) {
                adjustFrontState.act(model);
            }
            break;
        case TURN_e:
            if (turnState.mayAct) {
                turnState.act(model);
            }
            break;
        case MATCH_e:
            if (matchState.mayAct) {
                matchState.act(model);
            }
            break;
        case OLD_AIRFEILD_e:
            if (oldAirfeildState.mayAct) {
                oldAirfeildState.act(model);
            }
            break;
        case NEW_AIRFEILD_e:
            if (newAirfeildState.mayAct) {
                newAirfeildState.act(model);
            }
            break;


        default:
            std::cout << "state act not implemented" << std::endl;
            break;
    }

}


