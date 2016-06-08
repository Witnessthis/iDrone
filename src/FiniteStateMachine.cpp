#include <iostream>
#include "FiniteStateMachine.h"

state1 s1;
state2 s2;
state3 s3;

FiniteStateMachine::FiniteStateMachine() {
    currentState = state1_e;
}

FiniteStateMachine::~FiniteStateMachine() { }

void FiniteStateMachine::update() {
    //std::cout << "update" << std::endl;

    States_e next = NO_TRANSITION;

    switch (currentState){
        case state1_e:
            next = s1.getNext();
            break;

        case state2_e:
            next = s2.getNext();
            break;

        case state3_e:
            next = s3.getNext();
            break;

        default:
            std::cout << "state get next not implemented" << std::endl;
    }

    //std::cout << "next transition: " << next << std::endl;

    if(next != NO_TRANSITION){
        //TODO interrupt current action

        switch (next){
            case state1_e:
                s1.mayAct = true;
                break;

            case state2_e:
                s2.mayAct = true;
                break;

            case state3_e:
                s3.mayAct = true;
                break;

            default:
                std::cout << "state get next not implemented" << std::endl;
        }

        currentState = next;
    }
}

void FiniteStateMachine::act() {

    //std::cout << "currentState: " << currentState << std::endl;
    //std::cout << "Num States: " << NUM_STATES << std::endl;

    switch (currentState){
        case state1_e:
            if(s1.mayAct){
                s1.act();
            }
            break;

        case state2_e:
            if(s2.mayAct){
                s2.act();
            }
            break;

        case state3_e:
            if(s3.mayAct){
                s3.act();
            }
            break;

        default:
            std::cout << "state act not implemented" << std::endl;
            break;
    }

}


