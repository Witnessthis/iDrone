#include "State.h"
#include <iostream>

State::State() {
    mayAct = true;
}

State::~State() { }

States_e state1::getNext() {
    return state2_e;
}

void state1::act() {
    std::cout << "state1" << std::endl;
}

States_e state2::getNext() {
    return state3_e;
}

void state2::act() {
    std::cout << "state2" << std::endl;
}

States_e state3::getNext() {
    return state1_e;
}

void state3::act() {
    std::cout << "state3" << std::endl;
}