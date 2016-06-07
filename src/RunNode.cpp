//
// Created by lime on 6/6/16.
//

#include "RunNode.h"



RunNode::RunNode(ardrone_autonomy::Navdata* navdataptr) {
    navdataHandle = navdataptr;
    //set start state
}

RunNode::~RunNode() {
  //reset state
}

void* RunNode::iDroneFSM(void* arg) {

    while (1) {
    // read navdata through navdataHandle, values might change. investigate how to lock.

    //implement switch case here

    }
    return NULL;
}