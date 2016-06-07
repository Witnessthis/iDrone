//
// Created by lime on 6/6/16.
//

#include <ardrone_autonomy/Navdata.h>

#ifndef IDRONE_RUNNODE_H
#define IDRONE_RUNNODE_H


#endif //IDRONE_RUNNODE_H

class RunNode {


private:

public:
    ardrone_autonomy::Navdata* navdataHandle;
    RunNode(ardrone_autonomy::Navdata* navdataptr);
    ~RunNode();

    enum state {
        //States here
    };

    void* iDroneFSM(void* arg);

protected:



};