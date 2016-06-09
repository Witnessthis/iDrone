#ifndef __MODEL_H__DEFINED
#define __MODEL_H__DEFINED

#include <ardrone_autonomy/Navdata.h>

struct model_s{
    bool hasCalibrated;

    ardrone_autonomy::Navdata navdata;
};

#endif