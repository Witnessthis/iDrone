#ifndef __MODEL_H__DEFINED
#define __MODEL_H__DEFINED

#include <ardrone_autonomy/Navdata.h>
#include <iDrone/qrAdjust.h>
#include <string>

enum WallMarkings_e {
    W00_e, W01_e, W02_e, W03_e, W04_e,
    W10_e, W11_e, W12_e, W13_e, W14_e,
    W20_e, W21_e, W22_e, W23_e, W24_e,
    W30_e, W31_e, W32_e, W33_e, W34_e,
    NUM_WALL_MARKINGS};

struct WallMarking{
    bool hasBeenVisited;
    std::string id;
};

struct model_s{
    bool hasCalibrated;
    WallMarking wallMarkings[NUM_WALL_MARKINGS];

    ardrone_autonomy::Navdata navdata;
    iDrone::qrAdjust qrAdjust;
    std::string qrSpotted;
};

#endif