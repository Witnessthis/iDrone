#ifndef __MODEL_H__DEFINED
#define __MODEL_H__DEFINED

#include <ardrone_autonomy/Navdata.h>
#include <iDrone/qrAdjust.h>
#include <iDrone/afAdjust.h>
#include <string>
#include <ros/ros.h>

enum WallMarkings_e {
    W00_e, W01_e, W02_e, W03_e, W04_e,
    W10_e, W11_e, W12_e, W13_e, W14_e,
    W20_e, W21_e, W22_e, W23_e, W24_e,
    W30_e, W31_e, W32_e, W33_e, W34_e,
    NUM_WALL_MARKINGS};

enum AirFields_e {
    AF0_e, AF1_e, AF2_e, AF3_e, AF4_e, AF5_e, AF6_e, AF7_e, AF8_e, AF9_e, AF10_e, NUM_AIRFIELDS
};

enum AirFieldMatch {
    NO_MATCH_e, BAD_MATCH_e, GOOD_MATCH_e, PERFECT_MATCH_e
};

struct WallMarking {
    bool hasBeenVisited;
    std::string id;
};

struct AirField {
    bool hasLanded;
    //std::string wallMarking;
    int wallMarking;
    std::string airfieldQR;
    int x;
    int y;
};

struct model_s{
    bool hasCalibrated;
    WallMarking wallMarkings[NUM_WALL_MARKINGS];

    AirField airfields[NUM_AIRFIELDS];
    int nextAirfield;
    int currentWallMarking;

    ardrone_autonomy::Navdata navdata;
    iDrone::qrAdjust qrAdjust;
    std::string qrSpotted;
    iDrone::afAdjust afAdjust;

    int badMatchCounter;
    int consecutiveMatchesCounter;

};

#endif