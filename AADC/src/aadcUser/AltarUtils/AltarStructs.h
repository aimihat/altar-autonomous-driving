//
// Created by robin on 05/07/18.
//

#ifndef AADC_USER_ALTARSTRUCTS_H
#define AADC_USER_ALTARSTRUCTS_H

//DATA TYPES
struct sensorPoseGlobal {
    // x - y cordinates are relative to the COM of car
    float x;
    float y;
    float mounting_angle; //theta still local, mounting angle
};

struct robotPose {

    float x;
    float y;
    float theta;
};

struct sensorPoint {

    float x;
    float y;
};

struct mapPoint {

    float x;
    float y;
};

struct sensorPoseLocal {
    // x - y cordinates are relative to the COM of car

    float x;
    float y;
    float theta;
};

enum parkingStatus {
    START_PARKING = 0,
    BACKING,
    PARKED_IN,
    PULL_LEFT,
    PULL_RIGHT,
    NOT_PARKING
};


#endif //AADC_USER_ALTARSTRUCTS_H
