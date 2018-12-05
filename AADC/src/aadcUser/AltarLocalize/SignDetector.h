//
// Created by robin on 03/09/18.
//

#ifndef AADC_USER_SIGNDETECTION_H
#define AADC_USER_SIGNDETECTION_H

#include <vector>
#include <string>
#include <map>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <adtf_filtersdk.h>
//always include filtersdk, systemsdk or streaming3 sdk BEFORE adtfui!!

#include <locale>
#include "../AltarUtils/MapStructs.h"
#include "stdafx.h"
#include <stdlib.h>
#include <fstream>

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

class SignDetector {

public:
    SignDetector();

    bool GetRoadSign(std::vector<tPolarCoordiante>& laserScan, double timeStamp, Pose& roadSign);

    void ExpectedSign(double distance, double bearing, double timeStamp, int id);// Values are all given relative to the car
    vector<DebugPoint> GetDebugPoints();

    int GetSignInd();

    int signInd;

private:

    std::vector<tPolarCoordiante> signBuffer;
    double signBufferLength;

    double signDistance;
    double signBearing;
    double signX;
    double signY;
    double lastUpdate;


    double signWidth;
    double minBufferLength;
    double maxBufferLength;
    double maxRangeThreshold;
    double minRangeThreshold;
    double bearingThreshold;
    double regionThreshold;

    double timeThreshold;
    std::vector<tPolarCoordiante> cleanScan;
    std::vector<DebugPoint> debugPointList;

    std::vector<tPolarCoordiante> FilterScan(std::vector<tPolarCoordiante> laserScan);

    bool InAngluarThreshhold(tPolarCoordiante currPoint);

    bool InRangeThreshhold(tPolarCoordiante currPoint);

    bool InRegionThreshold(tPolarCoordiante currPoint, tPolarCoordiante nextPoint, double &nextDistance);

    bool ValidSignWidth(double signBufferLength);

    Pose ExtractRoadSign(vector<tPolarCoordiante>& signBuffer);

    bool RecentlyUpdated(double timeStamp);

    DebugPoint PolarToDebug(tPolarCoordiante point, int colour);
};


#endif //AADC_USER_SIGNDETECTION_H
