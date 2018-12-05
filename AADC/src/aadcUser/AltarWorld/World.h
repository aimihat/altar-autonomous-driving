/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once
#include "stdafx.h"

//INCLUDE ALTAR HEADER FILES
#include <cmath>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
//*************************************************************************************************
#define CID_MAPPING_FILTER "AltarWorld.filter.user.aadc.cid"

//Occupancy Map parameter
#define PI 3.14159265
#define DEGTORAD PI/180

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;


//DATA TYPES

struct GridObjects {
    // x - y cordinates are relative to the COM of car
    tInt16 top_pivot_x;
    tInt16 top_pivot_y;
    tInt16 bot_pivot_x;
    tInt16 bot_pivot_y;
};


struct sensorPoseGlobal {
    // x - y cordinates are relative to the COM of car
    tFloat32 x;
    tFloat32 y;
    tFloat32 theta; //theta still local, mounting angle
};

struct robotPose {

    tFloat32 x;
    tFloat32 y;
    tFloat32 theta;
};

struct sensorPoint {

    tFloat32 x;
    tFloat32 y;
};

struct MapPoint {

    tFloat32 x;
    tFloat32 y;
};

struct sensorPoseLocal {
    // x - y cordinates are relative to the COM of car

    tFloat32 x;
    tFloat32 y;
    tFloat32 theta;
};

#define OCCMAP_LENGTH 200
#define OCCMAP_WIDTH 200

class cWorld : public cTriggerFunction
{

private:


    //----------------------------
    //STUFF FOR POSITION INFORMATION
    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionIndex;

    /*! The position sample factory */
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;

    cPinReader PoseDataIn;




    // Vars to write image

    //Pins
    /*! Reader of an InPin. */
    cPinReader m_oReader;
    /*! Writer to an OutPin. */
    cPinWriter m_oWriter;

    //Stream Formats
    /*! The input format */
    adtf::streaming::tStreamImageFormat m_sImageFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;


    // -------------------------------------------

     struct ddlLaserScannerDataId
    {
        tSize size;
        tSize scanArray;
    } m_ddlLSDataId;

    adtf::mediadescription::cSampleCodecFactory m_LSStructSampleFactory;

    cPinReader laserDataIn;

private:

    //loops vars
    int count;
    sensorPoseGlobal globalSensor;
    // Localization Vars
    robotPose car;
    sensorPoseLocal lidarLocal;
    // TODO Ultrasonic Sensors INSERT

    // World Model Vars
    tFloat32 map_res; //mm
    tFloat32 occMap[OCCMAP_LENGTH][OCCMAP_WIDTH];
    tFloat32 occMapSize = OCCMAP_WIDTH * OCCMAP_LENGTH;
    tFloat32 wall_thickness;
    Mat gridMap;
    bool car_initalized;


    //Sensor Window Params
    tInt16 tp_y;
    tInt16 bp_y;
    tInt16 tp_x;
    tInt16 bp_x;

    tInt16 window_length;
    tInt16 window_rows;
    tInt16 window_width;
    tInt16 window_colums;

    //Viz Params
    tInt16 csv_counter;

    // Inverse Sensor model Params
    tFloat32 angular_offset_threshold;
    tFloat32 log_prior;


public:

    cWorld();
    virtual ~cWorld() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;


    // Custom Functions

    void Map(std::vector<tPolarCoordiante> laser_scan);
    sensorPoseGlobal SensorToGlobal(robotPose robot, sensorPoseLocal sensor);
    sensorPoint PointToGlobal(sensorPoseGlobal sensor, sensorPoint data);
    tFloat32 SensorRayAngle(sensorPoseGlobal sensor, sensorPoint data);
    tFloat32 GridAngle(MapPoint grid_center, sensorPoseGlobal data);
    tFloat32 InverseSensorModel(tFloat32 data_range, tFloat32 data_angle, tFloat32 grid_distance, tFloat32 grid_angle);
    tInt16 GrayValue(double occValue);
    void UpdateGrayGrid();
    void writeCSV(string filename, cv::Mat m);
    void CordsToMat(tInt16& i,tInt16& j);
    void FreeSpace(GridObjects grid_object);
     tResult UpdateCarPos();
    void StreamWorld();
    void UpdateCarUI();
    tResult CarInit();



    };


//*************************************************************************************************
