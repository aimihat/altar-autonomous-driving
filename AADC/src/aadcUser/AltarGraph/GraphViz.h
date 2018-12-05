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
#include <queue>
#include <deque>
#include <map>
//#include "openDriveReader.h"


#include <cmath>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include "../AltarUtils/audiParser.h"
#include "../AltarUtils/MapStructs.h"



//*************************************************************************************************
#define CID_MAPPING_FILTER "AltarGraphViz.filter.user.aadc.cid"

//Occupancy Map parameter
#define PI 3.14159265
#define DEG2RAD PI/180
#define RAD2DEG 180/PI

#define METERS_TO_PIXELS float(200.0)
#define METERS_TO_CELLS float(10.0)

#define WORLD_WIDTH (int)round(30 * METERS_TO_PIXELS)
#define WORLD_HEIGHT (int)round(30 * METERS_TO_PIXELS)

#define OCCMAP_WIDTH (int)round(30 * METERS_TO_CELLS)
#define OCCMAP_HEIGHT (int)round(30 * METERS_TO_CELLS)


#define WORLD_CENTER (int)round(WORLD_WIDTH*0.5)
#define OCCMAP_CENTER (int)round(OCCMAP_WIDTH*0.5)

//TODO Make it so World width is more in bottom left and reshape world Width and Height

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

class cGraphViz : public cTriggerFunction
{

private:

    adtf::base::property_variable<adtf_util::cFilename> m_mapFile;


    // ADTF STREAM

    struct ddlDebugPointId
    {
        tSize numPoints;
        tSize debugPoints;

    } m_ddlDebugPointId;

    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_DebugPointStructSampleFactory;

    cPinReader DebugPointIn;


    //POSITION DATA
    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionIndex;

    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;
    cPinReader PoseDataIn;

    //ULTRASONIC DATA
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;
    cPinWriter m_oSituationWriter;

    struct tBoolSignalValueId
    {
        tSize timeStamp;
        tSize bValue;
    } m_ddlBoolValueId;

    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;

    struct
    {
        tSignalValueId SideLeft;
        tSignalValueId SideRight;
        tSignalValueId RearLeft;
        tSignalValueId RearCenter;
        tSignalValueId RearRight;

    } m_ddlUltrasonicStructIndex;
    adtf::mediadescription::cSampleCodecFactory m_USDataSampleFactory;
    cPinReader USDataIn;

    //LAZER SCANNER DATA
    struct ddlLaserScannerDataId
    {
        tSize size;
        tSize scanArray;
    } m_ddlLSDataId;

    adtf::mediadescription::cSampleCodecFactory m_LSStructSampleFactory;
    cPinReader laserDataIn;


    struct LanePointDataId
    {
        tSize nPoints;
        tSize pointArray;
    } LanePointData;

    adtf::mediadescription::cSampleCodecFactory  LanePointSampleFactory;
    cPinWriter  LanePointOut;

    cPinReader m_oReader;
    cPinWriter m_oWriter;

    // Maneuver Array
    struct tManArrayId
    {
        tSize manArray;
    } m_ddlManArray;
    adtf::mediadescription::cSampleCodecFactory m_manArrayDataSampleFactory;
    cPinReader m_oManArrayReader;


    adtf::streaming::tStreamImageFormat m_sImageFormat;
    object_ptr<adtf::services::IReferenceClock> m_pClock;




    // VARIABLES FOR MAPPING

    MapPoint car;
    MapPoint goal;
    int count;

    int streamCount;
    Pose globalSensor;
    // Localization Vars
    Pose lidarLocal;

    Pose frontPoint; // For the pure pursuit controller
    Pose frontPointGlobal;
    Pose lidarGlobal;
    // Ultrasonic localization
    Pose US_SideLeftLocal;
    Pose US_SideRightLocal;
    Pose US_RearCenterLocal;
    Pose US_RearLeftLocal;
    Pose US_RearRightLocal;

    // World Model Vars
    tFloat32 map_res; //mm
    tFloat32 occMap[OCCMAP_HEIGHT][OCCMAP_WIDTH];
    tFloat32 occMapSize = OCCMAP_WIDTH * OCCMAP_HEIGHT;
    tFloat32 wall_thickness;
    Mat gridMap;
    Mat ready_map;
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

    // Inverse Sensor model Params
    tFloat32 angular_offset_threshold;
    tFloat32 log_prior;



    //MAPPING VARIABLES

    Mat mImageOccMapMasked;
    cv::Size windowSize;
    Mat gridMapMasked;
    // MAPPING FUNCTIONS
//    , tUltrasonicStruct US_data
    void MapLaserData(std::vector<tPolarCoordiante> &laser_scan);

     Pose SensorToGlobal(Pose robot, Pose sensor);
    MapPoint PointToGlobal(Pose sensor, MapPoint data);
    MapPoint  PointToLocal(Pose sensor, MapPoint data);

        tFloat32 SensorRayAngle(Pose sensor, MapPoint data);
    tFloat32 GridAngle(MapPoint grid_center, Pose data);
    tFloat32 InverseSensorModel(tFloat32 data_range, tFloat32 data_angle, tFloat32 grid_distance, tFloat32 grid_angle);
    cv::Vec4b GrayValue(double occValue);
    void UpdateGrayGrid(Rect frame);
    void CordsToMat(tInt16& i,tInt16& j);
    void FreeSpace(GridObjects grid_object);
//    tResult UpdateCarPos();
    void UpdateCarUI();
//    tResult CarInit();

    void preProcess();
    bool checkSurroundings(Mat map, tInt16 x,tInt16 y);
    tFloat32 probMean(Mat map, tInt16 x, tInt16 y);

    void InitalizeOccMap();
    void InitalizeSensorStream();
    tResult ReadSensorStream();

    void CropOccMap(Rect frame, Size window);
    void UpdateOccMap(Rect frame, Size window);
    void PlaceObstacle(Rect frame);


    // VIZUALZATION FUNCTIONS AND VARIABLES

    Mat mImageGrid;
    Mat mImagePath;
    Mat mImageOverlay;
    Mat mImage;
    Mat mImageGridMasked;
    Mat mImagePathMasked;
    Mat mImageOverlayMasked;
    Mat mImageSend;
    Mat mImageFlipped;
    Mat mImageNonFlipped;
    Mat mImageSendNonFlipped;
    Mat covmat;
    Mat mImageDebug;
    Mat mImageDebugTmp;

    std::vector<DebugPoint> debugPointList;

    tResult UpdateDebugPoints();
    void DrawDebugPoint(DebugPoint p);
    tResult DrawRoadSign(float x, float y, float theta, int id, bool detected = false);
    tResult DrawCar(float x, float y, float theta, bool debug = false);
    tResult DrawFrontLine(float x, float y, float theta);
    tResult DrawLaser(float x, float y, int angle);
    tResult DrawHitPoint(float x, float y, int colour);
    tResult DrawDetectedSign(float x, float y, float theta, int id, bool camera = false, bool lidar = false);

    int pointer;
    nodeIdWrapper goalNode;
    nodeIdWrapper startNode;

    std::set<nodeIdWrapper> visted_nodes;
    std::deque<nodeIdWrapper> to_vist;
    std::map<int,int> unique_map;
    std::queue<turn> command;
    double line_debug_length;

    Pose carPose;

    cv::Point2f mPreviousPos;

    tTimeStamp mLastTime;
    tTimeStamp mLastSendTime;
    tTimeStamp mTimeBetweenUpdates;


    std::vector<DebugPoint> mDrawPoints;
    Rect roi;
    Rect roiPixel;
    Rect roiCell;

    void PlotParticles();
    void StreamWorld();
    cv::RotatedRect getErrorEllipse(tFloat32 chisquare_val, cv::Point2f mean, cv::Mat covmat);
    void PlotErrorEllipse();
    void ComposeImage();
    void updateImage();
    tResult UpdateCarPos();
    tResult UpdateSensorPos();

        void ReCenterImage();


    bool carInitalized;
    bool pathInitalized;
    bool graphInitalized;
    // UTILS

    std::queue<turn> GetCommands();

    Quaternion toQuaternion(double pitch, double roll, double yaw);
    float CubicPoly(float a1, float b1, float c1, float d1, float ds);
    float RotateCCWX(float u2, float v2, float hdg2);
    float RotateCCWY(float u1, float v1, float hdg1);


    //Main GraphSearch Functions
    tResult GraphSearch(tTimeStamp tmTimeOfTrigger);

    //Graph Search Utils
    tResult GraphInit();
    tResult GraphViz(std::map<nodeIdWrapper, graphElement> &graph, Mat &mImage);
    std::map<nodeIdWrapper, graphElement> GenerateMapGraph(std::map<int, roadElementWrapper> &roads, int num);
    MapPoint GetGoalPosition();

    // GRAPHEARCH Variables
    map<int, roadElementWrapper> roads;
    map<int, ODReader::roadElement> roadsNew;

    std::map<nodeIdWrapper, graphElement> mapGraph;
    std::vector<int> history;
    bool graphSearched;





    // MAIN PathPlanning Functions
    tResult GenerateLattice(map<int, roadElementWrapper> &roads);
    void LatticePoints(std::map<int, roadElementWrapper> &roadNetwork, std::vector<int> segmentId, float lateral_density, float longitudinal_density);

    tResult PathPlanner();
    tResult PathInit(double carX,  double carY, std::map<nodeIdWrapper, graphNode> &graph);
    tResult UpdatePath();

        tResult PathReset();
    std::vector<MapPoint> GeneratePathPoints(std::vector<nodeIdWrapper> trajectory);
    tResult GeneratePath(double &pathCost, double lookAheadDistance, nodeIdWrapper startNode);
    tResult GeneratePath();
    void UpdatePathPoints(nodeIdWrapper nodeId, bool remove = false);

    //PathPlanning Utils
    double FutureObstacleCost(nodeIdWrapper childId, double lookAheadDistance);
    bool IsAhead(nodeIdWrapper nodeIdA);
    tResult DebugMove();


        bool NewObstacleInPath();
    tResult FindStartNode();
    double EvalCost(nodeIdWrapper parentId, nodeIdWrapper childId, bool lookAhead = false, bool driveStraight = false);
    nodeIdWrapper GetSuccesor(nodeIdWrapper currNodeId,  double &traversalCost, bool lookAhead = false, bool driveStraight = false);
    double GetDistance(nodeIdWrapper nodeIdA, nodeIdWrapper nodeIdB);
    nodeIdWrapper FindNearestNode(double x, double y, std::map<nodeIdWrapper, graphNode> &graph);
    nodeIdWrapper GetLastPathId();
    double GetAngle(nodeIdWrapper nodeIdA, nodeIdWrapper nodeIdB);

    //PathPlanning Variables
    std::map<nodeIdWrapper, graphNode> lattice;
    std::vector<MapPoint> pathPoints;
    std::vector<MapPoint> pathPointsDebug;
    double pathLength;
    double lookAheadDistance;
    std::vector<nodeIdWrapper> path;
    tResult PathViz(std::vector<nodeIdWrapper> &path, std::map<nodeIdWrapper, graphNode> &graph, Mat &mImage);

    tResult SendJunctionFlag(const tUInt32 &situation);
    vector<tUInt8> m_maneuverID;
    tResult ReadManeuverId();


public:
    cv::Scalar colour;
    bool flip;
    int WINDOW_WIDTH;
    int WINDOW_HEIGHT;
    cGraphViz();
    virtual ~cGraphViz() = default;

    tResult Configure() override;

    tResult Process(tTimeStamp tmTimeOfTrigger) override;

    void CreateROI(std::map<nodeIdWrapper, graphElement> &graph);

    void DrawRoad(map<int, roadElementWrapper> &roadNetwork);
    void DrawRoadNew(map<int,ODReader::roadElement> &roadNetwork);
    std::map<nodeIdWrapper, graphElement> GenerateMapGraphNew(std::map<int, ODReader::roadElement> &roads, int num);


    tFloat32 normalizeAngle(tFloat32 alpha, tFloat32 center);
/*! calculates modulus after division */
    tFloat32 mod(tFloat32 x, tFloat32 y);
    tResult SendLanePoints();
    int GetNodeType(roadElementWrapper segement);
    int currentNodeState;
    };

