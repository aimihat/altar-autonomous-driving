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

#include "stdafx.h"
#include "GraphViz.h"
#include "../AltarUtils/audiParser.h"
#include "../AltarUtils/mapReader/openDriveReader.h"
#include "ADTF3_OpenCV_helper.h"
#include <fstream>
#include <set>
#include <aadc_geometrics.h>
#include "ADTF3_helper.h"


//#include <openDriveReader.h>


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_MAPPING_FILTER,
"AltarGraphViz",
cGraphViz,
adtf::filter::timer_trigger(10000));


cGraphViz::cGraphViz()
{
    //MY STRUCTS
    RegisterPropertyVariable("OpenDrive Map File", m_mapFile);

    object_ptr<IStreamType> pDebugPoint;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tDebug", pDebugPoint, m_DebugPointStructSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_DebugPointStructSampleFactory, "numPoints", m_ddlDebugPointId.numPoints);
        adtf_ddl::access_element::find_array_index(m_DebugPointStructSampleFactory, "debugPoints", m_ddlDebugPointId.debugPoints);
    }

    else
    {
        LOG_WARNING("No mediadescription for tParticle found!");
    }

    Register(DebugPointIn, "DebugPoints", pDebugPoint);

    // Position Data
    object_ptr<IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32radius", m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32speed", m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32heading", m_ddlPositionIndex.heading);
    }
    else
    {
        LOG_WARNING("No mediadescription for tPosition found!");
    }

    Register(PoseDataIn, "Position" , pTypePositionData);


    object_ptr<IStreamType> pLanePointData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("LanePointData", pLanePointData, LanePointSampleFactory))
    {
        adtf_ddl::access_element::find_index(LanePointSampleFactory, "nPoints", LanePointData.nPoints);
        adtf_ddl::access_element::find_array_index(LanePointSampleFactory, "pointArray", LanePointData.pointArray);

    }
    else
    {
        LOG_WARNING("No mediadescription for LanePointData found!");
    }

    Register(LanePointOut, "LanePoints", pLanePointData);

    object_ptr<IStreamType> pSignalData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pSignalData, m_SignalValueSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, "ui32ArduinoTimestamp", m_ddlSignalValueId.timeStamp);
        adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, "f32Value", m_ddlSignalValueId.value);

    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }

    Register(m_oManeuverIdReader, "maneuver_id", pSignalData);
    Register(m_oSituationWriter, "situation", pSignalData);

    object_ptr<IStreamType> pTypeManArray;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tManArray", pTypeManArray, m_manArrayDataSampleFactory))
    {
        adtf_ddl::access_element::find_array_index(m_manArrayDataSampleFactory, cString("manArray"), m_ddlManArray.manArray);
    }
    else
    {
        LOG_WARNING("No mediadescription for tManArray found!");
    }

    Register(m_oManArrayReader, "maneuver_input", pTypeManArray);



//    object_ptr<IStreamType> pBoolSignalData;
//    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pBoolSignalData, m_BoolSignalValueSampleFactory))
//    {
//        adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, "ui32ArduinoTimestamp", m_ddlBoolValueId.timeStamp);
//        adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, "bValue", m_ddlBoolValueId.bValue);
//
//    }
//    else
//    {
//        LOG_WARNING("No mediadescription for tBoolSignalValue found!");
//    }



    InitalizeSensorStream();


    // Image Data
    // GREYSCALE_8
//    RGBA_32
//    RGB_24
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGBA_32);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

//    //create and set inital input format type
//    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
//    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
//    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oReader, "input", pType);

    //Register output pin
    Register(m_oWriter, "Map", pType);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
                                    {
                                        return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter);
                                    });



    LOG_INFO(cString::Format("Bottom Left: (%d, %d)",  WORLD_WIDTH, WORLD_HEIGHT));

    mImageGrid = cv::Mat(WORLD_WIDTH, WORLD_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0) );	// RGBA, 8 bit per channel
    mImagePath = cv::Mat( WORLD_WIDTH, WORLD_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0) );		// RGBA, 8 bit per channel
    mImageOverlay = cv::Mat(WORLD_WIDTH, WORLD_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0) );

    // Draw Lines on Map
    int gridLines = (float)WORLD_WIDTH/(float)METERS_TO_PIXELS;
    int start = -gridLines*0.5;
    for( int x = start; x <= gridLines - start; x ++ )
    {
        cv::Point2f start( round(x*METERS_TO_PIXELS) + WORLD_CENTER, 0 );
        cv::Point2f end( round(x*METERS_TO_PIXELS) + WORLD_CENTER, WORLD_HEIGHT );
        cv::line( mImageGrid, start, end, cv::Scalar(30, 30, 30, 255),2, 8,0  );
    }
    for( int y = start; y <= gridLines - start; y ++ )
    {
        cv::Point2f start( 0, round(y*METERS_TO_PIXELS) + WORLD_CENTER );
        cv::Point2f end( WORLD_WIDTH, round(y*METERS_TO_PIXELS) + WORLD_CENTER );
        cv::line( mImageGrid, start, end, cv::Scalar(30, 30, 30, 255),2, 8,0  );
    }
    // Draw x axis:
    cv::Point2f s( WORLD_CENTER, WORLD_CENTER );
    cv::Point2f e( WORLD_WIDTH*0.5 + WORLD_CENTER, WORLD_CENTER );
    cv::line( mImageGrid, s, e, cv::Scalar(255, 255, 255, 255),2, 8,0  );
    cv::Point2f textPos( WORLD_WIDTH - 20, WORLD_CENTER - 10 );
    putText( mImageGrid, "x", textPos, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0, 255) );
    // Draw y axis:
    e = cv::Point2f( WORLD_CENTER, 0 );
    textPos = cv::Point2f( WORLD_CENTER + 7, 10 );
    cv::line( mImageGrid, s, e, cv::Scalar(255, 255, 255, 255),2, 8,0  );
    putText( mImageGrid, "y", textPos, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0, 255) );

    // flip grid image:
    cv::Mat tmp( WORLD_WIDTH, WORLD_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0) );		// RGBA, 8 bit per channel
    cv::flip( mImageGrid, tmp, 0 );
    mImageGrid = tmp;

    mPreviousPos = cv::Point2f(WORLD_CENTER,WORLD_CENTER);
//    carPose.theta = 0;
//    carPose.x = 0;
//    carPose.y = 0;


    vector<segment> roadSegments;

//    m_odReader = new openDriveReader("/home/robin/Desktop/audi/AADC/src/aadcUser/AltarGraph/aadc2018#kickoff.xodr");
//    //Check if reader is success
//    LOG_INFO("File Read Error: %d", m_odReader->FileReadErr);
//    //Check the number of roads
//    LOG_INFO("Number of roads: %lu", m_odReader->RoadList.size());

    // YOU MUST INTIALIZE THSI BEFORE GETMAP BEFORE GETMAP NEW
//      roads = getMap("/home/aadc/Desktop/AltarMaster/LiveVisualization/adtfsessions/aadc2018#kickoff.xodr");
    roads = getMap("/home/robin/Desktop/audi/AADC/src/aadcUser/AltarGraph/aadc2018#kickoff.xodr");
//    roadsNew = getMapNew("/home/robin/Desktop/audi/AADC/src/aadcUser/AltarGraph/aadc2018#kickoff.xodr");
      DrawRoad(roads);
//    DrawRoadNew(roadsNew);
//    ODReader::roadElement;
    mapGraph = GenerateMapGraph(roads, 10);
//    mapGraph = GenerateMapGraphNew(roadsNew, 10);

    CreateROI(mapGraph);


    //mImageOverlay = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0));	// RGBA, 8 bit per channel
    mImage = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0));


//    mImageSend = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC3, CV_RGB(0,0,0) );	// RGB, 8 bit per channel
//    mImageFlipped = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC3, CV_RGB(0,0,0));
//    mImageSendNonFlipped = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC3, CV_RGB(0,0,0));
//    mImageNonFlipped = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC3, CV_RGB(0,0,0));

    mImageSend = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0));	// RGB, 8 bit per channel
    mImageFlipped = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0));
    mImageSendNonFlipped = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0));
    mImageNonFlipped = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0));

    mImageDebug = cv::Mat(WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0));
    mImageDebugTmp = cv::Mat(WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0));
    covmat = cv::Mat::eye(2,2, CV_32F);

    // CHECK ORDERING OF CREATION
    InitalizeOccMap();


    //LOG_INFO(cString::Format("graph: (%f, %f)",  m_odReader->graph[minId].p.x,  m_odReader->graph[minId].p.x));

    LOG_INFO("BEFORE ROI");
    mImageGridMasked = mImageGrid(roi);
    mImagePathMasked = mImagePath(roi);
    mImageOverlayMasked = mImageOverlay(roi);

    CropOccMap(roiCell, windowSize);
    LOG_INFO("AFTER");

    colour = cv::Scalar( 255, 0, 0, 255/2);
    flip = true;

//    carPose.x = 0;
//    carPose.y = 0;

    goal.x = 9;
    goal.y = 2;

    pointer = 0;

    // INTIALIZE Path Planning Variables

    pathInitalized = false;
    carInitalized = false;
    graphInitalized = false;
    pathLength = 0;
    lookAheadDistance = 5;

//    GraphViz(mapGraph, mImageGrid);

    graphSearched = false;

    streamCount = 0;
    //MAPPING

    currentNodeState = START;

}


// VIZULZATION PLUGINS

void cGraphViz::DrawRoad(map<int,roadElementWrapper> &roadNetwork){
    for (auto const& it : roadNetwork) {
        //Get the points from each road
//        vector<Pose3D> path = m_odReader->GetRoadPoints(it.second);
        // START CODE
        roadElementWrapper el = it.second;
        std::vector<Pose3D> list;
        Pose3D pose;
        int num = 10;
        //Finding heading change for lane shifting, Does not affect anything if there is no lane shift
        float curHeading = 0.0, headingChange = 0.0, lastHeading = 0.0;
        for (int i = 0; i < num; i++)
        {
            float ds = (float)(i + 0.1) / (num);
            roadGeometryWrapper geo = el.geometry[0];
            //Find points from parametric polynomial in local coordinate
            float tempx = CubicPoly(geo.aU, geo.bU, geo.cU, geo.dU, ds);
            float tempy = CubicPoly(geo.aV, geo.bV, geo.cV, geo.dV, ds);
            //Split to lane Point, use heading change to rotate lane shift
            float tempLanex = RotateCCWX(0, el.laneSplit / el.scale, headingChange);
            float tempLaney = RotateCCWY(0, el.laneSplit / el.scale, headingChange);
            //Rotate to global coordinate
            float newx = RotateCCWX(tempx + tempLanex, tempy + tempLaney, geo.hdg);
            float newy = RotateCCWY(tempx + tempLanex, tempy + tempLaney, geo.hdg);
            //Shift to global coordinate
            pose.p.x = (geo.x + newx) * el.scale;
            pose.p.y = (geo.y + newy) * el.scale;
            pose.p.z = el.altitude;
            pose.q = toQuaternion(0, 0, geo.hdg);
            //Store line
            list.push_back(pose);
            //Get the heading change from last MapPoint to current Point
            if (i != 0)
            {
                float x0 = list[i - 1].p.x, y0 = list[i - 1].p.y;
                float x1 = pose.p.x, y1 = pose.p.y;
                lastHeading = curHeading;
                curHeading = atan2(y1 - y0, x1 - x0);
                headingChange += normalizeAngle(curHeading - lastHeading, 0);
                headingChange = normalizeAngle(headingChange, 0);
            }
            else
            {
                curHeading = geo.hdg;
            }
        }
        // END CODE

        //Draw line for each Point
        for (unsigned int j = 0; j < list.size() - 1; j++)
        {//

            double x_start = list[j].p.x ;
            double y_start = list[j].p.y;
            double x_end = list[j + 1].p.x;
            double y_end = list[j + 1].p.y;

            Point2f line_start( (int)round(x_start*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER));
            Point2f line_end( (int)round(x_end*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_end*METERS_TO_PIXELS + WORLD_CENTER));
            line(mImageGrid, line_start, line_end, Scalar(255, 255, 255, 255 ), 2, 8, 0  );

            if (j == list.size()/2){


                ostringstream ss;
                ss << it.first;
//                ss << it.first;
                putText(mImageGrid, ss.str(), cv::Point((int)round(x_start * METERS_TO_PIXELS + WORLD_CENTER),
                                                    (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER)),
                        FONT_HERSHEY_PLAIN, 3, Scalar(0,0,255,255), 2, 8, true);

            }
        }
    }
}


void cGraphViz::DrawRoadNew(map<int,ODReader::roadElement> &roadNetwork){
    for (auto const& it : roadNetwork) {
        //Get the points from each road
//        vector<Pose3D> path = m_odReader->GetRoadPoints(it.second);
        // START CODE
        ODReader::roadElement el = it.second;
        std::vector<Pose3D> list;
        Pose3D pose;
        int num = 10;
        //Finding heading change for lane shifting, Does not affect anything if there is no lane shift
        float curHeading = 0.0, headingChange = 0.0, lastHeading = 0.0;
        for (int i = 0; i < num; i++)
        {
            float ds = (float)(i + 0.1) / (num);
            ODReader::roadGeometry geo = el.geometry[0];
            //Find points from parametric polynomial in local coordinate
            float tempx = CubicPoly(geo.aU, geo.bU, geo.cU, geo.dU, ds);
            float tempy = CubicPoly(geo.aV, geo.bV, geo.cV, geo.dV, ds);
            //Split to lane Point, use heading change to rotate lane shift
            float tempLanex = RotateCCWX(0, el.laneSplit / el.scale, headingChange);
            float tempLaney = RotateCCWY(0, el.laneSplit / el.scale, headingChange);
            //Rotate to global coordinate
            float newx = RotateCCWX(tempx + tempLanex, tempy + tempLaney, geo.hdg);
            float newy = RotateCCWY(tempx + tempLanex, tempy + tempLaney, geo.hdg);
            //Shift to global coordinate
            pose.p.x = (geo.x + newx) * el.scale;
            pose.p.y = (geo.y + newy) * el.scale;
            pose.p.z = el.altitude;
            pose.q = toQuaternion(0, 0, geo.hdg);
            //Store line
            list.push_back(pose);
            //Get the heading change from last MapPoint to current Point
            if (i != 0)
            {
                float x0 = list[i - 1].p.x, y0 = list[i - 1].p.y;
                float x1 = pose.p.x, y1 = pose.p.y;
                lastHeading = curHeading;
                curHeading = atan2(y1 - y0, x1 - x0);
                headingChange += normalizeAngle(curHeading - lastHeading, 0);
                headingChange = normalizeAngle(headingChange, 0);
            }
            else
            {
                curHeading = geo.hdg;
            }
        }
        // END CODE

        //Draw line for each Point
        for (unsigned int j = 0; j < list.size() - 1; j++)
        {//

            double x_start = list[j].p.x ;
            double y_start = list[j].p.y;
            double x_end = list[j + 1].p.x;
            double y_end = list[j + 1].p.y;

            Point2f line_start( (int)round(x_start*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER));
            Point2f line_end( (int)round(x_end*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_end*METERS_TO_PIXELS + WORLD_CENTER));
            line(mImageGrid, line_start, line_end, Scalar(255, 255, 255, 255 ), 2, 8, 0  );

            if (j == list.size()/2){


                ostringstream ss;
                ss << it.first;
//                ss << it.first;
                putText(mImageGrid, ss.str(), cv::Point((int)round(x_start * METERS_TO_PIXELS + WORLD_CENTER),
                                                        (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER)),
                        FONT_HERSHEY_PLAIN, 3, Scalar(0,0,255,255), 2, 8, true);

            }
        }
    }
}

void cGraphViz::CreateROI(std::map<nodeIdWrapper, graphElement> &graph) {//Find the minimum and maximum X and y pointsm_minX = 99999999.f, m_maxX = -99999999.f;

    tFloat32 m_minX, m_minY, m_maxX, m_maxY;

    m_minX = 99999999.f, m_maxX = -99999999.f;
    m_minY = 99999999.f, m_maxY = -99999999.f;
    for (auto const& it : graph) {
        float x = it.second.p.x;
        float y = it.second.p.y;
        if (x < m_minX)
        {
            m_minX = x;
        }
        else if (x > m_maxX)
        {
            m_maxX = x;
        }
        if (y < m_minY)
        {
            m_minY = y;
        }
        else if (y > m_maxY)
        {
            m_maxY = y;
        }
    }

    m_minX -= 1;
    m_minY -= 2;
    m_maxX += 1;
    m_maxY += 2;


    cv::Point botLeftPivot(
            (int)(round(m_minX * METERS_TO_PIXELS + WORLD_CENTER)),
            (int)(round(m_minY * METERS_TO_PIXELS + WORLD_CENTER))
    );

    cv::Point topRightPivot( (int)(round(m_maxX * METERS_TO_PIXELS + WORLD_CENTER)), (int)(round(m_maxY * METERS_TO_PIXELS + WORLD_CENTER)));

    roi = Rect(botLeftPivot,topRightPivot);
    LOG_INFO(cString::Format("Bottom Left: (%d, %d)",  (int)(round(m_minX * METERS_TO_PIXELS + WORLD_CENTER)), (int)(round(m_minY * METERS_TO_PIXELS + WORLD_CENTER))));
    LOG_INFO(cString::Format("Bottom Left: (%d, %d)",  (int)(round(m_maxX * METERS_TO_PIXELS + WORLD_CENTER)), (int)(round(m_maxY * METERS_TO_PIXELS + WORLD_CENTER))));


    cv::Point botLeftPivotPixel(
            (int)(round(m_minX * METERS_TO_PIXELS + WORLD_CENTER)),
            (int)(round(m_minY * METERS_TO_PIXELS + WORLD_CENTER)));

    cv::Point topRightPivotPixel(
            (int)(round(m_maxX * METERS_TO_PIXELS + WORLD_CENTER)),
            (int)(round(m_maxY * METERS_TO_PIXELS + WORLD_CENTER)));

    roiPixel = Rect(botLeftPivotPixel, topRightPivotPixel);


    cv::Point botLeftPivotCell(
            (int)(round(m_minX * METERS_TO_CELLS + OCCMAP_CENTER)),
            (int)(round(m_minY * METERS_TO_CELLS + OCCMAP_CENTER)));

    cv::Point topRightPivotCell(
            (int)(round(m_maxX * METERS_TO_CELLS + OCCMAP_CENTER)),
            (int)(round(m_maxY * METERS_TO_CELLS + OCCMAP_CENTER)));

    LOG_INFO(cString::Format("OCC MAP DIMS: (%d, %d)",  OCCMAP_WIDTH,OCCMAP_HEIGHT));

    LOG_INFO(cString::Format("Bottom Left CELL: (%d, %d)",  (int)(round(m_minX * METERS_TO_CELLS + OCCMAP_CENTER)),(int)(round(m_minY * METERS_TO_CELLS + OCCMAP_CENTER))));
    LOG_INFO(cString::Format("Bottom Left CELL: (%d, %d)",  (int)(round(m_maxX * METERS_TO_CELLS + OCCMAP_CENTER)),  (int)(round(m_maxY * METERS_TO_CELLS + OCCMAP_CENTER))));


    roiCell = Rect(botLeftPivotCell, topRightPivotCell);


    WINDOW_WIDTH = (int)(round(m_maxX * METERS_TO_PIXELS + WORLD_CENTER)) - (int)(round(m_minX * METERS_TO_PIXELS + WORLD_CENTER));
    WINDOW_HEIGHT = (int)(round(m_maxY * METERS_TO_PIXELS + WORLD_CENTER)) - (int)(round(m_minY * METERS_TO_PIXELS + WORLD_CENTER));

    windowSize = cv::Size(WINDOW_WIDTH, WINDOW_HEIGHT);
    LOG_INFO(cString::Format("WINDOW DIMS: (%d, %d)",  WINDOW_WIDTH, WINDOW_HEIGHT));

}


//implement the Configure function to read ALL Properties
tResult cGraphViz::Configure()
{


    // -------------------------------
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    //--------------------------------

    RETURN_NOERROR;
}


// PATH PLANNING FUNCTIONS

tResult cGraphViz::PathPlanner() {


    if (NewObstacleInPath()){
        PathReset();
        GeneratePath();
    }

    else if (pathInitalized) {
        UpdatePath();
        GeneratePath();

        pathPoints = GeneratePathPoints(path);
    }

    else {
        PathInit(carPose.x, carPose.y, lattice);
        pathInitalized = true;
    }

    // SEND NODES
    // TRIM PATH
    //carPose.x;

    //path.push_back(nextNode);
    //pathPoints = GeneratePathPoints();
    RETURN_NOERROR;

}

tResult cGraphViz::PathReset() {

    RETURN_NOERROR;

    }

tResult cGraphViz::UpdatePath() {

    if (path.size() > 0){
        if (IsAhead(path.front())){

            LOG_INFO("AHEAD");

        }

        else {

            LOG_INFO("BEHIND: UPDATING");

            if (path.size() > 4){
                double dist = GetDistance(path[0], path[1]);
                pathLength -= dist;
                path.erase(path.begin());
//                UpdatePathPoints(path.front(),true);
            }
        }
    }

    // Checking type of node, if a Turn node, then send Flag to state machine
    int frontNodeType = lattice[path[0]].type;

    LOG_INFO(cString::Format("node type: (%d)", frontNodeType));

    if (frontNodeType != currentNodeState) {

        currentNodeState = frontNodeType;
        SendJunctionFlag(frontNodeType); // Send Flag with type to state machine;
    }
    RETURN_NOERROR;

}

bool cGraphViz::IsAhead(nodeIdWrapper nodeIdA) {

    bool ahead = false;
    graphNode A =  lattice[nodeIdA];

    double relNodeAngle = atan2(A.p.x - carPose.x , A.p.y - carPose.y);
    relNodeAngle = carPose.theta;
    double frontLineAngle = 90 * DEG2RAD + relNodeAngle;
    double a = tan(frontLineAngle);

    Pose carFront;

    carFront = SensorToGlobal(carPose, frontPoint);

    double b = carFront.y - (carFront.x * a);
    double expY = (a * A.p.x) + b;
//    LOG_INFO(cString::Format("SCORE: (%f)", score));
//    LOG_INFO(cString::Format("Car Heading: (%f)", carPose.theta * 180/PI));

    // DEBUGGING PURPOSES
    if (relNodeAngle <= 180 * DEG2RAD && relNodeAngle >= 0 * DEG2RAD) { // Check if looking to right or left

        if (expY < A.p.y){

            ahead = true;

        }

        else ahead = false;
    }

    else {

        if (expY > A.p.y){

            ahead = true;

        }

        else ahead = false;

    }

    if (ahead == false){
        LOG_INFO("NOT AHEAD");

    }
    lattice[nodeIdA].inFront = ahead;


    DebugPoint p;

    p.x = carFront.x;
    p.y = carFront.y;
    p.theta = frontLineAngle;
    p.type = FRONT_LINE;

    debugPointList.push_back(p);

    LOG_INFO("PUSHING FRONT LINE POINT");
//    LOG_INFO(cString::Format("CAR POSE: (%f, %f)", carPose.x, carPose.y));
    return lattice[nodeIdA].inFront;

}

tResult cGraphViz::DebugMove() {

    graphNode A =  lattice[path[0]];
    graphNode B =  lattice[path[1]];
    double relNodeAngle = atan2(B.p.y - A.p.y , B.p.x - A.p.x);

    carPose.theta = relNodeAngle;
    carPose.x = A.p.x + 0.1;
    carPose.y = A.p.y;
    PathViz(path, lattice, mImageOverlay);

    RETURN_NOERROR;
}

nodeIdWrapper cGraphViz::GetLastPathId() {

    nodeIdWrapper lastNode;

    lastNode = path.back();

    return lastNode;
}

double cGraphViz::GetDistance(nodeIdWrapper nodeIdA, nodeIdWrapper nodeIdB) {

    graphNode A = lattice[nodeIdA];
    graphNode B = lattice[nodeIdB];

    return sqrt( pow(A.p.x - B.p.x, 2) + pow(A.p.y - B.p.y, 2) );
}

double cGraphViz::GetAngle(nodeIdWrapper nodeIdA, nodeIdWrapper nodeIdB) {

    graphNode A =  lattice[nodeIdA];
    graphNode B =  lattice[nodeIdB];

    return atan2(A.p.x - B.p.x , A.p.y - B.p.y);
}

bool cGraphViz::NewObstacleInPath() {

    return false;
}


void cGraphViz::UpdatePathPoints(nodeIdWrapper nodeId, bool remove) {

    if (remove){
        pathPoints.erase(pathPoints.begin());
    }

    else {

        graphNode node = lattice[nodeId];
        MapPoint pathPointLocal;
        pathPointLocal.x = node.p.x;
        pathPointLocal.y = node.p.y;
                pathPointLocal = PointToLocal(frontPointGlobal, pathPointLocal);

//        pathPointLocal = PointToLocal(carPose, pathPointLocal);

        pathPoints.push_back(pathPointLocal);
    }
}

std::vector<MapPoint>  cGraphViz::GeneratePathPoints(std::vector<nodeIdWrapper> trajectory) {

    std::vector<MapPoint>  points;
    std::vector<MapPoint>  debugPoints;

    Pose frontPointGlobal = SensorToGlobal(carPose, frontPoint);

    for (nodeIdWrapper nodeId : trajectory) { // MABYE ERROR HERE with thsi for loop

        graphNode node = lattice[nodeId];
        MapPoint pathPointLocal;
        pathPointLocal.x = node.p.x;
        pathPointLocal.y = node.p.y;
        pathPointLocal = PointToLocal(frontPointGlobal, pathPointLocal);
        points.push_back(pathPointLocal);

//        pathPointLocal = PointToGlobal(frontPointGlobal, pathPointLocal);
        debugPoints.push_back(pathPointLocal);
        }
    pathPointsDebug = debugPoints;
    return points;
}


tResult cGraphViz::PathViz(std::vector<nodeIdWrapper> &path, std::map<nodeIdWrapper, graphNode> &graph, Mat &mImageViz) {

    // CAR VIZUALTION
    //Draw rotated rectangle representing car + orientation lines.
//    float carHeight = CAR_HEIGHT*0.01;
//    float carWidth = CAR_WIDTH*0.01;
//
    float c = cos(carPose.theta);
    float s = sin(carPose.theta);
//
//    cv::RotatedRect rotatedRectangle(
//            //center Point
//            cv::Point((carPose.x + (carHeight/2)*c)* METERS_TO_PIXELS + WORLD_CENTER, ((carPose.y + (carHeight/2)*s)* METERS_TO_PIXELS + WORLD_CENTER)),
//            cv::Size(carHeight* METERS_TO_PIXELS,carWidth* METERS_TO_PIXELS),
//            90-carPose.theta*180.0/PI);
//
//    cv::Point2f vertices2f[4];
//    rotatedRectangle.points(vertices2f);
//
//    cv::Point vertices[4];
//    for(int i = 0; i < 4; ++i){
//        vertices[i] = vertices2f[i];
//    }
//
//    cv::fillConvexPoly(mImageOverlay,
//                       vertices,
//                       4,
//                       cv::Scalar( 30, 200, 0, 50));

    for (size_t i = 0; i < path.size(); i++) {
        nodeIdWrapper currId = path[i];
        graphNode currNode = graph[currId];

        double x = currNode.p.x;
        double y = currNode.p.y;

        cv::Point2f circle_point((int) round(x * METERS_TO_PIXELS + WORLD_CENTER),
                                 (int) round(y * METERS_TO_PIXELS + WORLD_CENTER));

        if (currNode.inFront){
            cv::circle(mImageViz, circle_point, 10, cv::Scalar(0, 255, 0, 250), CV_FILLED, 8, 0);
        }

        else {
            cv::circle(mImageViz, circle_point, 10, cv::Scalar(255, 0, 0, 250), CV_FILLED, 8, 0);
        }
    }

    for (size_t i = 0; i < pathPointsDebug.size(); i++) {
        MapPoint currPoint = pathPointsDebug[i];

        cv::Point2f circle_point((int) round(currPoint.x * METERS_TO_PIXELS + WORLD_CENTER),
                                 (int) round(currPoint.y * METERS_TO_PIXELS + WORLD_CENTER));

        cv::circle(mImageViz, circle_point, 5, cv::Scalar(255, 255, 255, 255), CV_FILLED, 8, 0);
    }

    cv::Point2f circle_point((int) round(frontPointGlobal.x * METERS_TO_PIXELS + WORLD_CENTER),
                               (int) round(frontPointGlobal.y * METERS_TO_PIXELS + WORLD_CENTER));
    cv::circle(mImageViz, circle_point, 20, cv::Scalar(255, 0, 0, 255), CV_FILLED, 8, 0);
//
//
//    for (auto const& it : lattice) {
////        LOG_INFO("DRAWING LATTICE");
//        nodeIdWrapper currId = it.first;
//        float x_start = it.second.p.x;
//        float y_start = it.second.p.y;
//
//        std::ostringstream ss;
//        //ss << it.second.id;
////        ss << currId.uniqueId;
//            ss << int(round(it.second.cost * 100))/100;
//
//        putText(mImageViz, ss.str(), cv::Point((int) round(x_start * METERS_TO_PIXELS + WORLD_CENTER),
//                                                   (int) round(y_start * METERS_TO_PIXELS + WORLD_CENTER)),
//                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255, 255), 2, 8, true);
//    }
//
//
////            cv::Point2f line_start( (int)round(x_start*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER));
////            cv::Point2f line_end( (int)round(x_end*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_end*METERS_TO_PIXELS + WORLD_CENTER));
////            cv::line( mImageGrid, line_start, line_end, cv::Scalar( 0, 200, 0, 0 ),1, 8,0);

    RETURN_NOERROR;

}

tResult cGraphViz::GraphViz(std::map<nodeIdWrapper, graphElement> &graph, Mat &mImage) {
    for (auto const& it : graph) {
        nodeIdWrapper currId = it.first;
        float x = it.second.p.x;
        float y = it.second.p.y;


        cv::Point2f circle_point((int) round(x * METERS_TO_PIXELS + WORLD_CENTER),
                                 (int) round(y * METERS_TO_PIXELS + WORLD_CENTER));

        cv::circle(mImage, circle_point, 5, cv::Scalar(255, 255, 0, 255), CV_FILLED, 8, 0);

        std::ostringstream ss;

        ss << currId.uniqueId;
        //ss << currId.roadId;

        //DOING THIS IS PROCESS NOW

        putText(mImage, ss.str(), cv::Point((int)round(x*METERS_TO_PIXELS + WORLD_CENTER),
                                                (int)round(y*METERS_TO_PIXELS + WORLD_CENTER)),
                cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(255,255,255,255), 2, 8,true);


        for (nodeIdWrapper i: it.second.succId){

            float nextX = graph[i].p.x;
            float nextY = graph[i].p.y;

            cv::Point2f line_start( (int)round(x*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y*METERS_TO_PIXELS + WORLD_CENTER));
            cv::Point2f line_end( (int)round(nextX*METERS_TO_PIXELS + WORLD_CENTER), (int)round(nextY*METERS_TO_PIXELS + WORLD_CENTER));
            cv::line( mImageGrid, line_start, line_end, cv::Scalar( 0, 200, 0, 100),2, 8,0);
        }


    }

    RETURN_NOERROR;

}

tResult cGraphViz::GeneratePath() {

    double lookaheadDistance = 1; //4 meters TODO make it based on speed?

    if (pathLength < lookaheadDistance && path.size() < 25) { //50 is size of point array
        double traversalCost = 0;
        LOG_INFO("FINDING NEXT NODE");
        nodeIdWrapper lastNode = path.back();

        nodeIdWrapper nextNode = GetSuccesor(lastNode, traversalCost, true, false);

        if (!(nextNode.uniqueId == lastNode.uniqueId && nextNode.roadId == lastNode.roadId )) { // Check to see if you reached end, then Succesor will return its self
            path.push_back(nextNode);
//            UpdatePathPoints(nextNode, false);
            double dist = GetDistance(lastNode,nextNode);
            pathLength += dist;
        }
    }


    RETURN_NOERROR;

}

tResult cGraphViz::GeneratePath(double &pathCost, double lookAheadDistance, nodeIdWrapper startNode) {

//    double lookaheadDistance = 4; //4 meters TODO make it based on speed?
    pathCost = 0;
    std::vector<nodeIdWrapper> history;
    history.push_back(startNode);

    while (pathLength < lookAheadDistance) {
        double traversalCost = 0;
        LOG_INFO("FINDING NEXT NODE");
        nodeIdWrapper lastNode = history.back();

        nodeIdWrapper nextNode = GetSuccesor(lastNode, traversalCost, false, true);

        if (!(nextNode.uniqueId == lastNode.uniqueId && nextNode.roadId == lastNode.roadId )) { // Check to see if you reached end, then Succesor will return its self
            history.push_back(nextNode);

            double dist = GetDistance(lastNode,nextNode);
            pathLength += dist;
            pathCost += traversalCost;
        }
        else {
            RETURN_NOERROR;
        }

    }

    RETURN_NOERROR;

}


nodeIdWrapper cGraphViz::GetSuccesor(nodeIdWrapper currNodeId, double &traversalCost, bool lookAhead, bool driveStraight) {

    graphNode currNode = lattice[currNodeId];

    double cost = 0;
    double minCost = 10000;
    nodeIdWrapper minId;
    graphNode minNode;

    double totalCost = 0;

    for (size_t i = 0; i < currNode.succ->size(); i++) {

        nodeIdWrapper childId = currNode.succ->at(i);
        graphNode childNode = lattice[childId];

        cost = EvalCost(currNodeId, childId, lookAhead, driveStraight);

        if (cost < minCost) {

            minCost = cost;
            minId = childId;
            minNode = childNode;
        }

        lattice[childId].cost = cost;
        totalCost += cost;
    }

    if (currNode.succ->size() == 0) { // If no Succesors

//        endReached = true;
        minId = currNodeId;
        minCost = 0;
    }

    if (lookAhead) {

        //FOR DEBUGGING
        for (size_t i = 0; i < currNode.succ->size(); i++) {

            nodeIdWrapper childId = currNode.succ->at(i);
            lattice[childId].intensity = lattice[childId].cost/totalCost ;
        }
    }

//    lattice[minId].edgeCost = minCost;
    // Store Information for Further Processing
    traversalCost = minCost;
    return minId;
}

double cGraphViz::FutureObstacleCost(nodeIdWrapper childId, double lookAheadDistance) {

    double pathCost = 0;
    double forwardSimDistance = 1;
    double pathLength = 0;

    std::vector<nodeIdWrapper> history;
    history.push_back(childId);

    while (pathLength < forwardSimDistance) {

        // Add Counter to break after a set number of steps
        // Do based on steps
        double traversalCost = 0;
        LOG_INFO("FINDING NEXT NODE");
        nodeIdWrapper lastNode = history.back();

        nodeIdWrapper nextNode = GetSuccesor(lastNode,  traversalCost, false, true);

        if (!(nextNode.uniqueId == lastNode.uniqueId && nextNode.roadId == lastNode.roadId )) { // Check to see if you reached end, then Succesor will return its self
            history.push_back(nextNode);

            double dist = GetDistance(lastNode,nextNode);
            pathLength += dist;
            pathCost += traversalCost;
        }
        else {
            return pathCost;
        }

    }

    return pathCost;
}

double cGraphViz::EvalCost(nodeIdWrapper parentId, nodeIdWrapper childId, bool lookAhead, bool driveStraight) {

    graphNode parentNode = lattice[parentId];
    graphNode node = lattice[childId];

    // Calculate Obstacle Cost

    double obstacleCost = 0;
    double startX = parentNode.p.x;
    double startY = parentNode.p.y;
    double endX = node.p.x;
    double endY = node.p.y;

    MapPoint innovationVector;
    innovationVector.x = (float)(endX - startX);
    innovationVector.y = (float)(endY - startY);

    double innovationLength = sqrt( pow(innovationVector.y,2) + pow(innovationVector.x,2));
    double queryDensity = METERS_TO_CELLS; //Points per meter
    int queryPoints = (int)round(queryDensity * innovationLength);
    double stepX = innovationVector.x / (queryDensity * innovationLength);
    double stepY = innovationVector.y / (queryDensity * innovationLength);

    double totalP = 0;
    double maxP = 0;
    //SLIGHTLY BIASED TOWARDS SHORTED SEGMENTS
    for (int i = 0; i < queryPoints; i++){

        double queryX = startX + (i * stepX);
        double queryY = startY + (i * stepY);

        cv::Point2f circle_point((int) round(queryX * METERS_TO_PIXELS + WORLD_CENTER),
                                 (int) round(queryY * METERS_TO_PIXELS + WORLD_CENTER));

//        cv::circle(mImageGrid, circle_point, 1, cv::Scalar(0, 255, 0, 255), CV_FILLED, 8, 0);

        int indX = (int)floor((queryX * METERS_TO_CELLS) + OCCMAP_CENTER);
        int indY = (int)floor((queryY * METERS_TO_CELLS) + OCCMAP_CENTER);

        double p = 1 - 1 / (1 + exp(occMap[indX][indY])); // p = 0 if not occupied - p = 1 if occupied

        totalP += p;
        if (p > maxP){
            maxP = p;
        }
    }

    obstacleCost = abs(1 / (1.1 - (totalP/queryPoints))); // Inverse of the average occpancy of traversal. No longer dependent on length
    obstacleCost = abs(1 / (1.1 - (maxP))); // Inverse of the Max occpancy of traversal. No longer dependent on length

    // WORLD TO OCC CELL

    float midlineOffset = abs(node.distanceToMid);
    float x = node.p.x;
    float y = node.p.y;

    //if not first time
    double cost = 0;

    double w1 = 1;
    double w2 = 1;
    double w3 = 1;
    double w4 = 1;


    double angleDifference = 0;
    double angleDistance = 0;

    if( path.size() > 1){

        double lastAngle = GetAngle(parentId, path[path.size() - 2]);
        double newAngle = GetAngle(childId, parentId);
        angleDifference = abs(newAngle - lastAngle);
    }

    else {

        angleDistance = GetDistance(childId, parentId);
    }

    if (driveStraight) {
        cost = w2 * angleDifference + w3 * angleDistance;
        lattice[childId].edgeCost = w2 * angleDifference + w3 * angleDistance + w4 * obstacleCost; // For generate path

    }

    else if (lookAhead){
        cost = w1 * midlineOffset + w2 * angleDifference + w3 * angleDistance + w4 * obstacleCost; //+ w4 * FutureObstacleCost(childId, 4);// + w4 * obstacleCost;
        lattice[childId].cost = cost;

    }

    else{

        cost = w1 * midlineOffset + w2 * angleDifference + w3 * angleDistance + w4 * obstacleCost;
    }
    //cost = angleDistance;

//    lattice[childId].cost = cost;

    return cost;
}


tResult cGraphViz::PathInit(double carX,  double carY, std::map<nodeIdWrapper, graphNode> &graph) {

    // determine how to find starting node assume I have the starting node
    nodeIdWrapper startNode = FindNearestNode(carX, carY, graph);
    path.push_back(startNode);

    RETURN_NOERROR;

}

nodeIdWrapper cGraphViz::FindNearestNode(double x, double y, std::map<nodeIdWrapper, graphNode> &graph) {

    // START NAIVELY, then node selector will prune the tree forward

    float minDist = 9999999;
    nodeIdWrapper minId;

    for (auto const& it : graph) {

        nodeIdWrapper currId = it.first;
        float nodeX = it.second.p.x;
        float nodeY = it.second.p.y;

        float dist = (float)sqrt( pow(x - nodeX, 2) + pow(y -nodeY, 2) );

        if (dist < minDist){

            minDist = dist;
            minId = currId;
        }
    }

    return minId;
}





// INTIAL GRAPH SEARCH FUNCTINOS


std::queue<turn> cGraphViz::GetCommands(){

    std::queue<turn> commandList;

//    commandList.push(LEFT);
//    commandList.push(RIGHT);
//    commandList.push(STRAIGHT);
//    commandList.push(RIGHT);
//    commandList.push(LEFT);

    commandList.push(LEFT);
    commandList.push(LEFT);


//    commandList.push(STRAIGHT);



    return commandList;
}
MapPoint cGraphViz::GetGoalPosition() {

    MapPoint endLocation;
    // Query end positon of parking spot
    endLocation.x = 9;
    endLocation.y = 2;

    return endLocation;

}

tResult cGraphViz::GraphInit() {

    command = GetCommands();
    goal = GetGoalPosition();

    float minStartDist = 9999999;
    nodeIdWrapper minStartId;

    float minGoalDist = 9999999;
    nodeIdWrapper minGoalId;

    //LOG_INFO(cString::Format("graph: (%f, %f)",  m_odReader->graph[minId].p.x,  m_odReader->graph[minId].p.x));

    for (auto const& it : mapGraph) {

        nodeIdWrapper currId = it.first;
        float x = it.second.p.x;
        float y = it.second.p.y;

        float distStart = (float)sqrt( pow(x - carPose.x, 2) + pow(y - carPose.y, 2) );
        float distGoal = (float)sqrt( pow(x - goal.x, 2) + pow(y - goal.y, 2) );

        if (distStart < minStartDist){

            minStartDist = distStart;
            minStartId = currId;
        }

        if (distGoal < minGoalDist){

            minGoalDist = distGoal;
            minGoalId = currId;
        }
    }

    goalNode = minGoalId;
    startNode = minStartId;
    to_vist.push_front(minStartId);

    RETURN_NOERROR;
}

tResult cGraphViz::GraphSearch(tTimeStamp tmTimeOfTrigger) {

    //LOG_INFO("Looping");


    if (to_vist.empty()) {
        GenerateLattice(roads);
        graphSearched = true;
//        mImagePath(roi) = cv::Scalar( 0, 0, 0, 0);
        LOG_INFO("RANOUT");
        //GraphInit();
    }

    else {

        LOG_INFO("Expanding");

        nodeIdWrapper currNode = to_vist.front();

        if (command.size() == 0 && currNode.roadId == goalNode.roadId && currNode.uniqueId == goalNode.uniqueId){ // TODO Mabye change command size to 0, which is when there are no commands left
            GenerateLattice(roads);

//            mImagePath(roi) = cv::Scalar( 0, 0, 0, 0);
            graphSearched = true;
            // REACHED END
            RETURN_NOERROR;
        }

        to_vist.pop_front();

        if (history.size() == 0) {
            history.push_back(currNode.roadId);
        }

        else if (currNode.roadId != history.back()) {
            history.push_back(currNode.roadId);
        }

//        if (!visted_nodes.count(currNode)){
//
//            visted_nodes.insert(currNode);

            float x_start = mapGraph[currNode].p.x;
            float y_start = mapGraph[currNode].p.y;
            cv::Point2f nodePoint( (int)round(x_start*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER));
            cv::circle( mImageGrid, nodePoint, 5, cv::Scalar( 255, 255, 255, 255),CV_FILLED, 8,0);

            int numSucc = mapGraph[currNode].succId.size();

            for (int j = 0; j < numSucc; j++) {

                nodeIdWrapper succNode = mapGraph[currNode].succId[j];

                if (numSucc > 1){

                    if (mapGraph[currNode].succType[j] == command.front()){
                        command.pop(); //add error checking
                        to_vist.push_front(succNode);
                        pointer++;

                        float x_end = mapGraph[mapGraph[currNode].succId[j]].p.x;
                        float y_end = mapGraph[mapGraph[currNode].succId[j]].p.y;

                        cv::Point2f line_start( (int)round(x_start*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER));
                        cv::Point2f line_end( (int)round(x_end*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_end*METERS_TO_PIXELS + WORLD_CENTER));
                        cv::line( mImagePath, line_start, line_end, cv::Scalar( 255, 0, 0, 255 ),10, 8,0);

                        break;
                    }

                }

                else {

                    to_vist.push_front(succNode);

                    float x_end = mapGraph[ mapGraph[currNode].succId[j]].p.x;
                    float y_end = mapGraph[ mapGraph[currNode].succId[j]].p.y;

                    cv::Point2f line_start( (int)round(x_start*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER));
                    cv::Point2f line_end( (int)round(x_end*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_end*METERS_TO_PIXELS + WORLD_CENTER));
                    cv::line( mImagePath, line_start, line_end,  cv::Scalar(  255, 0, 0, 255 ),10, 8,0);

                }
//            }
        }
    }

    RETURN_NOERROR;
}

tResult cGraphViz::UpdateCarPos(){

    object_ptr<const ISample> pReadSample;

    if (IS_OK(PoseDataIn.GetLastSample(pReadSample)))
    {
        LOG_DUMP("Updating carPose Position");

        carInitalized = true;

        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(pReadSample);
        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &carPose.x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &carPose.y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &carPose.theta));

        LOG_DUMP(cString::Format("carPose Pos: (%f, %f, %f)", carPose.x, carPose.y, carPose.theta));
        //LOG_DUMP(cString::Format("carPose Pos - map Coords: (%f, %f)", carPose.x/map_res, carPose.y/map_res));

    }

//    if (!carInitalized) { // Used to test path planning when your not reciving localization data
//        carInitalized = true;
//
//        carPose.x = 0;
//        carPose.y = 0;
//        carPose.theta = 60* DEG2RAD;
//
//    }

    UpdateSensorPos();
    RETURN_NOERROR;



}

tResult cGraphViz::UpdateSensorPos() {

    lidarGlobal = SensorToGlobal(carPose, lidarLocal);
    frontPointGlobal = SensorToGlobal(carPose, frontPoint);

    RETURN_NOERROR;
}

void cGraphViz::ReCenterImage() {

//    int x_offset = int(round(WINDOW_WIDTH/float(2)));
//    int y_offset = round(WINDOW_HEIGHT/float(2));
//
//
//    cv::Point2f centerPivot( (carPose.x*METERS_TO_PIXELS + WORLD_CENTER), (carPose.y*METERS_TO_PIXELS + WORLD_CENTER));
//    cv::Point2f botLeftPivot( (int)round(centerPivot.x - x_offset), (int)round(centerPivot.y - y_offset));
//    cv::Point2f topRightPivot( (int)round(centerPivot.x + x_offset), (int)round(centerPivot.y + y_offset));
//
//    roiPixel = Rect(botLeftPivot,topRightPivot );
//    //LOG_INFO(cString::Format("Eigen: (%f, %f)", botLeftPivot.x, botLeftPivot.y));
//
//    //roiPixel = Rect(int(round(WORLD_CENTER - WINDOW_WIDTH/2)),int(round(WORLD_CENTER - WINDOW_WIDTH/2)),int(WINDOW_WIDTH),int(WINDOW_WIDTH));

    mImageGridMasked = mImageGrid(roi);// + mImagePath(roi);
    mImagePathMasked = mImagePath(roi);
    mImageOverlayMasked = mImageOverlay(roi);
    //mImagePath = mImagePath(roiPixel);

}

void cGraphViz::ComposeImage()
{


    //mImage = mImageGridMasked; //+ mImagePath;
    mImageSend = mImageGridMasked + mImagePathMasked + mImageOccMapMasked + mImageOverlayMasked ;// + mImageDebug;
//    mImageSend = mImageOccMapMasked;
}

// FUNCTIONS TO GENERATE GRAPHS FOR COARSE AND LATTICE SEARCH

tResult cGraphViz::GenerateLattice(map<int,roadElementWrapper> &roads) {

    std::vector<int> segmentId;
    float lateral_density = 3; //nodes per meter
    float longitudinal_density = 1; //nodes per meter

    // Manually Enter in Segs
    //    segmentId.push_back(150);

    segmentId = history;


    LatticePoints(roads, segmentId, lateral_density, longitudinal_density);

    for (auto const& it : lattice) {
        nodeIdWrapper currId = it.first;
        float x_start = it.second.p.x;
        float y_start = it.second.p.y;

        cv::Point2f circle_point( (int)round(x_start*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER));

//        cv::circle( mImagePath, circle_point, 4, cv::Scalar( 255, 0, 0, 255),CV_FILLED, 8,0);

        std::ostringstream ss;
        //ss << it.second.id;
        //ss << currId.uniqueId;

        //DOING THIS IS PROCESS NOW
//
//        ss << int(round(it.second.distanceToMid));
//        putText(mImageGrid, ss.str(), cv::Point((int)round(x_start*METERS_TO_PIXELS + WORLD_CENTER),
//                                                (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER)),
//                cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(255,255,255,255), 2, 8,true);
//
//

        for (size_t i = 0; i < it.second.succ->size(); i++){
            LOG_INFO("PRINTING DOT");
            LOG_INFO(cString::Format("size: (%d)",  it.second.succ->size()));


            double x_end =             lattice[it.second.succ->at(i)].p.x;

            double y_end =            lattice[it.second.succ->at(i)].p.y;


            cv::Point2f line_start( (int)round(x_start*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER));
            cv::Point2f line_end( (int)round(x_end*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_end*METERS_TO_PIXELS + WORLD_CENTER));
//            cv::line( mImagePath, line_start, line_end, cv::Scalar( 0, 200, 0, 50 ),2, 8,0);

        }


    }

    RETURN_NOERROR;

}

int cGraphViz::GetNodeType(roadElementWrapper segement){

    int type = STRAIGHT;

    if (segement.junction != -1){ // Segement is part of a junction

        type = TURN;

    }

    return type;

}

void cGraphViz::LatticePoints(std::map<int,roadElementWrapper> &roadNetwork, std::vector<int> segmentId, float lateral_density, float longitudinal_density)
{
    int vect_count = 0;
    float road_width = 0.23;

    for (size_t i = 0; i < segmentId.size(); i++) {



        // TODO: Section 1: Intro
        int currSegId = segmentId[i];
        roadElementWrapper currEl = roadNetwork[currSegId];

        // To set type of nodes in this segement for Lukas, george please add this to your new lattice points

        int currSegNodeType = GetNodeType(currEl);

        roadGeometryWrapper geo = currEl.geometry[0];

        float road_length = geo.length;

        longitudinal_density = 4;

        int num_lat_points = (int)round(road_width * lateral_density);
        int num_long_points = (int)round(road_length * longitudinal_density);
        num_lat_points = 5; // ODD NUMBER
//        num_long_points = 4;

        int points_per_lane = 3; // must be odd
        num_lat_points = (points_per_lane * 2) - 1;

//        float midline_dist = abs(currEl.laneSplit / currEl.scale);
        float midline_dist = abs(road_width);

        float latStep = (midline_dist / (points_per_lane/2)); //lines from road line to road line
        float latScale = 0;

        std::vector<float> tempxs;
        std::vector<float> tempys;


        // Generate points along line segment. Points for straight driving are down the middle of the lane, points for curves are long middle of curve
        for (int z = 0; z < num_long_points; z++) {

            float ds = (float)(z + 0.1) / (num_long_points);

            //Find points from parametric polynomial in local coordinate
            tempxs.push_back(CubicPoly(geo.aU, geo.bU, geo.cU, geo.dU, ds));
            tempys.push_back(CubicPoly(geo.aV, geo.bV, geo.cV, geo.dV, ds));

        }

        // TODO: Section 2: Have generated a set amount of points along the geometry.
        // TODO: They are stored in tempx and tempy


        bool flag = true;

        //TODO speical case for turns
        // Generate Graph based on the sampled points

        for (int j = 0; j < num_lat_points; j++){ // for each lateral Point
            std::vector<Pose3D> list;
            Pose3D pose;

            float curHeading = 0.0, headingChange = 0.0, lastHeading = 0.0;
            // TODO: Section 1: Intro
            for (int k = 0; k < num_long_points; k++)
            {
                //TODO: Section 3: This takes a middle line and transforms it into a global reference frame
                //TODO: Correct coordinates should be stored in currGraphNode
                if (k == 0){
                    curHeading = 0;
                }
                else if (k == num_long_points - 1) {

                    float x0 = tempxs[k - 1], y0 = tempys[k - 1];
                    float x1 = tempxs[k], y1 = tempys[k];

                    curHeading = atan2(y1 - y0, x1 - x0);
                }

                else
                {
                    float x0 = tempxs[k - 1], y0 = tempys[k - 1];
                    float x1 = tempxs[k + 1], y1 = tempys[k + 1];

                    curHeading = atan2(y1 - y0, x1 - x0);

//                    lastHeading = curHeading;
//                    curHeading = atan2(y1 - y0, x1 - x0);
//                    headingChange += normalizeAngle(curHeading - lastHeading, 0);
//                    headingChange = normalizeAngle(headingChange, 0);
                }

                graphNode currGraphNode;

                currGraphNode.type = currSegNodeType;
                // LATERAL TRANSFORMATION

                pose.p.x = (tempxs[k]) * currEl.scale;
                pose.p.y = (tempys[k]) * currEl.scale;
                pose.p.z = currEl.altitude;
                pose.q = toQuaternion(0, 0, geo.hdg);

                float tempLanex = RotateCCWX(0, latScale / currEl.scale, curHeading);
                float tempLaney = RotateCCWY(0, latScale / currEl.scale, curHeading);

                currGraphNode.p.x =  pose.p.x + tempLanex * currEl.scale ;
                currGraphNode.p.y =  pose.p.y  + tempLaney * currEl.scale ;

                // GLOBAL TRANSFORMATION
                float newx = RotateCCWX(currGraphNode.p.x,currGraphNode.p.y, geo.hdg);
                float newy = RotateCCWY(currGraphNode.p.x, currGraphNode.p.y, geo.hdg);
                currGraphNode.p.x = newx + (geo.x* currEl.scale);
                currGraphNode.p.y = newy + (geo.y* currEl.scale);

                // TODO: Section 4: Turn stuff

                // ADD LATERAL OFFSET TO NODE FOR USE IN PATH PLANNING, NEED TO DEAL WITH TURNS SEPERATELY
                if (roadNetwork[currSegId].junction != -1) {

                    currGraphNode.distanceToMid = latScale / currEl.scale;
                }
                else {
                    currGraphNode.distanceToMid = (latScale - roadNetwork[currSegId].laneSplit) / currEl.scale; // Lane split will either be + or minues 10
                }
                // TOdO: Section 5: Pre- Have point in space [currGraphNode]
                // TODO: Section 5: Generating lattice

                // CURRENT NODE ID
                nodeIdWrapper currId;
                nodeIdWrapper neighbourId;

                currId.roadId = currSegId;
                neighbourId.roadId = currSegId;

                if (!roadNetwork[currSegId].reverseId) {

                    currId.uniqueId = j + k * num_lat_points + 1;

                    for (int t = 0; t < num_lat_points; t++) {

                        if (t != j) {
                            // This line loops through all neighbours to find where currGraphNode connects to via a neural net
                            neighbourId.uniqueId = t + k * num_lat_points + 1;
                            currGraphNode.neighbours->push_back(neighbourId);
                        }

                        if (k != 0 && k != num_long_points - 1) { // NORMAL CASE - nodes in the middle

                            nodeIdWrapper childId;
                            childId.roadId = currSegId;

                            childId.uniqueId = t + (k + 1) * num_lat_points + 1;
                            currGraphNode.succ->push_back(childId);

                            childId.uniqueId = t + (k - 1) * num_lat_points + 1;
                            currGraphNode.pred->push_back(childId);
                        }

                        else if (k == 0) { // START CASE - Node in first index position, can either be the start of end of segment

                            nodeIdWrapper childId;
                            childId.roadId = currSegId;


                            if (t == 0) { // Only assign once

                                if (i != 0) { // if not on the first line segment

                                    int prevSegId = segmentId[i - 1];
                                    currGraphNode.pred = &roadNetwork[prevSegId].endNodes;

                                }

                                roadNetwork[currSegId].startNodes.push_back(
                                        currId); // add CURRENT ID to road information so other line segments can query correctly
                            }

                            childId.uniqueId = t + (k + 1) * num_lat_points + 1;
                            currGraphNode.succ->push_back(childId);

                            LOG_INFO("WORKING FOR FIRST ONE");
                        }

                        else if (k == num_long_points - 1) { // END CASE - Node in last index position, usally at the end, but can be at the start

                            nodeIdWrapper childId;
                            childId.roadId = currSegId;

                            if (t == 0) { // Only assign once

                                if (i < segmentId.size() - 1) { // if not on the first line segment

                                    int nextSegId = segmentId[i + 1];
                                    currGraphNode.succ = &roadNetwork[nextSegId].startNodes;
                                    LOG_INFO("ASSIGNING LAST NODES NOT REVERSED");

                                }

                                roadNetwork[currSegId].endNodes.push_back(
                                        currId); // add CURRENT ID to road information so other line segments can query correctly

                            }

                            childId.uniqueId = t + (k - 1) * num_lat_points + 1;
                            currGraphNode.pred->push_back(childId);
                        }
                    }
               }

                else {

                    currId.uniqueId = j + (num_long_points - 1 - k) * num_lat_points + 1;

                    for (int t = 0; t < num_lat_points; t++) {

                        if (t != j) {
                            neighbourId.uniqueId = t + (num_long_points - 1 - k) * num_lat_points + 1;
                            currGraphNode.neighbours->push_back(neighbourId);
                        }

                        if (k != 0 && k != num_long_points - 1) { // NORMAL CASE - nodes in the middle

                            nodeIdWrapper childId;
                            childId.roadId = currSegId;

                            childId.uniqueId = t + ((num_long_points - 1 - k) + 1) * num_lat_points + 1;
                            currGraphNode.succ->push_back(childId);

                            childId.uniqueId = t + ((num_long_points - 1 - k)  - 1) * num_lat_points + 1;
                            currGraphNode.pred->push_back(childId);
                        }

                        else if (k == 0) { // START CASE - Node in first index position, can either be the start of end of segment

                            nodeIdWrapper childId;
                            childId.roadId = currSegId;

                            if (t == 0) { // Only assign once

                                if (i < segmentId.size() - 1) { // if not on the first line segment

                                    int nextSegId = segmentId[i + 1];
                                    currGraphNode.succ = &roadNetwork[nextSegId].startNodes;
                                    LOG_INFO("ASSIGNING LAST NODES NOT REVERSED");

                                }

                                roadNetwork[currSegId].endNodes.push_back(
                                        currId); // add CURRENT ID to road information so other line segments can query correctly

                            }

                            childId.uniqueId = t + ((num_long_points - 1 - k) - 1) * num_lat_points + 1;
                            currGraphNode.pred->push_back(childId);
                        }

                        else if (k == num_long_points - 1) { // END CASE - Node in last index position, usally at the end, but can be at the start
                            nodeIdWrapper childId;
                            childId.roadId = currSegId;


                            if (t == 0) { // Only assign once

                                if (i != 0) { // if not on the first line segment

                                    int prevSegId = segmentId[i - 1];
                                    currGraphNode.pred = &roadNetwork[prevSegId].endNodes;

                                }

                                roadNetwork[currSegId].startNodes.push_back(
                                        currId); // add CURRENT ID to road information so other line segments can query correctly
                            }

                            childId.uniqueId = t + ((num_long_points - 1 - k)  + 1) * num_lat_points + 1;
                            currGraphNode.succ->push_back(childId);

                            LOG_INFO("WORKING FOR FIRST ONE");
                        }
                    }
                }

//                nodeIdWrapper currId;
//                currId.roadId = currSegId;
//                currId.uniqueId =  j + k * num_lat_points + 1;
////
////                if (i == 0 && k == 0 && j == 0) {
////
////                    startNodeId = currId;
////                }
////
////                if (i == segmentId.size() - 1 && k == num_long_points - 1) {
////
////                    startNodeId = currId;
////
////                    //TODO all for many end nodes
////                }
//
//                // ADD CONNECTION INFORMATION
//
//                for (int t = 0; t < num_lat_points; t++) {
//
//                    if (k != 0 && k != num_long_points - 1) { // NORMAL CASE - nodes in the middle
//
//                        nodeIdWrapper childId;
//                        childId.roadId = currSegId;
//
//                        if (!roadNetwork[currSegId].reverse) {
//                            childId.uniqueId = t + (k + 1) * num_lat_points + 1;
//                            currGraphNode.succ->push_back(childId);
//
//                            childId.uniqueId = t + (k - 1) * num_lat_points + 1;
//                            currGraphNode.pred->push_back(childId);
//                        }
//
//                        else {
//                            childId.uniqueId = t + (k - 1) * num_lat_points + 1;
//                            currGraphNode.succ->push_back(childId);
//
//                            childId.uniqueId = t + (k + 1) * num_lat_points + 1;
//                            currGraphNode.pred->push_back(childId);
//                        }
//                    }
//
//                    else if (k == 0) { // START CASE - Node in first index position, can either be the start of end of segment
//
//                        nodeIdWrapper childId;
//                        childId.roadId = currSegId;
//
//                        if (!roadNetwork[currSegId].reverse) {
//
//                            if (t == 0) { // Only assign once
//
//                                if (i != 0) { // if not on the first line segment
//
//                                    int prevSegId = segmentId[i - 1];
//                                    currGraphNode.pred = &roadNetwork[prevSegId].endNodes;
//
//                                }
//
//                                roadNetwork[currSegId].startNodes.push_back(currId); // add CURRENT ID to road information so other line segments can query correctly
//                            }
//
//                            childId.uniqueId = t + (k + 1) * num_lat_points + 1;
//                            currGraphNode.succ->push_back(childId);
//
//                            LOG_INFO("WORKING FOR FIRST ONE");
//                        }
//
//                        else {
//
//                            LOG_INFO("NOT WORKING SHOULD NOT BE RESERVESE");
//
//
//                            if (t == 0) { // Only assign once
//
//                                if (i < segmentId.size() - 1) { // if not on last segment
//
//                                    int nextSegId = segmentId[i + 1];
//                                    currGraphNode.succ = &roadNetwork[nextSegId].startNodes;
//
//                                }
//
//                                roadNetwork[currSegId].endNodes.push_back(currId); // add CURRENT ID to road information so other line segments can query correctly
//
//                            }
//
//                            childId.uniqueId = t + (k + 1) * num_lat_points + 1;
//                            currGraphNode.pred->push_back(childId);
//                        }
//                    }
//
//
//                    else if (k == num_long_points - 1) { // END CASE - Node in last index position, usally at the end, but can be at the start
//
//                        nodeIdWrapper childId;
//                        childId.roadId = currSegId;
//
//                        if (!roadNetwork[currSegId].reverse) {
//                            if (t == 0) { // Only assign once
//
//                                if (i < segmentId.size() - 1) { // if not on the first line segment
//
//                                    int nextSegId = segmentId[i + 1];
//                                    currGraphNode.succ = &roadNetwork[nextSegId].startNodes;
//                                    LOG_INFO("ASSIGNING LAST NODES NOT REVERSED");
//
//                                }
//
//                                roadNetwork[currSegId].endNodes.push_back(currId); // add CURRENT ID to road information so other line segments can query correctly
//
//                            }
//
//                            childId.uniqueId = t + (k - 1) * num_lat_points + 1;
//                            currGraphNode.pred->push_back(childId);
//                        }
//
//                        else {
//                            LOG_INFO("ASSIGNING LAST NODES REVERSED");
//
//                            if (t == 0) { // Only assign once
//
//                                if (i != 0) { // if not on the first line segment
//
//                                    int prevSegId = segmentId[i - 1];
//                                    currGraphNode.pred = &roadNetwork[prevSegId].endNodes;
//
//                                }
//
//                                roadNetwork[currSegId].startNodes.push_back(currId); // add CURRENT ID to road information so other line segments can query correctly
//
//                            }
//                            childId.uniqueId = t + (k - 1) * num_lat_points + 1;
//                            currGraphNode.succ->push_back(childId);
//                        }
//                    }
//                }


                lattice[currId] = currGraphNode;

            }


            if (flag){

                latScale = abs(latScale) + latStep;
                flag = false;
            }

            else{

                latScale *= -1;
                flag = true;
            }

        }
    }
}

std::map<nodeIdWrapper, graphElement> cGraphViz::GenerateMapGraph(std::map<int, roadElementWrapper > &roads, int num)
{

    std::map<nodeIdWrapper, graphElement> newMapGraph;
    int numPoints;

    for (auto const& it : roads) { // Loop Through All Roads

        std::vector<Pose3D> list;
        Pose3D pose;
        roadElementWrapper el = it.second;
        roadGeometryWrapper geo = el.geometry[0];

        float curHeading = 0;
        numPoints = 4;
        std::vector<float> tempxs;
        std::vector<float> tempys;

        // Generate points along line segment. Points for straight driving are down the middle of the lane, points for curves are long middle of curve
        for (int z = 0; z < numPoints; z++) {
            float ds = (float)(z + 0.1) / (numPoints);

            //Find points from parametric polynomial in local coordinate
            tempxs.push_back(CubicPoly(geo.aU, geo.bU, geo.cU, geo.dU, ds));
            tempys.push_back(CubicPoly(geo.aV, geo.bV, geo.cV, geo.dV, ds));

        }

        for (int i = 0; i < numPoints; i++) { // Determine Number of points to Place //TODO place points based on desnity vs set number

            graphElement pathEl;

            if (i == 0){
                curHeading = 0;

            }
            else if (i == numPoints - 1) {

                float x0 = tempxs[i - 1], y0 = tempys[i - 1];
                float x1 = tempxs[i], y1 = tempys[i];
                curHeading = atan2(y1 - y0, x1 - x0);

            }

            else
            {
                float x0 = tempxs[i - 1], y0 = tempys[i - 1];
                float x1 = tempxs[i + 1], y1 = tempys[i + 1];

                curHeading = atan2(y1 - y0, x1 - x0);
            }


//            roadGeometryWrapper geo = el.geometry[0];

            pose.p.x = (tempxs[i]) * el.scale;
            pose.p.y = (tempys[i]) * el.scale;
            pose.p.z = el.altitude;
            pose.q = toQuaternion(0, 0, geo.hdg);

            float tempLanex = RotateCCWX(0, el.laneSplit / el.scale, curHeading);
            float tempLaney = RotateCCWY(0, el.laneSplit / el.scale, curHeading);

            pathEl.p.x =  pose.p.x + tempLanex * el.scale ;
            pathEl.p.y =  pose.p.y  + tempLaney * el.scale ;

            // GLOBAL TRANSFORMATION
            float newx = RotateCCWX(pathEl.p.x,pathEl.p.y, geo.hdg);
            float newy = RotateCCWY(pathEl.p.x, pathEl.p.y, geo.hdg);
            pathEl.p.x = newx + (geo.x* el.scale);
            pathEl.p.y = newy + (geo.y* el.scale);


            nodeIdWrapper currNode;

            currNode.roadId = it.first;
            currNode.uniqueId = i;

//            if (!el.reverse && el.uniqueId < 0){
            if (!el.reverseId){

                currNode.uniqueId = i;

                nodeIdWrapper succNode;

                if (i == 0) {
                    succNode.roadId = it.first;
                    succNode.uniqueId = currNode.uniqueId + 1;
                    pathEl.succId.push_back(succNode);
                    pathEl.succType.push_back(STRAIGHT);
                }

                else if (i == numPoints - 1) {
                    for (size_t k = 0; k < el.succIds.size(); k++){

                        LOG_INFO("SHOULD NOT BE LOOPNIG");
                        succNode.roadId = el.succIds[k];
                        succNode.uniqueId = 0;
                        pathEl.succId.push_back(succNode);
                        pathEl.succType.push_back(el.succType[k]);
                    }
                }

                else {
                    // Middle of path
                    succNode.roadId = it.first;
                    succNode.uniqueId = currNode.uniqueId + 1;
                    pathEl.succId.push_back(succNode);
                    pathEl.succType.push_back(STRAIGHT);

                    //Query George's Function here and add SuccId's
                }
            }

            else {

                currNode.uniqueId = numPoints - 1 - i;

                nodeIdWrapper succNode;

                if (i == 0) {

                    for (size_t k = 0; k < el.succIds.size(); k++){

                        LOG_INFO("SHOULD NOT BE LOOPNIG");
                        succNode.roadId = el.succIds[k];
                        succNode.uniqueId = 0;
                        pathEl.succId.push_back(succNode);
                        pathEl.succType.push_back(el.succType[k]);
                    }
                }

                else if (i == numPoints - 1) {
                    succNode.roadId = it.first;
                    succNode.uniqueId = currNode.uniqueId + 1;
                    pathEl.succId.push_back(succNode);
                    pathEl.succType.push_back(STRAIGHT);
                }

                else {
                    // Middle of path
                    succNode.roadId = it.first;
                    succNode.uniqueId = currNode.uniqueId + 1;
                    pathEl.succId.push_back(succNode);
                    pathEl.succType.push_back(STRAIGHT);

                    //Query George's Function here and add SuccId's
                }
            }

            newMapGraph[currNode] = pathEl;

        }
    }

    return newMapGraph;
//    RETURN_NOERROR;
}


std::map<nodeIdWrapper, graphElement> cGraphViz::GenerateMapGraphNew(std::map<int, ODReader::roadElement> &roads, int num)
{

    std::map<nodeIdWrapper, graphElement> newMapGraph;
    int numPoints;

    for (auto const& it : roads) { // Loop Through All Roads

        std::vector<Pose3D> list;
        Pose3D pose;
        ODReader::roadElement el = it.second;
        ODReader::roadGeometry geo = el.geometry[0];

        float curHeading = 0;
        numPoints = 4;
        std::vector<float> tempxs;
        std::vector<float> tempys;

        // Generate points along line segment. Points for straight driving are down the middle of the lane, points for curves are long middle of curve
        for (int z = 0; z < numPoints; z++) {
            float ds = (float)(z + 0.1) / (numPoints);

            //Find points from parametric polynomial in local coordinate
            tempxs.push_back(CubicPoly(geo.aU, geo.bU, geo.cU, geo.dU, ds));
            tempys.push_back(CubicPoly(geo.aV, geo.bV, geo.cV, geo.dV, ds));

        }

        for (int i = 0; i < numPoints; i++) { // Determine Number of points to Place //TODO place points based on desnity vs set number

            graphElement pathEl;

            if (i == 0){
                curHeading = 0;

            }
            else if (i == numPoints - 1) {

                float x0 = tempxs[i - 1], y0 = tempys[i - 1];
                float x1 = tempxs[i], y1 = tempys[i];
                curHeading = atan2(y1 - y0, x1 - x0);

            }

            else
            {
                float x0 = tempxs[i - 1], y0 = tempys[i - 1];
                float x1 = tempxs[i + 1], y1 = tempys[i + 1];

                curHeading = atan2(y1 - y0, x1 - x0);
            }


//            roadGeometryWrapper geo = el.geometry[0];

            pose.p.x = (tempxs[i]) * el.scale;
            pose.p.y = (tempys[i]) * el.scale;
            pose.p.z = el.altitude;
            pose.q = toQuaternion(0, 0, geo.hdg);

            float tempLanex = RotateCCWX(0, el.laneSplit / el.scale, curHeading);
            float tempLaney = RotateCCWY(0, el.laneSplit / el.scale, curHeading);

            pathEl.p.x =  pose.p.x + tempLanex * el.scale ;
            pathEl.p.y =  pose.p.y  + tempLaney * el.scale ;

            // GLOBAL TRANSFORMATION
            float newx = RotateCCWX(pathEl.p.x,pathEl.p.y, geo.hdg);
            float newy = RotateCCWY(pathEl.p.x, pathEl.p.y, geo.hdg);
            pathEl.p.x = newx + (geo.x* el.scale);
            pathEl.p.y = newy + (geo.y* el.scale);


            nodeIdWrapper currNode;

            currNode.roadId = it.first;
            currNode.uniqueId = i;

            if (!el.reverse){

                currNode.uniqueId = i;

                nodeIdWrapper succNode;

                if (i == 0) {
                    succNode.roadId = it.first;
                    succNode.uniqueId = currNode.uniqueId + 1;
                    pathEl.succId.push_back(succNode);
                    pathEl.succType.push_back(STRAIGHT);
                }

                else if (i == numPoints - 1) {
                    for (size_t k = 0; k < el.succIds.size(); k++){

                        LOG_INFO("SHOULD NOT BE LOOPNIG");
                        succNode.roadId = el.succIds[k];
                        succNode.uniqueId = 0;
                        pathEl.succId.push_back(succNode);
//                        pathEl.succType.push_back(el.succType[k]);
                        pathEl.succType.push_back((turn)el.succType[k]);

                    }
                }

                else {
                    // Middle of path
                    succNode.roadId = it.first;
                    succNode.uniqueId = currNode.uniqueId + 1;
                    pathEl.succId.push_back(succNode);
                    pathEl.succType.push_back(STRAIGHT);

                    //Query George's Function here and add SuccId's
                }
            }

            else {

                currNode.uniqueId = numPoints - 1 - i;

                nodeIdWrapper succNode;

                if (i == 0) {

                    for (size_t k = 0; k < el.succIds.size(); k++){

                        LOG_INFO("SHOULD NOT BE LOOPNIG");
                        succNode.roadId = el.succIds[k];
                        succNode.uniqueId = 0;
                        pathEl.succId.push_back(succNode);
                        pathEl.succType.push_back((turn)el.succType[k]);
                    }
                }

                else if (i == numPoints - 1) {
                    succNode.roadId = it.first;
                    succNode.uniqueId = currNode.uniqueId + 1;
                    pathEl.succId.push_back(succNode);
                    pathEl.succType.push_back(STRAIGHT);
                }

                else {
                    // Middle of path
                    succNode.roadId = it.first;
                    succNode.uniqueId = currNode.uniqueId + 1;
                    pathEl.succId.push_back(succNode);
                    pathEl.succType.push_back(STRAIGHT);

                    //Query George's Function here and add SuccId's
                }
            }

            newMapGraph[currNode] = pathEl;

        }
    }

    return newMapGraph;
//    RETURN_NOERROR;
}

Quaternion cGraphViz::toQuaternion(double pitch, double roll, double yaw)
{
    Quaternion q;
    double t0 = std::cos(yaw * 0.5);
    double t1 = std::sin(yaw * 0.5);
    double t2 = std::cos(roll * 0.5);
    double t3 = std::sin(roll * 0.5);
    double t4 = std::cos(pitch * 0.5);
    double t5 = std::sin(pitch * 0.5);

    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x = t0 * t3 * t4 - t1 * t2 * t5;
    q.y = t0 * t2 * t5 + t1 * t3 * t4;
    q.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q;
}

float cGraphViz::CubicPoly(float a1, float b1, float c1, float d1, float ds)
{

    return (a1 + b1*ds + c1*pow(ds, 2.0) + d1*pow(ds, 3.0));
}

float cGraphViz::RotateCCWX(float u2, float v2, float hdg2)
{
    return (u2*cos(hdg2) - v2*sin(hdg2));
}

float cGraphViz::RotateCCWY(float u1, float v1, float hdg1)
{
    return (u1*sin(hdg1) + v1*cos(hdg1));
}

//MAPPING FUNCTINOS
//void cGraphViz::CellToPixel() {
//}
void cGraphViz::UpdateOccMap(Rect frame, Size window) {

    // Slice from Grid Map * Still in Cells
    UpdateGrayGrid(frame);
    CropOccMap(frame, window);

}

void cGraphViz::CropOccMap(Rect frame, Size window) {

    // Slice from Grid Map * Still in Cells

    gridMapMasked = gridMap(frame);
    cv::resize(gridMapMasked, mImageOccMapMasked, window, 0, 0, cv::INTER_AREA);
//    mImageOccMapMasked = cv::Scalar(0,0,100,100);
}

void cGraphViz::InitalizeOccMap(){

    // WE ARE GOING TO METERS NOW

    // TODO CHANEG ALL TO METERS
    lidarLocal.x = 0.47;
    lidarLocal.y = 0;
    lidarLocal.theta = 0 * DEG2RAD;

    frontPoint.x = 0.47; //47 cm
    frontPoint.y = 0;
    frontPoint.theta = 0 * DEG2RAD;

    US_SideRightLocal.x = 0.250;
    US_SideRightLocal.y = -85;
    US_SideRightLocal.theta = 270 * DEG2RAD;

    US_SideLeftLocal.x = 250;
    US_SideLeftLocal.y = 85;
    US_SideLeftLocal.theta = 90 * DEG2RAD;

    US_RearCenterLocal.x = -100;
    US_RearCenterLocal.y = 0;
    US_RearCenterLocal.theta = 180 * DEG2RAD;

    US_RearRightLocal.x = -90;
    US_RearRightLocal.y = -11;
    US_RearRightLocal.theta = 270 * DEG2RAD;

    US_RearLeftLocal.x = -90;
    US_RearLeftLocal.y = 11;
    US_RearLeftLocal.theta = 90 * DEG2RAD;

    // Map Params
    map_res = (1 / METERS_TO_CELLS) * 1000; // in mm
    wall_thickness = 100;

    //Inverse Sensor model params
    angular_offset_threshold = 4.0  * DEG2RAD; //degrees
    log_prior = 0;

    // Grid Window
    window_length = 2000;//in mm
    window_width = 2000;

    window_rows = window_length / map_res;
    window_colums = window_width / map_res;

    // Map initlization

    car_initalized = false;

    for (int i = 0; i < OCCMAP_WIDTH; i++) {
        for (int j = 0; j < OCCMAP_HEIGHT; j++) {
            occMap[i][j] = log_prior;
        }
    }


    /// Place debuging obstacles
    cv::Point botLeftPivotCell(
            (int)(round(3 * METERS_TO_CELLS + OCCMAP_CENTER)),
            (int)(round(0 * METERS_TO_CELLS + OCCMAP_CENTER)));

    cv::Point topRightPivotCell(
            (int)(round(4 * METERS_TO_CELLS + OCCMAP_CENTER)),
            (int)(round(1 * METERS_TO_CELLS + OCCMAP_CENTER)));

//    PlaceObstacle(Rect(botLeftPivotCell, topRightPivotCell));

    gridMap = cv::Mat(OCCMAP_WIDTH, OCCMAP_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0) );
//    cv::Point2f textPos( OCCMAP_WIDTH/2, OCCMAP_HEIGHT/2);
//    putText( gridMap, "OCCP MAP", textPos, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0, 255), 2, 8, true);
//    gridMap = cv::Mat(OCCMAP_WIDTH, OCCMAP_HEIGHT, CV_8UC1, cv::Scalar::all(128));
//    mImageOccMapMasked = gridMap(roiPixel);

}

void cGraphViz::InitalizeSensorStream(){


    object_ptr<IStreamType> pLSData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pLSData, m_LSStructSampleFactory)) {

        (adtf_ddl::access_element::find_index(m_LSStructSampleFactory,"ui32Size", m_ddlLSDataId.size));
        (adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray));
    }

    else {

        LOG_WARNING("No mediadescription for tTemplateData found!");
    }

    Register(laserDataIn, "LaserScanner" , pLSData);


    // Ultrasonic Data

    object_ptr<IStreamType> pTypeUSData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tUltrasonicStruct", pTypeUSData, m_USDataSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideLeft") + cString(".f32Value"), m_ddlUltrasonicStructIndex.SideLeft.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideRight") + cString(".f32Value"), m_ddlUltrasonicStructIndex.SideRight.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearLeft") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearLeft.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearCenter") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearCenter.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearRight") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearRight.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }
    Register(USDataIn, "ultrasonic_struct", pTypeUSData);



}

tResult cGraphViz::ReadSensorStream(){


    object_ptr<const ISample> pSampleFromUS;

    if (IS_OK(USDataIn.GetLastSample(pSampleFromUS)))
    {
        auto oDecoderUS = m_USDataSampleFactory.MakeDecoderFor(*pSampleFromUS);

        RETURN_IF_FAILED(oDecoderUS.IsValid());


        // retrieve the values (using convenience methods that return a variant)
        //IMU
        tUltrasonicStruct US_data;

        //we do not need the timestamps here
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.SideLeft.value, &US_data.tSideLeft.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.SideRight.value, &US_data.tSideRight.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearLeft.value, &US_data.tRearLeft.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearCenter.value, &US_data.tRearCenter.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearRight.value, &US_data.tRearRight.f32Value));

    }



    object_ptr<const ISample> pReadLaserSample;


    if (IS_OK(laserDataIn.GetLastSample(pReadLaserSample)))
    {
        auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pReadLaserSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        tSize numOfScanPoints = 0;

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLSDataId.size, &numOfScanPoints));

        const tPolarCoordiante* pCoordiante = reinterpret_cast<const tPolarCoordiante*>(oDecoder.GetElementAddress(m_ddlLSDataId.scanArray));

        std::vector<tPolarCoordiante> scan;
        tPolarCoordiante scanPoint;

        for (tSize i = 0; i < numOfScanPoints; ++i){

            scanPoint.f32Radius = pCoordiante[i].f32Radius * (float)0.001;
            scanPoint.f32Angle = pCoordiante[i].f32Angle;

            if ((scanPoint.f32Angle < 90) || (scanPoint.f32Angle > 280)){

                scanPoint.f32Angle = normalizeAngle(scanPoint.f32Angle* static_cast<tFloat32>(DEG2RAD), 0) * static_cast<tFloat32>(RAD2DEG);
                scanPoint.f32Angle *= -1;

                scan.push_back(scanPoint);

            }

//            if (i % 3 == 0){

//                scanPoint.f32Angle = normalizeAngle(pCoordiante[i].f32Angle* static_cast<tFloat32>(DEG2RAD), 0) * static_cast<tFloat32>(RAD2DEG);
//                scanPoint.f32Angle *= -1;

//                float range = pCoordiante[i].f32Radius * (float)0.001;

//                MapPoint lidar_point;

//                lidar_point.x = range * cos(scanPoint.f32Angle* static_cast<tFloat32>(DEG2RAD));
//                lidar_point.y = range * sin(scanPoint.f32Angle* static_cast<tFloat32>(DEG2RAD)); //need to flip over x_axis

//                MapPoint lidar_point_global = PointToGlobal(lidarGlobal, lidar_point);

//                DebugPoint dp;

//                dp.x = lidar_point_global.x;
//                dp.y = lidar_point_global.y; //flip over x-axis
//                dp.type = LASER;
//                dp.Id = scanPoint.f32Angle;//(int)pCoordiante[i].f32Angle;

//                debugPointList.push_back(dp);
//            }
        }

//        LOG_INFO("MAPPING!");
        if (carInitalized){
//            MapLaserData(scan);
        }
    }

    RETURN_NOERROR;

}

void cGraphViz::UpdateCarUI(){

    tInt16 car_x = tInt16(roundf(carPose.x/map_res));
    tInt16 car_y = tInt16(roundf(carPose.y/map_res));
    CordsToMat(car_x,car_y);
    Pose lidar;
    lidar = SensorToGlobal(carPose, lidarLocal);

    tInt16 lidar_x = tInt16(roundf(lidar.x/map_res));
    tInt16 lidar_y = tInt16(roundf(lidar.y/map_res));

    CordsToMat(lidar_x,lidar_y);
  
    
    if ((lidar_x >= 0) && (lidar_x < OCCMAP_WIDTH) && (lidar_y >= 0) && (lidar_y < OCCMAP_HEIGHT)) {
        gridMap.at<std::uint8_t>(lidar_y, lidar_x) = static_cast<uint8_t>(80);

    }

    if ((car_x >= 0) && (car_x < OCCMAP_WIDTH) && (car_y >= 0) && (car_y < OCCMAP_HEIGHT)) {
        gridMap.at<std::uint8_t>(car_y, car_x) = static_cast<uint8_t>(0);

    }


}

bool cGraphViz::checkSurroundings(Mat map, tInt16 x,tInt16 y){

    tInt16 threshold = 3;

    for(tInt16 i = MAX(x-threshold, 0); i < MIN(x+threshold, OCCMAP_WIDTH); i++)
    {
        for(tInt16 j = MAX(y-threshold,0); j < MIN(y+threshold, OCCMAP_HEIGHT); j++)
        {
            if ( (map.at<tFloat32>(i,j)>0.5) && (i != x) && (j != y) )
            {
                return true;
            }
        }
    }
    return false;
}

tFloat32 cGraphViz::probMean(Mat map, tInt16 x, tInt16 y){
    tFloat32 sum = 0;
    tInt16 count = 0;

    for(tInt16 i = MAX(x-1, 0); i < MIN(x+1, OCCMAP_WIDTH); i++)
    {
        for(tInt16 j = MAX(y-1,0); j < MIN(y+1, OCCMAP_HEIGHT); j++)
        {
            if ( (i != x) && (j != y) )
            {
                sum += map.at<tFloat32>(i,j);
                count++;
            }
        }
    }
    return sum/count;

}

void cGraphViz::preProcess(){

    ready_map = gridMap.clone();

    auto min_x = (tInt16)MAX(carPose.x/2-50, 0);
    auto max_x = (tInt16)MIN(carPose.x/2+50, OCCMAP_WIDTH);
    auto min_y = (tInt16)MAX(carPose.y/2-50, 0);
    auto max_y = (tInt16)MIN(carPose.y/2+50, OCCMAP_HEIGHT);


    std::vector<std::vector<std::vector<tInt16>>> Clusters;
    std::vector<std::vector<tInt16>> cluster;

    std::vector<std::vector<tInt16>> occupied_points;
    std::vector<tInt16> Point;




    tInt16 iIndx, jIndx;
    for(tInt16 i = min_x; i < max_x; i++)
    {
        const double* Mi = ready_map.ptr<double>(i);
        for(tInt16 j = min_y; j < max_y; j++)
        {
            iIndx = i;
            jIndx = j;
            CordsToMat( iIndx, jIndx);
            if (ready_map.at<tFloat32>(iIndx,jIndx) > 0.5) {

                if (!checkSurroundings(ready_map, iIndx, jIndx)) {
                    ready_map.at<tFloat32>(iIndx, jIndx) = probMean(ready_map, i, j);
                } else {
                    occupied_points.push_back({i,j});
                }

            }
        }
    }





    return;
}

//tUltrasonicStruct US_data

void cGraphViz::MapLaserData(std::vector<tPolarCoordiante> &laser_scan){

    Pose lidarGlobal;
//    Pose US_SideLeftGlobal;
//    Pose US_SideRightGlobal;
//    Pose US_RearCenterGlobal;
//    Pose US_RearLeftGlobal;
//    Pose US_RearRightGlobal;


    lidarGlobal = SensorToGlobal(carPose, lidarLocal);
//    US_SideLeftGlobal = SensorToGlobal(carPose, US_SideLeftLocal);
//    US_SideRightGlobal = SensorToGlobal(carPose, US_SideRightLocal);
//    US_RearCenterGlobal = SensorToGlobal(carPose, US_RearCenterLocal);
//    US_RearLeftGlobal = SensorToGlobal(carPose, US_RearLeftGlobal);
//    US_RearRightGlobal = SensorToGlobal(carPose, US_RearLeftGlobal);
//    LOG_INFO("Mapping Lazzer Data");

    std::vector<tPolarCoordiante>::iterator it;
    globalSensor.x = lidarGlobal.x;
    globalSensor.y = lidarGlobal.y;

    for (tUInt16 i = 1; i < laser_scan.size();i+=20) {

        // determine MapPoint position in global
        tFloat32 range = laser_scan[i].f32Radius;
        tFloat32 angle = laser_scan[i].f32Angle * DEG2RAD; // MAKE SURE THIS ISN"T CAUSING AN ERROR
        MapPoint lidar_point;

        lidar_point.x = range * cos(angle);
        lidar_point.y = range * sin(angle) * -1; //need to flip over x_axis

        MapPoint lidar_point_global = PointToGlobal(lidarGlobal, lidar_point);


        tFloat32 laser_angle_global = SensorRayAngle(lidarGlobal, lidar_point_global);

// TODO loop through this way more intellgiently
        tp_y = tInt16(roundf((globalSensor.y / map_res) + (window_rows / 2)));
        bp_y = tInt16(roundf((globalSensor.y / map_res) - (window_rows / 2)));

        tp_x = tInt16(roundf((globalSensor.x / map_res) + (window_colums / 2)));
        bp_x = tInt16(roundf((globalSensor.x / map_res) - (window_colums / 2)));


        //REMEBER IF YOUR IN REAL WORLD COORDS OR GRID COORDS
        for (tInt16 grid_y = bp_y; grid_y < tp_y; grid_y += 1) {
            for (tInt16 grid_x = bp_x; grid_x < tp_x; grid_x += 1) {

                MapPoint center_point;
                center_point.x = (grid_x * map_res) + 0.5 * map_res;
                center_point.y = (grid_y * map_res) + 0.5 * map_res;

                tFloat32 map_distance = sqrt(pow((lidarGlobal.x - center_point.x), 2) +
                                             pow((lidarGlobal.y - center_point.y), 2));

                tFloat32 map_angle = GridAngle(center_point, lidarGlobal);

                if ((abs(laser_angle_global - map_angle)) < angular_offset_threshold) {// if within FOV of lazer

                    //LOG_INFO(cString::Format("Lazer Angle (%f)", laser_angle_global));
                    //LOG_INFO(cString::Format("Angle: (%f)", map_angle));

                    tInt16 i = grid_x;
                    tInt16 j = grid_y;

                    CordsToMat(i, j);
//
//                    LOG_INFO(cString::Format("Index in OccMap (%d, %d)", i, j ));
//                    LOG_INFO(cString::Format("OCCMAPDIM(%d, %d)", OCCMAP_HEIGHT, OCCMAP_HEIGHT));
                    if ((i >= 0) && (i <= OCCMAP_WIDTH) && (j >= 0) && (j <= OCCMAP_HEIGHT)) {
//                        LOG_INFO("Writing to OCC MAP");
                        occMap[i][j] =
                                occMap[i][j] + InverseSensorModel(range, laser_angle_global, map_distance, map_angle)
                                + log_prior; //set value

                    }

                }
            }
        }
    }

}

Pose cGraphViz::SensorToGlobal(Pose robot, Pose sensor){

    Pose sensorGlobal;

    tFloat32 c = cos(robot.theta);
    tFloat32 s = sin(robot.theta);

    sensorGlobal.x = c*sensor.x - s*sensor.y + robot.x;
    sensorGlobal.y = s*sensor.x + c*sensor.y + robot.y;;
    sensorGlobal.theta = sensor.theta + robot.theta;
    return sensorGlobal;

}

void cGraphViz::CordsToMat(tInt16& i,tInt16& j){

    i = i + OCCMAP_CENTER;
    j = j + OCCMAP_CENTER;

}

MapPoint  cGraphViz::PointToGlobal(Pose sensor, MapPoint data){

    MapPoint dataGlobal;

    float theta = sensor.theta; //because forward is along y-axis
    tFloat32 c = cos(theta);
    tFloat32 s = sin(theta);

    dataGlobal.x = c*data.x + -s*data.y + sensor.x;
    dataGlobal.y = s*data.x + c*data.y + sensor.y;;

    return dataGlobal;
}

MapPoint cGraphViz::PointToLocal(Pose sensor, MapPoint data){

    MapPoint dataLocal;

    float theta = sensor.theta; //because forward is along y-axis
    tFloat32 c = cos(theta);
    tFloat32 s = sin(theta);

    dataLocal.x = c*data.x + s*data.y - c*sensor.x - s*sensor.y;
    dataLocal.y = -s*data.x + c*data.y + s*sensor.x - c*sensor.y;

    return dataLocal;
}

tFloat32 cGraphViz::GridAngle(MapPoint grid_center, Pose sensor){

    return atan2( (grid_center.y - sensor.y), (grid_center.x - sensor.x) );
}

tFloat32 cGraphViz::SensorRayAngle(Pose sensor, MapPoint data){

    return atan2( (data.y-sensor.y), (data.x - sensor.x) );
}

void cGraphViz::FreeSpace(GridObjects grid_object){
    /*
    tInt16 tx = grid_object.tp_x;
    tInt16 ty = grid_object.tp_x;

    tInt16 bx = grid_object.bp_x;
    tInt16 by = grid_object.bp_y;

    for (tInt16 x_cell = tx; x_cell < bx; x_cell++) {
        for (tInt16 y_cell = ty; y_cell < by; y_cell++) {

            occMap[x_cell][y_cell] += 0;
        }
    }
     */

}

tFloat32 cGraphViz::InverseSensorModel(tFloat32 data_range, tFloat32 data_angle, tFloat32 grid_distance, tFloat32 grid_angle) {

    tFloat32 p_m = 0.5;

    if ((grid_distance < data_range - wall_thickness/2) && (grid_distance > 0)){

        p_m = 0.3;

        return std::log( p_m / (1-p_m) );

    }

    else if ((grid_distance > data_range - wall_thickness/2) && (grid_distance < data_range + wall_thickness/2)){

        //LOG_INFO("Updating Occupied Cell");

        p_m = 0.7;

        return std::log( p_m / (1-p_m) );

    }
    return std::log( p_m / (1-p_m) );

}

void cGraphViz::UpdateGrayGrid(Rect frame){

    for (tInt16 i = frame.x; i <= frame.x + frame.width; i++) {
        for (tInt16 j = frame.y; j <= frame.y + frame.height; j++) {
            gridMap.at<cv::Vec4b>(j, i) = GrayValue(occMap[i][j]); //GRID MAP INDEX IS REVERSED
//            cv::Vec4b grayValue;
//
//            if (i%2 == 0){
//            grayValue[0] = 255;
//            grayValue[1] = 0;
//            grayValue[2] = 255;
//            grayValue[3] = 100;
//
//        }
//        else {
//
//            grayValue[0] = 0;
//            grayValue[1] = 0;
//            grayValue[2] = 0;
//            grayValue[3] = 0;
//        }
//
//            gridMap.at<cv::Vec4b>(i, j) = grayValue; //GRID MAP INDEX IS REVERSED

        }
    }
}

void cGraphViz::PlaceObstacle(Rect frame){

    for (tInt16 i = frame.x; i <= frame.x + frame.width; i++) {
        for (tInt16 j = frame.y; j <= frame.y + frame.height; j++) {
            occMap[i][j] = 1000; //GRID MAP INDEX IS REVERSED
//            cv::Vec4b grayValue;
//
//            if (i%2 == 0){
//            grayValue[0] = 255;
//            grayValue[1] = 0;
//            grayValue[2] = 255;
//            grayValue[3] = 100;
//
//        }
//        else {
//
//            grayValue[0] = 0;
//            grayValue[1] = 0;
//            grayValue[2] = 0;
//            grayValue[3] = 0;
//        }
//
//            gridMap.at<cv::Vec4b>(i, j) = grayValue; //GRID MAP INDEX IS REVERSED

        }
    }
}

cv::Vec4b cGraphViz::GrayValue(double occValue) {
    cv::Vec4b grayValue;
    double p = 1 - 1 / (1 + exp(occValue));

    if (p <= 0.5){
        p = 0;
    }
    grayValue[0] = ((1 - p) * 255);
    grayValue[0] = ((p) * 255);
    grayValue[1] = 0;
    grayValue[2] = 0;
    grayValue[3] = ((p) * 255);


    return grayValue;
}

// UTILS
/*! calculates normalized angle */
tFloat32 cGraphViz::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
    return mod(alpha - center + M_PI, 2.0*M_PI) + center - M_PI;
}

/*! calculates modulus after division */
tFloat32 cGraphViz::mod(tFloat32 x, tFloat32 y)
{
    tFloat32 r;
    tFloat32 b_x;
    if (y == floor(y))
    {
        return x - floor(x / y) * y;
    }
    else
    {
        r = x / y;
        if (r < 0.0)
        {
            b_x = ceil(r - 0.5);
        }
        else
        {
            b_x = floor(r + 0.5);
        }
        if (fabs(r - b_x) <= 2.2204460492503131E-16 * fabs(r))
        {
            return 0.0;
        }
        else
        {
            return (r - floor(r)) * y;
        }
    }
}

tResult cGraphViz::SendLanePoints(){

    object_ptr<ISample> pSample;

    RETURN_IF_FAILED(alloc_sample(pSample))
    {
        {


            tUInt32 pathLength = pathPoints.size();

            auto oCodec = LanePointSampleFactory.MakeCodecFor(pSample);
            RETURN_IF_FAILED(oCodec.SetElementValue(LanePointData.nPoints, pathLength));

            lanePoint* lanePointsArray = reinterpret_cast<lanePoint*>(oCodec.GetElementAddress(LanePointData.pointArray));

            //init array with zeros
            memset(lanePointsArray, 0, 50 * sizeof(lanePoint)); //50 Size of Array

            // Points array has 5 slots
            for (tUInt32 i = 0; i < pathLength; i++)
            {
                lanePointsArray[i].x = (tInt32) (pathPoints[i].x * 1000); //Convert to mm
                lanePointsArray[i].y = (tInt32) (pathPoints[i].y * 1000);
            }

        }
    }

    LanePointOut << pSample << flush << trigger;

    RETURN_NOERROR;
}


void cGraphViz::DrawDebugPoint(DebugPoint p){

    if (p.type == HIT_POINT){

        DrawHitPoint(p.x,p.y,p.colour);

    }

    else if (p.type == FRONT_LINE){

        DrawFrontLine(p.x,p.y,p.theta);

    }

    else if (p.type == CAM_SIGN){
        DrawDetectedSign(p.x,p.y,p.theta,p.Id, true, false);

    }

    else if (p.type == LIDAR_SIGN){
        DrawDetectedSign(p.x,p.y,p.theta,p.Id, false, true);

    }

    else if (p.type == ROADSIGN){

        DrawRoadSign(p.x,p.y,p.theta,p.Id,false);
    }

    else if (p.type == CAR_DEBUG){

        DrawCar(p.x,p.y,p.theta,true);

    }

    else if (p.type == LASER){

        DrawLaser(p.x,p.y,p.Id);

    }
}


tResult cGraphViz::UpdateDebugPoints(){

    LOG_DUMP("Updating Debug Points");
    object_ptr<const ISample> pDebugPointSample;

    if (IS_OK(DebugPointIn.GetLastSample(pDebugPointSample))) {
        auto oDecoder = m_DebugPointStructSampleFactory.MakeDecoderFor(*pDebugPointSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        const DebugPoint *particleArray = reinterpret_cast<const DebugPoint *>(oDecoder.GetElementAddress(
                m_ddlDebugPointId.debugPoints));

        tUInt32 size;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDebugPointId.numPoints, &size));

        for (tUInt32 i = 0; i < size; i++){

            DebugPoint p;
            p.x = particleArray[i].x;
            p.y = particleArray[i].y;
            p.theta = particleArray[i].theta;
            p.type = particleArray[i].type;
            p.Id = particleArray[i].Id;
            p.colour = particleArray[i].colour;

            DrawDebugPoint(p);
        }
    }

    LOG_DUMP(cString::Format("debug list size [%d]", debugPointList.size()));

    for (tUInt32 i = 0; i < debugPointList.size(); i++){

        DrawDebugPoint(debugPointList.at(i));
    }

    debugPointList.clear();

    RETURN_NOERROR;
}
tResult cGraphViz::DrawHitPoint(float x, float y, int colour){

    MapPoint p;
    p.x = x;
    p.y = y;

    p = PointToGlobal(lidarGlobal, p);

    cv::Scalar drawColour = cv::Scalar(255, 255, 255, 255);

    if (colour == RED) drawColour = cv::Scalar(255, 0, 0, 255);

    if (colour == GREEN) drawColour = cv::Scalar(0, 255, 0, 255);

    if (colour == BLUE) drawColour = cv::Scalar(0, 0, 255, 255);

    if (colour == WHITE) drawColour = cv::Scalar(255, 255, 255, 255);

    if (colour == PURPLE) drawColour = cv::Scalar(255,0,255, 255);

    cv::circle(mImageOverlay,cv::Point((int)round(p.x* METERS_TO_PIXELS + WORLD_CENTER),(int)round(p.y* METERS_TO_PIXELS + WORLD_CENTER)), 4, drawColour, CV_FILLED, 8, 0);

    RETURN_NOERROR;

}

tResult cGraphViz::DrawLaser(float x, float y, int angle){

    cv::Point2f line_start( (int)round(lidarGlobal.x*METERS_TO_PIXELS + WORLD_CENTER), (int)round(lidarGlobal.y*METERS_TO_PIXELS + WORLD_CENTER));
    cv::Point2f line_end( (int)round(x*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y*METERS_TO_PIXELS + WORLD_CENTER));

    cv::line(mImageOverlay, line_start, line_end, cv::Scalar( 0, 200, 0, 255 ),2, 8,0);
//    cv::circle(mImageOverlay,cv::Point((int)round(x* METERS_TO_PIXELS + WORLD_CENTER),(int)round(y* METERS_TO_PIXELS + WORLD_CENTER)), 30, cv::Scalar(255, 0, 0, 255), CV_FILLED, 8, 0);

    std::ostringstream ss;
    ss << angle;
    putText(mImageOverlay, ss.str(), cv::Point((int) round((x + 0.1) * METERS_TO_PIXELS + WORLD_CENTER),
                                            (int) round((y + 0.1)* METERS_TO_PIXELS + WORLD_CENTER)),
            cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255, 255), 2, 8, true);

    RETURN_NOERROR;

}

tResult cGraphViz::DrawFrontLine(float x, float y, float theta){
// DEBUGGING

    double a = tan(theta);

    double b = y - (x * a);
    double debugLength = 10;

    double x1 = x + sqrt( pow(debugLength,2)/(1+pow(a,2)));
    double y1 = a * x1 + b;

    double x2 = x- sqrt( pow(debugLength,2)/(1+pow(a,2)));
    double y2 = a * x2 + b;

    cv::Point2f line_start( (int)round(x1*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y1*METERS_TO_PIXELS + WORLD_CENTER));
    cv::Point2f line_end( (int)round(x2*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y2*METERS_TO_PIXELS + WORLD_CENTER));
    cv::line(mImageOverlay, line_start, line_end, cv::Scalar( 0, 200, 0, 255 ),3, 8,0);
    cv::circle(mImageOverlay,cv::Point((int)round(x* METERS_TO_PIXELS + WORLD_CENTER),(int)round(y* METERS_TO_PIXELS + WORLD_CENTER)), 5, cv::Scalar(255, 0, 0, 255), CV_FILLED, 8, 0);

    LOG_DUMP("Drawing Front Line");
    RETURN_NOERROR;

}
tResult cGraphViz::DrawDetectedSign(float x, float y, float theta, int id, bool camera, bool lidar){


    if(camera){

        cv::circle(mImageOverlay,cv::Point((int)round(x* METERS_TO_PIXELS + WORLD_CENTER),(int)round(y* METERS_TO_PIXELS + WORLD_CENTER)), 30, cv::Scalar(255, 0, 0, 40), CV_FILLED, 8, 0);
        std::ostringstream ss;
        ss << id;
        putText(mImageOverlay, ss.str(), cv::Point((int) round((x + 0.3) * METERS_TO_PIXELS + WORLD_CENTER),
                                                   (int) round((y + 0.3)* METERS_TO_PIXELS + WORLD_CENTER)),
                cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255, 255), 2, 8, true);

    }

    if(lidar){

        MapPoint p;
        p.x = x;
        p.y = y;

        p = PointToGlobal(lidarGlobal,p);
        cv::circle(mImageOverlay,cv::Point((int)round(p.x* METERS_TO_PIXELS + WORLD_CENTER),(int)round(p.y* METERS_TO_PIXELS + WORLD_CENTER)), 30, cv::Scalar(255, 0, 0, 40), CV_FILLED, 8, 0);
        std::ostringstream ss;
        ss << id;
        putText(mImageOverlay, ss.str(), cv::Point((int) round((p.x + 0.3) * METERS_TO_PIXELS + WORLD_CENTER),
                                                   (int) round((p.y + 0.3)* METERS_TO_PIXELS + WORLD_CENTER)),
                cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255, 255), 2, 8, true);

        DrawRoadSign(p.x, p.y, theta, 0, true);
        RETURN_NOERROR;

    }

    RETURN_NOERROR;

}

tResult cGraphViz::DrawRoadSign(float x, float y, float theta, int id, bool detected){

    float a;
    float b;
    float signWidth = 0.1;
    if (detected) signWidth = 1; //meters
    double x1;
    double y1;

    double x2;
    double y2;

    // Careful, not that theta is in DEGREES
    if (theta == 0 || theta == 180 || theta == -180){

        x1 = x;
        y1 = y + signWidth;
        x2 = x;
        y2 = y - signWidth;
    }

    else {

        if (theta == -90 || theta == 90) {

            a = 0;
        }

        else{

            a = tan(theta * DEG2RAD);
        }

        b = y - (x * a);

        x1 = x + sqrt( pow(signWidth,2)/(1+pow(a,2)));
        y1 = a * x1 + b;

        x2 = x - sqrt( pow(signWidth,2)/(1+pow(a,2)));
        y2 = a * x2 + b;
    }


//    if(detected){
//
//        cv::circle(mImageOverlay,cv::Point((int)round(x* METERS_TO_PIXELS + WORLD_CENTER),(int)round(y* METERS_TO_PIXELS + WORLD_CENTER)), 30, cv::Scalar(255, 0, 0, 40), CV_FILLED, 8, 0);
//        std::ostringstream ss;
//        ss << id;
//        putText(mImageOverlay, ss.str(), cv::Point((int) round((x + 0.3) * METERS_TO_PIXELS + WORLD_CENTER),
//                                                (int) round((y + 0.3)* METERS_TO_PIXELS + WORLD_CENTER)),
//                cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255, 255), 2, 8, true);
//        RETURN_NOERROR;
//
//    }

    // Points for Sign
    cv::Point2f line_start( (int)round(x1*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y1*METERS_TO_PIXELS + WORLD_CENTER));
    cv::Point2f line_end( (int)round(x2*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y2*METERS_TO_PIXELS + WORLD_CENTER));

    // Draw Line front Center of sign showing direction
    double frontX = x - 0.1 * cos(theta*DEG2RAD);
    double frontY = y - 0.1 *sin(theta*DEG2RAD);

    cv::Point2f start( (int)round(x*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y*METERS_TO_PIXELS + WORLD_CENTER));
    cv::Point2f end( (int)round(frontX*METERS_TO_PIXELS + WORLD_CENTER), (int)round(frontY*METERS_TO_PIXELS + WORLD_CENTER));

    if (detected) {
        cv::line(mImageOverlay, line_start, line_end, cv::Scalar( 0, 0, 255, 255),4, 8,0);
        cv::line(mImageOverlay, start, end, cv::Scalar( 255, 255, 255, 255),4, 8,0);
    }

    else {
        cv::line(mImageGrid, line_start, line_end, cv::Scalar( 0, 255, 0, 255 ),4, 8,0);
        cv::line(mImageGrid, start, end, cv::Scalar( 255, 255, 255, 255 ),4, 8,0);
    }
    // Label Sign with ID //TODO add ID to debug nodes

    if(!detected) {
        std::ostringstream ss;
        ss << id;
        putText(mImageGrid, ss.str(), cv::Point((int) round((x + 0.1) * METERS_TO_PIXELS + WORLD_CENTER),
                                                (int) round((y + 0.1) * METERS_TO_PIXELS + WORLD_CENTER)),
                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255, 255), 2, 8, true);
    }

    RETURN_NOERROR;
}

tResult cGraphViz::DrawCar(float x, float y, float theta, bool debug){

    float c = cos(theta);
    float s = sin(theta);
    //For orientation, draw MapPoint 30cm away
    float orientationLen = 0.3;
//
    float destinationX = x + c * orientationLen;
    float destinationY = y + s * orientationLen;

    if (debug){
        cv::arrowedLine	(mImageOverlay,
                            cv::Point((int)round(x* METERS_TO_PIXELS + WORLD_CENTER),(int)round(y* METERS_TO_PIXELS + WORLD_CENTER)),
                            cv::Point((int)round(destinationX* METERS_TO_PIXELS + WORLD_CENTER),(int)round((destinationY* METERS_TO_PIXELS + WORLD_CENTER))),
                            cv::Scalar( 255, 0, 0, 255), 4);
        cv::circle(mImageOverlay,cv::Point((int)round(x* METERS_TO_PIXELS + WORLD_CENTER),(int)round(y* METERS_TO_PIXELS + WORLD_CENTER)), 10, cv::Scalar(255, 0, 0, 255), CV_FILLED, 8, 0);

    }

    else {
        cv::arrowedLine	(mImageOverlay,
                            cv::Point(x* METERS_TO_PIXELS + WORLD_CENTER,(y* METERS_TO_PIXELS + WORLD_CENTER)),
                            cv::Point(destinationX* METERS_TO_PIXELS + WORLD_CENTER,(destinationY* METERS_TO_PIXELS + WORLD_CENTER)),
                            cv::Scalar( 255, 255, 255, 255), 4);
        cv::circle(mImageOverlay,cv::Point(x* METERS_TO_PIXELS + WORLD_CENTER,(y* METERS_TO_PIXELS + WORLD_CENTER)), 10, cv::Scalar(255, 0, 255, 255), CV_FILLED, 8, 0);
    }
    RETURN_NOERROR;
}

void cGraphViz::StreamWorld(){

    mImageOverlay(roiPixel) = cv::Scalar( 0, 0, 0, 0);

    UpdateDebugPoints();
    PathViz(path, lattice, mImageOverlay);
    DrawCar(carPose.x, carPose.y, carPose.theta);

//    if (streamCount % 100 == 0){
//        UpdateOccMap(roiCell, windowSize);
//    }
    //PlotErrorEllipse();
    ReCenterImage();
//    CropOccMap(roiCell, windowSize);
    //updateImage();
    ComposeImage();
    //cv::cvtColor(mImageOverlay, mImageSendNonFlipped, CV_RGBA2RGB );

    //cv::cvtColor( mImage, mImageSend, CV_RGBA2RGB );
    cv::flip( mImageSend, mImageFlipped, 0 );

//    cv::flip( mImageSend, mImageFlipped, 0 );

    Mat outputImage = mImageFlipped;// + mImageSendNonFlipped;
//    Mat outputImage = mImageGridMasked;// + mImageSendNonFlipped;
    //cv::cvtColor( mImageDebug, mImageSend, CV_RGBA2RGB );
    //outputImage = mImageSend;
    //Write processed Image to Output Pin
    if (!outputImage.empty())
    {
        //update output format if matrix size does not fit to
//        if (outputImage.total() * outputImage.elemSize() != m_sImageFormat.m_szMaxByteSize)
//        {
//            setTypeFromMat(m_oWriter, outputImage);
//        }
        // write to pin
        adtf::streaming::tStreamImageFormat outputFormat;

        m_sImageFormat.m_ui32Height = outputImage.rows;
        m_sImageFormat.m_ui32Width = outputImage.cols;
        m_sImageFormat.m_szMaxByteSize = outputImage.cols * outputImage.rows * outputImage.channels();
        m_sImageFormat.m_ui8DataEndianess = PLATFORM_BYTEORDER;


        adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
        set_stream_type_image_format(*pTypeOutput, m_sImageFormat);

        m_oWriter << pTypeOutput;

        writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
    }
}

tResult cGraphViz::Process(tTimeStamp tmTimeOfTrigger)
{
    UpdateCarPos();
    ReadSensorStream();
    //DEBUG
//    MapPoint p1 = getParkingSpot(1);
//    MapPoint p2 = getParkingSpot(2);
//    MapPoint p3 = getParkingSpot(3);
////    MapPoint p4 = getParkingSpot(4);
//    LOG_INFO(cString::Format("P1: %f, %f)",p1.x, p1.y));
//    LOG_INFO(cString::Format("P2: %f, %f)",p2.x, p2.y));
//    LOG_INFO(cString::Format("P3: %f, %f)",p3.x, p3.y));


    if (carInitalized && !graphSearched) { //Flags to wait for positioning

        if (!graphInitalized){
            GraphInit();
            cv::Point2f start_point((int) round(mapGraph[startNode].p.x * METERS_TO_PIXELS + WORLD_CENTER),
                                    (int) round(mapGraph[startNode].p.y * METERS_TO_PIXELS + WORLD_CENTER));

            cv::circle(mImageGrid, start_point, 10, cv::Scalar(255, 255, 255, 255), CV_FILLED, 8, 0);
            putText( mImageGrid, "START", start_point, cv::FONT_HERSHEY_PLAIN, 5, cv::Scalar(255, 255, 0, 255), 2, 8, true);

            cv::Point2f end_point((int) round(mapGraph[goalNode].p.x * METERS_TO_PIXELS + WORLD_CENTER),
                                  (int) round(mapGraph[goalNode].p.y * METERS_TO_PIXELS + WORLD_CENTER));

            cv::circle(mImageGrid, end_point, 10, cv::Scalar(255, 255, 255, 255), CV_FILLED, 8, 0);
            putText( mImageGrid, "END", end_point, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 0, 255), 2, 8, true);


            graphInitalized = true;
        }

        GraphSearch(tmTimeOfTrigger);
    }

    if (graphSearched){
        PathPlanner();
        if (streamCount % 100 == 0) {
        LOG_INFO("DEBUGMOVE");
//            DebugMove();

        }

        SendLanePoints();
    }

    if (streamCount % 10 == 0) {

        StreamWorld();
    }

//    if (streamCount == 1000) {
//
//        mImagePath(roi) = cv::Scalar( 0, 0, 0, 0);
//    }

    streamCount++;

    RETURN_NOERROR;
}

tResult cGraphViz::SendJunctionFlag(const tUInt32 &situation) {

    tUInt32 ts = m_pClock->GetStreamTime();
    RETURN_IF_FAILED(transmitSignalValue(m_oSituationWriter, ts, m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, ts, m_ddlSignalValueId.value, situation));

    RETURN_NOERROR;

}

tResult cGraphViz::ReadManeuverId() {

    object_ptr<const ISample> pWriteManArraySample;

    if (IS_OK(m_oManArrayReader.GetLastSample(pWriteManArraySample))) {
        auto oDecoder = m_manArrayDataSampleFactory.MakeDecoderFor(pWriteManArraySample);

        RETURN_IF_FAILED(oDecoder.IsValid());
        auto pManArray = reinterpret_cast<const tUInt8*>(oDecoder.GetElementAddress(m_ddlManArray.manArray));

        for (tSize i = 0; i < 100; ++i) {
            if (pManArray[i] != 0) {
                m_maneuverID.push_back(pManArray[i]);
            }
            else {
                break;
            }
        }
    }

    RETURN_NOERROR;
}
