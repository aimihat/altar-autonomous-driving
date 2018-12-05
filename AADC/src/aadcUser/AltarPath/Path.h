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
#define CID_ALTAR_PATH_FILTER "AltarPath.filter.user.aadc.cid"
#define PI 3.1415

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;



class cPath : public cTriggerFunction {

private:
    //----------------------------
    //STUFF FOR POSITION INFORMATION
    struct {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionIndex;

    /*! The position sample factory */
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;

    cPinReader PoseDataIn;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    cv::Mat mImageGrid, outputImage;
    float METERS_TO_PIXELS, OFFSET_DRAW, GRIDSIZE;

private:
    //DATA TYPES
    struct junction {
        int id;
        std::vector<int> nodeList;
    };

//DATA TYPES
    struct robotPose {

        tFloat32 x;
        tFloat32 y;
        tFloat32 theta;
    };

    struct lanePoint {
        int x;
        int y;
        int cluster;
        int pctDiff;

        lanePoint()
        {
            x = 0;
            y = 0;
        }

        lanePoint(int xcoord, int ycoord)
        {
            x = xcoord;
            y = ycoord;
            cluster = 0;
            pctDiff = 0;
        }
    };

    struct point {
        float x;
        float y;
    };

    struct point_y_comparator {
        bool operator()(point pt1, point pt2) { return (pt1.y < pt2.y); }
    } mycomparatorY;

    struct segment {
        int id;
        vector<point> points;
        vector<point> left;
        vector<point> middle;
        vector<point> right;
    };

    robotPose carPose;
    adtf::streaming::tStreamImageFormat m_sImageFormat;
    cPinWriter m_oWriter;
    std::unordered_map<int, int> nodeSuccessor, nodePredecessor;
    std::vector<junction> junctions;

    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_lanePointSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_LanePointDataSampleFactory;

    struct ddllanePointId
    {
        tSize x;
        tSize y;
    } m_ddllanePointId;

    struct ddlLanePointDataId
    {
        tSize nPoints;
        tSize pointArray;
    } m_ddlLanePointDataId;

    cPinWriter m_oWriterLane;



public:
    vector<segment> segments;
    cPath();
    tResult transformPoint(point &transformedP, float cosine, float sine);
    tResult Transmit(vector<point>& relativePath);
    virtual ~cPath() = default;
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
};


//*************************************************************************************************
