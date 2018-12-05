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
#include "../AltarUtils/MapStructs.h"

//*************************************************************************************************
#define CID_MAPPING_FILTER "AltarWorldViz.filter.user.aadc.cid"

//Occupancy Map parameter
#define PI 3.14159265
#define DEGTORAD PI/180

#define METERS_TO_PIXELS float(50.0)

#define WORLD_WIDTH (int)round(50 * METERS_TO_PIXELS)
#define WORLD_HEIGHT (int)round(50 * METERS_TO_PIXELS)


#define WINDOW_WIDTH (int)round(10 * METERS_TO_PIXELS)
#define WINDOW_HEIGHT (int)round(10 * METERS_TO_PIXELS)


#define WORLD_CENTER (int)round(WORLD_WIDTH*0.5)
#define WINDOW_CENTER (int)round(WINDOW_WIDTH*0.5)




using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

class cWorldViz : public cTriggerFunction
{

private:

    std::vector<vizParticle> particles_debug;

    vizParticle* particleOuterA;
    vizParticle* particleMiddle;
    vizParticle* particleOuterB;

    // MY INFO




    struct ddlDebugPointId
    {
        tSize numPoints;
        tSize debugPoints;

    } m_ddlDebugPointId;

    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_DebugPointStructSampleFactory;

    cPinReader DebugPointIn;



    struct ddlParticleDataId
    {
        tSize particleArray;

    } m_ddlParticleDataId;

    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_ParticleStructSampleFactory;


    cPinReader ParticleDataIn;


    struct ddlPoseCovDataId
    {
        tSize covArray;

    } m_ddlPoseCovDataId;

    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_PoseCovStructSampleFactory;

    /*! Writer to an OutPin. */
    cPinReader PoseCovDataIn;
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

private:

    double line_debug_length;

    Mat mImageGrid;
    Mat mImagePath;
    Mat mImageOverlay;
    Mat mImage;
    Mat mImageGridMasked;
    Mat mImagePathMasked;

    Mat covmat;
    Mat mImageDebug;
    Mat mImageDebugTmp;

    Pose carPose;

    Mat mImageSend;
    Mat mImageFlipped;
    cv::Point2f mPreviousPos;

    tTimeStamp mLastTime;
    tTimeStamp mLastSendTime;
    tTimeStamp mTimeBetweenUpdates;

    std::vector<DebugPoint> debugPointList;

    Rect roi;
    void PlotParticles();
    void StreamWorld();
    cv::RotatedRect getErrorEllipse(tFloat32 chisquare_val, cv::Point2f mean, cv::Mat covmat);
    void PlotErrorEllipse();
    void ComposeImage();
    void updateImage();
    tResult UpdateCarPos();
    void ReCenterImage();

        public:

    cWorldViz();
    virtual ~cWorldViz() = default;

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
