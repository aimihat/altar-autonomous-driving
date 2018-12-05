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

//*************************************************************************************************
#define CID_ALTAR_OBJECT_DETECTION_DATA_TRIGGERED_FILTER "altar_object_detection.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace cv;
#include "arapaho.hpp"

bool fileExists(const char *file) 
    {
        struct stat st;
        if(!file) return false;
        int result = stat(file, &st);
        return (0 == result);
    }

/*! the main class of the open cv template. */
class cObjectDetection : public cTriggerFunction
{
private:

    //Pins
    /*! Reader of an InPin. */
    cPinReader m_oReader,m_oRearReader;
    /*! Writer to an OutPin. */
    cPinWriter m_oWriter, m_oBarbieWriter, m_oRearWriter;

    typedef struct
    {
        tFloat32 detectedClass;
        tFloat32 pctOffsetCenter;
    } tDetection;

    typedef struct
    {
        box boundingBox;
        int camera; //0 for front cam, 1 for rear cam
    } carDetection;

    struct ddlObjectDataId
    {
        tSize nObjects;
        tSize objectsArray;
    } m_ddlObjectDataId;
    adtf::mediadescription::cSampleCodecFactory m_ObjectDataSampleFactory;

    cPinWriter m_oWriterObject;

    property_variable<float> thresh = 0.20;
    property_variable<float> adult_ratio_threshold = 0.25;
    property_variable<int> roi_top_left_x = 410;
    property_variable<int> roi_top_left_y = 385;
    property_variable<int> roi_width = 500;
    property_variable<int> roi_height = 300;


    std::vector<tDetection> objectsDetected;
    std::vector<carDetection> carsDetected;

    //Stream Formats
        /*! The input format */
    adtf::streaming::tStreamImageFormat m_sImageFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;
    ArapahoV2Params araho_params, araho_params_coco;
    ArapahoV2* p;
    ArapahoV2* p_coco;
    int a;
public:
    tResult ExtractBoxes(box *bxs, std::string *lbls, int obj_count, cv::Mat& inputImage, int camera);
    /*! Default constructor. */
    cObjectDetection();


    /*! Destructor. */
    virtual ~cObjectDetection() = default;

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
