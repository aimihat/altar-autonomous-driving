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
#define CID_cCrosswalk_DATA_TRIGGERED_FILTER "crosswalk.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

#define LOW false
#define HIGH true



/*! the main class for the lane detection filter. */
class cCrosswalk : public cTriggerFunction
{
private:
    //Properties
    /*! Offset of the ROI in the Stream*/
    adtf::base::property_variable<int> m_ROIOffsetX = 375;
    /*! Offset of the ROI in the Stream*/
    adtf::base::property_variable<int> m_ROIOffsetY = 600;
    /*! Width of the ROI*/
    adtf::base::property_variable<int> m_ROIWidth = 365;
    /*! Height of the ROI*/
    adtf::base::property_variable<int> m_ROIHeight = 50;
    /*! number of detection lines searched in ROI */
    adtf::base::property_variable<int> m_detectionLines = 25;
    /*! Minimum Line Width in Pixel */
    adtf::base::property_variable<int> m_minLineWidth = 2;
    /*! Maximum Line Width in Pixel */
    adtf::base::property_variable<int> m_maxLineWidth = 16;
    /*! Mimimum line contrast in gray Values */
    adtf::base::property_variable<int> m_minLineContrast = 80;
    /*! Threshold for image binarization */
    adtf::base::property_variable<int> m_sizeSegmentImageBinarization = 145;
    adtf::base::property_variable<int> m_thresholdImageBinarization = -55;
    adtf::base::property_variable<int> m_clusterThreshold = 24;
    adtf::base::property_variable<int> m_minimumLaneSeparation = 135;
    adtf::base::property_variable<int> m_maximumLaneSeparation = 200;
    adtf::base::property_variable<int> m_frontCarPositionX = 634;
    adtf::base::property_variable<int> m_frontCarPositionY = -9;
    adtf::base::property_variable<float> m_pixelToMillimiterX = 2.97;   //using lane width
    adtf::base::property_variable<float> m_pixelToMillimiterY = 1.81;   //using lane width
    adtf::base::property_variable<int> m_clusterHeightLimit = 560;
    adtf::base::property_variable<int> m_steeringWeight = 25;
    //Pins

    /*! Media Descriptions. */
    struct tBoolSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlBoolSignalValueId;


    /*! The template data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;



    /*! Reader for the video. */
    cPinReader m_oReaderVideo;
    /*! Writer for the video. */
    cPinWriter m_oWriterVideo;
    /*! Writer to an OutPin. */
    cPinWriter m_oWriter;


    //Stream Formats
    /*! The input format */
    adtf::streaming::tStreamImageFormat m_InPinVideoFormat;
    /*! The output format */
    adtf::streaming::tStreamImageFormat m_OutPinVideoFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! lane detection roi bounding rectangle */
    cv::Rect m_LaneRoi = cv::Rect();
    tResult ChangeType(adtf::streaming::cDynamicSampleReader& inputPin, const adtf::streaming::ant::IStreamType& oType);





public:
    /*! Default constructor. */
    cCrosswalk();

    /*! Destructor. */
    virtual ~cCrosswalk() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;

    /*!
     * Overwrites the Process You need to implement the Reading and Writing of Samples within this
     * function MIND: Do Reading until the Readers queues are empty or use the
     * IPinReader::GetLastSample()
     * This FUnction will be called if the Run() of the TriggerFunction was called.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     *
     * \return  Standard Result Code.
     */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;



    struct ddlLineDataId
    {
        tSize linesFound;
        tSize lineArray;

    } m_ddlLineDataId;

    typedef struct
    {
        tFloat64 f64Distance;
        tFloat32 f32RoadId;
    } tLineCoordiante;


    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_LineStructSampleFactory;

    /*! Writer to an OutPin. */
    cPinWriter LineDataOut;

    tResult TransmitCrosswalk(tBoolSignalValue crosswalkSignal);




    /*!
    * Checks if the ROI is within the Image boundaries
    *
    *
    *
    * \return  Standard Result Code.
    */
    tResult checkRoi(void);

};

//*************************************************************************************************
