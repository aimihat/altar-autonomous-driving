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
#define CID_CLANEDETECTION_DATA_TRIGGERED_FILTER "lane_detection.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

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


typedef struct
{
    tFloat32 f32Distance;
    tFloat32 f32FrontDistance;
    tFloat32 f32RoadId;
} tLineCoordiante;

struct point_x_comparator {
    bool operator() (lanePoint pt1, lanePoint pt2) { return (pt1.x < pt2.x);}
} mycomparatorX;
struct point_y_comparator {
    bool operator() (lanePoint pt1, lanePoint pt2) { return (pt1.y > pt2.y);}
} mycomparatorY;
struct int_comparator {
    bool operator() (int a, int b) { return (a>b);}
} mycomparatorInt;


typedef struct
{
    tFloat32 f32x;
    tFloat32 f32y;
    tFloat32 f32radius;
    tFloat32 f32speed;
    tFloat32 f32heading;
} tPositionB;

struct GoalPoint {
    tFloat32 x;
    tFloat32 y;
} gP, pP;


/*! the main class for the lane detection filter. */
class cLaneDetection : public cTriggerFunction
{
private:
    //Properties
    /*! Offset of the ROI in the Stream*/
    adtf::base::property_variable<int> m_ROIOffsetX = 300;
    /*! Offset of the ROI in the Stream*/
    adtf::base::property_variable<int> m_ROIOffsetY = 0;
    /*! Width of the ROI*/
    adtf::base::property_variable<int> m_ROIWidth = 500;
    /*! Height of the ROI*/
    adtf::base::property_variable<int> m_ROIHeight = 920;
    /*! number of detection lines searched in ROI */
    adtf::base::property_variable<tInt> m_detectionLines = 25;
    /*! Minimum Line Width in Pixel */
    adtf::base::property_variable<int> m_minLineWidth = 4;
    /*! Maximum Line Width in Pixel */
    adtf::base::property_variable<int> m_maxLineWidth = 16;
    /*! Mimimum line contrast in gray Values */
    adtf::base::property_variable<int> m_minLineContrast = 80;
    /*! Threshold for image binarization */
    adtf::base::property_variable<int> m_sizeSegmentImageBinarization = 137;
    adtf::base::property_variable<int> m_thresholdImageBinarization = -55;
    adtf::base::property_variable<int> m_clusterThreshold = 24;
    adtf::base::property_variable<int> m_minimumLaneSeparation = 135;
    adtf::base::property_variable<int> m_maximumLaneSeparation = 200;
    adtf::base::property_variable<int> m_frontCarPositionX = 634;
    adtf::base::property_variable<int> m_frontCarPositionY = m_ROIHeight+9;
    adtf::base::property_variable<float> m_pixelToMillimiterX = 2.97;   //using lane width
    adtf::base::property_variable<float> m_pixelToMillimiterY = 1.81;   //using lane width
    adtf::base::property_variable<int> m_clusterHeightLimit = 560;
    adtf::base::property_variable<int> m_steeringWeight = 25;
    //Pins

    cPinReader m_oReaderSpd;
    cPinReader m_oReaderSteer;

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


    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_lanePointSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_LanePointDataSampleFactory;


    /*! Reader for the video. */
    cPinReader m_oReaderVideo;
    /*! Writer for the video. */
    cPinWriter m_oWriterVideo;

    int previousSpeed, previousSteer, lastUpdateTime;

    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_LPStructSampleFactory;

    //Stream Formats
    /*! The input format */
    adtf::streaming::tStreamImageFormat m_InPinVideoFormat;
    /*! The output format */
    adtf::streaming::tStreamImageFormat m_OutPinVideoFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! lane detection roi bounding rectangle */
    cv::Rect m_LaneRoi = cv::Rect();

    struct tPositionId
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionId;
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;
    /*! Writer to an OutPin. */
    cPinWriter m_oWriterSteer;
    cPinWriter m_oWriterSpd;
    /*! The template data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSpdSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSteerSampleFactory;

    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueSpdId, m_ddlSignalValueSteerId;



    struct ddlLineDataId
    {
        tSize linesFound;
        tSize lineArray;

    } m_ddlLineDataId;

    struct tLanePoint {
        tInt32 x;
        tInt32 y;
    };

    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_LineStructSampleFactory;

    /*! Writer to an OutPin. */
    cPinWriter LineDataOut;
    cv::cuda::GpuMat m_GpuInputImage, m_GpuGreyImage;



public:
    /*! Default constructor. */
    cLaneDetection();

    tResult Transmit(const lanePoint *points, const int &nPoints);
    /*! Destructor. */
    virtual ~cLaneDetection() = default;

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

    float pixelToRelativeCoordsX (int pixel);
    float pixelToRelativeCoordsY (int pixel);


private:

    /*!
     * Searches for the first line points.
     *
     * \param           detectionLines      The detection lines.
     * \param           image               The image.
     * \param [in,out]  detectedLinePoints  The detected line points.
     *
     * \return  Standard Result Code.
     */
    tResult findLinePoints(const vector<tInt>& detectionLines, const cv::Mat& image, vector <Point>& detectedLinePoints);
    tResult PercentageDifference(std::vector<lanePoint>& points);

    tResult TransmitSpeed(tSignalValue speedSignal);
    tResult TransmitDir(tSignalValue dirSignal);


    tSignalValue steerSignal_old;
    tSignalValue speedSignal_old;
    /*!
     * Gets detection lines.
     *
     * \param [in,out]  detectionLines  The detection lines.
     *
     * \return  Standard Result Code.
     */
    tResult getDetectionLines(vector<tInt>& detectionLines);

    /*!
    * Change type.
    *
    * \param [in,out]  inputPin    The input pin.
    * \param           oType       The type.
    *
    * \return  Standard Result Code.
    */
    tResult ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
                       const adtf::streaming::ant::IStreamType& oType);

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
