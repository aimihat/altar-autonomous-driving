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
#include "LaneDetection.h"
#include "ADTF3_OpenCV_helper.h"
#include <algorithm>
#include "spline.h"

/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CLANEDETECTION_DATA_TRIGGERED_FILTER,
                                    "Altar Lane Detection",
                                    cLaneDetection,
                                    adtf::filter::pin_trigger({"in"}));


cLaneDetection::cLaneDetection()
{
    //Register Properties
    RegisterPropertyVariable("ROIOffsetX [Pixel]",      m_ROIOffsetX);
    RegisterPropertyVariable("ROIOffsetY [Pixel]",      m_ROIOffsetY);
    RegisterPropertyVariable("ROIWidth [Pixel]",        m_ROIWidth);
    RegisterPropertyVariable("ROIHeight [Pixel]",       m_ROIHeight);
    RegisterPropertyVariable("detectionLines",  m_detectionLines);
    RegisterPropertyVariable("minLineWidth [Pixel]", m_minLineWidth);
    RegisterPropertyVariable("maxLineWidth [Pixel]", m_maxLineWidth);
    RegisterPropertyVariable("minLineContrast", m_minLineContrast);
    RegisterPropertyVariable("m_sizeSegmentImageBinarization", m_sizeSegmentImageBinarization);
    RegisterPropertyVariable("thresholdImageBinarization", m_thresholdImageBinarization);
    RegisterPropertyVariable("clusterThreshold", m_clusterThreshold);
    RegisterPropertyVariable("maximumLaneSeparation", m_maximumLaneSeparation);
    RegisterPropertyVariable("minimumLaneSeparation", m_minimumLaneSeparation);
    RegisterPropertyVariable("frontCarPositionX", m_frontCarPositionX);
    RegisterPropertyVariable("frontCarPositionY", m_frontCarPositionX);
    RegisterPropertyVariable("clusterHeightLimit", m_clusterHeightLimit);
    RegisterPropertyVariable("pixelToMillimiterX", m_pixelToMillimiterX);
    RegisterPropertyVariable("pixelToMillimiterY", m_pixelToMillimiterY);
    RegisterPropertyVariable("steeringWeight", m_steeringWeight);
    object_ptr<IStreamType> pTypeLPData;

    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeSignalValueSpd;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValueSpd, m_SignalValueSpdSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_SignalValueSpdSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueSpdId.timeStamp);
        adtf_ddl::access_element::find_index(m_SignalValueSpdSampleFactory, cString("f32Value"), m_ddlSignalValueSpdId.value);
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValueSpd found!");
    }

    object_ptr<IStreamType> pTypeSignalValueSteer;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValueSteer, m_SignalValueSteerSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_SignalValueSteerSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueSteerId.timeStamp);
        adtf_ddl::access_element::find_index(m_SignalValueSteerSampleFactory, cString("f32Value"), m_ddlSignalValueSteerId.value);
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValueSteer found!");
    }
    //create and set inital input format type
    m_InPinVideoFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pTypeInput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeInput, m_InPinVideoFormat);
    //Register input pin
    Register(m_oReaderVideo, "in", pTypeInput);

    //Register output pins
    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_OutPinVideoFormat);
    Register(m_oWriterVideo, "outVisual", pTypeOutput);


    object_ptr<IStreamType> pTypelanePoint;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("lanePoint", pTypelanePoint, m_lanePointSampleFactory))
    {
        //// find the indexes of the element for faster access in the process method.
        LOG_INFO("Found mediadescription for lanePoint!");

        adtf_ddl::access_element::find_index(m_lanePointSampleFactory, "x", m_ddllanePointId.x);
        adtf_ddl::access_element::find_index(m_lanePointSampleFactory, "y", m_ddllanePointId.y);
    }
    else
    {
        LOG_INFO("No mediadescription for lanePoint found!");
    }

    object_ptr<IStreamType> pTypeLanePointData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("LanePointData", pTypeLanePointData, m_LanePointDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_LanePointDataSampleFactory, "nPoints", m_ddlLanePointDataId.nPoints);
        adtf_ddl::access_element::find_array_index(m_LanePointDataSampleFactory, "pointArray", m_ddlLanePointDataId.pointArray);
    }
    else
    {
        LOG_INFO("No mediadescription for LanePointData found!");
    }

    Register(m_oWriterLane, "lane_points", pTypeLanePointData);



    //register callback for type changes
    m_oReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReaderVideo, *pType.Get());
    });

    object_ptr<IStreamType> pLineData;

    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLine", pLineData, m_LineStructSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_LineStructSampleFactory, "linesFound", m_ddlLineDataId.linesFound);

        adtf_ddl::access_element::find_array_index(m_LineStructSampleFactory, "lineArray", m_ddlLineDataId.lineArray);

    }
    else
    {
        LOG_WARNING("No mediadescription for tLine found!");
    }

    Register(LineDataOut, "Line", pLineData);

}


tResult cLaneDetection::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult cLaneDetection::ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
                                   const adtf::streaming::ant::IStreamType& oType)
{
    if (oType == adtf::streaming::stream_meta_type_image())
    {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pTypeInput;
        // get pType from input reader
        inputPin >> pTypeInput;
        adtf::streaming::get_stream_type_image_format(m_InPinVideoFormat, *pTypeInput);

        //set also output format
        adtf::streaming::get_stream_type_image_format(m_OutPinVideoFormat, *pTypeInput);
        //we always have a grayscale output image
        m_OutPinVideoFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
        // and set pType also to samplewriter
        m_oWriterVideo << pTypeInput;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}

tResult cLaneDetection::PercentageDifference(std::vector<lanePoint>& points)
{
    int diff;
    if (points.size() > 0)
        points.at(0).pctDiff = points.at(0).x;
    for (int i = 1; i < points.size(); i++) {
        diff = points.at(i).x-points.at(i-1).x;
        points.at(i).pctDiff = diff;
    }
    //Divide differences by the smallest difference
    RETURN_NOERROR;
}

float cLaneDetection::pixelToRelativeCoordsX(int pixel)
{
    return (pixel-m_frontCarPositionX)*m_pixelToMillimiterX;
}

float cLaneDetection::pixelToRelativeCoordsY(int pixel)
{
    return (m_frontCarPositionY-pixel)*m_pixelToMillimiterY;
}

tResult cLaneDetection::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oReaderVideo.GetNextSample(pReadSample)))
    {
        //GET RIGHT TURN BOOL
        bool RIGHT_TURN = 1;

        if (RIGHT_TURN) m_maximumLaneSeparation = m_maximumLaneSeparation + 40;
        if (RIGHT_TURN) m_ROIWidth = m_ROIWidth + 150;
        if (RIGHT_TURN) m_maxLineWidth = m_maxLineWidth + 10;
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;

        clock_t begin = clock();
        lanePoint relativePoints[50]; //double m_detectionLines to have safety margin
        std::vector<lanePoint> middleLanePoints;

        Mat outputImage;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {

            //create a opencv matrix from the media sample buffer

            Mat inputImage(cv::Size(m_InPinVideoFormat.m_ui32Width, m_InPinVideoFormat.m_ui32Height),
                           CV_8UC3, (uchar*)pReadBuffer->GetPtr());

            cv::cvtColor(inputImage, outputImage, COLOR_BGR2GRAY);

            //cv::threshold(outputImage, outputImage,0, 255, CV_THRESH_OTSU);
            cv::adaptiveThreshold(outputImage, outputImage, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, m_sizeSegmentImageBinarization, m_thresholdImageBinarization);// Generate Binary Image
            // Detect Lines
            // here we store the pixel lines in the image where we search for lanes
            vector<tInt> detectionLines;

            // here we have all the detected line points
            vector<Point> tempDetectedLinePoints;
            vector<lanePoint> detectedLinePoints;
            //calculate the detectionlines in image
            getDetectionLines(detectionLines);
            RETURN_IF_FAILED(findLinePoints(detectionLines, outputImage, tempDetectedLinePoints));
            for (int point = 0; point < tempDetectedLinePoints.size(); point++)
            {
                detectedLinePoints.push_back(lanePoint(tempDetectedLinePoints.at(point).x,tempDetectedLinePoints.at(point).y));
            }

            //First, sort points in x direction and remove points over height limit (for curvature clustering)
            detectedLinePoints.erase(
                        std::remove_if(detectedLinePoints.begin(),
                                       detectedLinePoints.end(),
                                       [this](const lanePoint& p){return p.y<m_clusterHeightLimit;}),
                        detectedLinePoints.end());

            //Check if right turn: if there are points near roi limits
            if (detectedLinePoints.size()>0)
                std::sort(detectedLinePoints.begin(), detectedLinePoints.end(), mycomparatorX);

            cvtColor(outputImage, outputImage, COLOR_GRAY2RGB);
            // draw ROI
            rectangle(outputImage, m_LaneRoi, Scalar(255), 10, 8, 0);

            //Next create vector of adjacent differences based on vector with x only
            PercentageDifference(detectedLinePoints);
            //Set threshold and start new cluster when threshold passed, get average of each cluster
            //Also if point jumps down in y by over two lines, different cluster (useful for turns)
            int current_cluster = 0;
            for (int p = 0; p < detectedLinePoints.size(); p++)
            {
                if ((detectedLinePoints.at(p).pctDiff > m_clusterThreshold))
                    current_cluster += 1;
                detectedLinePoints.at(p).cluster = current_cluster;
            }


            std::vector<Scalar> colorCycle;
            colorCycle.emplace_back(0,0,255);
            colorCycle.emplace_back(0,255,0);
            colorCycle.emplace_back(255,0,0);
            colorCycle.emplace_back(100,100,255);
            colorCycle.emplace_back(100,255,100);
            colorCycle.emplace_back(100,255,255);
            colorCycle.emplace_back(100,100,100);

            //Group points by y starting from down (inverted for opencv coordinates)
            if (!detectedLinePoints.empty())
                std::sort(detectedLinePoints.begin(), detectedLinePoints.end(), mycomparatorY);

            std::vector<int> sameLineIndexes;
            std::vector<int> indexesToDrop;
            std::vector<int> clustersAppear;

            //if cluster has multiple points for same y, remove them
            for (int point = 0; point < detectedLinePoints.size(); point++)
            {
                sameLineIndexes.push_back(point);
                //if new line or last point
                if ((point == detectedLinePoints.size()-1) || (detectedLinePoints.at(point).y != detectedLinePoints.at(point+1).y)) {
                    //for every cluster, check if appears more than once
                    for (int sameLineIndexe : sameLineIndexes) {
                        if (std::find(clustersAppear.begin(), clustersAppear.end(), detectedLinePoints.at(
                                          sameLineIndexe).cluster) != clustersAppear.end())
                        {
                            //add points for this cluster and this y to indexesToDrop
                            for (int j=0; j<detectedLinePoints.size(); j++)
                            {
                                if ((detectedLinePoints.at(sameLineIndexe).y == detectedLinePoints.at(j).y)
                                        && (detectedLinePoints.at(sameLineIndexe).cluster == detectedLinePoints.at(j).cluster))
                                {
                                    //if not in indexesToDrop
                                    if (std::find(indexesToDrop.begin(), indexesToDrop.end(), j) == indexesToDrop.end())
                                        indexesToDrop.push_back(j);
                                }
                            }
                        } else {
                            clustersAppear.push_back(detectedLinePoints.at(sameLineIndexe).cluster);
                        }
                    }
                    sameLineIndexes.clear();
                    clustersAppear.clear();
                }
            }

            std::sort(indexesToDrop.begin(), indexesToDrop.end(), mycomparatorInt);
            for (auto &ind : indexesToDrop) {
                detectedLinePoints.erase(detectedLinePoints.begin() + ind);
            }

            indexesToDrop.clear();

            //if cluster has <=2 points, remove it (assume noise)
            //If smallest value from cluster is high: drop it
            int clustersCount[current_cluster+1];
            int minClusterPoints;
            if (RIGHT_TURN) minClusterPoints = 2;
            if (!RIGHT_TURN) minClusterPoints = 2;

            memset(clustersCount, 0, sizeof(clustersCount));
            for (int point = 0; point < detectedLinePoints.size(); point++)
            {
                clustersCount[detectedLinePoints[point].cluster] += 1;
            }

            //Count how many clusters left
            int clustersLeft = 0;
            for (int i = current_cluster; i>=0; i--)
            {
                if (clustersCount[i]>minClusterPoints)
                    clustersLeft++;
            }
            //Eliminate right-most if the cluster before has way more points
            //Not if right turn
            int rightCluster = -1;
            if (clustersLeft>= 3 && !RIGHT_TURN)
            {
                for (int i = current_cluster; i>=0; i--)
                {
                    if (clustersCount[i]>minClusterPoints)
                    {
                        if (rightCluster<0)
                            rightCluster = i;
                        else
                        {
                            if ((clustersCount[rightCluster]<=4) && (clustersCount[i]-clustersCount[rightCluster] > 2))
                            {
                                clustersCount[rightCluster] = 1;
                                clustersLeft -=1;
                            }
                            break;
                        }
                    }
                }

            }
            //if right turn, delete clusters that aren't going right
            if (detectedLinePoints.size()>0 && RIGHT_TURN) {

                std::sort(detectedLinePoints.begin(), detectedLinePoints.end(), mycomparatorX);
                for (int point = 0; point < detectedLinePoints.size()-1; point ++)
                {
                    sameLineIndexes.push_back(point);
                    if (point==detectedLinePoints.size()-2) sameLineIndexes.push_back(point+1);
                    if ((point==detectedLinePoints.size()-2) || (detectedLinePoints[point].cluster != detectedLinePoints[point+1].cluster))
                    {
                        for (int i=1; i<sameLineIndexes.size(); i++)
                        {
                            if((clustersCount[detectedLinePoints.at(sameLineIndexes[i]).cluster] > minClusterPoints) && (detectedLinePoints[sameLineIndexes[i]].y-detectedLinePoints[sameLineIndexes[i-1]].y>0))
                            {
                                clustersCount[detectedLinePoints[point].cluster] = 1;
                                clustersLeft -=1;
                                break;
                            }

                        }

                        sameLineIndexes.clear();
                    }
                }
            }
            LOG_DUMP("%d clusters found", clustersLeft);
            int nClusters = 0;
            int nRightMost =0, nLeftMost = 0;
            for (int i = current_cluster; i>=0; i--)
            {
                if (clustersCount[i] <= minClusterPoints)
                {
                    for (int point = 0; point < detectedLinePoints.size(); point++)
                    {
                        if (detectedLinePoints[point].cluster == i)
                        {
                            indexesToDrop.push_back(point);
                        }
                    }
                    if (current_cluster == i)
                        //update current_cluster with value of right-most occupied cluster
                        current_cluster -= 1;
                } else {
                    //rightmost cant have more than left most in right turn
                    if (nRightMost == 0) nRightMost = clustersCount[i];
                    else nLeftMost = clustersCount[i] ;
                    nClusters += 1;
                }
            }
            std::sort(indexesToDrop.begin(), indexesToDrop.end(), mycomparatorInt);
            for (auto &ind : indexesToDrop)
            {
                detectedLinePoints.erase(detectedLinePoints.begin() + ind);
            }

            if (nClusters <3) goto useMap;
            if (nRightMost>nLeftMost && RIGHT_TURN) goto useMap;
            indexesToDrop.clear();

            sameLineIndexes.clear();

            int rightLaneX = 0;
            int laneWidth;

            if (detectedLinePoints.size()>0)
                std::sort(detectedLinePoints.begin(), detectedLinePoints.end(), mycomparatorY);

            std::vector<int> localizationPoints;

            for (int point = 0; point < detectedLinePoints.size(); point++)
            {
                sameLineIndexes.push_back(point);
                //if new line or last point
                if ((point == detectedLinePoints.size()-1) || (detectedLinePoints.at(point).y != detectedLinePoints.at(point+1).y)) {
                    //get x coordinate from point of last cluster if exists
                    for (int i = 0; i < sameLineIndexes.size(); i++) {
                        //find its cluster
                        if (detectedLinePoints[sameLineIndexes.at(i)].cluster == current_cluster)
                        {
                            rightLaneX = detectedLinePoints[sameLineIndexes.at(i)].x;
                            break;
                        }
                    }
                    if (rightLaneX != 0) {
                        //if last cluster point has corresponding second cluster point: add to middle points vector
                        for (int j = 0; j < sameLineIndexes.size(); j++) {

                            if (detectedLinePoints[sameLineIndexes.at(j)].x!=rightLaneX
                                    && rightLaneX-detectedLinePoints[sameLineIndexes.at(j)].x>m_minimumLaneSeparation
                                    && rightLaneX-detectedLinePoints[sameLineIndexes.at(j)].x<m_maximumLaneSeparation) {
                                //if different point, and in range
                                middleLanePoints.push_back(lanePoint((detectedLinePoints[sameLineIndexes.at(j)].x+rightLaneX)/2.0,detectedLinePoints[sameLineIndexes.at(j)].y));
                                if (middleLanePoints.size() == 1) //if first point, get lane width
                                    //TODO: possibly change to average lane width, or fixed
                                    laneWidth = rightLaneX-detectedLinePoints[sameLineIndexes.at(j)].x;
                            }
                        }
                    }
                    //Get 3 points on first and second line
                    if (localizationPoints.size()<6 && sameLineIndexes.size()==3)
                    {
                        cv::line(outputImage, Point(detectedLinePoints[sameLineIndexes[0]].x,detectedLinePoints[sameLineIndexes[0]].y),Point(detectedLinePoints[sameLineIndexes[1]].x,detectedLinePoints[sameLineIndexes[1]].y), Scalar(255,0,0), 3);
                        cv::line(outputImage, Point(detectedLinePoints[sameLineIndexes[1]].x,detectedLinePoints[sameLineIndexes[1]].y),Point(detectedLinePoints[sameLineIndexes[2]].x,detectedLinePoints[sameLineIndexes[2]].y), Scalar(255,0,0), 3);
                        localizationPoints.push_back(sameLineIndexes[0]);
                        localizationPoints.push_back(sameLineIndexes[1]);
                        localizationPoints.push_back(sameLineIndexes[2]);
                    }

                    rightLaneX = 0;
                    sameLineIndexes.clear();
                }
            }

            //send points
            if (localizationPoints.size()==6)
            {
                object_ptr<ISample> pLineSample;

                RETURN_IF_FAILED(alloc_sample(pLineSample))
                {
                    {
                        auto oCodec = m_LineStructSampleFactory.MakeCodecFor(pLineSample);

                        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlLineDataId.linesFound, 4));

                        tLineCoordiante* points = reinterpret_cast<tLineCoordiante*>(oCodec.GetElementAddress(m_ddlLineDataId.lineArray));
                        //2-3 normally, but for some reason inverted
                        points[0].f32Distance = (tFloat32)pixelToRelativeCoordsX(detectedLinePoints[localizationPoints[1]].x);
                        points[0].f32FrontDistance = (tFloat32)pixelToRelativeCoordsY(detectedLinePoints[localizationPoints[1]].y);
                        points[0].f32RoadId = (tFloat32)3;

                        points[1].f32Distance = (tFloat32)pixelToRelativeCoordsX(detectedLinePoints[localizationPoints[2]].x);
                        points[1].f32FrontDistance = (tFloat32)pixelToRelativeCoordsY(detectedLinePoints[localizationPoints[2]].y);
                        points[1].f32RoadId = (tFloat32)2;

                        points[2].f32Distance =(tFloat32) pixelToRelativeCoordsX(detectedLinePoints[localizationPoints[4]].x);
                        points[2].f32FrontDistance = (tFloat32)pixelToRelativeCoordsY(detectedLinePoints[localizationPoints[4]].y);
                        points[2].f32RoadId = (tFloat32)3;

                        points[3].f32Distance = (tFloat32)pixelToRelativeCoordsX(detectedLinePoints[localizationPoints[5]].x);
                        points[3].f32FrontDistance = (tFloat32)pixelToRelativeCoordsY(detectedLinePoints[localizationPoints[5]].y);
                        points[3].f32RoadId = (tFloat32)2;

                    }
                }

                LineDataOut << pLineSample << flush << trigger;
            }

            int colorIndex;
            for (int i = 0; i < detectedLinePoints.size(); i++)
            {
                colorIndex = detectedLinePoints.at(i).cluster-1;
                while (colorIndex >= colorCycle.size())
                    colorIndex -= colorCycle.size();

                circle(outputImage,
                       cv::Point(detectedLinePoints.at(i).x,detectedLinePoints.at(i).y),
                       5,
                       colorCycle.at(colorIndex),
                       -1,
                       8);
            }

            //LOG_INFO("%d",laneWidth);
            //Display middle of lane points
            for (int i = 0; i < middleLanePoints.size(); i ++)
            {
                circle(outputImage,
                       cv::Point(middleLanePoints.at(i).x,middleLanePoints.at(i).y),
                       5,
                       Scalar(255,255,255),
                       2,
                       8);
            }
            //Draw direction arrows, to tweak: first to last middle of lane
            if (!middleLanePoints.empty()) {
                arrowedLine(outputImage, cv::Point(middleLanePoints.at(0).x,middleLanePoints.at(0).y), cv::Point(middleLanePoints.back().x,middleLanePoints.back().y), Scalar(255, 0, 0), 2);

                //Display distance from center in % (assuming first point is the center)
                double distanceCenterPercentage = (middleLanePoints.at(0).x-m_frontCarPositionX)/(laneWidth*0.01);
                LOG_DUMP("%f center distance, first middle %d", distanceCenterPercentage, middleLanePoints.at(0).x);
                putText(outputImage, std::to_string(distanceCenterPercentage) + "% from center", Point(300,50), FONT_HERSHEY_PLAIN, 2, Scalar(0,143,143), 2);
            }

            //Get points relative to front of car, in mm
            if (!middleLanePoints.empty())
                relativePoints[0] = lanePoint(0,0);
            for (int i = 0; i < middleLanePoints.size() ; i++)
            {
                relativePoints[i+1] = lanePoint(pixelToRelativeCoordsX(middleLanePoints.at(i).x),
                                                pixelToRelativeCoordsY(middleLanePoints.at(i).y));
            }

            pReadBuffer->Unlock();
        }

        //not sending points and using map instead (uncertain about lanes)
        if (false) useMap: putText(outputImage, "uncertain use map", Point(500,50), FONT_HERSHEY_PLAIN, 2, Scalar(0,143,143), 2);

        if (RIGHT_TURN) m_maximumLaneSeparation = m_maximumLaneSeparation- 40;
        if (RIGHT_TURN) m_ROIWidth = m_ROIWidth - 150;
        if (RIGHT_TURN) m_maxLineWidth = m_maxLineWidth - 10;


        //Write processed Image to Output Pin
        if (!outputImage.empty())
        {
            //update output format if matrix size does not fit to
            if (outputImage.total() * outputImage.elemSize() != m_OutPinVideoFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_oWriterVideo, outputImage);
            }
            // write to pin
            writeMatToPin(m_oWriterVideo, outputImage, m_pClock->GetStreamTime());
        }

        if (!middleLanePoints.empty())
        {
            for (int i=0; i<middleLanePoints.size(); i++) {
                LOG_DUMP("y: %d mm, x: %d mm", relativePoints[i].y, relativePoints[i].x);
            }
            lanePoint p;
            Transmit(relativePoints, middleLanePoints.size()+1);
            //putText(outputImage, "Steering: " + std::to_string(steer) + ". Speed: "  + std::to_string(speed) + ".", Point(300,100), FONT_HERSHEY_PLAIN, 2, Scalar(0,143,143), 2);
            circle(outputImage,
                   cv::Point(m_frontCarPositionX+(p.x)/m_pixelToMillimiterX,-(p.y/m_pixelToMillimiterY+m_frontCarPositionY-outputImage.size().height)),
                   7,
                   Scalar(255,0,0),
                   2,
                   8);
        }

        clock_t c_end = clock();
        LOG_INFO(" TIME:: %f SUPPOSEDFPS:: %f", double(c_end - begin) / CLOCKS_PER_SEC, 1/(double(c_end - begin) / CLOCKS_PER_SEC));

    }

    RETURN_NOERROR;
}

tResult cLaneDetection::findLinePoints(const vector<tInt>& detectionLines, const cv::Mat& image, vector<Point>& detectedLinePoints)
{
    RETURN_IF_FAILED(checkRoi());
    //iterate through the calculated horizontal lines
    for (vector<tInt>::const_iterator nline = detectionLines.begin(); nline != detectionLines.end(); nline++)
    {
        uchar ucLastVal = 0;

        // create vector with line data
        const uchar* p = image.ptr<uchar>(*nline, m_ROIOffsetX);
        std::vector<uchar> lineData(p, p + m_ROIWidth);

        tBool detectedStartCornerLine = tFalse;
        tInt columnStartCornerLine = 0;

        for (std::vector<uchar>::iterator lineIterator = lineData.begin(); lineIterator != lineData.end(); lineIterator++)
        {
            uchar ucCurrentVal = *lineIterator;
            tInt currentIndex = tInt(std::distance(lineData.begin(), lineIterator));
            //look for transition from dark to bright -> start of line corner
            if ((ucCurrentVal - ucLastVal) > m_minLineContrast)
            {
                detectedStartCornerLine = tTrue;
                columnStartCornerLine = currentIndex;
            }//look for transition from bright to dark -> end of line
            else if ((ucLastVal - ucCurrentVal) > m_minLineContrast && detectedStartCornerLine)
            {
                //we already have the start corner of line, so check the width of detected line
                if ((abs(columnStartCornerLine - currentIndex) > m_minLineWidth)
                        && (abs(columnStartCornerLine - currentIndex) < m_maxLineWidth))
                {
                    detectedLinePoints.push_back(Point(tInt(currentIndex - abs(columnStartCornerLine - currentIndex) / 2 +
                                                            m_ROIOffsetX), *nline));

                    detectedStartCornerLine = tFalse;
                    columnStartCornerLine = 0;
                }
            }
            //we reached maximum line width limit, stop looking for end of line
            if (detectedStartCornerLine &&
                    abs(columnStartCornerLine - currentIndex) > m_maxLineWidth)
            {
                detectedStartCornerLine = tFalse;
                columnStartCornerLine = 0;
            }
            ucLastVal = ucCurrentVal;
        }
    }

    RETURN_NOERROR;
}

tResult cLaneDetection::getDetectionLines(vector<tInt>& detectionLines)
{
    tInt distanceBetweenDetectionLines = m_ROIHeight / (m_detectionLines + 1);

    for (int i = 1; i <= m_detectionLines; i++)
    {
        detectionLines.push_back(m_ROIOffsetY + i * distanceBetweenDetectionLines);
    }
    RETURN_NOERROR;
}

tResult cLaneDetection::checkRoi(void)
{
    // if width or heigt are not set ignore the roi
    if (static_cast<tFloat32>(m_ROIWidth) == 0 || static_cast<tFloat32>(m_ROIHeight) == 0)
    {
        LOG_ERROR("ROI width or height is not set!");
        RETURN_ERROR_DESC(ERR_INVALID_ARG, "ROI width or height is not set!");
    }

    //check if we are within the boundaries of the image
    if ((static_cast<tFloat32>(m_ROIOffsetX) + static_cast<tFloat32>(m_ROIWidth)) > m_InPinVideoFormat.m_ui32Width)
    {
        LOG_ERROR("ROI is outside of image");
        RETURN_ERROR_DESC(ERR_INVALID_ARG, "ROI is outside of image");
    }

    if ((static_cast<tFloat32>(m_ROIOffsetY) + static_cast<tFloat32>(m_ROIHeight)) > m_InPinVideoFormat.m_ui32Height)
    {
        LOG_ERROR("ROI is outside of image");
        RETURN_ERROR_DESC(ERR_INVALID_ARG, "ROI is outside of image");
    }

    //create the rectangle
    m_LaneRoi = cv::Rect2f(static_cast<tFloat32>(m_ROIOffsetX), static_cast<tFloat32>(m_ROIOffsetY), static_cast<tFloat32>(m_ROIWidth), static_cast<tFloat32>(m_ROIHeight));


    RETURN_NOERROR;
}

tResult cLaneDetection::Transmit(const lanePoint *points, const int &nPoints)
{

    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample, m_pClock->GetStreamTime()))
    {

        auto oCodec = m_LanePointDataSampleFactory.MakeCodecFor(pSample);

        oCodec.SetElementValue(m_ddlLanePointDataId.nPoints, nPoints);

        auto LanePoints_out = reinterpret_cast<tLanePoint*>(oCodec.GetElementAddress(m_ddlLanePointDataId.pointArray));

        //init array with zeros
        memset(LanePoints_out, 0, (nPoints) * sizeof(tLanePoint));

        for (int i = 0; i<nPoints; i++)
        {
            LanePoints_out[i].x = points[i].x;
            LanePoints_out[i].y = points[i].y;

        }


    }

    m_oWriterLane << pSample << flush << trigger;

    RETURN_NOERROR;
}
