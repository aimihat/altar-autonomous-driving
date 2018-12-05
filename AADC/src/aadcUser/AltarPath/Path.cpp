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
#include "Path.h"
#include "ADTF3_OpenCV_helper.h"
#include <fstream>
#include <map>
#include <math.h>
#include <aadc_geometrics.h>
//
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ALTAR_PATH_FILTER,
                                    "AltarPath",
                                    cPath,
                                    adtf::filter::pin_trigger({"Position"}));
//ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ALTAR_PATH_FILTER,
//                                    "AltarPath",
//                                    cPath,
//                                    adtf::filter::timer_trigger(10000));

cPath::cPath() {
    // Position Data
    object_ptr<IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData,
                                                                                       m_PositionSampleFactory)) {
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32radius", m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32speed", m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32heading", m_ddlPositionIndex.heading);
    } else {
        LOG_WARNING("No mediadescription for tPosition found!");
    }

    Register(PoseDataIn, "Position", pTypePositionData);



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

    // Image Data
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register output pin
    Register(m_oWriter, "Map", pType);

    //For visualization
    METERS_TO_PIXELS = 120.0;
    OFFSET_DRAW = 0;
    GRIDSIZE = 1000;
    mImageGrid = cv::Mat(GRIDSIZE, GRIDSIZE, CV_8UC3, cv::Scalar(0, 0, 0));


    // Create provisional node connections (previous and next)
    //junction j1;
    //j1.id;
    //j1.nodeList.push_back()

    nodeSuccessor[-5] = -4;
    nodePredecessor[-4] = -5;
    nodeSuccessor[-4] = -6;
    nodePredecessor[-6] = -4;
    nodeSuccessor[-6] = -12;
    nodePredecessor[-12] = -6;
    nodeSuccessor[-12] = -10;
    nodePredecessor[-10] = -12;
    nodeSuccessor[-10] = -11;
    nodePredecessor[-11] = -10;
    nodeSuccessor[-11] = -8;
    nodePredecessor[-8] = -11;
    nodeSuccessor[-8] = -7;
    nodePredecessor[-7] = -8;
    nodeSuccessor[-7] = -9;
    nodePredecessor[-9] = -7;
    nodeSuccessor[-9] = -3;
    nodePredecessor[-3] = -9;
    nodeSuccessor[-3] = -1;
    nodePredecessor[-1] = -3;
    nodeSuccessor[-1] = -2;
    nodePredecessor[-2] = -1;
    nodeSuccessor[-2] = -5;
    nodePredecessor[-5] = -2;

    //Use to load segments into any plugin from the text file generated
    ifstream myfile("/home/aadc/audi/AADC/src/aadcUser/AltarPath/map_segments.txt");

    if (!myfile)
    {
        LOG_WARNING("unable to open segments file");
    }
    int numSegments;
    myfile >> numSegments;
    for (int i = 0; i < numSegments; i++) {
        segment s;
        int sizeLeft, sizeRight, sizeMiddle, sizePoints;
        myfile >> s.id;
        myfile >> sizePoints;
        for (int j = 0; j < sizePoints; j++) {
            point p;
            myfile >> p.x;
            myfile >> p.y;
            s.points.push_back(p);
        }
        myfile >> sizeLeft;
        for (int j = 0; j < sizeLeft; j++) {
            point p;
            myfile >> p.x;
            myfile >> p.y;
            s.left.push_back(p);
        }
        myfile >> sizeMiddle;
        for (int j = 0; j < sizeMiddle; j++) {
            point p;
            myfile >> p.x;
            myfile >> p.y;
            s.middle.push_back(p);
        }
        myfile >> sizeRight;
        for (int j = 0; j < sizeRight; j++) {
            point p;
            myfile >> p.x;
            myfile >> p.y;
            s.right.push_back(p);
        }
        segments.push_back(s);
    }
    myfile.close();

    for (int i = 0; i < segments.size(); i++) {
        for (int p = 0; p < segments[i].left.size() - 1; p++) {
            cv::Point line_start_l((int) (segments[i].left[p].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                   (int) (GRIDSIZE - (segments[i].left[p].y * METERS_TO_PIXELS + OFFSET_DRAW)));
            cv::Point line_end_l((int) (segments[i].left[p + 1].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                 (int) (GRIDSIZE - (segments[i].left[p + 1].y * METERS_TO_PIXELS + OFFSET_DRAW)));
            cv::line(mImageGrid, line_start_l, line_end_l, cv::Scalar(255, 255, 0));
        }
        for (int p = 0; p < segments[i].right.size() - 1; p++) {
            cv::Point line_start_r((int) (segments[i].right[p].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                   (int) (GRIDSIZE - (segments[i].right[p].y * METERS_TO_PIXELS + OFFSET_DRAW)));
            cv::Point line_end_r((int) (segments[i].right[p + 1].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                 (int) (GRIDSIZE - (segments[i].right[p + 1].y * METERS_TO_PIXELS + OFFSET_DRAW)));
            cv::line(mImageGrid, line_start_r, line_end_r, cv::Scalar(255, 255, 255));
        }
        for (int p = 0; p < segments[i].middle.size() - 1; p++) {
            cv::Point line_start_m((int) (segments[i].middle[p].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                   (int) (GRIDSIZE-(segments[i].middle[p].y * METERS_TO_PIXELS + OFFSET_DRAW)));
            cv::Point line_end_m((int) (segments[i].middle[p + 1].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                 (int) (GRIDSIZE-(segments[i].middle[p + 1].y * METERS_TO_PIXELS + OFFSET_DRAW)));
            cv::line(mImageGrid, line_start_m, line_end_m, cv::Scalar(0, 255, 255));
        }
        cv::circle(
                mImageGrid,
                cv::Point((segments[i].middle[0].x * METERS_TO_PIXELS + OFFSET_DRAW),
                          (GRIDSIZE-(segments[i].middle[0].y * METERS_TO_PIXELS + OFFSET_DRAW))),
                5,
                cv::Scalar(255, 255, 255),
                -1);

        std::ostringstream ss;
        ss << segments[i].id;
        putText(mImageGrid, ss.str(), cv::Point(
                ((segments[i].middle[0].x + segments[i].middle[segments[i].middle.size() - 1].x) / 2.0 *
                 METERS_TO_PIXELS + OFFSET_DRAW),
                (GRIDSIZE-((segments[i].middle[0].y + segments[i].middle[segments[i].middle.size() - 1].y) / 2.0 *
                 METERS_TO_PIXELS + OFFSET_DRAW))), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 255));

        cv::circle(
                mImageGrid,
                cv::Point((segments[i].middle[segments[i].middle.size() - 1].x * METERS_TO_PIXELS + OFFSET_DRAW),
                          (GRIDSIZE-(segments[i].middle[segments[i].middle.size() - 1].y * METERS_TO_PIXELS + OFFSET_DRAW))), 5,
                cv::Scalar(255, 255, 255),
                -1);
    }
}

//implement the Configure function to read ALL Properties
tResult cPath::Configure() {
    // -------------------------------
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    //--------------------------------
    RETURN_NOERROR;
}

tResult cPath::Transmit(vector<point>& relativePath)
{
    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample, m_pClock->GetStreamTime()))
    {
        auto oCodec = m_LanePointDataSampleFactory.MakeCodecFor(pSample);

        oCodec.SetElementValue(m_ddlLanePointDataId.nPoints, relativePath.size());

        auto LanePoints_out = reinterpret_cast<lanePoint*>(oCodec.GetElementAddress(m_ddlLanePointDataId.pointArray));

        //init array with zeros
        memset(LanePoints_out, 0, (relativePath.size()) * sizeof(lanePoint));

        for (int i = 0; i<relativePath.size(); i++)
        {
            LanePoints_out[i].x = relativePath[i].x;
            LanePoints_out[i].y = relativePath[i].y;

        }
    }

    m_oWriterLane << pSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cPath::transformPoint(point& transformedP, float cosine, float sine)
{
    //change coordinate system
    //shift center
    transformedP.x = transformedP.x - carPose.x;
    transformedP.y = transformedP.y - carPose.y;
    float tempx = transformedP.x * 1000;
    float tempy = transformedP.y * 1000;
    //rotate and scale to mm
    transformedP.x = tempx*cosine + tempy*sine;
    transformedP.y = -tempx*sine + tempy*cosine;

    RETURN_NOERROR;
}

tResult cPath::Process(tTimeStamp tmTimeOfTrigger) {
    outputImage = mImageGrid.clone();
    vector<point> relativePath;
    vector<point> absolutePath;

    object_ptr<const ISample> pReadSample;

    if (IS_OK(PoseDataIn.GetLastSample(pReadSample))) {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(pReadSample);
        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &carPose.x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &carPose.y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &carPose.theta));

        carPose.x *= 0.001;
        carPose.y *= 0.001;
    }

    //Find nearest segment based on segment extremities

    float cosine = cos(carPose.theta);
    float sine = sin(carPose.theta);
    float transformCosine = cos(carPose.theta-PI/2);
    float transformSine = sin(carPose.theta-PI/2);

    //TODO: use quadtree instead of simple distances
    float minDist = 999.0;
    float dist;
    int closest, closestInd;
    for (int s = 0; s < segments.size(); s++) {
        for (int p = 0; p < segments[s].middle.size(); p++) {
            dist = sqrt(pow(segments[s].middle[p].x - carPose.x, 2) + pow(segments[s].middle[p].y - carPose.y, 2));
            if (dist < minDist) {
                minDist = dist;
                closest = segments[s].id;
                closestInd = s;
                };
        }
    }

    //Draw rotated rectangle representing car + orientation lines.
    float carHeight = CAR_HEIGHT*0.01;
    float carWidth = CAR_WIDTH*0.01;
    cv::RotatedRect rotatedRectangle(
            //center point
            cv::Point((carPose.x-(carHeight/2)*cosine)* METERS_TO_PIXELS + OFFSET_DRAW,GRIDSIZE-((carPose.y-(carHeight/2)*sine)* METERS_TO_PIXELS + OFFSET_DRAW)),
            cv::Size(carWidth* METERS_TO_PIXELS, carHeight* METERS_TO_PIXELS),
            90-carPose.theta*180.0/PI);

    cv::Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);

    cv::Point vertices[4];
    for(int i = 0; i < 4; ++i){
        vertices[i] = vertices2f[i];
    }

    cv::fillConvexPoly(outputImage,
                       vertices,
                       4,
                       cv::Scalar(255, 255, 255));

    //For orientation, draw point 30cm away
    float orientationLen = 0.3;

    float destinationX = carPose.x + cosine * orientationLen;
    float destinationY = carPose.y + sine * orientationLen;
    cv::arrowedLine	(outputImage,
            cv::Point(carPose.x* METERS_TO_PIXELS + OFFSET_DRAW,GRIDSIZE-(carPose.y* METERS_TO_PIXELS + OFFSET_DRAW)),
            cv::Point(destinationX* METERS_TO_PIXELS + OFFSET_DRAW,GRIDSIZE-(destinationY* METERS_TO_PIXELS + OFFSET_DRAW)),
            cv::Scalar(255, 255, 255), 1);
    cv::circle(outputImage,cv::Point(carPose.x* METERS_TO_PIXELS + OFFSET_DRAW,GRIDSIZE-(carPose.y* METERS_TO_PIXELS + OFFSET_DRAW)),4,cv::Scalar(255,0,0));

    //// From orientation, keep either successor or predecessor
    //Find line that splits (y = mx+b)
    bool successor;
    int nextNode, nextNode2;
    float m = tan(carPose.theta+PI/2);
    float b = carPose.y-(m*carPose.x);
    //Draw line
    //cv::Point a((int)0,(int)GRIDSIZE-(b* METERS_TO_PIXELS + OFFSET_DRAW));
    //cv::Point aa((int)GRIDSIZE,(int)GRIDSIZE-((m*(GRIDSIZE/METERS_TO_PIXELS)+b)* METERS_TO_PIXELS + OFFSET_DRAW));
    //cv::line(outputImage, a, aa, cv::Scalar(255, 0, 0))

    //Numerical stability theta = 0
    if (carPose.theta == 0)
        carPose.theta == 0.001;
    //Keep point or not based on sign and theta
    if (carPose.theta > 0 && carPose.theta<PI)
    {
        //Keep if above fnc
        for (int s = 0; s < segments.size(); s++) {
            if (segments[s].id ==  nodeSuccessor[closest]) {
                if (segments[s].middle[0].y > (m * segments[s].middle[0].x + b)) {
                    nextNode = nodeSuccessor[closest];
                    successor = 1;
                } else {
                    nextNode = nodePredecessor[closest]; //Interpolating that if not successor: predecessor
                    successor = 0;
                }
                break;
            }
        }
    } else if (carPose.theta < 0 && carPose.theta>-PI) {
        //Keep if below fnc
        for (int s = 0; s < segments.size(); s++) {
            if (segments[s].id ==  nodeSuccessor[closest]) {
                if (segments[s].middle[0].y < (m * segments[s].middle[0].x + b)) {
                    nextNode = nodeSuccessor[closest];
                    successor = 1;
                } else {
                    nextNode = nodePredecessor[closest];
                    successor = 0;
                }
                break;
            }
        }
    }


    if (successor) nextNode2 = nodeSuccessor[nextNode];
    if (!successor) nextNode2 = nodePredecessor[nextNode];

    //Feed path + filter points from existing segment

    for (int p = 0; p < segments[closestInd].points.size(); p++) {
        if (((carPose.theta < 0 && carPose.theta>-PI) && (segments[closestInd].points[p].y < (m * segments[closestInd].points[p].x + b)))
            || ((carPose.theta > 0 && carPose.theta<PI) && (segments[closestInd].points[p].y > (m * segments[closestInd].points[p].x + b)))) {

            //change coordinate system
            point transformedP = segments[closestInd].points[p];
            absolutePath.push_back(transformedP);
            transformPoint(transformedP, transformCosine, transformSine);
            relativePath.push_back(transformedP);
        }
    }

    //TODO: If multiple routes possible, choose based on instructions feed.
    for (int i = 0; i < segments.size(); i++)
    {
        if (segments[i].id == nextNode)
        {
            for (int p = 0; p < segments[i].points.size(); p++) {
                point transformedP = segments[i].points[p];
                absolutePath.push_back(transformedP);
                transformPoint(transformedP, transformCosine, transformSine);
                relativePath.push_back(transformedP);
            }
            break;
        }
    }
    for (int i = 0; i < segments.size(); i++)
    {
        if (segments[i].id == nextNode2)
        {
            for (int p = 0; p < segments[i].points.size(); p++) {
                point transformedP = segments[i].points[p];
                absolutePath.push_back(transformedP);
                transformPoint(transformedP, transformCosine, transformSine);
                relativePath.push_back(transformedP);
            }
            break;
        }
    }

    std::sort(relativePath.begin(), relativePath.end(), mycomparatorY);
    std::sort(absolutePath.begin(), absolutePath.end(), mycomparatorY);

    for (int p = 0; p < absolutePath.size(); p++)
    {
        cv::Point pathPoint((int) (absolutePath[p].x * METERS_TO_PIXELS + OFFSET_DRAW),
                               (int) (GRIDSIZE-(absolutePath[p].y * METERS_TO_PIXELS + OFFSET_DRAW)));
        //cv::Point line_end_m((int) (absolutePath[p + 1].x * METERS_TO_PIXELS + OFFSET_DRAW),
        //                     (int) (GRIDSIZE-(absolutePath[p + 1].y * METERS_TO_PIXELS + OFFSET_DRAW)));
        cv::circle(outputImage, pathPoint, 2, cv::Scalar(255,0,0), -1);
        //cv::line(outputImage, line_start_m, line_end_m, cv::Scalar(255, 0, 0));
    }

    if (!outputImage.empty())
    {
        //update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize() != m_sImageFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_oWriter, outputImage);
        }
        // write to pin
        writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
    }

    RETURN_NOERROR;
}
