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
#include "AltarTrajectoryGen.h"
#include "ADTF3_OpenCV_helper.h"
#include <algorithm>
#include "spline.h"
#include <unistd.h>

/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_cTrajectoryGen_DATA_TRIGGERED_FILTER,
                                    "TrajectoryGen",
                                    cTrajectoryGen,
                                    adtf::filter::pin_trigger({"lane_points", "speed_override", "steer_override"}));

cTrajectoryGen::cTrajectoryGen() {

    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeSignalValueSpd;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue",
                                                                                       pTypeSignalValueSpd,
                                                                                       m_SignalValueSpdSampleFactory)) {
        adtf_ddl::access_element::find_index(m_SignalValueSpdSampleFactory, cString("ui32ArduinoTimestamp"),
                                             m_ddlSignalValueSpdId.timeStamp);
        adtf_ddl::access_element::find_index(m_SignalValueSpdSampleFactory, cString("f32Value"),
                                             m_ddlSignalValueSpdId.value);
    } else {
        LOG_INFO("No mediadescription for tSignalValueSpd found!");
    }

    //Register(m_oReaderSpd, "speed_input" , pTypeSignalValueSpd);
    Register(m_oWriterSpd, "speed_output", pTypeSignalValueSpd);
    Register(m_oSpeedReader, "speed_override", pTypeSignalValueSpd);
    Register(m_oSteerReader, "steer_override", pTypeSignalValueSpd);


    object_ptr<IStreamType> pTypeSignalValueSteer;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue",
                                                                                       pTypeSignalValueSteer,
                                                                                       m_SignalValueSteerSampleFactory)) {
        adtf_ddl::access_element::find_index(m_SignalValueSteerSampleFactory, cString("ui32ArduinoTimestamp"),
                                             m_ddlSignalValueSteerId.timeStamp);
        adtf_ddl::access_element::find_index(m_SignalValueSteerSampleFactory, cString("f32Value"),
                                             m_ddlSignalValueSteerId.value);
    } else {
        LOG_INFO("No mediadescription for tSignalValueSteer found!");
    }

    // Register(m_oReaderSteer, "steering_input" , pTypeSignalValueSteer);
    Register(m_oWriterSteer, "steering_output", pTypeSignalValueSteer);


    object_ptr<IStreamType> pTypeLanePointData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("LanePointData",
                                                                                       pTypeLanePointData,
                                                                                       m_LanePointDataSampleFactory)) {
        adtf_ddl::access_element::find_index(m_LanePointDataSampleFactory, "nPoints", m_ddlLanePointDataId.nPoints);
        adtf_ddl::access_element::find_array_index(m_LanePointDataSampleFactory, "pointArray",
                                                   m_ddlLanePointDataId.pointArray);
    } else {
        LOG_INFO("No mediadescription for LanePointData found!");
    }

    Register(m_oReaderlane, "lane_points", pTypeLanePointData);


}


tResult cTrajectoryGen::Configure() {
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}


bool cTrajectoryGen::left_right(const GoalPoint &gP) {
    if (gP.x < 0) return 0;
    else return 1;
}


tResult cTrajectoryGen::TransmitSpeed(tSignalValue speedSignal) {
    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample, m_pClock->GetStreamTime())))
    {

        auto oCodec = m_SignalValueSpdSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueSpdId.timeStamp, speedSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueSpdId.value, speedSignal.f32Value));

         m_oWriterSpd << pWriteSample << flush << trigger;
    }


    RETURN_NOERROR;
}

tResult cTrajectoryGen::TransmitDir(tSignalValue steerSignal) {
    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample, m_pClock->GetStreamTime())))
    {
        auto oCodec = m_SignalValueSteerSampleFactory.MakeCodecFor(pWriteSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueSteerId.timeStamp, steerSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueSteerId.value, steerSignal.f32Value));

        m_oWriterSteer << pWriteSample << flush << trigger;
    }


    RETURN_NOERROR;
}


tResult cTrajectoryGen::trajectoryGen(tLanePoint *relativePoints, const int &nPoints) {

    tSignalValue speedSignal;
    tSignalValue steerSignal;

    if (nPoints <= 1)
    {
        if ( m_pClock->GetStreamTime() - lastUpdateTime < 300000)
        {

        }else{
            previousSteer = 0;
            previousSpeed = 0;
        }
        speedSignal.f32Value = previousSpeed;
        steerSignal.f32Value = previousSteer;
        speedSignal.ui32ArduinoTimestamp = static_cast<tUInt32>(lastUpdateTime);
        steerSignal.ui32ArduinoTimestamp = static_cast<tUInt32>(lastUpdateTime);
        TransmitDir(steerSignal);
        TransmitSpeed(speedSignal);

    } else {
        int xarr[nPoints];
        int yarr[nPoints];
        tSize emptyPoints = 0; // count if points are empty (i.e. 0)

        // Do the Processing
        for (int i = 0; i < nPoints; i++)
        {
            if (relativePoints[i].x == 0 && relativePoints[i].y == 0){
                ++emptyPoints;
            }

            xarr[i] = relativePoints[i].x;
            yarr[i] = relativePoints[i].y;
        }

        /* if there is only one or zero points (despite the count), return previous value */
        if (emptyPoints >= nPoints-1){
            speedSignal.f32Value = previousSpeed;
            steerSignal.f32Value = previousSteer;
            TransmitDir(steerSignal);
            TransmitSpeed(speedSignal);
            RETURN_NOERROR;
        }

        lin_spline path(xarr, yarr, nPoints);
        tFloat32 spd = previousSpeed;
        int k = 65;

        //IMPORTANT NOTE:::::: SPEED CALIBRATION OF CAR MUST BE: -10 in turns AND 15 MAX STRAIGHT LINE
        int lookahead_dist = static_cast<int>(k * spd-295);
//        int lookahead_dist = 450;
        if (lookahead_dist<200) lookahead_dist = 200;
        if (lookahead_dist>530) lookahead_dist = 530;

        path.create_spline();

        //calculate point of intersection based on lookahead distance
        //double y = 0;
        double dist_car = 0;
        double error = 5;
        bool found = false;

        for (auto point : path.spline_points)
        {
            dist_car = sqrt(pow(point[0],2) + pow(point[1],2) );

            if ((dist_car > lookahead_dist - error) && (dist_car < lookahead_dist + error))
            {
                gP.x = tFloat32(point[0]);
                gP.y = tFloat32(point[1]);
                found = true;
                break;
            }
        }

        if (!found)
        {
            gP.x = tFloat32(relativePoints[nPoints-1].x);
            gP.y = tFloat32(relativePoints[nPoints-1].y);
        }

        double gamma = 2*gP.x/( (pow(gP.x,2) + pow(gP.y,2)) );
        double steering_angle = atan(gamma*360)*180/3.14;
        double steering_output = (steering_angle/90)*100;
        int maxSpeed = 14.5;

        tFloat32 speed_output;
        //for speed of 30:0.28 pow, <25: 0.18
        double weighting = pow(abs(steering_output),0.175)/1.6;

        //double weighting = pow(abs(relativePoints[nPoints-1].x),0.5) / 8;
        if (weighting < 1) weighting = 1.0;
        speed_output = static_cast<tFloat32>(maxSpeed/weighting);

        // If very different steering from previous, take average
        if (abs(steering_output - steerSignal_old.f32Value)>30) {
            steering_output = (steering_output+steerSignal_old.f32Value)/2;
        }
        steerSignal.f32Value = static_cast<tFloat32>(steering_output);
        speedSignal.f32Value = static_cast<tFloat32>(speed_output);

        object_ptr<const ISample> pReadSpeedSample;
        object_ptr<const ISample> pReadSteerSample;

        static tTimeStamp speedTS = 0;
        static tTimeStamp steerTS = 0;

        if (IS_OK(m_oSpeedReader.GetLastSample(pReadSpeedSample))) {
            auto oDecoder = m_SignalValueSpdSampleFactory.MakeDecoderFor(*pReadSpeedSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            tTimeStamp ts;
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueSpdId.timeStamp, &ts));
            if (ts != speedTS){
                speedTS = ts;
                RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueSpdId.value, &speedSignal.f32Value));
            }
        }

        if (IS_OK(m_oSteerReader.GetLastSample(pReadSteerSample))) {
            auto oDecoder = m_SignalValueSpdSampleFactory.MakeDecoderFor(*pReadSteerSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            tTimeStamp ts;
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueSpdId.timeStamp, &ts));
            if (ts != steerTS){
                steerTS = ts;
                RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueSpdId.value, &steerSignal.f32Value));
            }
        }

        LOG_DUMP(cString::Format("steering: %f", steering_output));
        LOG_DUMP(cString::Format("speed: %f", speed_output));

        TransmitSpeed(speedSignal);
        TransmitDir(steerSignal);


        steerSignal_old = steerSignal;
        speedSignal_old = speedSignal;
        previousSteer = steerSignal.f32Value;
        previousSpeed = speedSignal.f32Value;
        lastUpdateTime = m_pClock->GetStreamTime();
    }
    RETURN_NOERROR;
}

tResult cTrajectoryGen::Process(tTimeStamp tmTimeOfTrigger) {
    object_ptr<const ISample> pReadPathSample;


    // changed to reading the last sample only from path
    if (IS_OK(m_oReaderlane.GetLastSample(pReadPathSample))) {
        auto oDecoder = m_LanePointDataSampleFactory.MakeDecoderFor(*pReadPathSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        int numOfLanePoints = 0;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLanePointDataId.nPoints, &numOfLanePoints));

        auto LanePoints_in = reinterpret_cast<const tLanePoint *>(oDecoder.GetElementAddress(
                m_ddlLanePointDataId.pointArray));

        /* TODO: clean up lanePoint mess */

        tLanePoint relativePoints[50] = {};
        tLanePoint point = {};

        for (tSize i = 0; i < numOfLanePoints; ++i) {
            point.x = LanePoints_in[i].x;
            point.y = LanePoints_in[i].y;
            relativePoints[i] = point;
        }

        trajectoryGen(relativePoints, numOfLanePoints);
    }

    RETURN_NOERROR;
}
