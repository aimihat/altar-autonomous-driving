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


#define CID_CMARKERPOS_DATA_TRIGGERED_FILTER "AltarParking.filter.user.aadc.cid"
using namespace adtf_util;
//using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define RAD2DEG 180/PI
#define DEG2RAD PI/180


/*! the main class of the marker positioning. */
class cParking : public cTriggerFunction
{

public:

    /*! Default constructor. */
    cParking();

    /*! Destructor. */
    virtual ~cParking() = default;

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
    tPosition car_pos;

private:


    bool PullOut_started = false;

    /*! holds the overall distance since starting the adtf config*/
    tFloat32 m_f32OverallDistance = 0.0f;

    /*! the wheel circumference in meter */
    property_variable<tFloat32> m_f32wheelCircumference = 0.34f;

    /*! the filter constant of first order */
    property_variable<tFloat32> m_f32FilterConstantfirstOrder = 0.3f;

    /*! enables or filtering enabled */
    property_variable<tBool> m_bEnableFiltering = tFalse;

    bool reach_park, manoeuvre_fwd, manoeuvre_back, parking_done = false;

    /*! Reader of an InPin speed. */
    cPinWriter m_oWriter;

    cPinWriter m_oWriterSpeed;
    cPinWriter m_oWriterSteer;

    cPinReader m_oReaderCarPos;
    cPinReader m_oReaderAction;


    tPosition initial_pos;
    tPosition fwd_manoeuvre_point;
    tPosition parking_done_point;


    struct tWheelDataId
    {
        tSize ArduinoTimestamp;
        tSize WheelTach;
        tSize WheelDir;
    } m_ddlWheelDataIndex;

    /*! The wheel data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_WheelDataSampleFactory;

    /*! the timestamp of the last left wheel struct */
    tWheelData m_tLastStructLeft;

    /*! the timestamp of the last right wheel struct */
    tWheelData m_tLastStructRight;

    /*! the timestamp of the last left wheel struct */
    tWheelData m_tBeforeLastStructLeft;

    /*! the timestamp of the last right wheel struct */
    tWheelData m_tBeforeLastStructRight;

    /*! the last received speed controller value */
    tSignalValue m_tLastSpeedControllerValue;

    /*! the last calculated speed of right wheel */
    tFloat32 m_f32LastCalculatedSpeedRight = 0;

    /*! the last calculated speed of left wheel */
    tFloat32 m_f32LastCalculatedSpeedLeft = 0;


    /*! first sample was received from left wheel */
    tBool m_bfirstSampleReceivedLeftWheel = tFalse;
    /*! first sample was received from right wheel */
    tBool m_bfirstSampleReceivedRightWheel = tFalse;



    /*! The codec factory */
//    cSampleCodecFactory m_oCodecFactory;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;




    struct tBoolSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlBoolSignalValueId;


    /*! The template data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;

    cPinReader m_oReader;


    /*! The ddl indices for a tSignalValue */
    struct
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId, m_ddlSignalValueSpdId, m_ddlSignalValueSteerId;

    /*! The signal data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalDataSampleFactory;

    /*! The ddl indices for a tPosition */
    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionIndex;


    adtf::mediadescription::cSampleCodecFactory m_SignalValueSpdSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSteerSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;


    //Pins
    /*! Input Pin for wheel struct*/
    cPinReader      m_oInputWheelLeft;
    /*! Input Pin for wheel struct*/
    cPinReader      m_oInputWheelRight;


    /*! The position sample factory */
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;


    tFloat32 Dist_to_point_front(tPosition point);
    tFloat32 Dist_to_point_rear(tPosition point);

    tResult TransmitDir(tSignalValue steerSignal);
    tResult TransmitSpeed(tSignalValue speedSignal);

    tResult UpdateSpeed();
    tResult UpdateDistance();
    tFloat32 calculateSpeed(const tUInt32 &ui32CurrentTimeStamp, const tUInt32 &ui32LastTimeStamp, const tUInt32 &ui32Ticks);



};
