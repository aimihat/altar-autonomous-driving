
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


/*********************************************************************
 * This code was provided by HERE
 *
 * *******************************************************************/

#define A_UTILS_NO_DEPRECATED_WARNING

#include <stdlib.h>
#include "Parking.h"
//#include <algorithm>
#include <string.h>
#include "stdafx.h"


#define CW_SLOT_COUNT 60.f
#define CW_ERROR_DIFFERENCE_SIDES 0.30f
#define CW_MIN_LIMIT_IGNORE 0.01f




/*! defines a data triggered filter and exposes it via a plugin class factory */
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CMARKERPOS_DATA_TRIGGERED_FILTER,
                                    "AltarParking",
                                    cParking,
                                    adtf::filter::pin_trigger({ "action", "car_position" }));

/*! initialize the trigger function */
cParking::cParking()
{

    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    //Register Properties
    RegisterPropertyVariable("wheel circumference [m]", m_f32wheelCircumference);
    RegisterPropertyVariable("filter constant of first order", m_f32FilterConstantfirstOrder);
    RegisterPropertyVariable("enable filtering", m_bEnableFiltering);


    object_ptr<IStreamType> pTypeWheelData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tWheelData", pTypeWheelData, m_WheelDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32ArduinoTimestamp", m_ddlWheelDataIndex.ArduinoTimestamp);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32WheelTach", m_ddlWheelDataIndex.WheelTach);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "i8WheelDir", m_ddlWheelDataIndex.WheelDir);
    }
    else
    {
        LOG_INFO("No mediadescription for tWheelData found!");
    }
    Register(m_oInputWheelLeft, "wheel_left", pTypeWheelData);
    Register(m_oInputWheelRight, "wheel_right", pTypeWheelData);


    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    Register(m_oReaderAction, "action", pTypeSignalValue);



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
    Register(m_oWriterSpeed, "speed", pTypeSignalValueSpd);


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
    Register(m_oWriterSteer, "steer", pTypeSignalValueSteer);




    //the position struct
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


    Register(m_oReaderCarPos, "car_position", pTypePositionData);


}



/*! implements the configure function to read ALL Properties */
tResult cParking::Configure()
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}


/*! funtion will be executed each time a trigger occured */
tResult cParking::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;

    tSignalValue actionSignal;

    if (IS_OK(m_oReaderAction.GetLastSample(pReadSample))) {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &actionSignal.f32Value));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &actionSignal.ui32ArduinoTimestamp));
    }






    tSignalValue speedSignal;
    tSignalValue steerSignal;


    if (actionSignal.f32Value == 1 || actionSignal.f32Value == 2){
        //Wheel Left
        object_ptr<const ISample> pSampleFromWheelLeft;

        if (IS_OK(m_oInputWheelLeft.GetLastSample(pSampleFromWheelLeft)))
        {
            // save the last struct to the struct beforeLast if it is not the first one
            if (m_bfirstSampleReceivedLeftWheel == tTrue)
            {
                m_tBeforeLastStructLeft = m_tLastStructLeft;
            }

            tUInt32 ui32Tach ;
            tInt8   i8Direction ;
            tUInt32 ui32Timestamp ;

            auto oDecoder = m_WheelDataSampleFactory.MakeDecoderFor(*pSampleFromWheelLeft);

            RETURN_IF_FAILED(oDecoder.IsValid());

            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &ui32Timestamp));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelTach, &ui32Tach));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelDir, &i8Direction));



            // if it is the first sample stop here and set to true
            if (m_bfirstSampleReceivedLeftWheel == tFalse)
            {
                m_bfirstSampleReceivedLeftWheel = tTrue;

                m_tLastStructLeft.i8WheelDir = i8Direction;
                m_tLastStructLeft.ui32ArduinoTimestamp = ui32Timestamp;
                m_tLastStructLeft.ui32WheelTach = ui32Tach;
            }
                // doing the calculation and the transmit
            else
            {
                // doing an minimal smoothing of the signal
                if (m_bEnableFiltering)
                    m_f32LastCalculatedSpeedLeft = m_f32LastCalculatedSpeedLeft +
                                                   m_f32FilterConstantfirstOrder * (calculateSpeed(ui32Timestamp, m_tLastStructLeft.ui32ArduinoTimestamp, ui32Tach - m_tLastStructLeft.ui32WheelTach)
                                                                                    - m_f32LastCalculatedSpeedLeft);
                else
                    m_f32LastCalculatedSpeedLeft = calculateSpeed(ui32Timestamp, m_tLastStructLeft.ui32ArduinoTimestamp, ui32Tach - m_tLastStructLeft.ui32WheelTach);

                m_tLastStructLeft.i8WheelDir = i8Direction;
                m_tLastStructLeft.ui32ArduinoTimestamp = ui32Timestamp;
                m_tLastStructLeft.ui32WheelTach = ui32Tach;
            }
        }



        //Wheel Right
        object_ptr<const ISample> pSampleFromWheelRight;

        if (IS_OK(m_oInputWheelRight.GetNextSample(pSampleFromWheelRight)))
        {
            // save the last struct to the struct beforeLast if it is not the first one
            if (m_bfirstSampleReceivedRightWheel == tTrue)
            {
                m_tBeforeLastStructRight = m_tLastStructRight;
            }

            tUInt32 ui32Tach = 0;
            tInt8 i8Direction = 0;
            tUInt32 ui32Timestamp = 0;

            {

                auto oDecoder = m_WheelDataSampleFactory.MakeDecoderFor(*pSampleFromWheelRight);

                RETURN_IF_FAILED(oDecoder.IsValid());

                RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &ui32Timestamp));
                RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelTach, &ui32Tach));
                RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelDir, &i8Direction));

            }


            // if it is the first sample stop here and set to true
            if (m_bfirstSampleReceivedRightWheel == tFalse)
            {
                m_bfirstSampleReceivedRightWheel = tTrue;

                m_tLastStructRight.i8WheelDir = i8Direction;
                m_tLastStructRight.ui32ArduinoTimestamp = ui32Timestamp;
                m_tLastStructRight.ui32WheelTach = ui32Tach;
            }
                // doing the calculation and the transmit
            else
            {
                // doing an minimal smoothing of the signal
                if (m_bEnableFiltering)
                    m_f32LastCalculatedSpeedRight = m_f32LastCalculatedSpeedRight + m_f32FilterConstantfirstOrder * (calculateSpeed(ui32Timestamp, m_tLastStructRight.ui32ArduinoTimestamp, ui32Tach - m_tLastStructRight.ui32WheelTach) - m_f32LastCalculatedSpeedRight);
                else
                    m_f32LastCalculatedSpeedRight = calculateSpeed(ui32Timestamp, m_tLastStructRight.ui32ArduinoTimestamp, ui32Tach - m_tLastStructRight.ui32WheelTach);

                m_tLastStructRight.i8WheelDir = i8Direction;
                m_tLastStructRight.ui32ArduinoTimestamp = ui32Timestamp;
                m_tLastStructRight.ui32WheelTach = ui32Tach;

                //TransmitSamples();
            }
        }

        UpdateDistance();
        LOG_INFO("Distance: %f",m_f32OverallDistance);
    }



    if (actionSignal.f32Value == 3){

        object_ptr<const ISample> pReadSampleCar;
        //LOG_INFO(cString::Format("process: %lu", tmTimeOfTrigger).GetPtr());

        if (IS_OK(m_oReaderCarPos.GetLastSample(pReadSampleCar))) {
            // store speed
            auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pReadSampleCar);

            oDecoder.GetElementValue(m_ddlPositionIndex.x, &car_pos.f32x);
            oDecoder.GetElementValue(m_ddlPositionIndex.y, &car_pos.f32y);
            oDecoder.GetElementValue(m_ddlPositionIndex.heading, &car_pos.f32heading);

            //LOG_INFO(cString::Format("process: %f", m_f32Speed));

        }

        //    object_ptr<const ISample> pReadSamplePark;
        //    tPosition park_pos;
        //    //LOG_INFO(cString::Format("process: %lu", tmTimeOfTrigger).GetPtr());
        //
        //    if (IS_OK(m_oReaderCarPos.GetNextSample(pReadSamplePark)))
        //    {
        //        // store speed
        //        auto oDecoder = m_SignalDataSampleFactory.MakeDecoderFor(*pReadSamplePark);
        //
        //        oDecoder.GetElementValue(m_ddlPositionIndex.x, &park_pos.f32x);
        //        oDecoder.GetElementValue(m_ddlPositionIndex.y, &park_pos.f32y);
        //        oDecoder.GetElementValue(m_ddlPositionIndex.x, &park_pos.f32x);
        //
        //        //LOG_INFO(cString::Format("process: %f", m_f32Speed));
        //
        //    }
        if (car_pos.f32heading < 0)
            car_pos.f32heading = 2 * PI + car_pos.f32heading;

        if (abs(car_pos.f32x) < 100) {
            car_pos.f32x = car_pos.f32x * 1000;
            car_pos.f32y = car_pos.f32y * 1000;
        }


        tPosition park_pos;
        park_pos.f32x = 600;
        park_pos.f32y = 1230;
        //    park_pos.f32y = 2760;

        tSignalValue speedSignal;
        tSignalValue steerSignal;


        if (manoeuvre_fwd) {

            LOG_INFO("car heading: %f", car_pos.f32heading);

            LOG_INFO(cString::Format("initial heading: %f", initial_pos.f32heading));

        }

        if (!reach_park) {

            //        if(Dist_to_point_front(park_pos) < 50)
            if (true) {
                reach_park = true;
                speedSignal.f32Value = 0;
                steerSignal.f32Value = 0;
                TransmitSpeed(speedSignal);
                TransmitDir(steerSignal);


            }
        } else if (!manoeuvre_fwd && abs(car_pos.f32x) < 100000 && abs(car_pos.f32y) < 100000 &&
                   abs(car_pos.f32x) > 2 && abs(car_pos.f32y) > 2) {
            initial_pos = car_pos;
            if (initial_pos.f32heading < 0)
                initial_pos.f32heading = 2 * PI + initial_pos.f32heading;


            fwd_manoeuvre_point.f32x = car_pos.f32x + 470 * sin(car_pos.f32heading) + 460 * cos(car_pos.f32heading) -
                                       580 * sin(car_pos.f32heading);
            fwd_manoeuvre_point.f32y = car_pos.f32y + 470 * cos(car_pos.f32heading) + 460 * sin(car_pos.f32heading) +
                                       580 * cos(car_pos.f32heading);


            parking_done_point.f32x = car_pos.f32x + 470 * cos(car_pos.f32heading) + 420 * sin(car_pos.f32heading);
            parking_done_point.f32y = car_pos.f32y + 470 * sin(car_pos.f32heading) + 420 * cos(car_pos.f32heading);
            steerSignal.f32Value = -100;
            speedSignal.f32Value = 13;
            TransmitSpeed(speedSignal);
            TransmitDir(steerSignal);

            manoeuvre_fwd = true;
        } else if (!manoeuvre_back) {
            if ((car_pos.f32heading > fmod(initial_pos.f32heading + 0.92 * PI / 3 - 0.05, 2 * PI)) &&
                (car_pos.f32heading < fmod(initial_pos.f32heading + 0.92 * PI / 3 + 0.05, 2 * PI))) {
                manoeuvre_back = true;
                steerSignal.f32Value = 100;
                speedSignal.f32Value = -13;
                TransmitSpeed(speedSignal);
                TransmitDir(steerSignal);
            }
        } else if (!parking_done) {
            if ((car_pos.f32heading > fmod(initial_pos.f32heading + 0.96 * PI / 2 - 0.05, 2 * PI)) &&
                (car_pos.f32heading < fmod(initial_pos.f32heading + 0.96 * PI / 2 + 0.05, 2 * PI))) {
                parking_done = true;
                steerSignal.f32Value = 0;
                speedSignal.f32Value = 0;
                TransmitSpeed(speedSignal);
                TransmitDir(steerSignal);
            }
        }
    }else if (actionSignal.f32Value == 1) {
        if (!PullOut_started) {
            PullOut_started = true;
            speedSignal.f32Value = 113;
            steerSignal.f32Value = 0;
            TransmitSpeed(speedSignal);
            TransmitDir(steerSignal);
        }
        if (m_f32OverallDistance > 0.2) {
            speedSignal.f32Value = 13;
            steerSignal.f32Value = 100;

            TransmitSpeed(speedSignal);
            TransmitDir(steerSignal);
        }
        if (m_f32OverallDistance > 0.3) {
            speedSignal.f32Value = 0;
            steerSignal.f32Value = 0;

            TransmitSpeed(speedSignal);
            TransmitDir(steerSignal);
        }


    }else if (actionSignal.f32Value == 2){
        if (!PullOut_started) {
            PullOut_started = true;
            speedSignal.f32Value = 13;
            steerSignal.f32Value = 0;
            TransmitSpeed(speedSignal);
            TransmitDir(steerSignal);
        }
        if (m_f32OverallDistance > 0.3) {
            speedSignal.f32Value = 0;
            steerSignal.f32Value = -100;

            TransmitSpeed(speedSignal);
            TransmitDir(steerSignal);
        }
        if (m_f32OverallDistance > 0.4) {
            speedSignal.f32Value = 0;
            steerSignal.f32Value = 0;

            TransmitSpeed(speedSignal);
            TransmitDir(steerSignal);
        }
    }else if (actionSignal.f32Value == 0){
        PullOut_started = false;
        m_f32OverallDistance = 0;
        reach_park, manoeuvre_fwd, manoeuvre_back, parking_done = false;

    }




    RETURN_NOERROR;
}



tFloat32 cParking::Dist_to_point_front(tPosition point) {
    tFloat32 car_front_x = car_pos.f32x + 470 * cos(car_pos.f32heading);
    tFloat32 car_front_y = car_pos.f32y + 470 * sin(car_pos.f32heading);

    return sqrt(pow(car_front_x - point.f32x, 2) + pow(car_front_y - point.f32y, 2));
}

tFloat32 cParking::Dist_to_point_rear(tPosition point) {
    tFloat32 car_rear_x = car_pos.f32x - 105 * cos(car_pos.f32heading);
    tFloat32 car_rear_y = car_pos.f32y - 105 * sin(car_pos.f32heading);

    return sqrt(pow(car_rear_x - point.f32x, 2) + pow(car_rear_y - point.f32y, 2));
}

tFloat32 cParking::calculateSpeed(const tUInt32 &ui32CurrentTimeStamp, const tUInt32 &ui32LastTimeStamp, const tUInt32 &ui32Ticks)
{
    // return if time difference is 0, if time difference is smaller than 0, if ticks are 0 or smaller 0
    if ((ui32CurrentTimeStamp - ui32LastTimeStamp == 0) || (ui32Ticks == 0)) return 0;
    //          circumference      SlotsInTimeDiff
    // speed =  -------------- *  -------------
    //           TotalSlots*          TimeDiff
    return (m_f32wheelCircumference / CW_SLOT_COUNT * static_cast<tFloat32>(ui32Ticks)) /
           (static_cast<tFloat32>(ui32CurrentTimeStamp - ui32LastTimeStamp) / static_cast<tFloat32>(1e6));
}

tResult cParking::UpdateDistance() {
    // static variable for warning outputs to console
    static tInt32 i32WarningCounter = 0;

    // calculate the average of both wheel speeds
    tFloat32 f32speed = (m_f32LastCalculatedSpeedRight + m_f32LastCalculatedSpeedLeft) / 2;
    LOG_INFO("Speed of car: %f" ,f32speed);


    if (fabs((m_f32LastCalculatedSpeedRight - m_f32LastCalculatedSpeedLeft)) >
        fabs(m_f32LastCalculatedSpeedRight) * CW_ERROR_DIFFERENCE_SIDES) {
        if (m_f32LastCalculatedSpeedRight < CW_MIN_LIMIT_IGNORE) {
            f32speed = m_f32LastCalculatedSpeedLeft;
            if (m_tLastStructLeft.i8WheelDir == 1)
                f32speed = f32speed * -1;
        } else if (m_f32LastCalculatedSpeedLeft < CW_MIN_LIMIT_IGNORE) {
            f32speed = m_f32LastCalculatedSpeedRight;
            if (m_tLastStructRight.i8WheelDir == 1)
                f32speed = f32speed * -1;
        }
        i32WarningCounter++;
        if (i32WarningCounter % 200 == 0)
            LOG_WARNING(cString::Format(
                    "Wheel speed from left and right side are very different. Please check cables and connections! Right: %f, Left: %f, Result: %f",
                    m_f32LastCalculatedSpeedRight, m_f32LastCalculatedSpeedLeft, f32speed));
    } else {
        // if direction is backwards speed should be negative
        if (m_tLastStructLeft.i8WheelDir == 1 && m_tLastStructRight.i8WheelDir == 1)
            f32speed = f32speed * -1;
    }
    // distance since last sample
    tFloat32 f32distance = 0;

    // calculate the overall distance
    // if the speed is negative (car is going backward, distance is decreasing)
    if (m_tLastStructLeft.ui32ArduinoTimestamp != 0 && m_tLastStructRight.ui32ArduinoTimestamp != 0 &&
        m_tBeforeLastStructRight.ui32ArduinoTimestamp != 0 && m_tBeforeLastStructLeft.ui32ArduinoTimestamp != 0) {
        //                (TimeDiffLeft + TimeDiffRight)
        //   distance =   ------------------------------   *  speed
        //                            2
        f32distance = ((m_tLastStructLeft.ui32ArduinoTimestamp - m_tBeforeLastStructLeft.ui32ArduinoTimestamp) +
                       (m_tLastStructRight.ui32ArduinoTimestamp - m_tBeforeLastStructRight.ui32ArduinoTimestamp))
                      / (2 * static_cast<tFloat32>(1e6)) * fabs(f32speed);
        m_f32OverallDistance = m_f32OverallDistance + f32distance;
    }




    RETURN_NOERROR;
}

tResult cParking::TransmitSpeed(tSignalValue speedSignal)
{
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {

        auto oCodec = m_SignalValueSpdSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueSpdId.timeStamp, speedSignal.ui32ArduinoTimestamp));

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueSpdId.value, speedSignal.f32Value));

    }

    m_oWriterSpeed << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cParking::TransmitDir(tSignalValue steerSignal)
{
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {

        auto oCodec = m_SignalValueSteerSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueSteerId.timeStamp, steerSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueSteerId.value, steerSignal.f32Value));

    }

    m_oWriterSteer << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}
