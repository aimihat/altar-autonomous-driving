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
#include "AltarPullOut.h"
#include "ADTF3_helper.h"
#include "aadc_structs.h"


#define CW_SLOT_COUNT 60.f
#define CW_ERROR_DIFFERENCE_SIDES 0.30f
#define CW_MIN_LIMIT_IGNORE 0.01f





ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "AltarPullOut",
    cPullOut,
    adtf::filter::pin_trigger({"action"}));


cPullOut::cPullOut()
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
        (adtf_ddl::access_element::find_index(m_SignalValueSpdSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueSpdId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSpdSampleFactory, cString("f32Value"), m_ddlSignalValueSpdId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValueSpd found!");
    }
    Register(m_oWriterSpd, "speed_out", pTypeSignalValueSpd);


    object_ptr<IStreamType> pTypeSignalValueSteer;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValueSteer, m_SignalValueSteerSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSteerSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueSteerId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSteerSampleFactory, cString("f32Value"), m_ddlSignalValueSteerId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValueSteer found!");
    }

    Register(m_oWriterSteer, "steer_out", pTypeSignalValueSteer);




}


//implement the Configure function to read ALL Properties
tResult cPullOut::Configure()
{
    RETURN_IF_FAILED(cTriggerFunction::Configure());
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));


    RETURN_NOERROR;
}

tResult cPullOut::Process(tTimeStamp tmTimeOfTrigger)
{

//    object_ptr<const ISample> pReadSample;
//
//    tFloat32 inputData;
//
//    if (IS_OK(m_oReader.GetLastSample(pReadSample)))
//    {
//        auto oDecoder = m_templateDataSampleFactory.MakeDecoderFor(*pReadSample);
//
//        RETURN_IF_FAILED(oDecoder.IsValid());
//
//        // retrieve the values (using convenience methods that return a variant)
//        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTemplateDataId.f32Value, &inputData));
//
//    }












    // Do the Processing
//    tFloat32 outputData = inputData * 0.001;
//
//    object_ptr<ISample> pWriteSample;
//
//    if (IS_OK(alloc_sample(pWriteSample)))
//    {
//
//        auto oCodec = m_templateDataSampleFactory.MakeCodecFor(pWriteSample);
//
//        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlTemplateDataId.f32Value, outputData));
//
//    }
//    m_oWriter << pWriteSample << flush << trigger;
    
    RETURN_NOERROR;
}



tResult cPullOut::UpdateSpeed()
{

    RETURN_NOERROR;
}



tResult cPullOut::TransmitSpeed(tSignalValue speedSignal)
{
    object_ptr<ISample> pWriteSample;

        RETURN_IF_FAILED(alloc_sample(pWriteSample))
        {

            auto oCodec = m_SignalValueSpdSampleFactory.MakeCodecFor(pWriteSample);

            RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueSpdId.timeStamp, speedSignal.ui32ArduinoTimestamp));

            RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueSpdId.value, speedSignal.f32Value));

        }

        m_oWriterSpd << pWriteSample << flush << trigger;
        RETURN_NOERROR;
}

tResult cPullOut::TransmitDir(tSignalValue steerSignal)
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
