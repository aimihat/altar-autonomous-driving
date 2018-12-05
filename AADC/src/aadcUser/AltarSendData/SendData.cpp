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
#include "SendData.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "AltarSendData",
    cSendData, adtf::filter::timer_trigger(1000000/4));

const tChar* strMyStructDescription =
        "<struct alignment=\"4\" name=\"tSimple\" version=\"2\">"
        "<element name=\"fValue\" type=\"tFloat32\" arraysize=\"1\">"
        "<deserialized alignment=\"4\"/>"
        "<serialized byteorder=\"LE\" bytepos=\"0\"/>"
        "</element>"
        "</struct>";



cSendData::cSendData()
{

    // Position Data
    object_ptr<IStreamType> pTypeSignalData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalData, m_SignalDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_SignalDataSampleFactory, "ui32ArduinoTimestamp", m_ddlSignalDataIndex.timeStamp);
        adtf_ddl::access_element::find_index(m_SignalDataSampleFactory, "f32Value", m_ddlSignalDataIndex.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }

    Register(PoseDataIn, "speed" , pTypeSignalData);



    object_ptr<IStreamType> pTypeMyData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tMyData", pTypeMyData, m_MyStructSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_MyStructSampleFactory, "ui32Size", m_ddlMyDataId.data1);
        adtf_ddl::access_element::find_index(m_MyStructSampleFactory, "tMyFloat32", m_ddlMyDataId.data2);
        adtf_ddl::access_element::find_array_index(m_MyStructSampleFactory, "tMyArray", m_ddlMyDataId.data3);

    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }


    LOG_INFO(cString::Format("INTIALIZE!!!!"));

    Register(MyDataOut, "output", pTypeMyData);


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


//implement the Configure function to read ALL Properties
tResult cSendData::Configure()
{
    RETURN_NOERROR;
}

tResult cSendData::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    tFloat32 f32x, f32y, f32heading;

    f32x = 0;
    if (IS_OK(PoseDataIn.GetLastSample(pReadSample)))
    {
        auto oDecoder = m_SignalDataSampleFactory.MakeDecoderFor(pReadSample);
        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalDataIndex.value, &f32x));

        //LOG_INFO(cString::Format("Speed: (%f)", f32x));

    }
    //LOG_INFO(cString::Format("Process"));


    object_ptr<ISample> pSample;

    RETURN_IF_FAILED(alloc_sample(pSample))
    {
        {

            tUInt32 c = 1;
            tFloat32 g = 2.0;
            auto oCodec = m_MyStructSampleFactory.MakeCodecFor(pSample);
            RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlMyDataId.data1, c));
            RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlMyDataId.data2, g));
            //LOG_INFO(cString::Format("Hello World"));



            tMySimpleData* points = reinterpret_cast<tMySimpleData*>(oCodec.GetElementAddress(m_ddlMyDataId.data3));

            //init array with zeros
                    int count = 5;
            memset(points, 0, count * sizeof(tMySimpleData));

            // Points array has 5 slots
            for (tUInt32 i = 0; i < count; i++)
            {
                points[i].f32num1 = 0;
                points[i].f32num2 = f32x;
            }




        }
    }

    MyDataOut << pSample << flush << trigger;
    

    // CUT HERE

    

    object_ptr<ISample> pLineSample;

    RETURN_IF_FAILED(alloc_sample(pLineSample))
    {
        {

            auto oCodec = m_LineStructSampleFactory.MakeCodecFor(pLineSample);

            RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlLineDataId.linesFound, 4));

            tLineCoordiante* points = reinterpret_cast<tLineCoordiante*>(oCodec.GetElementAddress(m_ddlLineDataId.lineArray));

            //init array with zero

            //memset(points, 0, count * sizeof(tFloat32));

            points[0].f64Distance = -0.22 * 1000;
            points[0].f64FrontDistance = 0.1 * 1000;
            points[0].f32RoadId = 2;

            points[1].f64Distance = 0.22 * 1000;
            points[1].f64FrontDistance = 0.1 * 1000;
            points[1].f32RoadId = 3;

            points[2].f64Distance = -0.22 * 1000;
            points[2].f64FrontDistance = 0.2 * 1000;
            points[2].f32RoadId = 2;

            points[3].f64Distance = 0.22 * 1000;
            points[3].f64FrontDistance = 0.2 * 1000;
            points[3].f32RoadId = 3;


        }
    }

    LineDataOut << pLineSample << flush << trigger;



    RETURN_NOERROR;
}
