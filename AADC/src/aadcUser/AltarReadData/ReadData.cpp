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
#include "ReadData.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "AltarReadData",
    cReadData,
    adtf::filter::pin_trigger({"input"}));


cReadData::cReadData()
{
    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description

    object_ptr<IStreamType> pTypeMyData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tMyData", pTypeMyData, m_MyStructSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_MyStructSampleFactory, "ui32Size", m_ddlMyDataId.data1);
        adtf_ddl::access_element::find_index(m_MyStructSampleFactory, "tMyFloat32", m_ddlMyDataId.data2);
        adtf_ddl::access_element::find_index(m_MyStructSampleFactory, "tMyArray", m_ddlMyDataId.data3);

    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }


    LOG_INFO(cString::Format("INTIALIZE!!!!"));

    Register(MyDataIn, "input", pTypeMyData);
}


//implement the Configure function to read ALL Properties
tResult cReadData::Configure()
{
    RETURN_NOERROR;
}

tResult cReadData::Process(tTimeStamp tmTimeOfTrigger)
{

    object_ptr<const ISample> pReadSample;
    tUInt32 b;
    tFloat32 a;
    tMySimpleData z;



    if (IS_OK(MyDataIn.GetLastSample(pReadSample))) {
        auto oDecoder = m_MyStructSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlMyDataId.data1, &b));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlMyDataId.data2, &a));

        const tMySimpleData *pCoordiante = reinterpret_cast<const tMySimpleData *>(oDecoder.GetElementAddress(
                m_ddlMyDataId.data3));

        tMySimpleData myPoint;

        for (tSize i = 0; i < 6; ++i) {
            myPoint.f32num1 = pCoordiante[i].f32num1;
            myPoint.f32num2 = pCoordiante[i].f32num2;

            LOG_INFO(cString::Format("(%f, %f)", myPoint.f32num1, myPoint.f32num2));

        }
    }


    LOG_INFO(cString::Format("(%f, %f)", a,b));

    RETURN_NOERROR;
}