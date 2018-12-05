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
#include <aadc_structs.h>
#include <unistd.h>
#include "EmergencyBreak.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
                                    "EmergencyBreak",
                                    cEmergencyBreak,
                                    adtf::filter::pin_trigger({"laser_scanner"}));


cEmergencyBreak::cEmergencyBreak():m_doEmergencyBreak(tFalse)
{
    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.timeStamp);
        adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue"), m_ddlBoolSignalValueId.value);
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    Register(m_oWriterBool, "output", pTypeBoolSignalValue);


    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp);
        adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }

    Register(m_oReaderSpeed, "speed" , pTypeSignalValue);
    Register(m_oWriter, "speed_out" , pTypeSignalValue);



    object_ptr<IStreamType> pTypeLSData;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pTypeLSData, m_LSStructSampleFactory))
    {
        //// find the indices of the element for faster access in the process method
        LOG_INFO("Found mediadescription for tLaserScannerData!");
        adtf_ddl::access_element::find_index(m_LSStructSampleFactory, "ui32Size", m_ddlLSDataId.size);
        adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray);
    }
    else
    {
        LOG_INFO("No mediadescription for tLaserScannerData found!");
    }
    Register(m_oInputLaserScanner, "laser_scanner", pTypeLSData);






    //register properties
    RegisterPropertyVariable("Braking distance square coefficient", m_propBrakeCoSq);
    RegisterPropertyVariable("Braking distance linear coefficient", m_propBrakeCoLi);
    RegisterPropertyVariable("Braking distance offset", m_propBrakeOff);


    RegisterPropertyVariable("Min distance to object [mm]", m_propMinObstacleDistance);

}


//implement the Configure function to read ALL Properties
tResult cEmergencyBreak::Configure()
{
    RETURN_NOERROR;
}

tResult cEmergencyBreak::Process(tTimeStamp tmTimeOfTrigger) {


    object_ptr<const ISample> pReadSampleLS;
    std::vector<tPolarCoordiante> scan;



    if(IS_OK(m_oInputLaserScanner.GetLastSample(pReadSampleLS)))
    {
        auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pReadSampleLS);

        RETURN_IF_FAILED(oDecoder.IsValid());
        tSize numOfScanPoints = 0;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLSDataId.size, &numOfScanPoints));

        const tPolarCoordiante* pCoordinates = reinterpret_cast<const tPolarCoordiante*>(oDecoder.GetElementAddress(m_ddlLSDataId.scanArray));

        tPolarCoordiante scanPoint;

        for (tSize i = 0; i < numOfScanPoints; ++i)
        {
            scanPoint.f32Radius = pCoordinates[i].f32Radius;
            scanPoint.f32Angle = pCoordinates[i].f32Angle;
            scan.push_back(scanPoint);
        }

        //init with some max value

    }



    object_ptr<const ISample> pReadSample;

    tSignalValue speedSignal;
    tBoolSignalValue emergencySignal;

    if (IS_OK(m_oReaderSpeed.GetLastSample(pReadSample))) {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &speedSignal.f32Value));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &speedSignal.ui32ArduinoTimestamp));
        drivenSpeed = speedSignal.f32Value;


        DrivePossible(drivenSpeed, scan, speedSignal.f32Value);

        if (m_doEmergencyBreak) {
            speedSignal.f32Value = 0;
        }
        emergencySignal.bValue = m_doEmergencyBreak;


        RETURN_IF_FAILED(TransmitSpeed(speedSignal));


        RETURN_IF_FAILED(TransmitEmergency(emergencySignal));
    }

    RETURN_NOERROR;
}

void cEmergencyBreak::CheckEmergencyBreak(std::vector<tPolarCoordiante>& scan, const tFloat32 &speed)
{
    tPolarCoordiante closestObstacle;
    closestObstacle.f32Angle = 0.0;
    closestObstacle.f32Radius = 999999999.9;
    int minFoVAngle, maxFoVAngle;
    if (speed < 20) {minFoVAngle = 45; maxFoVAngle = 315;}
    else if (speed < 25) {minFoVAngle = 30; maxFoVAngle = 330;}
    else if (speed < 30) {minFoVAngle = 20; maxFoVAngle = 340;}
    else {minFoVAngle = 15; maxFoVAngle = 345;}

    for (auto element : scan)
    {
        if (element.f32Angle <= tFloat32(minFoVAngle) || element.f32Angle >= tFloat32(maxFoVAngle))
        {
            if (element.f32Radius != 0.0 && closestObstacle.f32Radius > element.f32Radius)
            {
                closestObstacle = element;
            }
        }
    }



    //m_doEmergencyBreak = closestObstacle.f32Radius < tFloat32(m_propMinObstacleDistance);
    m_doEmergencyBreak = closestObstacle.f32Radius < BrakingDistance(speed) || closestObstacle.f32Radius>20000;


    LOG_INFO(cString::Format("%s obstacle found, closest is at %f deg with %f mm dist, FoV (max/min) (%f/%f)",
                             (m_doEmergencyBreak?"  ":"no"),closestObstacle.f32Angle,closestObstacle.f32Radius,
                             tFloat32(maxFoVAngle),tFloat32(minFoVAngle)));

}




tFloat32 cEmergencyBreak::BrakingDistance(const tFloat32 &speed) {
    if (FLEX_BRAKE) {
        // braking formula is empirical tested and developed
        // made some tests with the braking distance of the model car
        // calculated trend formula from the values
        // formula returns value, which is greater than the real braking distance to avoid collisions
        tFloat32 dist;



        if (speed < 16) dist = 200;
        else if (speed < 19) dist = 400;
        else if (speed < 35) dist = (m_propBrakeCoSq * (speed * speed) + m_propBrakeCoLi * std::abs(speed) + m_propBrakeOff);
        else dist = 1500;
        LOG_INFO("Braking distance: %f", dist);
        return dist;

    } else {
        return minFixedLsPropValue;
    }
}


tResult cEmergencyBreak::TransmitEmergency(tBoolSignalValue emergencySignal)
{
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {

        auto oCodec = m_BoolSignalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.timeStamp, emergencySignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.value, emergencySignal.bValue));

    }

    m_oWriterBool << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cEmergencyBreak::TransmitSpeed(tSignalValue speedSignal)
{
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {

        auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, speedSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, speedSignal.f32Value));

    }

    m_oWriter << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

void cEmergencyBreak::DrivePossible(const tFloat32 &drivenSpeed,
                                    std::vector<tPolarCoordiante>& scan,
                                    const tFloat32 &wantedSpeed) {

//    tFloat32 brakingDistance = EmergencyStop::BrakingDistance(drivenSpeed.f32Value);
//        check if driving forward
    if (drivenSpeed >= 0) {
        CheckEmergencyBreak( scan, drivenSpeed);//        check if driving backward
    } else if (drivenSpeed < -0.01) { //drive backward
        m_doEmergencyBreak = false;
//        return BackFree(ultrasonicStruct, brakingDistance);
//        check if standing
    } else {
//          check if want to drive forward
        if (wantedSpeed > 0.01) {
            CheckEmergencyBreak( scan, wantedSpeed);//        check if driving backward
//                or want to drive backward
        } else if (wantedSpeed < 0.01) {
//            return BackFree(ultrasonicStruct, brakingDistance);
        } else {
//                if car is standing and it should remain standing
            return;
        }
    }

}
