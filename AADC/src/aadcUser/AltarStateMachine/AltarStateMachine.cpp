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

#include "AltarStateMachine.h"
#include "DecisionMaker.h"

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ALTAR_STATE_MACHINE_DATA_TRIGGERED_FILTER,
    "Altar State Machine",
    cAltarStateMachine,
    adtf::filter::timer_trigger(10));


using namespace aadc::jury;

// Define before constructing
struct cAltarStateMachine::decisionMaker : public boost::sml::sm<decisionMakerImpl, sml::process_queue<std::queue>> {
    explicit decisionMaker(cAltarStateMachine& parent_)
            : boost::sml::sm<decisionMakerImpl, sml::process_queue<std::queue>>(static_cast<cAltarStateMachine&>(parent_)) {}
};

cAltarStateMachine::cAltarStateMachine()
{

    RegisterStateStruct();
    RegisterManeuverStruct();
    RegisterSignalStruct();
    RegisterBoolSignal();
    RegisterLaneData();
    RegisterWheelStruct();
    RegisterParkingStruct();

    m_lastWheelSpeed.fill(0);
    m_pDM = std::make_shared<decisionMaker>(*this);
}

tResult cAltarStateMachine::RegisterManeuverStruct() {

    object_ptr<IStreamType> pTypeManeuverData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tManeuver", pTypeManeuverData, m_maneuverDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_maneuverDataSampleFactory, cString("id"), m_ddlManeuverStruct.id);
        adtf_ddl::access_element::find_index(m_maneuverDataSampleFactory, cString("maneuver"), m_ddlManeuverStruct.action);
        adtf_ddl::access_element::find_index(m_maneuverDataSampleFactory, cString("extra"), m_ddlManeuverStruct.extra);
    }
    else
    {
        LOG_WARNING("No mediadescription for tManeuver found!");
    }

    Register(m_oManeuverReader, "jury_maneuver_input", ucom_object_ptr_cast<const IStreamType>(pTypeManeuverData));

    RETURN_NOERROR;
}


tResult cAltarStateMachine::RegisterStateStruct() {

    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeJuryStateData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tDriverStruct", pTypeJuryStateData, m_juryStateDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_juryStateDataSampleFactory, cString("i16StateID"), m_ddlDriverStructId.stateId);
        adtf_ddl::access_element::find_index(m_juryStateDataSampleFactory, cString("i16ManeuverEntry"), m_ddlDriverStructId.maneuverEntry);
    }
    else
    {
        LOG_WARNING("No mediadescription for tDriverModule found!");
    }

    Register(m_oJuryStateReader, "jury_state_input" , ucom_object_ptr_cast<const IStreamType>(pTypeJuryStateData));
    Register(m_oDriveStateWriter, "drive_state_output", ucom_object_ptr_cast<const IStreamType>(pTypeJuryStateData));

    RETURN_NOERROR;

}

tResult cAltarStateMachine::RegisterSignalStruct() {
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue,
                                                                                       m_signalValueSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_signalValueSampleFactory, cString("ui32ArduinoTimestamp"),
                                             m_ddlSignalValueId.timeStamp);
        adtf_ddl::access_element::find_index(m_signalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }

    Register(m_oSpeedWriter, "speed_output", ucom_object_ptr_cast<const IStreamType>(pTypeSignalValue));
    Register(m_oSteerWriter, "steer_output", ucom_object_ptr_cast<const IStreamType>(pTypeSignalValue));
    Register(m_oMapSituationReader, "map_situation", ucom_object_ptr_cast<const IStreamType>(pTypeSignalValue));
    RETURN_NOERROR;

}

tResult cAltarStateMachine::RegisterBoolSignal() {

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue",
                                                                                       pTypeBoolSignalValue,
                                                                                       m_boolSignalDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_boolSignalDataSampleFactory, cString("ui32ArduinoTimestamp"),
                                             m_ddlBoolSignalValueId.timeStamp);
        adtf_ddl::access_element::find_index(m_boolSignalDataSampleFactory, cString("bValue"),
                                             m_ddlBoolSignalValueId.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tBoolSignalValue found!");
    }

    Register(m_oEmergencyBrakeReader, "emergency_break", ucom_object_ptr_cast<const IStreamType>(pTypeBoolSignalValue));
    Register(m_oCrossingReader, "crossing_in", ucom_object_ptr_cast<const IStreamType>(pTypeBoolSignalValue));

    Register(m_oBrakeLightWriter, "brake_light", ucom_object_ptr_cast<const IStreamType>(pTypeBoolSignalValue));
    Register(m_oLeftLightWriter, "left_light", ucom_object_ptr_cast<const IStreamType>(pTypeBoolSignalValue));
    Register(m_oRightLightWriter, "right_light", ucom_object_ptr_cast<const IStreamType>(pTypeBoolSignalValue));
    Register(m_oHazardLightWriter, "hazard_light", ucom_object_ptr_cast<const IStreamType>(pTypeBoolSignalValue));
    Register(m_oHeadLightWriter, "head_light", ucom_object_ptr_cast<const IStreamType>(pTypeBoolSignalValue));
    Register(m_oReverseLightWriter, "reverse_light", ucom_object_ptr_cast<const IStreamType>(pTypeBoolSignalValue));

    RETURN_NOERROR;

}

tResult cAltarStateMachine::RegisterLaneData() {

    object_ptr<IStreamType> pTypeLanePointData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("LanePointData",pTypeLanePointData,m_LanePointDataSampleFactory)) {
        adtf_ddl::access_element::find_index(m_LanePointDataSampleFactory, cString("nPoints"), m_ddlLanePointDataId.nPoints);
        adtf_ddl::access_element::find_array_index(m_LanePointDataSampleFactory, cString("pointArray"),m_ddlLanePointDataId.pointArray);
    } else {
        LOG_INFO("No mediadescription for LanePointData found!");
    }

    Register(m_oMapPathReader, "map_path", ucom_object_ptr_cast<const IStreamType>(pTypeLanePointData));
    Register(m_oLaneReader, "lane_follow", ucom_object_ptr_cast<const IStreamType>(pTypeLanePointData));
    Register(m_oPathWriter, "path_output", ucom_object_ptr_cast<const IStreamType>(pTypeLanePointData));

    RETURN_NOERROR;

}

tResult cAltarStateMachine::RegisterWheelStruct() {

    object_ptr<IStreamType> pTypeWheelData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeWheelData, m_wheelDataSampleFactory))
    {
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_wheelDataSampleFactory, "ui32ArduinoTimestamp", m_ddlSignalValueId.timeStamp));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_wheelDataSampleFactory, "f32Value", m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }

    Register(m_oWheelReader, "wheel_speed", pTypeWheelData);

    RETURN_NOERROR;
}

tResult cAltarStateMachine::RegisterParkingStruct() {

    object_ptr<IStreamType> pTypeParkingData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tParking", pTypeParkingData, m_parkingDataSampleFactory))
    {
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_parkingDataSampleFactory, "status", m_ddlParkingValueId.status));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_parkingDataSampleFactory, "speed", m_ddlParkingValueId.speed));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_parkingDataSampleFactory, "steering", m_ddlParkingValueId.steering));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_struct_index(m_parkingDataSampleFactory, "parkPath", m_ddlParkingValueId.pathArray));
    }
    else
    {
        LOG_INFO("No mediadescription for tParking found!");
    }


    Register(m_oParkingReader, "parking_indicator", ucom_object_ptr_cast<const IStreamType>(pTypeParkingData));
    RETURN_NOERROR;
}

//implement the Configure function to read ALL Properties
tResult cAltarStateMachine::Configure()
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cAltarStateMachine::Process(tTimeStamp tmTimeOfTrigger)
{

    RETURN_IF_FAILED(ProcessJuryInput());

    if (m_driverStruct.i16StateID == statecar_running) {


        RETURN_IF_FAILED(CheckInputs());

        RETURN_IF_FAILED(ProcessEventQ());
    }

    RETURN_IF_FAILED(TransmitLight());

    RETURN_NOERROR;
}

tResult cAltarStateMachine::ProcessJuryInput() {

    if(IS_OK(ReadJury())) {

        switch (m_juryStruct.i16StateID) {
            case statecar_startup:
                // should not be received from Jury
                break;
            case statecar_ready:
                    m_pDM->process_event(stop{});
                    LOG_INFO("Car starting...");
//                    m_driverStruct.i16StateID = statecar_startup;
//                    m_driverStruct.i16ManeuverEntry = m_juryStruct.i16ManeuverEntry;
//                    RETURN_IF_FAILED(OutputDriverStruct());
                    // do start up stuff
                    m_driverStruct = m_juryStruct;
                    LOG_INFO("Car is ready");

                    RETURN_IF_FAILED(OutputDriverStruct());
                break;
            case statecar_running:
                m_driverStruct = m_juryStruct;

                RETURN_IF_FAILED(OutputDriverStruct());

                m_pDM->process_event(start{m_driverStruct.i16ManeuverEntry});

                break;

            case statecar_complete:
                m_driverStruct = m_juryStruct;
                m_pDM->process_event(complete{});
                m_pDM->process_event(completed{});

                // change state to ready
                break;
            default:
                LOG_INFO("No input from Jury, continuing with last state");
                break;
        }
    }

    RETURN_NOERROR;
}

tResult cAltarStateMachine::CheckInputs() {

    using namespace sml;

    RETURN_IF_FAILED(ReadEverything());

    if (m_emergencyBrake) {
        m_pDM->process_event(eBrakeOn{});
        addEvent({EMERG_STOP,TOP_PRIORITY,0 /* TODO: add distance */});
    }
    else if (eBrakeApplied && !m_emergencyBrake) {
        m_pDM->process_event(eBrakeOff{});
        LOG_INFO("Emergency Break Release..."); // TODO: solve loop
    }




    switch (m_MapSituation) {
        case DRIVE_STRAIGHT:
            if (!eBrakeApplied) {
                addEvent({FOLLOW_LANE, LOWEST_PRIORITY, 0});
            }
            break;
        case EXIT:
            break;
        case LEFT_TURN:
            addEvent({TURN_LEFT, LOW_PRIORITY, 0});
            break;
        case RIGHT_TURN:
            addEvent({TURN_RIGHT, LOW_PRIORITY, 0});
            break;
        case TURN:
            break;
        case START:
            break;
        case OBSTACLE:
            break;
        default:
            if (!eBrakeApplied) {
                addEvent({FOLLOW_LANE, LOWEST_PRIORITY, 0});
            }
    }

    switch (m_ParkingStruct.status) {

        case START_PARKING:
            addEvent({PARKING,MED_PRIORITY, 0});
            break;
        case BACKING:
            addEvent({BACKING_IN, HIGH_PRIORITY,0});
            break;
        case PARKED_IN:
            addEvent({PARKED,HIGH_PRIORITY,0});
            break;
        case PULL_LEFT:
            addEvent({PULL_OUT_LEFT,HIGH_PRIORITY,0});
            break;
        case PULL_RIGHT:
            addEvent({PULL_OUT_RIGHT,HIGH_PRIORITY,0});
            break;
        case NOT_PARKING:
            m_pDM->process_event(endParking{});
            break;
    }


    RETURN_NOERROR;
}

tResult cAltarStateMachine::ReadEverything() {

    ReadEBrake();

    ReadWheelSpeed();

    ReadMapSituation();

    ReadParking();

    ReadCrossing();

    RETURN_NOERROR;
}

tResult cAltarStateMachine::ReadEBrake() {


    object_ptr<const ISample> pReadSample;

    if (IS_OK(m_oEmergencyBrakeReader.GetLastSample(pReadSample)))
    {
        auto oDecoder = m_boolSignalDataSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.value, &m_emergencyBrake));

    }

    RETURN_NOERROR;
}

tResult cAltarStateMachine::ReadJury() {

    object_ptr<const ISample> pReadSample;

    if (IS_OK(m_oJuryStateReader.GetNextSample(pReadSample))) // GetNextSample returns error if you already read the newest
    {
//        m_lastReadJury = pReadSample->GetTime();
        auto oDecoder = m_juryStateDataSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriverStructId.stateId, &m_juryStruct.i16StateID));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriverStructId.maneuverEntry, &m_juryStruct.i16ManeuverEntry));

    }

    RETURN_NOERROR;
}

tResult cAltarStateMachine::ReadMapSituation() {

    object_ptr<const ISample> pMapSituationSample;

    while (IS_OK(m_oMapSituationReader.GetNextSample(pMapSituationSample))) {

        auto oDecoder = m_signalValueSampleFactory.MakeDecoderFor(pMapSituationSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &m_MapSituation));
    }

    RETURN_NOERROR;
}

tResult cAltarStateMachine::ReadParking() {

    object_ptr<const ISample> pReadSample;

    if (IS_OK(m_oParkingReader.GetNextSample(pReadSample))) // GetNextSample returns error if you already read the newest
    {
        auto oDecoder = m_parkingDataSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlParkingValueId.status, &m_ParkingStruct.status));
        if (m_ParkingStruct.status == START_PARKING) {
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlParkingValueId.speed, &m_ParkingStruct.speed));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlParkingValueId.steering, &m_ParkingStruct.steering));
        } else {

            auto pParkingPath = reinterpret_cast<const tLanePointData *>(oDecoder.GetElementAddress(m_ddlParkingValueId.pathArray));

            m_ParkingStruct.parkPath.nPoints = pParkingPath->nPoints;

            for (tSize i = 0; i < pParkingPath->nPoints; i++) {
                m_ParkingStruct.parkPath.path.push_back(pParkingPath->path[i]);
            }
        }

    }

    RETURN_NOERROR;
}

tResult cAltarStateMachine::ReadWheelSpeed() {

    object_ptr<const ISample> pWheelSample;
    while (IS_OK(m_oWheelReader.GetNextSample(pWheelSample))) {
        m_lastWheelSpeed.back() = m_lastWheelSpeed.front();

        // store speed
        auto oDecoder = m_wheelDataSampleFactory.MakeDecoderFor(*pWheelSample);
        m_lastWheelSpeed.front() = adtf_ddl::access_element::get_value(oDecoder, m_ddlSignalValueId.value);
        //LOG_INFO(cString::Format("process: %f", m_f32Speed));

    }

    RETURN_NOERROR;


}

tResult cAltarStateMachine::ReadCrossing() {

    object_ptr<const ISample> pCrossingSample;
    if (IS_OK(m_oCrossingReader.GetLastSample(pCrossingSample))) {

        auto oDecoder = m_wheelDataSampleFactory.MakeDecoderFor(*pCrossingSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.value, &m_atCrossing));

    }

    RETURN_NOERROR;
}

const bool cAltarStateMachine::Braking() const {
    return ((abs(m_lastWheelSpeed[1]) - abs(m_lastWheelSpeed[0]))>0.1 || eBrakeApplied || m_lastWheelSpeed[0] == 0);
}

tResult cAltarStateMachine::RequestManeuver() {

    RETURN_IF_FAILED(OutputDriverStruct());

    object_ptr<const ISample> pReadSample;

    while (IS_OK(m_oManeuverReader.GetLastSample(pReadSample)) && m_maneuverStruct.id != m_driverStruct.i16ManeuverEntry)
    {
        auto oDecoder = m_maneuverDataSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

//        std::lock_guard<std::mutex> lock{m_maneuverLock};
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlManeuverStruct.id, &m_maneuverStruct.id));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlManeuverStruct.action, &m_maneuverStruct.action));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlManeuverStruct.extra, &m_maneuverStruct.extra));
    }

    LOG_INFO(cString::Format("Maneuver no. %d: %s", m_maneuverStruct.id,aadc::jury::maneuverToString(m_maneuverStruct.action).c_str()));

    RETURN_NOERROR;
}

void cAltarStateMachine::onCompletion() {


    // send struct to jury

}

void cAltarStateMachine::requestEBrake() {
    LOG_DUMP("Emergency Break Engage...");

     LOG_RESULT(OutputSpeed(0));

}

void cAltarStateMachine::useLaneFollow() {

    TransmitPath(m_oLaneReader);

}

void cAltarStateMachine::useMap() {

    TransmitPath(m_oMapPathReader);

}

void cAltarStateMachine::useParkingPath() {

    OutputPath(m_ParkingStruct.parkPath);
}

void cAltarStateMachine::useParkingSS() {

    OutputSteering(m_ParkingStruct.steering);
    OutputSpeed(m_ParkingStruct.speed);
}

void cAltarStateMachine::addEvent(const evtDetail &evt) {

    m_qEventQueue.push(evt);
}

tResult cAltarStateMachine::ProcessEventQ() {
//    std::lock_guard<std::mutex> lock{m_qLock};

// might need to remove the while loop
    while (!m_qEventQueue.empty()) {

        ProcessEvent(m_qEventQueue.top());

        m_qEventQueue.pop();
    }

    RETURN_NOERROR;
}

tResult cAltarStateMachine::ProcessEvent(const evtDetail &evt) {

    switch (evt.id) {
        case EMERG_STOP:
            m_pDM->process_event(eBrakeOn{});
            break;
        case TURN_LEFT:
            m_pDM->process_event(junction{false});
            break;
        case TURN_RIGHT:
            m_pDM->process_event(junction{true});
            break;
        case FOLLOW_LANE:
            m_pDM->process_event(exitJunction{});
            m_pDM->process_event(followLane{});
            break;
        case PARKING:
            m_pDM->process_event(parkingIn{});
            break;
        case BACKING_IN:
            m_pDM->process_event(backing{});
            break;
        case PARKED:
            m_pDM->process_event(parked{});
            break;
        case PULL_OUT_LEFT:
            m_pDM->process_event(pullOut{false});
            break;
        case PULL_OUT_RIGHT:
            m_pDM->process_event(pullOut{true});
            break;

        default:
            break;
    }

    RETURN_NOERROR;
}

tResult cAltarStateMachine::TransmitPath(cPinReader &stream) {

    tUInt32 nPoints = 0;
    vector<tLanePoint> points;

    object_ptr<const ISample> pReadPathSample;

    while (IS_OK(stream.GetNextSample(pReadPathSample))) {
        auto oDecoder = m_LanePointDataSampleFactory.MakeDecoderFor(*pReadPathSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLanePointDataId.nPoints, &nPoints));

        auto *LanePoints_in = reinterpret_cast<const tLanePoint *>(oDecoder.GetElementAddress(
                m_ddlLanePointDataId.pointArray));

        for (tSize i = 0; i < nPoints; ++i) {
            points.push_back(LanePoints_in[i]);
        }
    }

    RETURN_IF_FAILED(OutputPath(LanePointData(nPoints,points)));

    RETURN_NOERROR;
}

tResult cAltarStateMachine::TransmitLight() {

    using namespace sml;

    if (Braking()) {
        m_pDM->process_event(brakeLightOn{});
    }
    else {
        m_pDM->process_event(brakeLightOff{});
    }
    if ((m_lastWheelSpeed[0] + m_lastWheelSpeed[1])/2 < 0) {
        m_pDM->process_event(reverseLightOn{});
    }
    else {
        m_pDM->process_event(reverseLightOff{});
    }


    RETURN_NOERROR;
}

tResult cAltarStateMachine::TransmitLight(const lightType &light) {

    switch (light) {
        case HEAD_ON:
            RETURN_IF_FAILED(WriteLight(m_oHeadLightWriter, true));
            break;
        case BRAKE_ON:
            RETURN_IF_FAILED(WriteLight(m_oBrakeLightWriter, true));
            break;
        case LEFT_ON:
            RETURN_IF_FAILED(WriteLight(m_oLeftLightWriter, true));
            break;
        case RIGHT_ON:
            RETURN_IF_FAILED(WriteLight(m_oRightLightWriter, true));
            break;
        case HAZARD_ON:
            RETURN_IF_FAILED(WriteLight(m_oHazardLightWriter, true));
            break;
        case REVERSE_ON:
            RETURN_IF_FAILED(WriteLight(m_oReverseLightWriter, true));
            break;
            /* switch off */
        case HEAD_OFF:
            RETURN_IF_FAILED(WriteLight(m_oHeadLightWriter, false));
            break;
        case LEFT_OFF:
            RETURN_IF_FAILED(WriteLight(m_oLeftLightWriter, false));
            break;
        case RIGHT_OFF:
            RETURN_IF_FAILED(WriteLight(m_oRightLightWriter, false));
            break;
        case HAZARD_OFF:
            RETURN_IF_FAILED(WriteLight(m_oHazardLightWriter, false));
            break;
        case REVERSE_OFF:
            RETURN_IF_FAILED(WriteLight(m_oReverseLightWriter, false));
            break;
        case BRAKE_OFF:
            RETURN_IF_FAILED(WriteLight(m_oBrakeLightWriter, false));
            break;
    }


    RETURN_NOERROR;
}

tResult cAltarStateMachine::WriteLight(cPinWriter &stream, const tBool &b) {

    object_ptr<ISample> pLightSample;
    if (IS_OK(alloc_sample(pLightSample, m_pClock->GetStreamTime()))){
        auto oCodec = m_boolSignalDataSampleFactory.MakeCodecFor(pLightSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.timeStamp, m_pClock->GetStreamTime()));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.value, b));

        stream << pLightSample << flush << trigger;

    }


    RETURN_NOERROR;
}

tResult cAltarStateMachine::OutputDriverStruct() {

    object_ptr<ISample> pWriteDriverSample;
    if (IS_OK(alloc_sample(pWriteDriverSample))) {
        auto oCodec = m_juryStateDataSampleFactory.MakeCodecFor(pWriteDriverSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.stateId, m_driverStruct.i16StateID));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.maneuverEntry, m_driverStruct.i16ManeuverEntry));
        m_oDriveStateWriter << pWriteDriverSample << flush << trigger;

    }

    RETURN_NOERROR;
}

tResult cAltarStateMachine::OutputSpeed(const tFloat32 &speed) {

    object_ptr<ISample> pWriteSpeedSample;
    if (IS_OK(alloc_sample(pWriteSpeedSample, m_pClock->GetStreamTime()))) {

        auto oCodec = m_signalValueSampleFactory.MakeCodecFor(pWriteSpeedSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, speed));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, m_pClock->GetStreamTime()));

        m_oSpeedWriter << pWriteSpeedSample << flush << trigger;

    }

    RETURN_NOERROR;
}

tResult cAltarStateMachine::OutputSteering(const tFloat32 &steer) {

    object_ptr<ISample> pWriteSteeringSample;
    if (IS_OK(alloc_sample(pWriteSteeringSample, m_pClock->GetStreamTime()))) {

        auto oCodec = m_signalValueSampleFactory.MakeCodecFor(pWriteSteeringSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, steer));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, m_pClock->GetStreamTime()));

        m_oSteerWriter << pWriteSteeringSample << flush << trigger;

    }

    RETURN_NOERROR;
}

tResult cAltarStateMachine::OutputPath(const LanePointData &path) {
    object_ptr<ISample> pWritePathSample;

    if (IS_OK(alloc_sample(pWritePathSample))) {

        auto oCodec = m_LanePointDataSampleFactory.MakeCodecFor(pWritePathSample);


        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlLanePointDataId.nPoints, path.nPoints));

        auto pointArray = reinterpret_cast<tLanePoint *>(oCodec.GetElementAddress(m_ddlLanePointDataId.pointArray));
        memset(pointArray, 0, 50 * sizeof(tLanePoint));



        for (tSize i = 0; i < path.nPoints; ++i) {
            pointArray[i].x = path.path[i].x;
            pointArray[i].y = path.path[i].y;
        }


        m_oPathWriter << pWritePathSample << flush << trigger;

    }

    RETURN_NOERROR;
}

