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

//*************************************************************************************************
#define CID_ALTAR_STATE_MACHINE_DATA_TRIGGERED_FILTER "altar_state_machine.filter.user.aadc.cid"


#include "stdafx.h"
#include "aadc_jury.h"
#include "aadc_structs.h"
#include "../AltarUtils/AltarStructs.h"
#include "../AltarUtils/MapStructs.h"
#include "ADTF3_helper.h"
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include "Queue_Struct.h"
#include <mutex>

//#include "StateMachine.h"
#include <typeinfo>

class cAltarStateMachine : public cTriggerFunction {


private:

    struct tLanePoint {

        tInt32 x;
        tInt32 y;

    };

    struct tLanePointData {
        tUInt32 nPoints;
        tLanePoint path[50];
    };

    struct LanePointData {
        tUInt32 nPoints;
        vector<tLanePoint> path;

        LanePointData() {
            nPoints = 0;
        }

        LanePointData(const tUInt32& _nPoints, vector<tLanePoint>& _path) {
            nPoints = _nPoints;
            path = std::move(_path);
        }
    };

    struct tParking {
        parkingStatus status;
        tFloat32 speed;
        tFloat32 steering;
        LanePointData parkPath;
    };

    /*! Media Descriptions. */
    struct tDriverStructId {
        tSize stateId;
        tSize maneuverEntry;
    } m_ddlDriverStructId;

    struct tManeuverId {
        tSize id;
        tSize action;
        tSize extra;
    } m_ddlManeuverStruct;

    struct tBoolSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlBoolSignalValueId;

    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    struct LanePointDataId
    {
        tSize nPoints;
        tSize pointArray;
    } m_ddlLanePointDataId;

    struct tParkingId {
        tSize status;
        tSize speed;
        tSize steering;
        tSize pathArray;
    } m_ddlParkingValueId;

    tSize m_ddlSpeedValueId;

    /*! The template data sample factory */
    cSampleCodecFactory m_juryStateDataSampleFactory;
    cSampleCodecFactory m_maneuverDataSampleFactory;
    cSampleCodecFactory m_boolSignalDataSampleFactory;
    cSampleCodecFactory m_signalValueSampleFactory;
    cSampleCodecFactory m_LanePointDataSampleFactory;
    cSampleCodecFactory m_wheelDataSampleFactory;
    cSampleCodecFactory m_parkingDataSampleFactory;

    object_ptr<adtf::services::IReferenceClock> m_pClock;

public:

    /*! Default constructor. */
    cAltarStateMachine();

    /*! Destructor. */
    virtual ~cAltarStateMachine() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;

    tResult RegisterManeuverStruct();
    tResult RegisterStateStruct();

    tResult RegisterSignalStruct();
    tResult RegisterBoolSignal();
    tResult RegisterLaneData();
    tResult RegisterWheelStruct();
    tResult RegisterParkingStruct();

    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

    tResult ProcessJuryInput();

    tResult OutputDriverStruct();

    tResult RequestManeuver();

    tResult ReadEverything();

    tResult ReadEBrake();

    tResult ReadWheelSpeed();

    tResult ReadJury();

    tResult ReadMapSituation();

    tResult ReadParking();

    tResult ReadCrossing();

    tResult TransmitPath(cPinReader &stream);

    tResult CheckInputs();

    tResult ProcessEventQ();

    tResult ProcessEvent(const evtDetail& evt);

    tResult OutputSpeed(const tFloat32& speed);

    tResult OutputSteering(const tFloat32& steer);

    tResult OutputPath(const LanePointData& path);

    tResult TransmitLight();
    tResult TransmitLight(const lightType &light);

    tResult WriteLight(cPinWriter &stream, const tBool &b);

    const bool Braking() const;

    void addEvent(const evtDetail &evt);

    void onCompletion();

    void requestEBrake();

    void useLaneFollow();

    void useMap();

    void useParkingPath();

    void useParkingSS();

private:

    tDriverStruct m_juryStruct = {aadc::jury::statecar_startup,0};  // what we receive from jury
    tDriverStruct m_driverStruct = m_juryStruct; // what we output to jury, dictates the state of the car
    aadc::jury::tManeuver m_maneuverStruct = {0, aadc::jury::manuever_undefined,0};  // the order taken from DriverModule
    tBool m_emergencyBrake = false;
    tTimeStamp m_lastReadJury = 0;
    std::array<tFloat32, 2> m_lastWheelSpeed;
    situations m_MapSituation = DRIVE_STRAIGHT;
    tParking m_ParkingStruct = {NOT_PARKING,0.0,0.0, LanePointData()};
    tBool m_atCrossing = false;


    /*! Reader of an InPin. */
    cPinReader m_oRampReader;
    cPinReader m_oObstacleReader;
    cPinReader m_oMapPathReader;
    cPinReader m_oMapSituationReader;
    cPinReader m_oLaneReader;
    cPinReader m_oJuryStateReader;
    cPinReader m_oManeuverReader;
    cPinReader m_oParkingReader;
    cPinReader m_oEmergencyBrakeReader;
    cPinReader m_oWheelReader;
    cPinReader m_oCrossingReader;
    /*! Writer to an OutPin. */
    cPinWriter m_oSpeedWriter;
    cPinWriter m_oSteerWriter;
    cPinWriter m_oBrakeLightWriter;
    cPinWriter m_oLeftLightWriter;
    cPinWriter m_oRightLightWriter;
    cPinWriter m_oHeadLightWriter;
    cPinWriter m_oHazardLightWriter;
    cPinWriter m_oReverseLightWriter;
    cPinWriter m_oDriveStateWriter;
    cPinWriter m_oPathWriter;

    /*! Event Queue */
    std::priority_queue<evtDetail> m_qEventQueue;

    /* Locks */
    std::mutex m_qLock;
    std::mutex m_maneuverLock;
    std::mutex m_speedLock;


    struct decisionMaker; //forward decl
    std::shared_ptr<decisionMaker> m_pDM; // delay evaluation until the class is constructed

};



//*************************************************************************************************
