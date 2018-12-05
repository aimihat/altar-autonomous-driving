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

//otherwise cDOM will cause a deprecated warning, however there is no alternative yet
#define A_UTILS_NO_DEPRECATED_WARNING

#include "DriverModuleFilter.h"

#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, A_UTILS_NS::log::tLogLevel::Info)                        //!< log info messages


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CAR_CONTROLLER,
                                    LABEL_CAR_CONTROLLER,
                                    DriverModule,
                                    adtf::filter::timer_trigger(1000));

using namespace aadc::jury;

DriverModule::DriverModule() : m_clientConnectionEstablished(tFalse)
{
    //Register Properties
    RegisterPropertyVariable("Port number", m_propTCPPort);
    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);

    object_ptr<IStreamType> pTypeManeuverStruct;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tManeuver", pTypeManeuverStruct, m_maneuverDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_maneuverDataSampleFactory, cString("id"), m_ddlManeuver.id);
        adtf_ddl::access_element::find_index(m_maneuverDataSampleFactory, cString("maneuver"), m_ddlManeuver.maneuver);
        adtf_ddl::access_element::find_index(m_maneuverDataSampleFactory, cString("extra"), m_ddlManeuver.extra);
    }
    else
    {
        LOG_WARNING("No mediadescription for tManeuver found!");
    }

    object_ptr<IStreamType> pTypeCarState;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tDriverStruct", pTypeCarState, m_driverStructDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_driverStructDataSampleFactory, cString("i16StateID"), m_ddlDriverStructId.stateId);
        adtf_ddl::access_element::find_index(m_driverStructDataSampleFactory, cString("i16ManeuverEntry"), m_ddlDriverStructId.maneuverEntry);
    }
    else
    {
        LOG_WARNING("No mediadescription for tDriverStruct found!");
    }

    object_ptr<IStreamType> pTypeManArray;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tManArray", pTypeManArray, m_manArrayDataSampleFactory))
    {
        adtf_ddl::access_element::find_array_index(m_manArrayDataSampleFactory, cString("manArray"), m_ddlManArray.manArray);
    }
    else
    {
        LOG_WARNING("No mediadescription for tManArray found!");
    }

    Register(m_oOutputManeuverStruct, "maneuver_struct", ucom_object_ptr_cast<const IStreamType>(pTypeManeuverStruct));
    Register(m_oInputCarState, "car_state", ucom_object_ptr_cast<const IStreamType>(pTypeCarState));
    Register(m_oOutputCarState, "state_change", ucom_object_ptr_cast<const IStreamType>(pTypeCarState));
    Register(m_oOutputMapManeuver, "map_maneuver", ucom_object_ptr_cast<const IStreamType>(pTypeManArray));

}

DriverModule::~DriverModule()
{
    m_streamSocket.Close();
    m_serverSocket.Close();
    m_clientConnectionEstablished = tFalse;
}

tResult DriverModule::Configure()
{
    /* start ports and prep TCP */
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_IF_FAILED_DESC(m_serverSocket.Open(m_propTCPPort, cServerSocket::SS_Exclusive),
                          cString::Format("Could not open server socket with port %d", static_cast<tInt>(m_propTCPPort)));
    LOG_INFO(cString::Format("Server Socket was opened with port %d", static_cast<tInt>(m_propTCPPort)));
    RETURN_IF_FAILED_DESC(m_serverSocket.Listen(),
                          cString::Format("Could not listen to port %d",static_cast<tInt>(m_propTCPPort)));
    LOG_INFO(cString::Format("Server Socket now listens on port %d", static_cast<tInt>(m_propTCPPort)));

    RETURN_NOERROR;
}


tResult DriverModule::ReceiveTCPData(std::vector<tChar>& data)
{
    // no stream connected yet
    if (!m_clientConnectionEstablished)
    {
        // try to connect to client
        if (m_serverSocket.IsConnected(static_cast<tTimeStamp>(2e2)))
        {
            RETURN_IF_FAILED(m_serverSocket.Accept(m_streamSocket));
            LOG_INFO("TCP Connection was established");
            m_clientConnectionEstablished = tTrue;

            SendDriverStruct({statecar_startup, 0});
        }
    }
    else
    {
        if (m_streamSocket.DataAvailable())
        {
            cString strBuffer;
            const tSize bufferSize = 65536;
            tInt bytesRead = 0;
            //make some space for data
            strBuffer.SetBuffer(bufferSize);
            // if read ok
            tResult res = m_streamSocket.Read((void*) strBuffer.GetPtr(), bufferSize, &bytesRead);
            if (IS_OK(res))
            {
                CONSOLE_LOG_INFO(cString::Format("Received from client: %s", strBuffer.GetPtr()));
                data.clear();
                data.resize(bytesRead);
                memcpy(data.data(), strBuffer.GetPtr(), bytesRead);
            }
            else
            {
                LOG_INFO("TCP Connection was disconnected");
                m_clientConnectionEstablished = tFalse;
            }
        }
        else
        {
            RETURN_ERROR(ERR_NOT_READY);
        }
    }
    RETURN_NOERROR;
}

tResult DriverModule::Process(tTimeStamp tmTime)
{
    std::vector<tChar> data;

    if(IS_OK(ReceiveTCPData(data))){

        RETURN_IF_FAILED(ProcessJury(data));
    }

    RETURN_IF_FAILED(ProcessCar());

    RETURN_NOERROR;
}

tResult DriverModule::LoadManeuverList()
{
    lastId.clear();
    m_sectorList.clear();
    // create dom from string received from pin
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;

    //read first Sector Elem
    if (IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {
        object_ptr<ISample> pWriteManArraySample;

        if (IS_OK(alloc_sample(pWriteManArraySample))) {
            auto oCodec = m_manArrayDataSampleFactory.MakeCodecFor(pWriteManArraySample);
            auto manArray = reinterpret_cast<tUInt8 *>(oCodec.GetElementAddress(m_ddlManArray.manArray));
            memset(manArray, 0, 100*sizeof(tUInt8));

            //iterate through sectors
            for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin();
                 itSectorElem != oSectorElems.end(); ++itSectorElem) {
                //if sector found
                tSector sector;
                sector.id = (*itSectorElem)->GetAttributeUInt32("id");

                if (IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems))) {
                    //iterate through maneuvers
                    for (cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin();
                         itManeuverElem != oManeuverElems.end(); ++itManeuverElem) {
                        aadc::jury::tManeuver man;
                        man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                        man.action = maneuverFromString((*itManeuverElem)->GetAttribute("action").GetPtr());
                        man.extra = (*itManeuverElem)->GetAttributeInt("extra");
                        sector.sector.push_back(man);

                        manArray[man.id] = static_cast<tUInt8>(man.action);

                        if (std::next(itManeuverElem) == oManeuverElems.end()) lastId.push_back(man.id);
                    }
                }

                m_sectorList.push_back(sector);
            }

            m_oOutputMapManeuver << pWriteManArraySample << flush << trigger;

        }
    }
    if (oSectorElems.size() > 0)
    {
        LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
    }
    else {
        LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}

tResult DriverModule::ProcessJury(const std::vector<tChar> &data) {
    const tSize sizeOfJuryStruct = sizeof(tJuryStruct);
    if (data.size() == sizeOfJuryStruct)
    { //jurystruct

        tJuryStruct* juryStruct = (tJuryStruct*) data.data();
        tInt8 i8ActionID = juryStruct->i16ActionID;
        tInt16 i16entry = juryStruct->i16ManeuverEntry;


        switch (i8ActionID)
        {
            case action_getready: {
                CONSOLE_LOG_INFO(
                        cString::Format("Driver Module: Received Request Ready with maneuver ID %d", i16entry));
//                RETURN_IF_FAILED(SendDriverStruct({statecar_startup, i16entry}));

//                RETURN_IF_FAILED(OutputDriverStruct(statecar_startup, i16entry));

//                /*! read state machine until ready */
//                tDriverStruct inputDriverStruct{statecar_startup,0};
//                while (inputDriverStruct.i16StateID != statecar_ready) {
//                    RETURN_IF_FAILED(ReadState(inputDriverStruct));
//                }
                m_driverStruct = {statecar_ready,i16entry};
                RETURN_IF_FAILED(SendDriverStruct(m_driverStruct));
                RETURN_IF_FAILED(OutputDriverStruct((stateCar)m_driverStruct.i16StateID, m_driverStruct.i16ManeuverEntry));
                break;
            }
            case action_start:
                CONSOLE_LOG_INFO(cString::Format("Driver Module: Received Run with maneuver ID %d", i16entry));
                RETURN_IF_FAILED(SendDriverStruct({statecar_running, i16entry}));
                RETURN_IF_FAILED(OutputDriverStruct(statecar_running, i16entry));
                break;
            case action_stop:
                CONSOLE_LOG_INFO(cString::Format("Driver Module: Received Stop with maneuver ID %d", i16entry));
                RETURN_IF_FAILED(SendDriverStruct({statecar_ready, i16entry}));
                RETURN_IF_FAILED(OutputDriverStruct(statecar_ready, i16entry));
                break;

            default:
                LOG_ERROR("Error, action ID undefined!");
        }
    }
    else if (data.size() > 0)
    {//maneuverlist
        m_strManeuverFileString.Set(data.data(),data.size());
        LoadManeuverList();
    }

    RETURN_NOERROR;
}

tResult DriverModule::ProcessCar() {

    RETURN_IF_FAILED(ReadState(m_driverStruct));

    if (m_driverStruct.i16StateID == statecar_error){
        SendDriverStruct(m_driverStruct);
        RETURN_NOERROR;
    }

    if ((!lastId.empty())) {
        if (m_driverStruct.i16ManeuverEntry > lastId.back()) {
            RETURN_IF_FAILED(OutputDriverStruct(statecar_complete,lastId.back()))
            RETURN_IF_FAILED(OutputManeuver({-1,manuever_undefined,0}));
            RETURN_IF_FAILED(SendDriverStruct({statecar_complete, static_cast<tInt16>(lastId.back())}));
            RETURN_NOERROR;
        }

        RETURN_IF_FAILED(OutputManeuver(findManeuver(m_driverStruct.i16ManeuverEntry)));
    };

//    RETURN_IF_FAILED(SendDriverStruct(m_driverStruct));


    RETURN_NOERROR;
}

tResult DriverModule::SendDriverStruct(const tDriverStruct &driverStruct) {

    if (m_propEnableConsoleOutput)
    {
        switch (driverStruct.i16StateID)
        {
            case statecar_ready:
                LOG_INFO(cString::Format("Driver Module: Send state: READY, Maneuver ID %d", driverStruct.i16ManeuverEntry));
                break;
            case statecar_running:
                LOG_INFO(cString::Format("Driver Module: Send state: RUNNING, Maneuver ID %d", driverStruct.i16ManeuverEntry));
                break;
            case statecar_complete:
                LOG_INFO(cString::Format("Driver Module: Send state: COMPLETE, Maneuver ID %d", driverStruct.i16ManeuverEntry));
                break;
            case statecar_error:
                LOG_INFO(cString::Format("Driver Module: Send state: ERROR, Maneuver ID %d", driverStruct.i16ManeuverEntry));
                break;
            case statecar_startup:
                LOG_INFO(cString::Format("Driver Module: Send state: STARTUP, Maneuver ID %d", driverStruct.i16ManeuverEntry));
                break;
        }
    }

    if (m_clientConnectionEstablished)
    {
        RETURN_IF_FAILED(m_streamSocket.Write(&driverStruct, sizeof(tDriverStruct)));
    }

    RETURN_NOERROR;
}

tResult DriverModule::OutputDriverStruct(const stateCar &id, const tInt16 &i16ManeuverEntry) {

    tDriverStruct driverStruct;
    driverStruct.i16StateID = id;
    driverStruct.i16ManeuverEntry = i16ManeuverEntry;

    object_ptr<ISample> pWriteDriverSample;
    if (IS_OK(alloc_sample(pWriteDriverSample))) {
        auto oCodec = m_driverStructDataSampleFactory.MakeCodecFor(pWriteDriverSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.stateId, driverStruct.i16StateID));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.maneuverEntry, driverStruct.i16ManeuverEntry));
        m_oOutputCarState << pWriteDriverSample << flush << trigger;

    }

    RETURN_NOERROR;
}

tResult DriverModule::OutputManeuver(const tManeuver &maneuver) {

    object_ptr<ISample> pWriteManeuverSample;
    if (IS_OK(alloc_sample(pWriteManeuverSample))) {
        auto oCodec = m_maneuverDataSampleFactory.MakeCodecFor(pWriteManeuverSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlManeuver.id, maneuver.id));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlManeuver.maneuver, maneuver.action));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlManeuver.extra, maneuver.extra));
        m_oOutputManeuverStruct << pWriteManeuverSample << flush << trigger;

    }

    RETURN_NOERROR;
}

tResult DriverModule::ReadState(tDriverStruct& driverStruct) {
    object_ptr<const ISample> pReadSample;

    if (IS_OK(m_oInputCarState.GetNextSample(pReadSample))) {
        auto oDecoder = m_driverStructDataSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriverStructId.stateId, &(driverStruct.i16StateID)));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriverStructId.maneuverEntry, &(driverStruct.i16ManeuverEntry)));

        LOG_INFO("Current Car state: %s", aadc::jury::stateCarToString((stateCar)driverStruct.i16StateID));
    }\
    else {
        RETURN_ERROR(ERR_NOT_READY);
    }

    RETURN_NOERROR;
}

tManeuver DriverModule::findManeuver(const tUInt16 &id_) {

    auto sectorId = upper_bound(lastId.begin(), lastId.end(), id_) - lastId.begin();

    auto it = std::find_if(m_sectorList.at(sectorId).sector.begin(), m_sectorList.at(sectorId).sector.end(), [&] (const aadc::jury::tManeuver& man) {
        return man.id == id_;
    } );

    if (it->id != id_) {
        LOG_ERROR("Can't find maneuver after narrowing down");
        return aadc::jury::tManeuver(-2,manuever_undefined,0);
    }

    return aadc::jury::tManeuver(it->id, it->action, it->extra);

}
