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
#define CID_cTrajectoryGen_DATA_TRIGGERED_FILTER "trajectory_gen.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;




/*! the main class for the lane detection filter. */
class cTrajectoryGen : public cTriggerFunction
{
private:
    cPinReader m_oReaderlane;
//    cPinReader m_oReaderSpd;
//    cPinReader m_oReaderSteer;

    cPinWriter m_oWriterSteer;
    cPinWriter m_oWriterSpd;
    cPinReader m_oSpeedReader;
    cPinReader m_oSteerReader;
    /*! The template data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSpdSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSteerSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_LanePointSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_LanePointDataSampleFactory;

    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueSpdId, m_ddlSignalValueSteerId;

    object_ptr<adtf::services::IReferenceClock> m_pClock;

    struct lanePoint {
        int x;
        int y;
        int cluster;
        int pctDiff;

        lanePoint()
        {
            x = 0;
            y = 0;
        }

        lanePoint(int xcoord, int ycoord)
        {
            x = xcoord;
            y = ycoord;
            cluster = 0;
            pctDiff = 0;
        }
    };

/* for transmission */
    struct tLanePoint {
        tInt32 x;
        tInt32 y;
    };


    typedef struct
    {
        tFloat32 f32x;
        tFloat32 f32y;
        tFloat32 f32radius;
        tFloat32 f32speed;
        tFloat32 f32heading;
    } tPositionB;

    struct GoalPoint {
        tFloat32 x;
        tFloat32 y;
    } gP, pP;

public:

    /*! Default constructor. */
    cTrajectoryGen();

    /*! Destructor. */
    virtual ~cTrajectoryGen() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;

    /*!
     * Overwrites the Process You need to implement the Reading and Writing of Samples within this
     * function MIND: Do Reading until the Readers queues are empty or use the
     * IPinReader::GetLastSample()
     * This FUnction will be called if the Run() of the TriggerFunction was called.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     *
     * \return  Standard Result Code.
     */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

    tResult trajectoryGen(tLanePoint *relativePoints, const int &nPoints);

private:

    tResult TransmitSpeed(tSignalValue speedSignal);
    tResult TransmitDir(tSignalValue dirSignal);
    bool left_right(const GoalPoint &gP);

    tFloat32 previousSpeed = 0.0;
    tFloat32 previousSteer = 0.0;
    tTimeStamp lastUpdateTime = 0;

    tSignalValue steerSignal_old;
    tSignalValue speedSignal_old;

    struct ddllanePointId
    {
        tSize x;
        tSize y;
    } m_ddllanePointId;

    struct ddlLanePointDataId
    {
        tSize nPoints;
        tSize pointArray;
    } m_ddlLanePointDataId;

    std::mutex m_speedLock;
    std::mutex m_steerLock;

};

//*************************************************************************************************
