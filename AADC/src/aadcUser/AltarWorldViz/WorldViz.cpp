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
#include "WorldViz.h"
#include "ADTF3_OpenCV_helper.h"
#include <fstream>


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_MAPPING_FILTER,
                                    "AltarWorldViz",
                                    cWorldViz,
                                    adtf::filter::pin_trigger({"Position"}));


cWorldViz::cWorldViz()
{
    //MY STRUCTS

    object_ptr<IStreamType> pDebugPoint;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tDebug", pDebugPoint, m_DebugPointStructSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_DebugPointStructSampleFactory, "numPoints", m_ddlDebugPointId.numPoints);
        adtf_ddl::access_element::find_array_index(m_DebugPointStructSampleFactory, "debugPoints", m_ddlDebugPointId.debugPoints);
    }

    else
    {
        LOG_WARNING("No mediadescription for tParticle found!");
    }

    Register(DebugPointIn, "DebugPoints", pDebugPoint);



    object_ptr<IStreamType> pParticleData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tParticles", pParticleData, m_ParticleStructSampleFactory))
    {
        adtf_ddl::access_element::find_array_index(m_ParticleStructSampleFactory, "particleArray", m_ddlParticleDataId.particleArray);

    }
    else
    {
        LOG_WARNING("No mediadescription for tParticle found!");
    }

    Register(ParticleDataIn, "Particles", pParticleData);


    object_ptr<IStreamType> pPoseCovData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPoseCov", pPoseCovData, m_PoseCovStructSampleFactory))
    {
        adtf_ddl::access_element::find_array_index(m_PoseCovStructSampleFactory, "covArray", m_ddlPoseCovDataId.covArray);

    }
    else
    {
        LOG_WARNING("No mediadescription for tPoseCov found!");
    }

    Register(PoseCovDataIn, "PoseCov", pPoseCovData);



    // Position Data
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

    Register(PoseDataIn, "Position" , pTypePositionData);


    // Image Data
    // GREYSCALE_8
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register output pin
    Register(m_oWriter, "Map", pType);

    /*
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
                                    { return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter); });

*/

    mImageGrid = cv::Mat(WORLD_WIDTH, WORLD_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0) );	// RGBA, 8 bit per channel
    mImagePath = cv::Mat( WORLD_WIDTH, WORLD_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0) );		// RGBA, 8 bit per channel


    mImageOverlay = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0));	// RGBA, 8 bit per channel
    mImage = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0));
    mImageSend = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC3, CV_RGB(0,0,0) );	// RGB, 8 bit per channel
    mImageFlipped = cv::Mat( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC3, CV_RGB(0,0,0));

    mImageDebug = cv::Mat(WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0));
    mImageDebugTmp = cv::Mat(WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0));
    covmat = cv::Mat::eye(2,2, CV_32F);


    // Draw Lines on Map
    int gridLines = (float)WORLD_WIDTH/(float)METERS_TO_PIXELS;
    int start = -gridLines*0.5;
    for( int x = start; x <= gridLines - start; x ++ )
    {
        cv::Point2f start( round(x*METERS_TO_PIXELS) + WORLD_CENTER, 0 );
        cv::Point2f end( round(x*METERS_TO_PIXELS) + WORLD_CENTER, WORLD_HEIGHT );
        cv::line( mImageGrid, start, end, cv::Scalar(30, 30, 30, 255) );
    }
    for( int y = start; y <= gridLines - start; y ++ )
    {
        cv::Point2f start( 0, round(y*METERS_TO_PIXELS) + WORLD_CENTER );
        cv::Point2f end( WORLD_WIDTH, round(y*METERS_TO_PIXELS) + WORLD_CENTER );
        cv::line( mImageGrid, start, end, cv::Scalar(30, 30, 30, 255) );
    }
    // Draw x axis:
    cv::Point2f s( WORLD_CENTER, WORLD_CENTER );
    cv::Point2f e( WORLD_WIDTH*0.5 + WORLD_CENTER, WORLD_CENTER );
    cv::line( mImageGrid, s, e, cv::Scalar(64, 128, 64 ) );
    cv::Point2f textPos( WORLD_WIDTH - 20, WORLD_CENTER - 10 );
    putText( mImageGrid, "x", textPos, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(64,128,64) );
    // Draw y axis:
    e = cv::Point2f( WORLD_CENTER, 0 );
    textPos = cv::Point2f( WORLD_CENTER + 7, 10 );
    cv::line( mImageGrid, s, e, cv::Scalar(64, 64, 128 ) );
    putText( mImageGrid, "y", textPos, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(64,64,128) );

    // flip grid image:
    cv::Mat tmp( WORLD_WIDTH, WORLD_HEIGHT, CV_8UC4, cv::Scalar(0,0,0,0) );		// RGBA, 8 bit per channel
    cv::flip( mImageGrid, tmp, 0 );
    mImageGrid = tmp;

    mPreviousPos = cv::Point2f(WORLD_CENTER,WORLD_CENTER);
    carPose.theta = 0;
    carPose.x = 0;
    carPose.y = 0;

    mImageGrid = mImageGrid;
    //ifstream fin("/home/aadc/audi/AADC/src/aadcUser/AltarWorldVizcirclelines.in");
//
//    ifstream fin("/home/robin/Desktop/audi/AADC/src/aadcUser/AltarWorldViz/circlelines.in");
//    for (int i = 0; i < 96*3; i++){
//
//        float x1,y1, x2, y2;
//        fin>>x1>>y1>>x2>>y2;
//
//        cv::Point2f line_start( (int)round(x1*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y1*METERS_TO_PIXELS + WORLD_CENTER));
//        cv::Point2f line_end( (int)round(x2*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y2*METERS_TO_PIXELS + WORLD_CENTER));
//        cv::line( mImageGrid, line_start, line_end, cv::Scalar( 0, 0, 255, 255 ) );
//        LOG_INFO(cString::Format("Eigen: (%f, %f)", x1, x2));
//
//    }
    vector<segment> roadSegments;


    ifstream myfile("/home/aadc/audi/AADC/src/aadcUser/AltarLocalize/map_segments.txt");

    //ifstream myfile("/home/robin/Desktop/audi/AADC/src/aadcUser/AltarLocalize/map_segments.txt");
//    while (!myfile.eof()) {
//        segment lineSegments;
//        int sizeLeft, sizeRight, sizeMiddle;
//        myfile >> lineSegments.id;
//        myfile >> sizeLeft;
//        for (int j = 0; j < sizeLeft; j++) {
//            point p;
//            myfile >> p.x;
//            myfile >> p.y;
//            lineSegments.left.push_back(p);
//        }
//        myfile >> sizeMiddle;
//        for (int j = 0; j < sizeMiddle; j++) {
//            point p;
//            myfile >> p.x;
//            myfile >> p.y;
//            lineSegments.middle.push_back(p);
//        }
//        myfile >> sizeRight;
//        for (int j = 0; j < sizeRight; j++) {
//            point p;
//            myfile >> p.x;
//            myfile >> p.y;
//            lineSegments.right.push_back(p);
//        }
//        roadSegments.push_back(lineSegments);
//    }
//
//    int numSegments;
//    myfile >> numSegments;
//    for (int i = 0; i < numSegments; i++) {
//        segment s;
//        int sizeLeft, sizeRight, sizeMiddle, sizePoints;
//        myfile >> s.id;
//        myfile >> sizePoints;
//        for (int j = 0; j < sizePoints; j++) {
//            MapPoint p;
//            myfile >> p.x;
//            myfile >> p.y;
//            s.points.push_back(p);
//        }
//
//        myfile >> sizeLeft;
//        for (int j = 0; j < sizeLeft; j++) {
//            point p;
//            myfile >> p.x;
//            myfile >> p.y;
//            s.left.push_back(p);
//        }
//        myfile >> sizeMiddle;
//        for (int j = 0; j < sizeMiddle; j++) {
//            point p;
//            myfile >> p.x;
//            myfile >> p.y;
//            s.middle.push_back(p);
//        }
//        myfile >> sizeRight;
//        for (int j = 0; j < sizeRight; j++) {
//            point p;
//            myfile >> p.x;
//            myfile >> p.y;
//            s.right.push_back(p);
//        }
//        roadSegments.push_back(s);
//    }
//
//
//
//    for (int i = 0; i < roadSegments.size(); i++) {
//        for (int p = 1; p < roadSegments[i].left.size() ; p++) {
//            cv::Point line_start_l((int) (roadSegments[i].left[p - 1].x * METERS_TO_PIXELS + WORLD_CENTER),
//                                   (int) (roadSegments[i].left[p - 1].y * METERS_TO_PIXELS + WORLD_CENTER));
//            cv::Point line_end_l((int) (roadSegments[i].left[p].x * METERS_TO_PIXELS + WORLD_CENTER),
//                                 (int) (roadSegments[i].left[p].y * METERS_TO_PIXELS + WORLD_CENTER));
//            cv::line(mImageGrid, line_start_l, line_end_l, cv::Scalar(0, 255, 255, 255));
//
//            cv::Point line_start_r((int) (roadSegments[i].right[p - 1].x * METERS_TO_PIXELS + WORLD_CENTER),
//                                   (int) (roadSegments[i].right[p - 1].y * METERS_TO_PIXELS + WORLD_CENTER));
//            cv::Point line_end_r((int) (roadSegments[i].right[p].x * METERS_TO_PIXELS + WORLD_CENTER),
//                                 (int) (roadSegments[i].right[p].y * METERS_TO_PIXELS + WORLD_CENTER));
//            cv::line(mImageGrid, line_start_r, line_end_r, cv::Scalar(0, 255, 255, 255));
//
//            cv::Point line_start_m((int) (roadSegments[i].middle[p - 1].x * METERS_TO_PIXELS + WORLD_CENTER),
//                                   (int) (roadSegments[i].middle[p - 1].y * METERS_TO_PIXELS + WORLD_CENTER));
//            cv::Point line_end_m((int) (roadSegments[i].middle[p].x * METERS_TO_PIXELS + WORLD_CENTER),
//                                 (int) (roadSegments[i].middle[p].y * METERS_TO_PIXELS + WORLD_CENTER));
//            cv::line(mImageGrid, line_start_m, line_end_m, cv::Scalar(0, 255, 255, 255));
//
////            cv::Point line_start_p((int) (roadSegments[i].points[p - 1].x * METERS_TO_PIXELS + WORLD_CENTER),
////                                   (int) (roadSegments[i].points[p - 1].y * METERS_TO_PIXELS + WORLD_CENTER));
////            cv::Point line_end_p((int) (roadSegments[i].points[p].x * METERS_TO_PIXELS + WORLD_CENTER),
////                                 (int) (roadSegments[i].points[p].y * METERS_TO_PIXELS + WORLD_CENTER));
////            cv::line(mImageGrid, line_start_p, line_end_p, cv::Scalar(0, 255, 255, 255));
//        }
//        cv::circle(
//                mImageGrid,
//                cv::Point((int)(roadSegments[i].middle[0].x * METERS_TO_PIXELS + WORLD_CENTER),
//                          (int)(roadSegments[i].middle[0].y * METERS_TO_PIXELS + WORLD_CENTER)),
//                2,
//                cv::Scalar(0, 255, 255, 255),
//                -1);
//
//        cv::circle(
//                mImageGrid,
//                cv::Point((int)(roadSegments[i].middle[roadSegments[i].middle.size() - 1].x * METERS_TO_PIXELS + WORLD_CENTER),
//                          (int)(roadSegments[i].middle[roadSegments[i].middle.size() - 1].y * METERS_TO_PIXELS + WORLD_CENTER)),
//                2,
//                cv::Scalar(0, 255, 255, 255),
//                -1);
////
//        std::ostringstream ss;
//        ss << roadSegments[i].id;
//        putText(mImageGrid, ss.str(), cv::Point(
//                ((roadSegments[i].middle[0].x + roadSegments[i].middle[roadSegments[i].middle.size() - 1].x) / 2.0 *
//                 METERS_TO_PIXELS + WORLD_CENTER),
//                ((roadSegments[i].middle[0].y + roadSegments[i].middle[roadSegments[i].middle.size() - 1].y) / 2.0 *
//                 METERS_TO_PIXELS + WORLD_CENTER)), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 255));
//    }





//    double line_segs_3[5][2] = {11.5,282,13,185,46,93,120,28,215,9};
//    double line_segs_2[4][2] = {58.5, 262, 64, 165, 115, 86, 215, 54};
//
//
//// NOTE THAT YOU USING METERS HERE NOW,
//    for (int i = 0; i < 4; i++){
//
//        double x_start = line_segs_3[i][0]* 0.01;
//        double y_start = line_segs_3[i][1]* 0.01;
//        double x_end = line_segs_3[i+1][0]* 0.01;
//        double y_end = line_segs_3[i+1][1]* 0.01;
//
//        cv::Point2f line_start( (int)round(x_start*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER));
//        cv::Point2f line_end( (int)round(x_end*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_end*METERS_TO_PIXELS + WORLD_CENTER));
//        cv::line( mImageGrid, line_start, line_end, cv::Scalar( 0, 0, 255, 255 ) );
//
//    }
//
//    for (int i = 0; i < 3; i++){
//
//        double x_start = line_segs_2[i][0]* 0.01;
//        double y_start = line_segs_2[i][1]* 0.01;
//        double x_end = line_segs_2[i+1][0]* 0.01;
//        double y_end = line_segs_2[i+1][1]* 0.01;
//        cv::Point2f line_start( (int)round(x_start*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_start*METERS_TO_PIXELS + WORLD_CENTER));
//        cv::Point2f line_end( (int)round(x_end*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y_end*METERS_TO_PIXELS + WORLD_CENTER));
//        cv::line( mImageGrid, line_start, line_end, cv::Scalar( 0, 0, 255, 255 ) );
//
//
//    }



    cv::Point2f line1_start( (int)round(1.13*METERS_TO_PIXELS + WORLD_CENTER), (int)round(3.45*METERS_TO_PIXELS + WORLD_CENTER));
    cv::Point2f line1_end( (int)round(3.13*METERS_TO_PIXELS + WORLD_CENTER), (int)round(3.45*METERS_TO_PIXELS + WORLD_CENTER));

    cv::Point2f line2_start( (int)round(1.13*METERS_TO_PIXELS + WORLD_CENTER), (int)round(2.97*METERS_TO_PIXELS + WORLD_CENTER));
    cv::Point2f line2_end( (int)round(3.13*METERS_TO_PIXELS + WORLD_CENTER), (int)round(2.97*METERS_TO_PIXELS + WORLD_CENTER));

    cv::Point2f line3_start( (int)round((1.13)*METERS_TO_PIXELS + WORLD_CENTER), (int)round(2.52*METERS_TO_PIXELS + WORLD_CENTER));
    cv::Point2f line3_end( (int)round((3.13)*METERS_TO_PIXELS + WORLD_CENTER), (int)round(2.52*METERS_TO_PIXELS + WORLD_CENTER));

    cv::line( mImageGrid, line1_start, line1_end, cv::Scalar( 0, 0, 255, 255 ) );
    cv::line( mImageGrid, line2_start, line2_end, cv::Scalar( 0, 0, 255, 255 ) );
    cv::line( mImageGrid, line3_start, line3_end, cv::Scalar( 0, 0, 255, 255 ) );


    roi = Rect(int(round(WORLD_CENTER - WINDOW_WIDTH/2)),int(round(WORLD_CENTER - WINDOW_WIDTH/2)),int(WINDOW_WIDTH),int(WINDOW_WIDTH));
    mImageGridMasked = mImageGrid(roi);
    mImagePathMasked = mImagePath(roi);

    particleOuterA = new vizParticle();
    particleMiddle = new vizParticle();
    particleOuterB = new vizParticle();

    line_debug_length = 1;

}


//implement the Configure function to read ALL Properties
tResult cWorldViz::Configure()
{
    // -------------------------------
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    //--------------------------------

    RETURN_NOERROR;
}


tResult cWorldViz::Process(tTimeStamp tmTimeOfTrigger)
{


    UpdateCarPos();
    StreamWorld();

    RETURN_NOERROR;
}

tResult cWorldViz::UpdateCarPos(){

    LOG_DUMP("Updating carPose Position");

    object_ptr<const ISample> pReadSample;

    if (IS_OK(PoseDataIn.GetLastSample(pReadSample)))
    {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(pReadSample);
        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &carPose.x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &carPose.y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &carPose.theta));

        carPose.x *= 0.001;
        carPose.y *= 0.001;

        LOG_DUMP(cString::Format("carPose Pos: (%f, %f, %f)", carPose.x, carPose.y, carPose.theta));
        //LOG_DUMP(cString::Format("carPose Pos - map Coords: (%f, %f)", carPose.x/map_res, carPose.y/map_res));

    }

    object_ptr<const ISample> pReadCovSample;

    if (IS_OK(PoseCovDataIn.GetLastSample(pReadCovSample))) {
        auto oDecoder = m_PoseCovStructSampleFactory.MakeDecoderFor(*pReadCovSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        const tFloat32 *convArray = reinterpret_cast<const tFloat32 *>(oDecoder.GetElementAddress(
                m_ddlPoseCovDataId.covArray));

        covmat.at<float>(0, 0) = static_cast<tFloat32 >(convArray[0]);
        covmat.at<float>(0, 1) = static_cast<tFloat32 >(convArray[1]);

        covmat.at<float>(1, 0) = static_cast<tFloat32 >(convArray[2]);
        covmat.at<float>(1, 1) = static_cast<tFloat32 >(convArray[3]);
    }



     object_ptr<const ISample> pDebugPointSample;

     if (IS_OK(DebugPointIn.GetLastSample(pDebugPointSample))) {
         auto oDecoder = m_DebugPointStructSampleFactory.MakeDecoderFor(*pDebugPointSample);

         RETURN_IF_FAILED(oDecoder.IsValid());

         const DebugPoint *particleArray = reinterpret_cast<const DebugPoint *>(oDecoder.GetElementAddress(
                 m_ddlDebugPointId.debugPoints));

         debugPointList.clear();

         tUInt32 size;
         RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDebugPointId.numPoints, &size));

         for (tUInt32 i = 0; i < size; i++){

             DebugPoint p;
             p.x = particleArray[i].x;
             p.y = particleArray[i].y;
             debugPointList.push_back(p);

         }
     }


    object_ptr<const ISample> pReadParticleSample;

    if (IS_OK(ParticleDataIn.GetLastSample(pReadParticleSample))) {
        auto oDecoder = m_ParticleStructSampleFactory.MakeDecoderFor(*pReadParticleSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        const vizParticle *particleArray = reinterpret_cast<const vizParticle *>(oDecoder.GetElementAddress(
                m_ddlParticleDataId.particleArray));

        particles_debug.clear();

        int size = particleArray[0].x;

        for (int i = 1; i < size + 1; i++){

            vizParticle p;
            p.x = particleArray[i].x ;
            p.y = particleArray[i].y ;
            p.a = particleArray[i].a;
            p.b = particleArray[i].b;
            p.skip = particleArray[i].skip;
            particles_debug.push_back(p);

        }
    }


    RETURN_NOERROR;


 }
void cWorldViz::ReCenterImage() {

    int x_offset = int(round(WINDOW_WIDTH/float(2)));
    int y_offset = round(WINDOW_HEIGHT/float(2));


    cv::Point2f centerPivot( (carPose.x*METERS_TO_PIXELS + WORLD_CENTER), (carPose.y*METERS_TO_PIXELS + WORLD_CENTER));
    cv::Point2f botLeftPivot( (int)round(centerPivot.x - x_offset), (int)round(centerPivot.y - y_offset));
    cv::Point2f topRightPivot( (int)round(centerPivot.x + x_offset), (int)round(centerPivot.y + y_offset));

    roi = Rect(botLeftPivot,topRightPivot );
    //LOG_INFO(cString::Format("Eigen: (%f, %f)", botLeftPivot.x, botLeftPivot.y));

    //roi = Rect(int(round(WORLD_CENTER - WINDOW_WIDTH/2)),int(round(WORLD_CENTER - WINDOW_WIDTH/2)),int(WINDOW_WIDTH),int(WINDOW_WIDTH));

     mImageGridMasked = mImageGrid(roi);// + mImagePath(roi);
     mImagePathMasked = mImagePath(roi);

     //mImagePath = mImagePath(roi);

}


 void cWorldViz::PlotParticles() {

}

void cWorldViz::StreamWorld(){


//    PlotErrorEllipse();
    ReCenterImage();
    updateImage();
    ComposeImage();
    cv::cvtColor( mImage, mImageSend, CV_RGBA2RGB );
    cv::flip( mImageSend, mImageFlipped, 0 );

    Mat outputImage = mImageFlipped;

    //cv::cvtColor( mImageDebug, mImageSend, CV_RGBA2RGB );
    //outputImage = mImageSend;
    //Write processed Image to Output Pin
    if (!outputImage.empty())
    {
        //update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize() != m_sImageFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_oWriter, outputImage);
        }
        // write to pin
        writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
    }

}
 void cWorldViz::ComposeImage()
 {


     //mImage = mImageGridMasked; //+ mImagePath;
     mImage = mImageGridMasked + mImagePathMasked + mImageOverlay + mImageDebug;
 }


 void cWorldViz::updateImage()
 {
     //cv::Point2f newPos( round(carPose.x*METERS_TO_PIXELS + WORLD_CENTER), round(carPose.y*METERS_TO_PIXELS + WORLD_CENTER ));
     cv::Point2f newPos( WINDOW_CENTER, WINDOW_CENTER);
     //cv::line( mImagePath, mPreviousPos, newPos, cv::Scalar( 255, 255, 255, 255 ) );
     // Reset overlay:
     mImageOverlay = cv::Scalar( 0, 0, 0, 0 );
     // Draw debug points:
             /*
     std::vector<DebugPoint>::iterator it;
     int i = 0;
     for( it = mDrawPoints.begin(); it != mDrawPoints.end(); )
     {
         cv::circle( mImageOverlay, it->pos, 1, it->color );
         if( it->killTime > mLastTime )
         {
              // COMMENT OUT THIS MIDDLE STUFF
             tFloat32 numer = (tFloat32(mLastTime) - tFloat32(it->startTime));
             tFloat32 denom = (tFloat32(it->killTime) - tFloat32(it->startTime));
             tFloat32 amount = std::min(std::max(numer/denom,0.0),1.0);
             //tFloat32 amount = (tFloat32(mLastTime) - tFloat32(it->startTime))/(tFloat32(it->killTime) - tFloat32(it->startTime));
             it->color.val[3] = amount;
             if( i == mDrawPoints.size()-1 )
             {
                 std::cout << "amount: " << amount << " " << numer << " " << denom << std::endl;
             }
             it ++;
         } else {
             it = mDrawPoints.erase(it);
         }
         i++;
     }
     */

     // Draw line indicating carPose direction
     cv::Point2f movement( round( 10*cos( carPose.theta ) ), round( 10*sin( carPose.theta ) ) );

     cv::Point2f lookAtPos = newPos + movement;
     cv::line( mImageOverlay, newPos, lookAtPos, cv::Scalar( 0, 255, 0, 255 ) );

     mImagePath = cv::Scalar( 0, 0, 0, 0 );

     LOG_INFO("THINKING AOBUT IT");
     for (int i = 0; i < debugPointList.size(); i++) {
         LOG_INFO("DOING IT");

         cv::circle(mImagePath,
                    cv::Point((int)round(debugPointList[i].x * METERS_TO_PIXELS + WORLD_CENTER), (int)round(debugPointList[i].y * METERS_TO_PIXELS + WORLD_CENTER)),
                    10,
                    cv::Scalar( 255, 255, 255, 255),
                    2,
                    8);

         cv::circle(mImagePath,
                    cv::Point((int)round( WORLD_CENTER), (int)round(WORLD_CENTER)),
                    10,
                    cv::Scalar( 255, 255, 255, 255),
                    2,
                    8);

     }
     mPreviousPos = newPos;

 }


void cWorldViz::PlotErrorEllipse(){
    //The mean of our data
    cv::Point2f mean(WINDOW_CENTER, WINDOW_CENTER);

    //Calculate the error ellipse for a 95% confidence intervanl
    cv::RotatedRect ellipse = getErrorEllipse(2.4477, mean, covmat);

    //Show the result
    mImageDebug = cv::Scalar( 0, 0, 0, 0 );
    mImagePath = cv::Scalar( 0, 0, 0, 0 );
    //mImagePath = cv::Scalar( 0, 0, 0, 0 );
    cv::ellipse(mImageDebug, ellipse, cv::Scalar::all(255), 1);

    int point_pairs = 1;
    cv::Scalar colour = cv::Scalar( 255, 0, 0, 255);
    int radius =1;
    for (int i = 0; i < particles_debug.size(); i++){

        if (!particles_debug[i].skip){
            cv::circle(mImagePath,
                       cv::Point((int)round(particles_debug[i].x * METERS_TO_PIXELS + WORLD_CENTER), (int)round(particles_debug[i].y * METERS_TO_PIXELS + WORLD_CENTER)),
                       radius,
                       colour,
                       2,
                       8);

            double x2 = sqrt( pow(line_debug_length,2) / (1 + pow(particles_debug[i].a,2) )) + particles_debug[i].x;
            double y2 = particles_debug[i].a * x2 + particles_debug[i].b;

            double x3 = -1 * sqrt( pow(line_debug_length,2) / (1 + pow(particles_debug[i].a,2) )) + particles_debug[i].x;
            double y3 = particles_debug[i].a * x3 + particles_debug[i].b;

            cv::Point2f line_outerA_start( (int)round(x2*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y2*METERS_TO_PIXELS + WORLD_CENTER));
            cv::Point2f line_outerA_end( (int)round(x3*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y3*METERS_TO_PIXELS + WORLD_CENTER));

            cv::line( mImagePath, line_outerA_start, line_outerA_end, colour );
        }

        else {
            cv::circle(mImagePath,
                       cv::Point((int)round(particles_debug[i].x * METERS_TO_PIXELS + WORLD_CENTER), (int)round(particles_debug[i].y * METERS_TO_PIXELS + WORLD_CENTER)),
                       radius + 1,
                       cv::Scalar::all(255),
                       2,
                       8);

            double x2 = sqrt( pow(line_debug_length,2) / (1 + pow(particles_debug[i].a,2) )) + particles_debug[i].x;
            double y2 = particles_debug[i].a * x2 + particles_debug[i].b;

            double x3 = -1 * sqrt( pow(line_debug_length,2) / (1 + pow(particles_debug[i].a,2) )) + particles_debug[i].x;
            double y3 = particles_debug[i].a * x3 + particles_debug[i].b;

            cv::Point2f line_outerA_start( (int)round(x2*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y2*METERS_TO_PIXELS + WORLD_CENTER));
            cv::Point2f line_outerA_end( (int)round(x3*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y3*METERS_TO_PIXELS + WORLD_CENTER));

            cv::line( mImagePath, line_outerA_start, line_outerA_end, cv::Scalar::all(255));
        }

        if ((point_pairs%2) == 0){

            colour =  cv::Scalar( 0, 255, 0, 255);
            //radius = 5;
        }

        point_pairs += 1;

    }
//
//
//
//
//    cv::circle(mImagePath,
//               cv::Point((int)round(particleMiddle->x * METERS_TO_PIXELS + WORLD_CENTER), (int)round(particleMiddle->y * METERS_TO_PIXELS + WORLD_CENTER)),
//               5,
//               cv::Scalar::all(255),
//               2,
//               8);
//
//    x2 = sqrt( pow(line_debug_length,2) / (1 + pow(particleMiddle->a,2) )) + particleMiddle->x;
//    y2 = particleMiddle->a * x2 + particleMiddle->b;
//
//    x3 = -1 * sqrt( pow(line_debug_length,2) / (1 + pow(particleMiddle->a,2) )) + particleMiddle->x;
//    y3 = particleMiddle->a * x3 + particleMiddle->b;
//
//    cv::Point2f line_middle_start( (int)round(x2*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y2*METERS_TO_PIXELS + WORLD_CENTER));
//    cv::Point2f line_middle_end( (int)round(x3*METERS_TO_PIXELS + WORLD_CENTER), (int)round(y3*METERS_TO_PIXELS + WORLD_CENTER));
//
//    cv::line( mImagePath, line_middle_start, line_middle_end, cv::Scalar::all(255) );



    cv::flip( mImageDebug,mImageDebugTmp, 0 );
    mImageDebug = mImageDebugTmp;
}


cv::RotatedRect cWorldViz::getErrorEllipse(tFloat32 chisquare_val, cv::Point2f mean, cv::Mat covmat){

    //taken from http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/

    //Get the eigenvalues and eigenvectors
    cv::Mat eigenvalues, eigenvectors;
    //cv::eigen(covmat, true, eigenvalues, eigenvectors);
    cv::eigen(covmat, eigenvalues, eigenvectors);

    //Calculate the angle between the largest eigenvector and the x-axis
    tFloat32 angle = atan2(eigenvectors.at<tFloat32>(0,1), eigenvectors.at<tFloat32>(0,0));

    //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
    if(angle < 0)
        angle += 6.28318530718;

    //Conver to degrees instead of radians
    angle = 180*angle/3.14159265359;

    //Calculate the size of the minor and major axes
    tFloat32 halfmajoraxissize=chisquare_val*sqrt(eigenvalues.at<tFloat32>(0));
    tFloat32 halfminoraxissize=chisquare_val*sqrt(eigenvalues.at<tFloat32>(1));


    if (isnan(halfmajoraxissize ) || isnan(halfminoraxissize )){
        halfmajoraxissize = 1;
        halfminoraxissize = 1;
        angle = 0;
    }
//
//     LOG_INFO(cString::Format("Position Covariance (x = %f, xy = %f, yx = %f, x = %f)", covmat.at<tFloat32>(0, 0),
//                              covmat.at<tFloat32>(0, 1),covmat.at<tFloat32>(1, 0),covmat.at<tFloat32>(1, 1)));
//
//     LOG_INFO(cString::Format("Eigen: (%f, %f)", eigenvalues.at<tFloat32>(0), eigenvalues.at<tFloat32>(1)));
//
//     LOG_INFO(cString::Format("Ellipse: (%f, %f, %f)", halfmajoraxissize, halfminoraxissize, angle));
//     LOG_INFO(cString::Format("Theta: (%f, %f, %f)", carPose.x, carPose.y, carPose.theta));

    //Return the oriented ellipse
    //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
    return cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize * METERS_TO_PIXELS, halfminoraxissize * METERS_TO_PIXELS), -angle);

}



