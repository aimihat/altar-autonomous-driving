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
#include "Crosswalk.h"
#include "ADTF3_OpenCV_helper.h"
#include <algorithm>

/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_cCrosswalk_DATA_TRIGGERED_FILTER,
                                    "Crosswalk Detector",
                                    cCrosswalk,
                                    adtf::filter::pin_trigger({"in"}));


cCrosswalk::cCrosswalk()
{
    //Register Properties
    RegisterPropertyVariable("ROIOffsetX [Pixel]",      m_ROIOffsetX);
    RegisterPropertyVariable("ROIOffsetY [Pixel]",      m_ROIOffsetY);
    RegisterPropertyVariable("ROIWidth [Pixel]",        m_ROIWidth);
    RegisterPropertyVariable("ROIHeight [Pixel]",       m_ROIHeight);
    RegisterPropertyVariable("detectionLines",  m_detectionLines);
    RegisterPropertyVariable("minLineWidth [Pixel]", m_minLineWidth);
    RegisterPropertyVariable("maxLineWidth [Pixel]", m_maxLineWidth);
    RegisterPropertyVariable("minLineContrast", m_minLineContrast);
    RegisterPropertyVariable("m_sizeSegmentImageBinarization", m_sizeSegmentImageBinarization);
    RegisterPropertyVariable("thresholdImageBinarization", m_thresholdImageBinarization);
    RegisterPropertyVariable("clusterThreshold", m_clusterThreshold);
    RegisterPropertyVariable("maximumLaneSeparation", m_maximumLaneSeparation);
    RegisterPropertyVariable("minimumLaneSeparation", m_minimumLaneSeparation);
    RegisterPropertyVariable("frontCarPositionX", m_frontCarPositionX);
    RegisterPropertyVariable("frontCarPositionY", m_frontCarPositionX);
    RegisterPropertyVariable("clusterHeightLimit", m_clusterHeightLimit);
    RegisterPropertyVariable("pixelToMillimiterX", m_pixelToMillimiterX);
    RegisterPropertyVariable("pixelToMillimiterY", m_pixelToMillimiterY);
    RegisterPropertyVariable("steeringWeight", m_steeringWeight);
    object_ptr<IStreamType> pTypeLPData;

    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.timeStamp);
        adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue"), m_ddlBoolSignalValueId.value);
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    Register(m_oWriter, "output", pTypeBoolSignalValue);

    //create and set inital input format type
    m_InPinVideoFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pTypeInput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeInput, m_InPinVideoFormat);
    //Register input pin
    Register(m_oReaderVideo, "in", pTypeInput);

    //Register output pins
    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_OutPinVideoFormat);
    Register(m_oWriterVideo, "outVisual", pTypeOutput);


    //register callback for type changes
    m_oReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
                                         {
                                             return ChangeType(m_oReaderVideo, *pType.Get());
                                         });




}


tResult cCrosswalk::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}


tResult cCrosswalk::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oReaderVideo.GetNextSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        Mat outputImage;

        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {
            //create a opencv matrix from the media sample buffer
            Mat inputImage(cv::Size(m_InPinVideoFormat.m_ui32Width, m_InPinVideoFormat.m_ui32Height),
                           CV_8UC3, (uchar *) pReadBuffer->GetPtr());

//            Mat inputImage = imread("/home/robin/Desktop/audi/AADC/src/aadcUser/AltarCrosswalk/crosswalk.jpg", CV_LOAD_IMAGE_COLOR);


            cvtColor(inputImage, outputImage, COLOR_BGR2GRAY);
            adaptiveThreshold(outputImage, outputImage, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY,
                              m_sizeSegmentImageBinarization, m_thresholdImageBinarization);// Generate Binary Image


            Mat ScanZone = Mat(0, outputImage.cols, CV_8U);
            Mat ScanAvg = Mat(1, m_ROIWidth, CV_8U);

            for(int i = m_ROIOffsetY; i < m_ROIOffsetY + m_ROIHeight; i++)
            {
                ScanZone.push_back(outputImage.row(i));
            }

//            LOG_INFO("Cols: %d rows: %d, val at (0,0): %d", ScanZone.cols, ScanZone.rows, ScanZone.at<uchar>(0,0));
//            LOG_INFO("1");
            for(int col = m_ROIOffsetX; col< m_ROIOffsetX + m_ROIWidth; col++)
            {
                int sum = 0;
                for(int row = 0; row < m_ROIHeight; row++)
                {
                    sum += ScanZone.at<uchar>(row,col);
                }
                ScanAvg.at<uchar>(0,col-m_ROIOffsetX) = uchar(sum/m_ROIHeight);
            }
//            LOG_INFO("Cols: %d rows: %d, val at (0,0): %d", ScanAvg.cols, ScanAvg.rows, ScanAvg.at<uchar>(0,0));


            int count = 0;
            int last_count = 0;
            int periods = 0;
            bool count_start = false;
            bool state = false;

//            for(int col = m_ROIOffsetX; col< m_ROIOffsetX + m_ROIWidth; col++){
//                LOG_INFO("%d", (int)ScanZone.at<uchar>(0, col) );
//            }


            for(int i =0; i<ScanAvg.cols; i++)
                LOG_INFO("%d", (int)ScanAvg.at<uchar>(0, i) );

            int width = 0;
            for(int i = 0; i < ScanAvg.cols; i++)
            {

                if (state == LOW) {
                    if ((int)ScanAvg.at<uchar>(0, i) > 127) {

                        last_count = count;
                        state = HIGH;
                        count_start = true;
                    }else{
                        state = LOW;
                    }
                }else {
                    if ((int)ScanAvg.at<uchar>(0, i) > 127) {
                        state = HIGH;
                        width++;
                    }else{
                        state = LOW;
                        if (count_start && (width > 20)) periods ++;
                        width = 0;
                    }
                }


                if (count_start)
                    count++;
            }

//            LOG_INFO("Periods: %d Count: %d Last count: %d", periods, count, last_count);
//            double frequency = double(periods)/last_count;

//            LOG_INFO("Frequency: %f", frequency);
            tBoolSignalValue crosswalkSignal;


            if (periods == 4) crosswalkSignal.bValue = true;
            else crosswalkSignal.bValue = false;

            TransmitCrosswalk(crosswalkSignal);

            rectangle(outputImage, m_LaneRoi, Scalar(255), 10, 8, 0);

            m_LaneRoi = cv::Rect2f(static_cast<tFloat32>(m_ROIOffsetX), static_cast<tFloat32>(m_ROIOffsetY), static_cast<tFloat32>(m_ROIWidth), static_cast<tFloat32>(m_ROIHeight));


            //Write processed Image to Output Pin
            if (!outputImage.empty()) {
                //update output format if matrix size does not fit to
                if (outputImage.total() * outputImage.elemSize() != m_OutPinVideoFormat.m_szMaxByteSize) {
                    setTypeFromMat(m_oWriterVideo, outputImage);
                }
                // write to pin
                writeMatToPin(m_oWriterVideo, outputImage, m_pClock->GetStreamTime());
            }
        }
    }

    RETURN_NOERROR;
}


tResult cCrosswalk::checkRoi(void)
{
    // if width or heigt are not set ignore the roi
    if (static_cast<tFloat32>(m_ROIWidth) == 0 || static_cast<tFloat32>(m_ROIHeight) == 0)
    {
        LOG_ERROR("ROI width or height is not set!");
        RETURN_ERROR_DESC(ERR_INVALID_ARG, "ROI width or height is not set!");
    }

    //check if we are within the boundaries of the image
    if ((static_cast<tFloat32>(m_ROIOffsetX) + static_cast<tFloat32>(m_ROIWidth)) > m_InPinVideoFormat.m_ui32Width)
    {
        LOG_ERROR("ROI is outside of image");
        RETURN_ERROR_DESC(ERR_INVALID_ARG, "ROI is outside of image");
    }

    if ((static_cast<tFloat32>(m_ROIOffsetY) + static_cast<tFloat32>(m_ROIHeight)) > m_InPinVideoFormat.m_ui32Height)
    {
        LOG_ERROR("ROI is outside of image");
        RETURN_ERROR_DESC(ERR_INVALID_ARG, "ROI is outside of image");
    }

    //create the rectangle
    m_LaneRoi = cv::Rect2f(static_cast<tFloat32>(m_ROIOffsetX), static_cast<tFloat32>(m_ROIOffsetY), static_cast<tFloat32>(m_ROIWidth), static_cast<tFloat32>(m_ROIHeight));


    RETURN_NOERROR;
}

tResult cCrosswalk::ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
                                   const adtf::streaming::ant::IStreamType& oType)
{
    if (oType == adtf::streaming::stream_meta_type_image())
    {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pTypeInput;
        // get pType from input reader
        inputPin >> pTypeInput;
        adtf::streaming::get_stream_type_image_format(m_InPinVideoFormat, *pTypeInput);

        //set also output format
        adtf::streaming::get_stream_type_image_format(m_OutPinVideoFormat, *pTypeInput);
        //we always have a grayscale output image
        m_OutPinVideoFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
        // and set pType also to samplewriter
        m_oWriterVideo << pTypeInput;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}

tResult cCrosswalk::TransmitCrosswalk(tBoolSignalValue crosswalkSignal)
{
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {

        auto oCodec = m_BoolSignalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.timeStamp, crosswalkSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.value, crosswalkSignal.bValue));

    }

    m_oWriter << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}
