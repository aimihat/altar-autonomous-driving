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
#include "FlipVerticalCam.h"
#include "ADTF3_OpenCV_helper.h"
#include <algorithm>
#include "spline.h"

/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CLANEDETECTION_DATA_TRIGGERED_FILTER,
                                    "Lane Detection",
                                    cLaneDetection,
                                    adtf::filter::pin_trigger({"in"}));


cLaneDetection::cLaneDetection()
{

    object_ptr<IStreamType> pTypeLPData;


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


tResult cLaneDetection::Configure()
{
    //get clock object
    RETURN_NOERROR;
}

tResult cLaneDetection::ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
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

tResult cLaneDetection::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oReaderVideo.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        Mat outputImage;


        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat inputImage(cv::Size(m_InPinVideoFormat.m_ui32Width, m_InPinVideoFormat.m_ui32Height),
                           CV_8UC3, (uchar*)pReadBuffer->GetPtr());


            cvtColor(inputImage, outputImage, COLOR_BGR2GRAY);
            adaptiveThreshold(outputImage, outputImage, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, m_sizeSegmentImageBinarization, m_thresholdImageBinarization);// Generate Binary Image
            // Detect Lines

            cvtColor(outputImage, outputImage, COLOR_GRAY2RGB);



        //Write processed Image to Output Pin
        if (!outputImage.empty())
        {
            //update output format if matrix size does not fit to
            if (outputImage.total() * outputImage.elemSize() != m_OutPinVideoFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_oWriterVideo, outputImage);
            }
            // write to pin
            writeMatToPin(m_oWriterVideo, outputImage, m_pClock->GetStreamTime());
        }
    }

    RETURN_NOERROR;
}


