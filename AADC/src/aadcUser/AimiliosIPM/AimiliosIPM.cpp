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
#include "AimiliosIPM.h"
#include "ADTF3_OpenCV_helper.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CAIMILIOS_IPM_DATA_TRIGGERED_FILTER,
                                    "Aimilios IPM",
                                    cAimiliosIPM,
                                    adtf::filter::pin_trigger({ "input" }));

cAimiliosIPM::cAimiliosIPM()
{

    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oReader, "input", pType);
    //Register output pin
    Register(m_oWriter, "output", pType);

    const int width = 1280;
    const int height = 960;
    vector<Point2f> origPoints;
    origPoints.push_back(Point2f(170, height-210));
    origPoints.push_back(Point2f(width-170, height-210));
    origPoints.push_back(Point2f(width, height-450));
    origPoints.push_back(Point2f(0, height-450));
    vector<Point2f> dstPoints;
    dstPoints.push_back(Point2f(510, height));
    dstPoints.push_back(Point2f(width-510, height));
    dstPoints.push_back(Point2f(width+530,100));
    dstPoints.push_back(Point2f(0-450,100));
    m_ipm = std::make_unique<IPM> (Size(width,height), Size(width,height), origPoints, dstPoints);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter);
    });

    //GPU
    m_bGpuProcessing = tFalse;

    try
    {
        if (cv::cuda::getCudaEnabledDeviceCount() > 0)
        {
            m_bGpuAvailable = tTrue;

            //only set the properties if we have a cuda capable gpu.
            RegisterPropertyVariable("GPU processing", m_bGpuProcessing);
        }
        else
        {
            m_bGpuAvailable = tFalse;
            m_bGpuProcessing = tFalse;
        }
    }
    catch (cv::Exception& e)
    {
        const char* err_msg = e.what();
        LOG_ERROR(cString("OpenCV exception caught: ") + err_msg);
    }
}

tResult cAimiliosIPM::Configure()
{

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    
    RETURN_NOERROR;
}
int to_vary = 450;

tResult cAimiliosIPM::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    Mat outputImage;
    cuda::Stream stream;

    while (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                   CV_8UC3, const_cast<unsigned char*>(static_cast<const unsigned char*>(pReadBuffer->GetPtr())));

            outputImage.create(inputImage.size(), inputImage.type());

            m_GpuInputImage.upload(inputImage, stream);
            m_GpuGreyImage.upload(outputImage, stream);

            //Do the image processing and copy to destination image buffer
            cuda::cvtColor(m_GpuInputImage, m_GpuGreyImage, CV_BGR2GRAY, 0, stream);

            m_GpuInputImage.download(inputImage,stream);
            m_GpuGreyImage.download(inputImage, stream);

            m_ipm->applyHomography( inputImage, outputImage );
            cv::cvtColor(outputImage, outputImage, CV_GRAY2BGR);
            //outputImage = inputImage;
            //            clock_t end = clock();
            //            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            //            printf("%.2f (ms)\r", 1000*elapsed_secs);
            //            ipm.drawPoints(origPoints, inputImageGray );

        }
    }

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

    RETURN_NOERROR;
}
