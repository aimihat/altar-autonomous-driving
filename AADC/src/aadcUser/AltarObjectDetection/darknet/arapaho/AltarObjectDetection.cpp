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
#include "AltarObjectDetection.h"
#include "ADTF3_OpenCV_helper.h"
#include <string>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <sys/types.h>
#include <sys/stat.h>

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ALTAR_OBJECT_DETECTION_DATA_TRIGGERED_FILTER,
                                    "AltarObjectDetection",
                                    cObjectDetection,
                                    adtf::filter::pin_trigger({"input_front","input_rear"}));

cObjectDetection::cObjectDetection() {
    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(
                stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);
    
    
    RegisterPropertyVariable("detection_threshold", thresh);
    RegisterPropertyVariable("age_threshold", adult_ratio_threshold);
    RegisterPropertyVariable("ROI Top-Left x", roi_top_left_x);
    RegisterPropertyVariable("ROI Top-Left y", roi_top_left_y);
    RegisterPropertyVariable("ROI Width", roi_width);
    RegisterPropertyVariable("ROI Height", roi_height);
    
    object_ptr<IStreamType> pTypeObjectData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("ObjectData", pTypeObjectData, m_ObjectDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_ObjectDataSampleFactory, "nObjects", m_ddlObjectDataId.nObjects);
        adtf_ddl::access_element::find_array_index(m_ObjectDataSampleFactory, "objectsArray", m_ddlObjectDataId.objectsArray);
    }
    else
    {
        LOG_INFO("No mediadescription for ObjectData found!");
    }
    
    Register(m_oWriterObject, "objects_detected", pTypeObjectData);
    
    //Register input pin
    Register(m_oReader, "input_front", pType);
    Register(m_oRearReader, "input_rear", pType);
    //Register output pin
    Register(m_oWriter, "image coco", pType);
    Register(m_oBarbieWriter, "image barbie", pType);
    Register(m_oRearWriter, "image rear", pType);
    
    //register callback for type changes
    m_oReader.SetAcceptTypeCallback(
                [this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType> &pType) -> tResult {
        return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter);
    });

    //ARAPAHO FOR CARS (COCO DATASET)

    araho_params_coco = {};
    p_coco = new ArapahoV2();
    
    static char INPUT_DATA_FILE_COCO[] = "/home/aadc/audi/AADC/src/aadcUser/AltarObjectDetection/darknet/arapaho/input.data";
    static char INPUT_CFG_FILE_COCO[] = "/home/aadc/audi/AADC/src/aadcUser/AltarObjectDetection/darknet/arapaho/input.cfg";
    static char INPUT_WEIGHTS_FILE_COCO[] = "/home/aadc/audi/AADC/src/aadcUser/AltarObjectDetection/darknet/arapaho/input.weights";
    if (!fileExists(INPUT_DATA_FILE_COCO) || !fileExists(INPUT_CFG_FILE_COCO) || !fileExists(INPUT_WEIGHTS_FILE_COCO)) {
        LOG_ERROR("Setup failed as input files do not exist or not readable!\n");
    }
    araho_params_coco.datacfg = INPUT_DATA_FILE_COCO;
    araho_params_coco.cfgfile = INPUT_CFG_FILE_COCO;
    araho_params_coco.weightfile = INPUT_WEIGHTS_FILE_COCO;
    araho_params_coco.nms = 0.4;
    araho_params_coco.maxClasses = 2;
    
    int im_width = m_sImageFormat.m_ui32Width;
    int im_height = m_sImageFormat.m_ui32Height;
    
    bool ret = false;
    ret = p_coco->Setup(araho_params_coco, im_width, im_height);
    
    if (!ret) {
        LOG_ERROR("Araho Setup failed!\n");
        if (p_coco) delete p_coco;
        p_coco = 0;
    } else {
        LOG_INFO("Araho Setup Successful");
    }

    //ARAPAHO FOR BARBIES

    araho_params = {};
    p = new ArapahoV2();

    static char INPUT_DATA_FILE[] = "/home/aadc/audi/AADC/src/aadcUser/AltarObjectDetection/darknet/arapaho/barbies.data";
    static char INPUT_CFG_FILE[] = "/home/aadc/audi/AADC/src/aadcUser/AltarObjectDetection/darknet/arapaho/barbies-yolov3.cfg";
    static char INPUT_WEIGHTS_FILE[] = "/home/aadc/audi/AADC/src/aadcUser/AltarObjectDetection/darknet/arapaho/barbies-yolov3.backup";
    if (!fileExists(INPUT_DATA_FILE) || !fileExists(INPUT_CFG_FILE) || !fileExists(INPUT_WEIGHTS_FILE)) {
        LOG_ERROR("Setup failed as input files do not exist or not readable!\n");
    }
    araho_params.datacfg = INPUT_DATA_FILE;
    araho_params.cfgfile = INPUT_CFG_FILE;
    araho_params.weightfile = INPUT_WEIGHTS_FILE;
    araho_params.nms = 0.4;
    araho_params.maxClasses = 2;

    ret = false;
    ret = p->Setup(araho_params, im_width, im_height);

    if (!ret) {
        LOG_ERROR("Araho Setup failed!\n");
        if (p) delete p;
        p = 0;
    } else {
        LOG_INFO("Araho Setup Successful");
    }
    
}

tResult cObjectDetection::Configure() {
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    
    RETURN_NOERROR;
}

tResult cObjectDetection::ExtractBoxes(box *bxs, std::string *lbls, int obj_count, cv::Mat& inputImage, int camera) {
    Mat box_im;
    int img_height = inputImage.size().height; //in pixels
    int img_width = inputImage.size().width;

    //Extract particular box
    //Clip to avoid out-of-bonds
    int objId, leftTopX, leftTopY, rightBotX, rightBotY;

    for (objId = 0; objId < obj_count; objId++) {
        leftTopX = 1 + img_width * (bxs[objId].x - bxs[objId].w / 2);
        leftTopY = 1 + img_height * (bxs[objId].y - bxs[objId].h / 2);
        rightBotX = 1 + img_width * (bxs[objId].x + bxs[objId].w / 2);
        rightBotY = 1 + img_height * (bxs[objId].y + bxs[objId].h / 2);

        if (lbls[objId] == "barbie") {
            if ((bxs[objId].w * img_width) / (bxs[objId].h * img_height) >
                    adult_ratio_threshold) {
                lbls[objId] = "child";
            } else { lbls[objId] = "adult"; }
        }

        //add to outputpin objects

        tDetection objDetected;

        if (lbls[objId]=="car")
        {
            objDetected.detectedClass = 1;
            carDetection i;
            i.camera = camera;
            i.boundingBox = bxs[objId];
            carsDetected.push_back(i);
        } else if (lbls[objId] == "adult") {
            objDetected.detectedClass = 2;
        } else if (lbls[objId] == "child") {
            objDetected.detectedClass = 3;
        } else if (lbls[objId] == "person") {
            objDetected.detectedClass = 4;
        } else if (lbls[objId] == "siren") {
            objDetected.detectedClass = 5;
        }

        if (camera == 0) {// front cam only
            objDetected.pctOffsetCenter = (-0.5 + bxs[objId].x) * 100;
            objectsDetected.push_back(objDetected);
        }
        LOG_INFO("Box #%d: center {x,y}, box {w,h} = [%f, %f, %f, %f]\n",
                 objId, bxs[objId].x, bxs[objId].y, bxs[objId].w, bxs[objId].h);
        // Show image and overlay using OpenCV
        rectangle(inputImage,
                  cvPoint(leftTopX, leftTopY),
                  cvPoint(rightBotX, rightBotY),
                  CV_RGB(255, 0, 0), 1, 8, 0);
        // Show lbls
        if (lbls[objId].c_str()) {
            DPRINTF("Label:%s\n\n", lbls[objId].c_str());
            putText(inputImage, lbls[objId].c_str(), cvPoint(leftTopX, leftTopY),
                    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
        }
    }
    RETURN_NOERROR;
}

tResult cObjectDetection::Process(tTimeStamp tmTimeOfTrigger) {
    object_ptr<const ISample> pReadSample;
    Mat outputImage;
    int numObjects, numObjectsCoco, numObjectsRearCoco;
    std::string *labels = 0;
    box *boxes = 0;
    std::string *labelsCoco = 0;
    box *boxesCoco = 0;
    std::string *labelsRearCoco = 0;
    box *boxesRearCoco = 0;

    objectsDetected.clear();
    carsDetected.clear();
    
    //TODO: ignore multiple samples (they otherwise accumulate)
    while (IS_OK(m_oReader.GetNextSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {
            //create a opencv matrix from the media sample buffer
            Mat inputImage_uncropped = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                           CV_8UC3,
                                           const_cast<unsigned char *>(static_cast<const unsigned char *>(pReadBuffer->GetPtr())));
            
            
            //Fish-eye cropping for ROI (only 416x416 model or runs out of memory)
            Mat inputImage;
            inputImage_uncropped(Rect(roi_top_left_x, roi_top_left_y, roi_width, roi_height)).copyTo(inputImage);

            numObjects = numObjectsCoco = numObjectsRearCoco = 0;
            
            // Detect the objects in the image

            p->Detect(
                        inputImage,
                        0.24,
                        0.5,
                        numObjects);

            p_coco->Detect(
                    inputImage_uncropped,
                        0.24,
                        0.5,
                        numObjectsCoco);
            
            if (numObjectsCoco + numObjects > 0) {
                if (numObjects)
                {
                    boxes = new box[numObjects];
                    labels = new std::string[numObjects];
                }
                if (numObjectsCoco)
                {
                    boxesCoco = new box[numObjectsCoco];
                    labelsCoco = new std::string[numObjectsCoco];
                }
                
                // Get boxes and labels
                
                if (numObjects)
                {
                    p->GetBoxes(
                                boxes,
                                labels,
                                numObjects
                                );
                    ExtractBoxes(boxes, labels, numObjects, inputImage, 0);
                }
                if (numObjectsCoco)
                {
                    p_coco->GetBoxes(
                                boxesCoco,
                                labelsCoco,
                                numObjectsCoco
                                );
                    ExtractBoxes(boxesCoco, labelsCoco, numObjectsCoco, inputImage_uncropped, 0);
                }

                if (boxes) {
                    delete[] boxes;
                    boxes = 0;
                }
                if (boxesCoco) {
                    delete[] boxesCoco;
                    boxesCoco = 0;
                }
                if (labels) {
                    delete[] labels;
                    labels = 0;
                }
                if (labelsCoco) {
                    delete[] labelsCoco;
                    labelsCoco = 0;
                }
            }

            //Transmit output image
            object_ptr<ISample> pObjectSample;
            RETURN_IF_FAILED(alloc_sample(pObjectSample))
            {

                    auto oCodec = m_ObjectDataSampleFactory.MakeCodecFor(pObjectSample);

                    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlObjectDataId.nObjects, (tUInt32)objectsDetected.size()));

                    tDetection* objects = reinterpret_cast<tDetection*>(oCodec.GetElementAddress(m_ddlObjectDataId.objectsArray));

                    memset(objects,0,10*sizeof(tDetection));
                    //2-3 normally, but for some reason inverted
                    for (uint i = 0; i < objectsDetected.size(); i++)
                    {
                        LOG_INFO("OUTPUTING %d objects, %f, %f", objectsDetected.size(), objectsDetected[i].detectedClass,objectsDetected[i].pctOffsetCenter);
                        objects[i].detectedClass = (tFloat32)objectsDetected[i].detectedClass;
                        objects[i].pctOffsetCenter = (tFloat32)objectsDetected[i].pctOffsetCenter;
                    }

             }

             m_oWriterObject << pObjectSample << flush << trigger;
             outputImage = inputImage;

            //Write processed Image to Output Pin
            if (!inputImage.empty()) {
                //update output format if matrix size does not fit to
                if (inputImage.total() * inputImage.elemSize() != m_sImageFormat.m_szMaxByteSize) {
                    setTypeFromMat(m_oBarbieWriter, inputImage);
                }
                // write to pin
                writeMatToPin(m_oBarbieWriter, inputImage, m_pClock->GetStreamTime());
            }
            //Write processed Image to Output Pin
            if (!inputImage_uncropped.empty()) {
                //update output format if matrix size does not fit to
                if (inputImage_uncropped.total() * inputImage_uncropped.elemSize() != m_sImageFormat.m_szMaxByteSize) {
                    setTypeFromMat(m_oWriter, inputImage_uncropped);
                }
                // write to pin
                writeMatToPin(m_oWriter, inputImage_uncropped, m_pClock->GetStreamTime());
            }
        }

    }

    while (IS_OK(m_oRearReader.GetNextSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        Mat inputImageRear = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                       CV_8UC3,
                                       const_cast<unsigned char *>(static_cast<const unsigned char *>(pReadBuffer->GetPtr())));

        if (IS_OK(pReadSample->Lock(pReadBuffer))) {
            p_coco->Detect(
                    inputImageRear,
                    0.24,
                    0.5,
                    numObjectsRearCoco);
        }

        if (numObjectsRearCoco) {
            boxesRearCoco = new box[numObjectsRearCoco];
            labelsRearCoco = new std::string[numObjectsRearCoco];

            p_coco->GetBoxes(
                    boxesRearCoco,
                    labelsRearCoco,
                    numObjectsRearCoco
            );
            ExtractBoxes(boxesRearCoco, labelsRearCoco, numObjectsRearCoco, inputImageRear, 1);

            if (boxesRearCoco) {
                delete[] boxesRearCoco;
                boxesRearCoco = 0;
            }
            if (labelsRearCoco) {
                delete[] labelsRearCoco;
                labelsRearCoco = 0;
            }

        }
        if (!inputImageRear.empty()) {
            //update output format if matrix size does not fit to
            if (inputImageRear.total() * inputImageRear.elemSize() != m_sImageFormat.m_szMaxByteSize) {
                setTypeFromMat(m_oRearWriter, inputImageRear);
            }
            // write to pin
            writeMatToPin(m_oRearWriter, inputImageRear, m_pClock->GetStreamTime());
        }
    }

    RETURN_NOERROR;
}
