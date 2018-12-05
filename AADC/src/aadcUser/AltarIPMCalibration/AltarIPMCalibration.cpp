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

#include "AltarIPMCalibration.h"
#include "ADTF3_helper.h"
#include "ADTF3_OpenCV_helper.h"

ADTF_PLUGIN("IPM Calibration Plugin", cAltarIPMCalibrationFilter)


cAltarIPMCalibrationFilter::cAltarIPMCalibrationFilter() : m_pWidget(nullptr)
{
    RegisterPropertyVariable("Calibration Pattern"         ,  m_calibPatter                  );
    RegisterPropertyVariable("Debug Output to Console"     ,  m_calibDebugOutputToConsole    );
    RegisterPropertyVariable("Width [number of squares]"   ,  m_calibWidth                   );
    RegisterPropertyVariable("Height [number of squares]"  ,  m_calibHeight                  );
    RegisterPropertyVariable("Delay [s]"                   ,  m_calibDelay                   );
    RegisterPropertyVariable("Number of Datasets to use"   ,  m_calibDatasetsToUse           );
    RegisterPropertyVariable("Square Size [m]"             ,  m_calibSquareSize              );
    RegisterPropertyVariable("Aspect Ratio"                ,  m_calib_AspectRation           );

    //create and set inital input format type
    m_sInputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sInputFormat);

    //Register input pin
    create_pin(*this, m_oPinInputVideo, "video_rgb", pType);

    //register callback for type changes
    m_oPinInputVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oPinInputVideo, m_sInputFormat, *pType.Get());
    });
}


cAltarIPMCalibrationFilter::~cAltarIPMCalibrationFilter()
{

}

QWidget* cAltarIPMCalibrationFilter::CreateView()
{
    // use single UI File in background
    m_pWidget = new cAltarIPMCalibrationWidget(nullptr);

    connect(this, SIGNAL(newImage(const QImage &)), m_pWidget, SLOT(OnNewImage(const QImage &)));
    connect(m_pWidget->m_btStartIPM, SIGNAL(clicked()), this, SLOT(OnStartCalibrationIPM()));
    connect(m_pWidget->m_btCancel, SIGNAL(clicked()), this, SLOT(OnCancel()));
    connect(m_pWidget, SIGNAL(SendSaveAs(QString)), this, SLOT(OnSaveAs(QString)));
    connect(this, SIGNAL(sendState(int)), m_pWidget, SLOT(OnSetState(int)));
    connect(this, SIGNAL(sendMessage(cString)), m_pWidget, SLOT(OnSendMessage(cString)));

    return m_pWidget;
}


tVoid cAltarIPMCalibrationFilter::ReleaseView()
{
    delete m_pWidget;
    m_pWidget = nullptr;
}

tResult cAltarIPMCalibrationFilter::OnIdle()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    RETURN_NOERROR;
}

tResult cAltarIPMCalibrationFilter::OnTimer()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    object_ptr<const ISample> pReadSample;

    if(IS_OK(m_oPinInputVideo.GetLastSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat m_inputImage = Mat(cv::Size(m_sInputFormat.m_ui32Width, m_sInputFormat.m_ui32Height),
                                   CV_8UC3, (uchar*) pReadBuffer->GetPtr());

            //Do the image processing and copy to destination image buffer
            ProcessVideo(m_inputImage);
        }
        pReadBuffer->Unlock();
    }

    RETURN_NOERROR;
}

tResult cAltarIPMCalibrationFilter::Init(tInitStage eStage)
{   
    // set the member variables from the properties
    // debug mode or not
    m_bDebugModeEnabled = m_calibDebugOutputToConsole;
    // get calibration pattern
    if (m_calibPatter == 1) m_calibrationSettings.calibrationPattern = ipmCalibrationSettings::CHESSBOARD;
    // get width and height of board
    m_calibrationSettings.boardSize = Size(m_calibWidth, m_calibHeight);

    // set the member variables for the calibration (see header)
    m_calibrationSettings.delay = m_calibDelay * 1e6;
    m_calibrationSettings.nrFrames = m_calibDatasetsToUse;
    m_calibrationSettings.nrFramesDataAq = m_calibrationSettings.nrFrames * 5;
    m_calibrationSettings.squareSize = m_calibSquareSize;
    m_calibrationSettings.aspectRatio = m_calib_AspectRation;
    m_calibrationSettings.writeExtrinsics = true;
    m_calibrationSettings.writePoints = false;
    m_calibrationSettings.calibZeroTangentDist = true;
    m_calibrationSettings.calibFixPrincipalPoint = true;
    m_calibrationSettings.flag = 0;
    // set the state and other necessary variables
    m_prevTimestamp = 0;
    m_imagePoints.clear();
    m_calibrationState = WAITING;
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));
    RETURN_NOERROR;
}

tResult cAltarIPMCalibrationFilter::ProcessVideo(Mat& inputMat)
{
    //copy to new mat and convert to grayscale otherwise the image in original media sample is modified
    Mat frame_gray;
    cvtColor(inputMat, frame_gray, COLOR_BGR2GRAY);

    //variables for detection
    tBool bFoundPattern = tFalse;
    vector<Point2f> pointbuf;

    // is not calibrated and is not waiting
    if (m_calibrationState == CAPTURING)
    {
     
        //m_countAttemptsSearchChessboard++;
        //LOG_INFO(cString::Format("Attempt %i/%i", m_countAttemptsSearchChessboard, m_maxCountAttemptsSearchChessboard));


        int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

        chessBoardFlags |= CALIB_CB_FAST_CHECK;

        //find pattern in input image, result is written to pointbuf
        switch (m_calibrationSettings.calibrationPattern)
        {
            case ipmCalibrationSettings::CHESSBOARD:
                bFoundPattern = findChessboardCorners(inputMat, m_calibrationSettings.boardSize, pointbuf,
                                                      chessBoardFlags);
                break;
            default:
                LOG_ERROR("Unknown Calibration Pattern chosen");
                break;
        }

        // improve the found corners' coordinate accuracy
        if (bFoundPattern)
        {
            //m_countAttemptsSearchChessboard = 0;

            if (m_calibrationSettings.calibrationPattern == ipmCalibrationSettings::CHESSBOARD)
            {
                cornerSubPix(frame_gray, pointbuf, Size(11, 11),
                             Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 300, 0.1));
            }

            //if the delay time is passed and the pattern is found create a valid calibration dataset in m_imagePoints
            if ((abs(m_prevTimestamp - GetTime()) > m_calibrationSettings.delay))
            {
                m_imagePoints.push_back(pointbuf);
                m_prevTimestamp = GetTime();
                emit sendMessage(cString::Format("Processed %d of %d needed datasets", m_imagePoints.size(), m_calibrationSettings.nrFramesDataAq));
                LOG_INFO(cString::Format("Processed %d of %d needed datasets", m_imagePoints.size(), m_calibrationSettings.nrFramesDataAq));
            }
            else
            {
                emit sendMessage(cString::Format("Delay time was not passed"));
                LOG_INFO(cString::Format("Delay time was not passed"));
            }

            //draw the corners in the gray image
            drawChessboardCorners(frame_gray, m_calibrationSettings.boardSize, Mat(pointbuf), bFoundPattern);
        }
        //if more image points than the defined numbers are found switch to state CALIBRATED
        if (m_imagePoints.size() >= unsigned(m_calibrationSettings.nrFramesDataAq))
        {
            m_matrixSize = frame_gray.size();
            m_calibrationState = CAPTURING_FINISHED;
            emit sendState(m_calibrationState);
        }

    }

    //create an qimage from the data to print to qt gui
    QImage image(frame_gray.data, frame_gray.cols, frame_gray.rows, static_cast<int>(frame_gray.step), QImage::Format_Indexed8);

    // for a grayscale image there is no suitable definition in qt so we have to create our own color table
    QVector<QRgb> grayscaleTable;
    for (int i = 0; i < 256; i++) grayscaleTable.push_back(qRgb(i, i, i));
    image.setColorTable(grayscaleTable);

    //send the image to the gui
    emit newImage(image.scaled(320, 240, Qt::KeepAspectRatio));

    RETURN_NOERROR;
}

tBool cAltarIPMCalibrationFilter::runCalibrationAndSave(Size imageSize, Mat& homoMatrix,
                                                      vector<vector<Point2f>> imagePoints)
{
    // for more details and comments look to original Source at opencv/samples/cpp/calibration.cpp
    bool ok = tFalse;

    try
    {
        ok = runCalibration(imageSize, homoMatrix, imagePoints);
    }
    catch (cv::Exception& e)
    {
        LOG_ERROR(cString::Format("CV exception caught: %s", e.what()));
    }

    if (ok)
        LOG_INFO(cString::Format("Calibration succeeded"));
    else
        LOG_INFO(cString::Format("Calibration failed"));

    if (ok)
        saveCameraParams(imageSize, homoMatrix, imagePoints);
    return ok;
}

tBool cAltarIPMCalibrationFilter::runCalibration(Size& imageSize, Mat& homoMatrix,
                                                    vector<vector<Point2f>> imagePoints)
{


    vector<vector<Point2f> > objectPoints(1);
    calcChessboardCorners(m_calibrationSettings.boardSize, m_calibrationSettings.squareSize, objectPoints[0], m_calibrationSettings.calibrationPattern);

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    bool ok = false;

    vector<Point2f> objCorners = {objectPoints[0][0],objectPoints[0][8],objectPoints[0][45],objectPoints[0][53]};
    vector<Point2f> imgCorners = {imagePoints[0][45], imagePoints[0][53], imagePoints[0][0], imagePoints[0][8]};

//    Mat object, image;

//    fillPoly(object, objCorners, cv::Scalar (255, 255, 255), 8);
//    fillPoly(image, imgCo rners, cv::Scalar (255, 255, 255), 8);

//    imshow("object coordinate", object);
//    imshow("image coordinates", image);

    homoMatrix = getPerspectiveTransform(imgCorners, objCorners/* , RANSAC, 3 */);

    ok = !(homoMatrix.empty());

    return ok;
}


void cAltarIPMCalibrationFilter::saveCameraParams(Size& imageSize, Mat& homoMatrix,
                                                     const vector<vector<Point2f> >& imagePoints)
{
    FileStorage fs(m_calibrationSettings.outputFileName, FileStorage::WRITE);

    time_t tm;
    time(&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << m_calibrationSettings.boardSize.width;
    fs << "board_height" << m_calibrationSettings.boardSize.height;
    fs << "square_size" << m_calibrationSettings.squareSize;

    if (m_calibrationSettings.flag & CALIB_FIX_ASPECT_RATIO)
        fs << "fix_aspect_ratio" << m_calibrationSettings.aspectRatio;

    fs << "flags" << m_calibrationSettings.flag;

    fs << "homography_matrix" << homoMatrix;

    if (m_calibrationSettings.writePoints && !imagePoints.empty())
    {
        Mat imagePtMat((int) imagePoints.size(), (int) imagePoints[0].size(), CV_32FC2);
        for (size_t i = 0; i < imagePoints.size(); i++)
        {
            Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}


void cAltarIPMCalibrationFilter::calcChessboardCorners(Size boardSize, float squareSize, vector<Point2f>& corners, ipmCalibrationSettings::Pattern patternType)
{
    corners.clear();

    switch (patternType)
    {
        case ipmCalibrationSettings::CHESSBOARD:
        case ipmCalibrationSettings::CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; ++i)
                for (int j = 0; j < boardSize.width; ++j)
                    corners.push_back(Point2f(j*squareSize, i*squareSize));
            break;

        case ipmCalibrationSettings::ASYMMETRIC_CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.push_back(Point2f((2 * j + i % 2)*squareSize, i*squareSize));
            break;
        default:
            break;
    }
}

tResult cAltarIPMCalibrationFilter::resetCalibrationResults()
{
    m_matrixSize = cv::Size(0, 0);
    m_imagePoints.clear();
    m_prevTimestamp = 0;
    m_homoMatrix = Mat::eye(3, 3, CV_64F);

    RETURN_NOERROR;
}

tTimeStamp cAltarIPMCalibrationFilter::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime() : cSystem::GetTime();
}

void cAltarIPMCalibrationFilter::OnStartCalibrationIPM()
{
    resetCalibrationResults();
    // switch state to CAPTURING
    m_calibrationState = CAPTURING;
    m_calibrationSettings.flag = 0;
    m_calibrationSettings.useIPM = tTrue;
    emit sendState(CAPTURING);
    LOG_INFO("Starting Calibration IPM");
}

void cAltarIPMCalibrationFilter::OnSaveAs(QString qFilename)
{
    // save the calibration results to the given file in argument
    if (m_calibrationState == CAPTURING_FINISHED)
    {

        // convert file name to absolute file
        cFilename filename = qFilename.toStdString().c_str();
        //ADTF_GET_CONFIG_FILENAME(filename);
        filename = filename.CreateAbsolutePath(".");
        m_calibrationSettings.outputFileName = filename;
        LOG_INFO(cString::Format("Attempt to save calibration to %s", filename.GetPtr()));

        if (runCalibrationAndSave(m_matrixSize, m_homoMatrix, m_imagePoints))
        {
            LOG_INFO(cString::Format("Saved calibration to %s", filename.GetPtr()));
            // switch to state WAITING again
            emit sendMessage("");
            m_calibrationState = WAITING;
        }
    }
}

void cAltarIPMCalibrationFilter::OnCancel()
{
    m_calibrationState = WAITING;
    emit sendState(m_calibrationState);
    emit sendMessage("");
}
