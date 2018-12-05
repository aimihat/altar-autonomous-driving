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


/*********************************************************************
 * This code was provided by HERE
 *
 * *******************************************************************/

#define A_UTILS_NO_DEPRECATED_WARNING

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/core/cvdef.h"
#include "opencv2/core/hal/interface.h"
#include <opencv2/core/hal/hal.hpp>

#include <adtf3.h>
#include <stdlib.h>
#include "../AltarUtils/quadtree.h"

#include "Localize.h"

/*! configuration parameters */
//sudo chmod -R 777 src/

// road sign distance and pose estimation
#define MP_LIMIT_ALPHA    60.0 // [degrees]
#define MP_LIMIT_YAW      15.0 // [degrees]
#define MP_LIMIT_YAW_INIT  8.0 // [degrees]
#define MP_LIMIT_DISTANCE  4 // [m]

// process covariances
#define MP_PROCESS_X                    1e-3
#define MP_PROCESS_Y                    1e-3
#define MP_PROCESS_HEADING              3e-4
#define MP_PROCESS_HEADING_DRIFT        5e-8
#define MP_PROCESS_SPEED                2e-3
#define MP_PROCESS_SPEED_SCALE          1e-6

// initial covariance values
#define MP_PROCESS_INIT_X               10.0
#define MP_PROCESS_INIT_Y               10.0
#define MP_PROCESS_INIT_HEADING         0.55
#define MP_PROCESS_INIT_HEADING_DRIFT   0.25
#define MP_PROCESS_INIT_SPEED           1.0
#define MP_PROCESS_INIT_SPEED_SCALE     0.5

// measurement covariances
#define MP_MEASUREMENT_X                0.5
#define MP_MEASUREMENT_Y                0.5
#define MP_MEASUREMENT_HEADING          1.0 // [radians]

/*! defines a data triggered filter and exposes it via a plugin class factory */
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CMARKERPOS_DATA_TRIGGERED_FILTER,
                                    "AltarLocalize",
                                    cLocalize,
                                    adtf::filter::pin_trigger({ "imu" }));

/*! initialize the trigger function */

void Cholesky( const Mat& A, Mat& S )
{
//    S = Scalar::all(0);
    CV_Assert(A.type() == CV_64F);

    int dim = A.rows;
    S.create(dim, dim, CV_64F);

    int i, j, k;

    for( i = 0; i < dim; i++ )
    {
        for( j = 0; j < i; j++ )
            S.at<double>(i,j) = double(0);

        double sum = double(0);
        for( k = 0; k < i; k++ )
        {
            double val = S.at<double>(k,i);
            sum += val*val;
        }

        S.at<double>(i,i) = std::sqrt(std::max(A.at<double>(i,i) - sum, double(0)));
        double ival = double(1)/S.at<double>(i, i);

        for( j = i + 1; j < dim; j++ )
        {
            sum = 0;
            for( k = 0; k < i; k++ )
                sum += S.at<double>(k, i) * S.at<double>(k, j);

            S.at<double>(i, j) = (A.at<double>(i, j) - sum)*ival;
        }
    }
}

cLocalize::cLocalize()
{
    SetName("MarkerPos");

    //register properties
    RegisterPropertyVariable("Roadsign File", m_roadSignFile);

    RegisterPropertyVariable("Speed Scale", m_f32SpeedScale);
    RegisterPropertyVariable("World Scale", m_f32WorldScale);

    RegisterPropertyVariable("Camera Offset::Lateral", m_f32CameraOffsetLat);
    RegisterPropertyVariable("Camera Offset::Longitudinal", m_f32CameraOffsetLon);
    RegisterPropertyVariable("Sign Init", sign_init);


    //MY STRUCTS

    object_ptr<IStreamType> pLSData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pLSData, m_LSStructSampleFactory)) {

        (adtf_ddl::access_element::find_index(m_LSStructSampleFactory,"ui32Size", m_ddlLSDataId.size));
        (adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray));
    }

    else {

        LOG_WARNING("No mediadescription for tTemplateData found!");
    }

    Register(laserDataIn, "LaserScanner" , pLSData);


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

    Register(DebugPointOut, "DebugPoints", pDebugPoint);



    object_ptr<IStreamType> pParticleData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tParticles", pParticleData, m_ParticleStructSampleFactory))
    {
        adtf_ddl::access_element::find_array_index(m_ParticleStructSampleFactory, "particleArray", m_ddlParticleDataId.particleArray);

    }
    else
    {
        LOG_WARNING("No mediadescription for tParticle found!");
    }

    Register(ParticleDataOut, "Particles", pParticleData);


    object_ptr<IStreamType> pPoseCovData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPoseCov", pPoseCovData, m_PoseCovStructSampleFactory))
    {
        adtf_ddl::access_element::find_array_index(m_PoseCovStructSampleFactory, "covArray", m_ddlPoseCovDataId.covArray);

    }
    else
    {
        LOG_WARNING("No mediadescription for tPoseCov found!");
    }

    Register(PoseCovDataOut, "PoseCov", pPoseCovData);





    object_ptr<IStreamType> pLineData;

    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLine", pPoseCovData, m_LineStructSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_LineStructSampleFactory, "linesFound", m_ddlLineDataId.linesFound);
        adtf_ddl::access_element::find_array_index(m_LineStructSampleFactory, "lineArray", m_ddlLineDataId.lineArray);

    }
    else
    {
        LOG_WARNING("No mediadescription for tLine found!");
    }

    Register(LineDataIn, "Line", pLineData);




    //the imu struct
    object_ptr<IStreamType> pTypeIMUData;

    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tInerMeasUnitData", pTypeIMUData, m_IMUDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "ui32ArduinoTimestamp", m_ddlInerMeasUnitDataIndex.timeStamp);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_x", m_ddlInerMeasUnitDataIndex.A_x);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_y", m_ddlInerMeasUnitDataIndex.A_y);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_z", m_ddlInerMeasUnitDataIndex.A_z);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_x", m_ddlInerMeasUnitDataIndex.G_x);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_y", m_ddlInerMeasUnitDataIndex.G_y);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_z", m_ddlInerMeasUnitDataIndex.G_z);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_x", m_ddlInerMeasUnitDataIndex.M_x);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_y", m_ddlInerMeasUnitDataIndex.M_y);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_z", m_ddlInerMeasUnitDataIndex.M_z);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32roll", m_ddlInerMeasUnitDataIndex.roll);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32pitch", m_ddlInerMeasUnitDataIndex.pitch);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32yaw", m_ddlInerMeasUnitDataIndex.yaw);
    }
    else
    {
        LOG_WARNING("No mediadescription for tInerMeasUnitData found!");
    }

    //the signal struct
    object_ptr<IStreamType> pTypeSignalData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalData, m_SignalDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_SignalDataSampleFactory, "ui32ArduinoTimestamp", m_ddlSignalDataIndex.timeStamp);
        adtf_ddl::access_element::find_index(m_SignalDataSampleFactory, "f32Value", m_ddlSignalDataIndex.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }


    //the roadsignext struct
    object_ptr<IStreamType> pTypeRoadSignData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tRoadSignExt", pTypeRoadSignData, m_RoadSignSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "i16Identifier", m_ddlRoadSignIndex.id);
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "f32Imagesize", m_ddlRoadSignIndex.size);
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "af32TVec", m_ddlRoadSignIndex.tvec);
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "af32RVec", m_ddlRoadSignIndex.rvec);
    }
    else
    {
        LOG_WARNING("No mediadescription for tRoadSignExt found!");
    }


    //the position struct
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

    // cast to const type for the method calls below
    object_ptr<const IStreamType> pConstTypeRoadSignData = pTypeRoadSignData;


    object_ptr<const IStreamType> pConstTypeSignalData = pTypeSignalData;
    object_ptr<const IStreamType> pConstTypeIMUData = pTypeIMUData;
    object_ptr<const IStreamType> pConstTypePositionData = pTypePositionData;

    //register input pin(s)
    Register(m_oReaderRoadSign, "roadSign", pConstTypeRoadSignData);
    Register(m_oReaderSpeed, "speed", pConstTypeSignalData);

    Register(DistanceDelta, "last_distance", pConstTypeSignalData);

    Register(m_oReaderIMU, "imu", pConstTypeIMUData);

    //register output pin
    Register(m_oWriter, "position", pConstTypePositionData);

    // initialize EKF variables
    m_state = Mat(6, 1, CV_64F, Scalar::all(0));
    m_errorCov = Mat(6, 6, CV_64F, Scalar::all(0));

    tFloat64 T = 0.1;
    m_errorCov.at<double>(0, 0) = MP_PROCESS_INIT_X;
    m_errorCov.at<double>(1, 1) = MP_PROCESS_INIT_Y;
    m_errorCov.at<double>(2, 2) = MP_PROCESS_INIT_HEADING;
    m_errorCov.at<double>(2, 3) = MP_PROCESS_INIT_HEADING / T;
    m_errorCov.at<double>(3, 3) = MP_PROCESS_INIT_HEADING_DRIFT;
    m_errorCov.at<double>(4, 4) = MP_PROCESS_INIT_SPEED;
    m_errorCov.at<double>(4, 5) = MP_PROCESS_INIT_SPEED / T;
    m_errorCov.at<double>(5, 5) = MP_PROCESS_INIT_SPEED_SCALE;

    m_transitionMatrix = Mat(6, 6, CV_64F, Scalar::all(0));
    setIdentity(m_transitionMatrix);

    m_processCov = Mat(6, 6, CV_64F, Scalar::all(0));
    m_processCov.at<double>(0, 0) = MP_PROCESS_X;
    m_processCov.at<double>(1, 1) = MP_PROCESS_Y;
    m_processCov.at<double>(2, 2) = MP_PROCESS_HEADING;
    m_processCov.at<double>(3, 3) = MP_PROCESS_HEADING_DRIFT;
    m_processCov.at<double>(4, 4) = MP_PROCESS_SPEED;
    m_processCov.at<double>(5, 5) = MP_PROCESS_SPEED_SCALE;

    m_isInitialized = tFalse;

    // initialize translation and rotation vectors
    m_Tvec = Mat(3, 1, CV_32F, Scalar::all(0));
    m_Rvec = Mat(3, 1, CV_32F, Scalar::all(0));

    // initialize other variables
    m_f32Speed = 0;
    m_f32YawRate = 0;

    m_ui32ArduinoTimestamp = 0;

    m_ui32Cnt = 0;
    q_count = 0;
    q_scale_factor = 2;

    particle_dist_threshold = 0.05; // meters
    lookaround_dist_threshold = 10;
    boundary_check = 2;
    particle_boundary_check = 5;

    stateDim = 5;
    camDim = 3;
    lidarDim = 3;

    lidarOffsetLong = 0.47;
    lidarOffsetLat = 0;

    mState = Mat(stateDim, 1, CV_64F, Scalar::all(0));

////    mState.at<double>(0, 0) = 0; // x
////    mState.at<double>(1, 0) = 0; // y
////    mState.at<double>(2, 0) = 0; // yaw
//    mState.at<double>(3, 0) = 0; //yawdrift - mean is zero b/c gaussian
////    mState.at<double>(4, 0) = 0; //speed
//    mState.at<double>(5, 0) = 0; //speeddrift


    mCov = Mat(stateDim, stateDim, CV_64F, Scalar::all(0));

    mCov.at<double>(0, 0) = 10; //x
    mCov.at<double>(1, 1) = 10; //y
    mCov.at<double>(2, 2) = 0.5; // theta
    mCov.at<double>(3, 3) = 5e-8; // Yaw Drift
    mCov.at<double>(4, 4) = 1e-6; // Speed Drift
//    mCov.at<double>(5, 5) = 1; // range sensor var
//    mCov.at<double>(6, 6) = 1; // bearing sensor var

//        mCov.at<double>(0, 0) = 1;
//    mCov.at<double>(1, 1) = 1;
//    mCov.at<double>(2, 2) = 1;
//    mCov.at<double>(3, 3) = 1;
//    mCov.at<double>(4, 4) = 1; // Yaw Rate
//    mCov.at<double>(5, 5) = 1;
//    mCov.at<double>(6, 6) = 1;
//
//    //Measurement Variance
//    mCov.at<double>(7, 7) = 1;
//    mCov.at<double>(8, 8) = 1;
//    mCov.at<double>(9, 9) = 1; // Heading Variance additive;


    mMeasurementPoints = Mat(2*stateDim+1, stateDim, CV_64F, Scalar::all(0));
    mMeasurementMean = Mat(stateDim, 1, CV_64F, Scalar::all(0));
    mMeasurementCov = Mat(stateDim, stateDim, CV_64F, Scalar::all(0));

    mMeanWeights = Mat(1, 2*stateDim + 1, CV_64F, Scalar::all(0));
    mCovWeights = Mat(1, 2*stateDim + 1, CV_64F, Scalar::all(0));
    mSigmaPoints = Mat(2*stateDim+1, stateDim, CV_64F, Scalar::all(0));


//    mSigmaRoot = Mat(stateDim, stateDim, CV_64F, Scalar::all(0));
    mCamMeasurementVar = Mat(3, 3, CV_64F, Scalar::all(0));
//    mLaserMeasurementVar = Mat(3, 3, CV_64F, Scalar::all(0));

    mCamMeasurementVar.at<double>(0, 0) = 1;
    mCamMeasurementVar.at<double>(1, 1) = 1;
    mCamMeasurementVar.at<double>(2, 2) = 0.5;

//    mCamMeasurementVar.at<double>(0, 0) = 1;
//    mCamMeasurementVar.at<double>(1, 1) = 1;


    alpha = 0.01;
    k = 0;
    beta = 2;

    GetWeights(mState, mMeanWeights, mCovWeights, alpha, beta, k);


    RoadSignDetector = new SignDetector();
}

tResult cLocalize::GetSigmaPoints(Mat &state, Mat& covariance, Mat &sigmaPoint, double alpha, double beta, double k) {
    mSigmaRoot = cv::Scalar(0.0);
    sigmaPoint = cv::Scalar(0.0);
    int n = stateDim;
    Mat stateRow = state.t();

    double lambda = (pow(alpha, 2) * (n + k)) - n;
    double kappa = 3 - n;
    Mat covSquaredRooted = covariance.clone();// + 0.0001 * Mat::eye(n, n, CV_64F); // Add small num for numerical stability
    bool success = cv::hal::Cholesky64f((double*)covSquaredRooted.ptr(),covSquaredRooted.step, stateDim,NULL, 0, 0);
    if (!success){

        LOG_INFO("Mat Eigen CHOLESKY FAILED");

        //TODO IF Failed, then reset covariance.
    }
//    hal_ni_Cholesky64f
    for( int i = 0; i < covSquaredRooted.rows; i++ )
    {
        for( int j = i + 1; j < covSquaredRooted.cols; j++ )
            covSquaredRooted.at<double>(i,j) = double(0);
    }

    Mat test =(covSquaredRooted * covSquaredRooted.t()) - covariance;

    LOG_INFO(cString::Format("Eigen test sum[%.20f",cv::sum(test)));

//    covSquaredRooted *= sqrt(n + lambda);
    covSquaredRooted *= sqrt(n + kappa);

    // CUTOUT THE MIDDLE STEP AND JUST INDEX FROM THE LOWER TRIANGLE for speed up.

    sigmaPoint.row(0) = state.t();

    for (int i = 0; i < n; i++){
            Mat tempPlus = state + covSquaredRooted.col(i);
            Mat tempMinus = state - covSquaredRooted.col(i);

            sigmaPoint.row(i + 1) = tempPlus.t();
            sigmaPoint.row(n + i + 1) = tempMinus.t();
    }

    RETURN_NOERROR;
}


tResult cLocalize::GetWeights(Mat &state, Mat& weightsMean, Mat & weightsCov, double alpha, double beta, double k) {

    int n = state.rows;

    double lambda = (pow(alpha, 2) * (n + k)) - n;
    double kappa = 3 - n;


//    weightsMean.at<double>(0,0) = lambda / (n + lambda);
//    weightsCov.at<double>(0,0) = (lambda / (n + lambda)) + (1 - pow(alpha,2) + beta);

    weightsMean.at<double>(0,0) = kappa / (n + kappa);
    weightsCov.at<double>(0,0) = kappa / (n + kappa);


    for (int i = 0; i < n; i++){

//        weightsMean.at<double>(0,i + 1) = 1 / (2 * (n + lambda));
//        weightsMean.at<double>(0,n + i + 1) = 1 / (2 * (n + lambda));

//        weightsCov.at<double>(0,i + 1) =  1 / (2 * (n + lambda));
//        weightsCov.at<double>(0,n + i + 1) =  1 / (2 * (n + lambda));

        weightsMean.at<double>(0,i + 1) = 1 / (2 * (n + kappa));
        weightsMean.at<double>(0,n + i + 1) = 1 / (2 * (n + kappa));

        weightsCov.at<double>(0,i + 1) =  1 / (2 * (n + kappa));
        weightsCov.at<double>(0,n + i + 1) =  1 / (2 * (n + kappa));


    }
//    double s = cv::sum(weightsMean)[0];

    RETURN_NOERROR;
}


tResult cLocalize::UTSensor(Mat &measurePoints, Mat &sigmaPoints, Mat& weightsMean, Mat &weightsCov, Mat &stateVector, Mat &measureVector, std::vector<bool> measureAngles, std::vector<bool> stateAngles, Mat &covariance, Mat &crossVariance) {
//    UTSensor(mMeasurementPoints, mSigmaPoints, mMeanWeights, mCovWeights, mState, mMeasureVector, measureAngles, stateAngles, mMeasureCov, mCrossVariance);

    Mat covarianceTemp = Mat(measurePoints.cols, measurePoints.cols, CV_64F, Scalar::all(0));
    measureVector = Mat(measurePoints.cols, 1, CV_64F, Scalar::all(0));

    crossVariance = Mat(stateVector.rows, measurePoints.cols, CV_64F, Scalar::all(0));

    for (int k = 0; k < measurePoints.cols; k++){

        // Careful Angles may be problem with this average, //TODO CHECK ANGLES
            Mat pointAvg = (weightsMean * measurePoints.col(k));
            measureVector.at<double>(k, 0) = pointAvg.at<double>(0, 0);

    }

    Mat measurePointTemp = measurePoints.clone();

    for (int i = 0; i < measurePoints.rows; i++){
        for (int j = 0; j < measurePoints.cols; j++){
            measurePointTemp.at<double>(i,j) = measurePoints.at<double>(i,j) - measureVector.at<double>(j,0);

            if (measureAngles.at(j)) {
                measurePointTemp.at<double>(i,j) = normalizeAngle(measurePointTemp.at<double>(i,j), 0);
            }
        }
    }

    Mat sigmaPointTemp = sigmaPoints.clone();
    for (int i = 0; i < sigmaPoints.rows; i++){
        for (int j = 0; j < sigmaPoints.cols; j++){
            sigmaPointTemp.at<double>(i,j) = sigmaPoints.at<double>(i,j) - stateVector.at<double>(j,0);

            if (stateAngles.at(j)) {
                sigmaPointTemp.at<double>(i,j) = normalizeAngle(sigmaPointTemp.at<double>(i,j), 0);
            }

        }
    }


    Mat outerProduct;

    for (int i = 0; i < weightsCov.cols; i++){

        outerProduct = measurePointTemp.row(i).t() * measurePointTemp.row(i);
        covarianceTemp += weightsCov.at<double>(0,i) * outerProduct;
    }

    covariance = covarianceTemp;
    covariance += mCamMeasurementVar;

    for (int i = 0; i < weightsCov.cols; i++){

        outerProduct = sigmaPointTemp.row(i).t() * measurePointTemp.row(i);
        crossVariance += weightsCov.at<double>(0,i) * outerProduct;
    }

    RETURN_NOERROR;
}

tResult cLocalize::UTMotion(Mat &sigmaPoints, Mat& weightsMean, Mat &weightsCov, Mat &meanVector, Mat &covariance, std::vector<bool> angles) {

    Mat covarianceTemp = Mat(stateDim, stateDim, CV_64F, Scalar::all(0));

    for (int k = 0; k < sigmaPoints.cols; k++){
        if (angles.at(k)){

            Mat pointAvg = (weightsMean * sigmaPoints.col(k));
            meanVector.at<double>(k, 0) = pointAvg.at<double>(0, 0);
        }
    }

    Mat sigmaPointTemp = sigmaPoints.clone();
    Mat outerProduct;

    for (int i = 0; i < sigmaPoints.rows; i++){
        for (int j = 0; j < sigmaPoints.cols; j++){

            if (angles.at(j)) {
                double delta = sigmaPoints.at<double>(i,j) - meanVector.at<double>(j,0);
                sigmaPointTemp.at<double>(i,j) = normalizeAngle(delta, 0);
            }

            else{
                sigmaPointTemp.at<double>(i,j) = sigmaPoints.at<double>(i,j) - meanVector.at<double>(j,0);
            }
        }
    }

    for (int i = 0; i < weightsCov.cols; i++){
        outerProduct = sigmaPointTemp.row(i).t() * sigmaPointTemp.row(i);
        covarianceTemp += weightsCov.at<double>(0,i) * outerProduct;
    }

    covariance = covarianceTemp;


    RETURN_NOERROR;
}

tResult cLocalize::MotionModel(Mat &sigmaPoints, double dt, double yawRate, double velocity) {

    yawRate *= static_cast<tFloat32>(DEG2RAD);

    if (velocity < 0.00001){
       yawRate = 0;
    }
//    dt = 0.001; //for debugging
    if (dt > 5){
        RETURN_NOERROR;
    }

    Mat sigmaPointTemp = sigmaPoints.clone();
//    double hk;
    double vk;
    double yk;


    for (int i = 0; i < sigmaPoints.rows; i++){

        yk = yawRate + sigmaPoints.at<double>(i,3);
        vk = velocity + sigmaPoints.at<double>(i,4);
        const double theta = sigmaPoints.at<double>(i,2);

        if (yk <= 0.001) { //moving in straight line;

            sigmaPointTemp.at<double>(i,0) += vk*cos(theta + (yk*dt))*dt;
            sigmaPointTemp.at<double>(i,1) += vk*sin(theta+ (yk*dt))*dt;
            sigmaPointTemp.at<double>(i,2) = theta + (yk*dt);
            sigmaPointTemp.at<double>(i,2) = normalizeAngle(sigmaPointTemp.at<double>(i,2), 0);

        }

        else {

            const double k = vk / yk;
            sigmaPointTemp.at<double>(i,0) += - k * sin(theta) + k * sin(theta + (yk*dt));
            sigmaPointTemp.at<double>(i,1) += k * cos(theta) - k * cos(theta + (yk*dt));
            sigmaPointTemp.at<double>(i,2) = theta + (yk*dt);
            sigmaPointTemp.at<double>(i,2) = normalizeAngle(sigmaPointTemp.at<double>(i,2), 0);
        }
    }

    sigmaPoints = sigmaPointTemp;

    RETURN_NOERROR;
}

tResult cLocalize::CameraModel(Mat &sigmaPoints, Mat &predPoints, RoadSignPose sign) {

    // CURRENT PROBLEM is That the Camera Mean is not being placed in the middle, so you get a massive uncertainity and I it grows,
    // then agian your only subtracting it. Still doesn't seem right......
    //Test with x,y,theta to make sure kalman updates ect working then extend

    predPoints = Mat(2*stateDim + 1, camDim, CV_64F, Scalar::all(0));

    for (int i = 0; i < sigmaPoints.rows; i++){

        double deltaX = sign.x - sigmaPoints.at<double>(i,0);
        double deltaY = sign.y - sigmaPoints.at<double>(i,1);

        //Debuggings
//        predPoints.at<double>(i,0) = sigmaPoints.at<double>(i,0);
//        predPoints.at<double>(i,1) = sigmaPoints.at<double>(i,1);

//        predPoints.at<double>(i,0) =  sqrt(pow(deltaX,2) + pow(deltaY,2)); // Distance
//        predPoints.at<double>(i,1) =  normalizeAngle((atan2(deltaY, deltaX) - sigmaPoints.at<double>(i,2)),0); // Heading
//        predPoints.at<double>(i,2) =  sigmaPoints.at<double>(i,2);

//        predPoints.at<double>(i,0) =  sqrt(pow(deltaX,2) + pow(deltaY,2)); // Distance
//        predPoints.at<double>(i,1) =  normalizeAngle((atan2(deltaY, deltaX) - sigmaPoints.at<double>(i,2)),0); // Heading
//        predPoints.at<double>(i,2) =  sigmaPoints.at<double>(i,2);

        predPoints.at<double>(i,0) =  sigmaPoints.at<double>(i,0); // Distance
        predPoints.at<double>(i,1) =  sigmaPoints.at<double>(i,1); // Heading
        predPoints.at<double>(i,2) =  sigmaPoints.at<double>(i,2);
    }


    RETURN_NOERROR;
}

tResult cLocalize::LidarModel(Mat &sigmaPoints, Mat &predPoints, Pose sign) {

    // CURRENT PROBLEM is That the Camera Mean is not being placed in the middle, so you get a massive uncertainity and I it grows,
    // then agian your only subtracting it. Still doesn't seem right......
    //Test with x,y,theta to make sure kalman updates ect working then extend

    predPoints = Mat(2*stateDim + 1, lidarDim, CV_64F, Scalar::all(0));

    for (int i = 0; i < sigmaPoints.rows; i++){

        double deltaX = sign.x - sigmaPoints.at<double>(i,0);
        double deltaY = sign.y - sigmaPoints.at<double>(i,1);

        //Debuggings
//        predPoints.at<double>(i,0) = sigmaPoints.at<double>(i,0);
//        predPoints.at<double>(i,1) = sigmaPoints.at<double>(i,1);

//        predPoints.at<double>(i,0) =  sqrt(pow(deltaX,2) + pow(deltaY,2)); // Distance
//        predPoints.at<double>(i,1) =  normalizeAngle((atan2(deltaY, deltaX) - sigmaPoints.at<double>(i,2)),0); // Heading
//        predPoints.at<double>(i,2) =  sigmaPoints.at<double>(i,2);

//        predPoints.at<double>(i,0) =  sqrt(pow(deltaX,2) + pow(deltaY,2)); // Distance
//        predPoints.at<double>(i,1) =  normalizeAngle((atan2(deltaY, deltaX) - sigmaPoints.at<double>(i,2)),0); // Heading
//        predPoints.at<double>(i,2) =  sigmaPoints.at<double>(i,2);

        predPoints.at<double>(i,0) =  sigmaPoints.at<double>(i,0); // Distance
        predPoints.at<double>(i,1) =  sigmaPoints.at<double>(i,1); // Heading
        predPoints.at<double>(i,2) =  sigmaPoints.at<double>(i,2);
    }


    RETURN_NOERROR;
}

tResult cLocalize::KalmanUpdate(Mat &measurmentData, Mat &measureVector, Mat &measureCov, Mat &crossVariance, Mat &stateVector, Mat &stateCov, std::vector<bool> angles) {

    Mat kalmanGain;
    Mat newStateVector;
    Mat newStateCov;
    Mat innovationVector;


    kalmanGain = crossVariance * measureCov.inv();

    innovationVector = measurmentData - measureVector;

    for (int i = 0; i < innovationVector.rows; i++){

        if (angles.at(i)){
            innovationVector.at<double>(i,0) = normalizeAngle(innovationVector.at<double>(i,0),0);
        }
    }

    LOG_INFO("INNOVATION VECTOR");
//    LOG_INFO(cString::Format("Mat [%f, %f, %f]", innovationVector.at<double>(0, 0), innovationVector.at<double>(1,0),innovationVector.at<double>(2,0)));
    LOG_INFO("END ");


    newStateVector = stateVector + kalmanGain * innovationVector;

    newStateCov = stateCov - kalmanGain * measureCov * kalmanGain.t();

    stateVector = newStateVector;
    stateCov = newStateCov;

    RETURN_NOERROR;
}

/*! implements the configure function to read ALL Properties */
tResult cLocalize::Configure()
{
    //Load Road lines

    double width = 30;
    double length = 30;

    // NOTE THE QUAD TREE IS NOT USING 3 of intial branches

    double dim = max(width, length)/2;
    Boundary world;
    world.center.x = dim;
    world.center.y = dim;
    world.half_distance = dim;

    // open roadsign configuration file
    cFilename fileRoadSign = m_roadSignFile;
    adtf::services::ant::adtf_resolve_macros(fileRoadSign);

    //check if roadsign file exits
    if (fileRoadSign.IsEmpty())
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, "RoadSign file not found");
    }
    if (!(cFileSystem::Exists(fileRoadSign)))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, "RoadSign file not found");
    }
    else
    {
        tInt i = 0;

        cDOM oDOM;
        oDOM.Load(fileRoadSign);
        cDOMElementRefList oElems;

        if (IS_OK(oDOM.FindNodes("configuration/roadSign", oElems)))
        {
            for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            {
                roadSign item;
                item.u16Id = tUInt16((*itElem)->GetAttribute("id", "0").AsInt32());
                item.f32X = tFloat32((*itElem)->GetAttribute("x", "0").AsFloat64());
                item.f32Y = tFloat32((*itElem)->GetAttribute("y", "0").AsFloat64());

                //adjust for mm
                item.f32X =  item.f32X;
                item.f32Y =  item.f32Y;


                item.f32Radius = tFloat32((*itElem)->GetAttribute("radius", "0").AsFloat64());

                item.f32Radius =  item.f32Radius;

                item.f32Direction = tFloat32((*itElem)->GetAttribute("direction", "0").AsFloat64());

                DebugPoint p;
                p.x = item.f32X;
                p.y = item.f32Y;
                p.theta = item.f32Direction;
                p.type = ROADSIGN;
                p.Id = item.u16Id;

                roadSignList.push_back(p);

                LOG_INFO("debugpoint ROADSIGN pushback");


                //item.bInit = ((*itElem)->GetAttribute("init","0").AsInt32());
                if ((*itElem)->GetAttribute("init", "0").AsInt32() == 0)
                {
                    item.bInit = false;
                }
                else
                {
                    item.bInit = true;
                }


                item.u16Cnt = 0;
                item.u32ticks = GetTime();

                item.f32Direction *= static_cast<tFloat32>(DEG2RAD); // convert to radians

                LOG_INFO(cString::Format("LoadConfiguration::Id %d XY %f %f Radius %f Direction %f",
                                         item.u16Id, item.f32X, item.f32Y, item.f32Radius, item.f32Direction).GetPtr());

                m_roadSigns.push_back(item);

                i++;
            }
        }

    }

    RETURN_NOERROR;
}

/*! calculates normalized angle difference */
tFloat32 cLocalize::angleDiff(tFloat32 angle1, tFloat32 angle2)
{
    // normalization
    angle1 = normalizeAngle(angle1, static_cast<tFloat32>(M_PI));
    angle2 = normalizeAngle(angle2, static_cast<tFloat32>(M_PI));

    // compute difference and normalize in [-pi pi]
    return normalizeAngle(angle2 - angle1, 0);
}

/*! calculates normalized angle */
tFloat32 cLocalize::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
    return mod(alpha - center + static_cast<tFloat32>(M_PI), 2.0f*static_cast<tFloat32>(M_PI)) + center - static_cast<tFloat32>(M_PI);
}

/*! calculates modulus after division */
tFloat32 cLocalize::mod(tFloat32 x, tFloat32 y)
{
    tFloat32 r;
    tFloat32 b_x;
    if (y == floor(y))
    {
        return x - floor(x / y) * y;
    }
    else
    {
        r = x / y;
        if (r < 0.0)
        {
            b_x = ceil(r - 0.5f);
        }
        else
        {
            b_x = floor(r + 0.5f);
        }
        if (fabs(r - b_x) <= 2.2204460492503131E-16 * fabs(r))
        {
            return 0.0;
        }
        else
        {
            return (r - floor(r)) * y;
        }
    }
}

/*! Calculates orientation, distance and pose of the given road sign,
 * and updates the positioning filter accordingly */
tResult cLocalize::ProcessRoadSignStructExt(tTimeStamp tmTimeOfTrigger, const adtf::streaming::ISample &oSample)
{

    LOG_DUMP("Detected Road Sign");
    // parse the data
    auto oDecoder = m_RoadSignSampleFactory.MakeDecoderFor(oSample);

    // fetch marker id
    m_i16ID = access_element::get_value(oDecoder, m_ddlRoadSignIndex.id);

    const tVoid* pArray;
    tSize size;

    // fetch marker translation and rotation arrays
    access_element::get_array(oDecoder, "af32TVec", pArray, size);
    m_Tvec.data = const_cast<uchar*>(static_cast<const uchar*>(pArray));

    access_element::get_array(oDecoder, "af32RVec", pArray, size);
    m_Rvec.data = const_cast<uchar*>(static_cast<const uchar*>(pArray));

    // ignore initial noisy markers
            // Decrease for testing purposes

    if (m_ui32Cnt < 5)
//    if (m_ui32Cnt < 50)
    {
        LOG_WARNING(cString::Format("Noise %d", m_ui32Cnt));
        m_ui32Cnt++;
        RETURN_NOERROR;
    }

    cv::Mat R;
    cv::Rodrigues(m_Rvec, R); // rot is 3x3

    // calculate translation
    tFloat32 lateral = m_Tvec.at<float>(0);
    tFloat32 longitudinal = m_Tvec.at<float>(2);

    // add camera offset
    lateral += m_f32CameraOffsetLat;
    longitudinal += m_f32CameraOffsetLon;

    tFloat32 d0 = sqrt(lateral*lateral + longitudinal*longitudinal);

    tFloat32 a0 = atan2(lateral, longitudinal);
    a0 = (tFloat32)normalizeAngle(a0, 0)*static_cast<tFloat32>(RAD2DEG); // normalize angle -pi:pi

    a0 *= -1.0; // and change direction

    // calculate pose of the road sign
    float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) + R.at<float>(1, 0) * R.at<float>(1, 0));

    tFloat32 yawE = atan2(-R.at<float>(2, 0), sy) * static_cast<tFloat32>(RAD2DEG);


    // check angle and distance limit
    if (fabs(a0) > MP_LIMIT_ALPHA || d0 > MP_LIMIT_DISTANCE)
    {
        LOG_WARNING("Over Distance and Angle Limit");

        RETURN_NOERROR;
    }

    //LOG_INFO(cString::Format("ID %d: d0 %f a0 %f yawE %f", m_i16ID, d0, a0, yawE).GetPtr());

    // wait for start-marker, and then initialize the filter
    if ((m_isInitialized == tFalse) && (sign_init == true))
    {
        for (unsigned int i = 0; i < m_roadSigns.size(); i++)
        {
            if ((m_roadSigns[i].u16Id == m_i16ID) &&
                (m_roadSigns[i].bInit == tTrue))
            {

                //Found Sign Update

                RoadSignDetector->ExpectedSign(d0, a0, GetTime(),i);

                // calculate the vehicle position and heading based on
                // road-sign measurement

                // estimate heading
                tFloat32 heading = m_roadSigns[i].f32Direction + yawE*static_cast<tFloat32>(DEG2RAD);

                heading = normalizeAngle(heading, 0);

                tFloat32 shift = -1.0f*d0;

                tFloat32 correction = heading + a0*static_cast<tFloat32>(DEG2RAD);
                correction = normalizeAngle(correction, 0);

                // estimate location
                tFloat32 x = m_roadSigns[i].f32X + cos(correction)*shift;
                tFloat32 y = m_roadSigns[i].f32Y + sin(correction)*shift;

                LOG_INFO(cString::Format("initialize e %f n %f h %f x %f y %f", x, y, heading*RAD2DEG,
                m_roadSigns[i].f32X, m_roadSigns[i].f32Y).GetPtr());

                // initialize filter state
                m_state.at<double>(0) = x;
                m_state.at<double>(1) = y;
                m_state.at<double>(2) = heading;
                m_state.at<double>(3) = 0;
                m_state.at<double>(4) = 0;
                m_state.at<double>(5) = 0;

                m_isInitialized = tTrue;


                mState.at<double>(0,0) = x;
                mState.at<double>(1,0) = y;
                mState.at<double>(2,0) = heading;
                mState.at<double>(3, 0) = 0; //yawdrift - mean is zero b/c gaussian
                mState.at<double>(4,0) = 0;//speeddrift
//                mState.at<double>(5, 0) = 0;
//                mState.at<double>(6, 0) = 0;



                // I overide some of these assignments in motion update
            }

        }

        RETURN_NOERROR;

    }
    // find a matching road sign

    tFloat64 dt = 0;

    tInt ind = -1;
    for (unsigned int i = 0; i < m_roadSigns.size(); i++)
    {
        if (m_roadSigns[i].u16Id == m_i16ID)
        {
            // calculate heading wrt marker
            tFloat32 heading = static_cast<tFloat32>(m_state.at<double>(2)) + a0*static_cast<tFloat32>(DEG2RAD);
            heading = normalizeAngle(heading, 0);

            // estimate marker location based on current vehicle location
            // and marker measurement
            tFloat32 x0 = static_cast<tFloat32>(m_state.at<double>(0)) + cos(heading)*d0;
            tFloat32 y0 = static_cast<tFloat32>(m_state.at<double>(1)) + sin(heading)*d0;

            // calculate error distance
            tFloat32 dx = x0 - m_roadSigns[i].f32X;
            tFloat32 dy = y0 - m_roadSigns[i].f32Y;

            tFloat32 distance = sqrt(dx*dx + dy*dy);

            tInt found = tFalse;

            // re-initialize with init-signs
            if (m_roadSigns[i].bInit == tTrue && fabs(m_f32Speed) < 0.01 && fabs(m_f32YawRate) < 1)
            {
//                LOG_INFO("Reintializing");

                tFloat32 shift = -1.0f*d0;

                // estimate location
                tFloat32 x = m_roadSigns[i].f32X + cos(heading)*shift;
                tFloat32 y = m_roadSigns[i].f32Y + sin(heading)*shift;
                tFloat32 heading = m_roadSigns[i].f32Direction + yawE*static_cast<tFloat32>(DEG2RAD);


                DebugPoint p;

                p.x = m_roadSigns[i].f32X;
                p.y = m_roadSigns[i].f32Y;
                p.Id = (int)a0;
                p.type = CAM_SIGN;

                debugPointList.push_back(p);

                // initialize filter but keep heading states
                m_state.at<double>(0) = x;
                m_state.at<double>(1) = y;
                m_state.at<double>(2) = heading;
                m_state.at<double>(4) = 0;
//                m_state.at<double>(5) = 0;
                found = tTrue;
//
//                mState.at<double>(0,0) = x;
//                mState.at<double>(1,0) = y;
//                mState.at<double>(2,0) = heading;
////                mState.at<double>(3, 0) = 0; //yawdrift - mean is zero b/c gaussian
//                //dont change speed or yawrate
//                mState.at<double>(5, 0) = 0; //speeddrift
//                mState.at<double>(6, 0) = 0; //speeddrift

            }
                // marker found within the radius
            else if (distance < m_roadSigns[i].f32Radius)
            {
                found = tTrue;
            }

            // found a suitable marker
            if (found)
            {
                ind = i;

                RoadSignDetector->ExpectedSign(d0, a0, GetTime(),i);

                // calculate time from previous marker measurement
                dt = (GetTime() - m_roadSigns[i].u32ticks)*1e-6;
                m_roadSigns[i].u32ticks = GetTime();

                // reset sample counter when marker reappears
                if (dt > 1.0)
                {
                    m_roadSigns[i].u16Cnt = 0;
                }

                break;
            }

        }

    }

    // update sample counter
    m_roadSigns[ind].u16Cnt++;

    // conditions:
    // #1 no matching marker found
    // #2 too long time from previous marker input
    // #3 dropping samples when a new marker is found

//    if (ind < 0 || dt > 0.3 || m_roadSigns[ind].u16Cnt < 10)
//    {
//        LOG_WARNING("Not Robust");

//        RETURN_NOERROR;
//    }

    if (ind < 0 || m_roadSigns[ind].u16Cnt < 10)
    {
        LOG_WARNING("Not Robust");

        RETURN_NOERROR;
    }

    // If it makes matches the marker then update
    // EKF update step

    // create pseudo measurement for heading update
    tFloat32 headingUpdate = static_cast<tFloat32>(m_state.at<double>(2));

    tFloat32 shift = -1.0f*d0; // reversed translation direction

    // calculate translation direction
    tFloat32 correction = headingUpdate + a0*static_cast<tFloat32>(DEG2RAD);
    correction = normalizeAngle(correction, 0);

    // update location estimate
    tFloat32 x = m_roadSigns[ind].f32X + cos(correction)*shift;
    tFloat32 y = m_roadSigns[ind].f32Y + sin(correction)*shift;

    Mat measCov = Mat(3, 3, CV_64F, Scalar::all(0));
    measCov.at<double>(0, 0) = MP_MEASUREMENT_X;
    measCov.at<double>(1, 1) = MP_MEASUREMENT_Y;
    measCov.at<double>(2, 2) = MP_MEASUREMENT_HEADING;

    Mat measurementMatrix = Mat(3, 6, CV_64F, Scalar::all(0));

    measurementMatrix.at<double>(0, 0) = 1.0;
    measurementMatrix.at<double>(1, 1) = 1.0;
    measurementMatrix.at<double>(2, 2) = 1.0;

    Mat identity = Mat(6, 6, CV_64F, Scalar::all(0));
    setIdentity(identity);

    Mat measurement = Mat(3, 1, CV_64F, Scalar::all(0));

    measurement.at<double>(0) = x;
    measurement.at<double>(1) = y;
    measurement.at<double>(2) = headingUpdate;

    // propagate covariance
    m_errorCov = m_transitionMatrix*m_errorCov*m_transitionMatrix.t() + m_processCov;
    //m_errorCov = (m_errorCov + m_errorCov.t())/2;

    // calculate innovation
    Mat innovation = measurement - measurementMatrix*m_state;

    // modulo of the heading measurement
    innovation.at<double>(2) = angleDiff(static_cast<tFloat32>(mod(static_cast<tFloat32>(m_state.at<double>(2)), static_cast<tFloat32>(2.0*M_PI)) - M_PI),
                                         mod(headingUpdate, 2.0f*static_cast<tFloat32>(M_PI)) - static_cast<tFloat32>(M_PI));

    Mat tmp = measurementMatrix*m_errorCov*measurementMatrix.t() + measCov;

    Mat gain = m_errorCov*measurementMatrix.t()*tmp.inv();

    // update state and covariance matrix
    m_state += gain*innovation;
    m_state.at<double>(2) = normalizeAngle(static_cast<tFloat32>(m_state.at<double>(2)), 0.0f);
    m_errorCov = (identity - gain*measurementMatrix)*m_errorCov;
    //m_errorCov = (m_errorCov + m_errorCov.t())/2;

    //cv::completeSymm(m_errorCov);
    //m_errorCov = (m_errorCov + m_errorCov.t())/2;
    RoadSignPose currRoadSign;

    currRoadSign.x =  (double)m_roadSigns[ind].f32X;
    currRoadSign.y =  (double)m_roadSigns[ind].f32Y;

    std::vector<bool> measureAngles = {false, false, true};
    std::vector<bool> stateAngles = {false, false, true, false, false};
    Mat mCrossVariance;
    Mat mMeasureVector = Mat(2, 1, CV_64F, Scalar::all(0));
    Mat mMeasureCov = Mat(2, 1, CV_64F, Scalar::all(0));

    double thetaUpdate = mState.at<double>(2,0);
    thetaUpdate = m_roadSigns[ind].f32Direction + yawE*DEG2RAD;

    Mat data = Mat(camDim, 1, CV_64F, Scalar::all(0));
    data.at<double>(0, 0) = x;
    data.at<double>(1, 0) = y;
    data.at<double>(2, 0) = thetaUpdate; //psuedo measeurement

    GetWeights(mState, mMeanWeights, mCovWeights, alpha, beta, k);
    GetSigmaPoints(mState, mCov, mSigmaPoints, alpha, beta, k); // Resample in case laser update has happened and changed Cov
    CameraModel(mSigmaPoints, mMeasurementPoints, currRoadSign);
    UTSensor(mMeasurementPoints, mSigmaPoints, mMeanWeights, mCovWeights, mState,
             mMeasureVector, measureAngles, stateAngles, mMeasureCov, mCrossVariance);
    KalmanUpdate(data, mMeasureVector, mMeasureCov, mCrossVariance, mState, mCov, measureAngles);


//    KalmanUpdate(Mat &measurmentData, Mat &sigmaPoints, Mat &measurementPoints, Mat &meanStateVector, Mat &stateCov, Mat &meanMeasurementVector, Mat &measurementCov, Mat &weightsCov)
//    LOG_INFO("Measurment Update");

    RETURN_NOERROR;
}


/*! support function for getting time */
tTimeStamp cLocalize::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}


tResult cLocalize::SendDebugPoints(){
        object_ptr<ISample> pDebugPointSample;

        RETURN_IF_FAILED(alloc_sample(pDebugPointSample))
        {
            {
                auto oCodec = m_DebugPointStructSampleFactory.MakeCodecFor(pDebugPointSample);

                DebugPoint* points = reinterpret_cast<DebugPoint*>(oCodec.GetElementAddress(m_ddlDebugPointId.debugPoints));

                //init array with zero
                memset(points, 0, 200 * sizeof(DebugPoint));

                int debugSize = debugPointList.size();
                for (unsigned int i = 0; i < debugSize; i++){

                    points[i].x = debugPointList[i].x;
                    points[i].y = debugPointList[i].y;
                    points[i].theta = debugPointList[i].theta;
                    points[i].type = debugPointList[i].type;
                    points[i].Id = debugPointList[i].Id;
                    points[i].colour = debugPointList[i].colour;

                }

                debugPointList.clear();

                if (!m_isInitialized) {
                    for (unsigned int i = 0; i < roadSignList.size(); i++){

                        points[i].x = roadSignList[i].x;
                        points[i].y = roadSignList[i].y;
                        points[i].theta = roadSignList[i].theta;
                        points[i].type = roadSignList[i].type;
                        points[i].Id = roadSignList[i].Id;
                        points[i].colour = roadSignList[i].colour;

                    }

                    debugSize += roadSignList.size();
                }

                std::vector<DebugPoint> detectorPoints = RoadSignDetector->GetDebugPoints();

                for (unsigned int i = 0; i < detectorPoints.size(); i++){

                    uint z = i + debugSize;
                    points[z].x = detectorPoints[i].x;
                    points[z].y = detectorPoints[i].y;
                    points[z].theta = detectorPoints[i].theta;
                    points[z].type = detectorPoints[i].type;
                    points[z].Id = detectorPoints[i].Id;
                    points[z].colour = detectorPoints[i].colour;

                }
//                LOG_INFO(cString::Format("detectorPoints size[%d]", detectorPoints.size()));

//                LOG_INFO(cString::Format("Sending points size[%d]", debugSize + detectorPoints.size()));

                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDebugPointId.numPoints, debugSize + detectorPoints.size()));

            }
        }

        DebugPointOut << pDebugPointSample << flush << trigger;
        RETURN_NOERROR;

}

/*! sends position data out */
tResult cLocalize::sendPositionStruct(const tTimeStamp &timeOfFix, const tFloat32 &f32X, const tFloat32 &f32Y, const tFloat32 &f32Radius,
                                       const tFloat32 &f32Heading, const tFloat32 &f32Speed)
{
    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample, timeOfFix));

    auto oCodec = m_PositionSampleFactory.MakeCodecFor(pSample);

    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.x, f32X));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.y, f32Y));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.radius, f32Radius));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.speed, f32Speed));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.heading, f32Heading));

    //LOG_INFO(cString::Format("sendPositionStruct: %.3f %.3f %.3f %.3f %.3f", f32X, f32Y,
    //f32Radius, f32Heading, f32Speed).GetPtr());

    // the sample buffer lock is released in the destructor of oCodec
    m_oWriter << pSample << flush << trigger;

    //LOG_INFO(cString::Format("COVARIANCE %f , %f", m_errorCov.at<double>(0, 1), m_errorCov.at<double>(1, 0) ) );


    object_ptr<ISample> pCovSample;

    RETURN_IF_FAILED(alloc_sample(pCovSample))
    {
        {
            auto oCodec = m_PoseCovStructSampleFactory.MakeCodecFor(pCovSample);

            tFloat32* points = reinterpret_cast<tFloat32*>(oCodec.GetElementAddress(m_ddlPoseCovDataId.covArray));

            //init array with zero

            //memset(points, 0, count * sizeof(tFloat32));
            points[0] = (tFloat32)m_errorCov.at<double>(0, 0);
            points[1] = (tFloat32)m_errorCov.at<double>(0, 1);
            points[2] = (tFloat32)m_errorCov.at<double>(1, 0);
            points[3] = (tFloat32)m_errorCov.at<double>(1, 1);

        }
    }

    PoseCovDataOut << pCovSample << flush << trigger;

    RETURN_NOERROR;
}

/*! processes inertial measurement data sample, and runs EKF prediction
 *  based on heading rate and speed measurements */
tResult cLocalize::ProcessInerMeasUnitSample(tTimeStamp tmTimeOfTrigger, const adtf::streaming::ISample &oSample)
{

    if ((m_isInitialized == tFalse) && (sign_init == false))
    {

        LOG_DUMP("Initalized CAR!");
        // estimate heading
        tFloat32 heading = 0;
        // estimate location
        tFloat32 x = 1;
        tFloat32 y = 1;

        //LOG_INFO(cString::Format("initialize e %f n %f h %f x %f y %f", x, y, heading*RAD2DEG,
        //m_roadSigns[i].f32X, m_roadSigns[i].f32Y).GetPtr());

        // initialize filter state
        m_state.at<double>(0) = x;
        m_state.at<double>(1) = y;
        m_state.at<double>(2) = heading;
        m_state.at<double>(3) = 0;
        m_state.at<double>(4) = 0;
        m_state.at<double>(5) = 0;

        LOG_INFO(cString::Format("initialize e %f n %f h %f", x, y, heading*RAD2DEG));

        m_isInitialized = tTrue;


        mState.at<double>(0,0) = x;
        mState.at<double>(1,0) = y;
        mState.at<double>(2,0) = heading; // yaw
        mState.at<double>(3, 0) = 0; //speed
        mState.at<double>(4, 0) = 0; //yawrate
//        mState.at<double>(5, 0) = 0;//yawdrift - mean is zero b/c gaussian
//        mState.at<double>(6, 0) = 0; //speeddrift

        RETURN_NOERROR;
    }


    tFloat64 dt = 0;
    tUInt32 ui32ArduinoTimestamp = 0;

    // parse the data
    auto oDecoder = m_IMUDataSampleFactory.MakeDecoderFor(oSample);

    // yaw-rate gyro measurment
    m_f32YawRate = access_element::get_value(oDecoder, m_ddlInerMeasUnitDataIndex.G_z);

    // fetch timestamp and calculate dt
    ui32ArduinoTimestamp = access_element::get_value(oDecoder, m_ddlInerMeasUnitDataIndex.timeStamp);

    dt = (tFloat64)(ui32ArduinoTimestamp - m_ui32ArduinoTimestamp)*1e-6;

    m_ui32ArduinoTimestamp = ui32ArduinoTimestamp;

    //LOG_INFO(cString::Format("processInertial: %.6f", m_f32YawRate).GetPtr());

    // filter not initialized
    if (m_isInitialized == tFalse)
    {
        RETURN_NOERROR;
    }

    // update heading

    double hk = static_cast<double>(m_state.at<double>(2) + (m_f32YawRate*static_cast<tFloat32>(DEG2RAD) + m_state.at<double>(3))*dt);

    // normalize heading -pi:pi
    hk = normalizeAngle(hk, 0);

    tFloat32 sc = m_f32SpeedScale;

    // update speed and scale
    tFloat32 ak = static_cast<tFloat32>(m_state.at<double>(5));
    tFloat32 vk = m_f32Speed*(sc - ak);


    // update transition matrix; F = I + Fc*dt
    m_transitionMatrix.at<double>(0, 2) = -vk*sin(hk)*dt;
    m_transitionMatrix.at<double>(0, 3) = -vk*sin(hk)*dt;
    m_transitionMatrix.at<double>(0, 4) = cos(hk)*dt;
    m_transitionMatrix.at<double>(0, 5) = -vk / (sc - ak)*cos(hk)*dt;

    m_transitionMatrix.at<double>(1, 2) = vk*cos(hk)*dt;
    m_transitionMatrix.at<double>(1, 3) = vk*cos(hk)*dt;
    m_transitionMatrix.at<double>(1, 4) = sin(hk)*dt;
    m_transitionMatrix.at<double>(1, 5) = -vk / (sc - ak)*sin(hk)*dt;

    m_transitionMatrix.at<double>(2, 3) = dt;

    // propagate state and covariance

    m_state.at<double>(0) += vk*cos(hk)*dt;
    m_state.at<double>(1) += vk*sin(hk)*dt;
    m_state.at<double>(2) = hk;
    m_state.at<double>(4) = vk;

//    debugPointList.clear();

    GetWeights(mState, mMeanWeights, mCovWeights, alpha, beta, k);
    GetSigmaPoints(mState, mCov, mSigmaPoints, alpha, beta, k);
    MotionModel(mSigmaPoints, dt, m_f32YawRate, m_f32Speed);
    std::vector<bool> angles = {false, false, true, false, false};
    UTMotion(mSigmaPoints, mMeanWeights,mCovWeights, mState, mCov, angles);

    DebugPoint p;
    p.x =  (tFloat32)mState.at<double>(0,0);
    p.y =  (tFloat32)mState.at<double>(1,0);
    p.theta = (tFloat32)mState.at<double>(2,0);
    p.type = CAR_DEBUG;
    debugPointList.push_back(p);

    p.x =  (tFloat32)m_state.at<double>(0,0);
    p.y =  (tFloat32)m_state.at<double>(1,0);
    p.theta =  (tFloat32)m_state.at<double>(2,0);
    p.type = CAR;

    debugPointList.push_back(p);


//    for (int i = 0; i < 10; i++){
//            LOG_INFO(cString::Format("Mat [%f, %f, %f, %f, %f, $f]", m_transitionMatrix.at<double>(i, 0), m_transitionMatrix.at<double>(i, 1),m_transitionMatrix.at<double>(i, 2)
//                    ,m_transitionMatrix.at<double>(i, 3),m_transitionMatrix.at<double>(i, 4),m_transitionMatrix.at<double>(i, 5)));
//    }
//    RETURN_NOERROR;
//    for (int i = 0; i < ; i++){
//        LOG_INFO(cString::Format("Mat [%f, %f, %f, %f, %f, $f]", m_errorCov.at<double>(i, 0), m_errorCov.at<double>(i, 1),m_errorCov.at<double>(i, 2)
//                ,m_errorCov.at<double>(i, 3),m_errorCov.at<double>(i, 4),m_errorCov.at<double>(i, 5)));
//    }
//    LOG_INFO("SPACE0");

    m_errorCov = m_transitionMatrix*m_errorCov*m_transitionMatrix.t() + m_processCov;
    //LOG_INFO(cString::Format("COV error %f", error));


    // TEMP FIX FOR NOW, IF ISSUES IN THE FUTURE COME BACK AND LOOK ^^^^ THIS MULTI DOES NOT RESUTL IN SYMTERY MATRIX
    //m_errorCov = (m_errorCov + m_errorCov.t())/2;


    sendPositionStruct(tmTimeOfTrigger, static_cast<tFloat32>(m_state.at<double>(0)), static_cast<tFloat32>(m_state.at<double>(1)),
                       static_cast<tFloat32>(sqrt(m_errorCov.at<double>(0, 0) + m_errorCov.at<double>(1, 1))),
                       static_cast<tFloat32>(m_state.at<double>(2)), static_cast<tFloat32>(m_state.at<double>(4)));
//    LOG_INFO("Motion Update");

    RETURN_NOERROR;
}

bool sortByAngle(const tPolarCoordiante &lhs, const tPolarCoordiante &rhs)
{
    return lhs.f32Angle < rhs.f32Angle;
}

tResult cLocalize::LaserUpdate(tTimeStamp tmTimeOfTrigger, const adtf::streaming::ISample &oSample)
{

    auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(oSample);
    LOG_DUMP("LASER ROADSIGN: START");
    RETURN_IF_FAILED(oDecoder.IsValid());

    tSize numOfScanPoints = 0;

    // retrieve the values (using convenience methods that return a variant)
    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLSDataId.size, &numOfScanPoints));

    const tPolarCoordiante* pCoordiante = reinterpret_cast<const tPolarCoordiante*>(oDecoder.GetElementAddress(m_ddlLSDataId.scanArray));

    std::vector<tPolarCoordiante> scan;
    tPolarCoordiante scanPoint;

    for (tSize i = 0; i < numOfScanPoints; ++i){
        scanPoint.f32Radius = pCoordiante[i].f32Radius * (float)0.001;
        scanPoint.f32Angle = pCoordiante[i].f32Angle;

        if ((scanPoint.f32Angle < 90) || (scanPoint.f32Angle > 280)){

            if(scanPoint.f32Radius > 0.0001){ //! For now remove zero's b/c it messes up sign detection, mabye we are throwing away information though
                scanPoint.f32Angle = normalizeAngle(scanPoint.f32Angle* static_cast<tFloat32>(DEG2RAD), 0) * static_cast<tFloat32>(RAD2DEG);
                scanPoint.f32Angle *= -1;

                scan.insert(std::upper_bound( scan.begin(), scan.end(), scanPoint, sortByAngle), scanPoint);
//            std::upper_bound( scan.begin(), scan.end(), scanPoint, sortByAngle);
//            scan.push_back(scanPoint);
            }
        }
    }

    Pose currRoadSign;
    double timeStamp = (double)GetTime();

    if (RoadSignDetector->GetRoadSign(scan, timeStamp, currRoadSign)){

       // Measurement Update UKF
        DebugPoint p;

        // X Y relative to car reference Frame
        p.x = currRoadSign.x;
        p.y = currRoadSign.y;
        p.theta = currRoadSign.theta;

        p.Id = 100;
        p.type = LIDAR_SIGN;

        debugPointList.push_back(p);

        LOG_DUMP("LASER ROADSIGN: FOUND!!!");


        double lateral = p.y;
        double longitudinal = p.x;

        // add camera offset
        lateral += lidarOffsetLat;
        longitudinal += lidarOffsetLong;

        double d0 = sqrt(lateral*lateral + longitudinal*longitudinal);

        double a0 = atan2(lateral, longitudinal);
        a0 = (tFloat32)normalizeAngle(a0, 0); // normalize angle -pi:pi

//        a0 *= -1.0; // and change direction

        //TODO make headingUpdate an actual Update

        double headingUpdate = mState.at<double>(2);

        double shift = -1.0f*d0; // reversed translation direction

        // calculate translation direction
        double correction = headingUpdate + a0;
        correction = normalizeAngle(correction, 0);

        int ind = RoadSignDetector->GetSignInd();
        // update location estimate
        double x = (double)m_roadSigns[ind].f32X + cos(correction)*shift;
        double y = (double)m_roadSigns[ind].f32Y + sin(correction)*shift;

        std::vector<bool> measureAngles = {false, false, true};
        std::vector<bool> stateAngles = {false, false, true, false, false};
        Mat mCrossVariance;
        Mat mMeasureVector = Mat(2, 1, CV_64F, Scalar::all(0));
        Mat mMeasureCov = Mat(2, 1, CV_64F, Scalar::all(0));

        Mat data = Mat(lidarDim, 1, CV_64F, Scalar::all(0));
        data.at<double>(0, 0) = x;
        data.at<double>(1, 0) = y;
        data.at<double>(2, 0) = headingUpdate;

        GetSigmaPoints(mState, mCov, mSigmaPoints, alpha, beta, k); // Resample in case laser update has happened and changed Cov
        LidarModel(mSigmaPoints, mMeasurementPoints, currRoadSign);
        UTSensor(mMeasurementPoints, mSigmaPoints, mMeanWeights, mCovWeights, mState,
                 mMeasureVector, measureAngles, stateAngles, mMeasureCov, mCrossVariance);
        KalmanUpdate(data, mMeasureVector, mMeasureCov, mCrossVariance, mState, mCov, measureAngles);

   }
    LOG_DUMP("LASER ROADSIGN: END");

    RETURN_NOERROR;
}

MapPoint cLocalize::LaserToGlobal(tPolarCoordiante laserPoint){

    MapPoint p;

    p.x = laserPoint.f32Radius * cos(laserPoint.f32Angle * static_cast<tFloat32>(DEG2RAD));
    p.y = laserPoint.f32Radius * sin(laserPoint.f32Angle * static_cast<tFloat32>(DEG2RAD));

    return p;
}


/*! funtion will be executed each time a trigger occured */
tResult cLocalize::Process(tTimeStamp tmTimeOfTrigger)
{
    LOG_INFO("Mat START STATE");

    for (int i = 0; i < stateDim; i++){
        LOG_INFO(cString::Format("Mat [%f, %f, %f, %f, %f]", mCov.at<double>(i, 0), mCov.at<double>(i,1),mCov.at<double>(i,2)
                , mCov.at<double>(i,3), mCov.at<double>(i,4)));
    }

    LOG_INFO("Mat END STATE");

////    LOG_INFO(cString::Format("Mat [%f, %f, %f, %f, %f, %f, %f]", mMeanWeights.at<double>(0, 0), mMeanWeights.at<double>(0, 1),mMeanWeights.at<double>(0, 2),
////            mMeanWeights.at<double>(0, 3),mMeanWeights.at<double>(0, 4),mMeanWeights.at<double>(0, 5),mMeanWeights.at<double>(0, 6)));
////    LOG_INFO(cString::Format("Mat [%f, %f, %f, %f, %f, %f, %f]", mCovWeights.at<double>(0, 0), mCovWeights.at<double>(0, 1),mCovWeights.at<double>(0, 2),
////                             mCovWeights.at<double>(0, 3),mCovWeights.at<double>(0, 4),mCovWeights.at<double>(0, 5),mCovWeights.at<double>(0, 6)));
//
//    LOG_INFO("START STATE");
//    LOG_INFO(cString::Format("Mat [%f, %f, %f, %.9f, %f]", mState.at<double>(0, 0), mState.at<double>(1,0),mState.at<double>(2,0)
//            , mState.at<double>(3, 0), mState.at<double>(4,0)));
//    LOG_INFO("END STATE");

//
    object_ptr<const ISample> pReadSample;

    //LOG_INFO(cString::Format("process: %lu", tmTimeOfTrigger).GetPtr());

    while (IS_OK(m_oReaderSpeed.GetNextSample(pReadSample)))
    {
        // store speed
        auto oDecoder = m_SignalDataSampleFactory.MakeDecoderFor(*pReadSample);
        m_f32Speed = adtf_ddl::access_element::get_value(oDecoder, m_ddlSignalDataIndex.value);
        //LOG_INFO(cString::Format("process: %f", m_f32Speed));

    }

    while (IS_OK(m_oReaderIMU.GetNextSample(pReadSample)))
    {
        // predict

        ProcessInerMeasUnitSample(tmTimeOfTrigger, *pReadSample);

    }

    while (IS_OK(m_oReaderRoadSign.GetNextSample(pReadSample)))
    {
        // update
        //LOG_INFO("I Got Road Sample");
        ProcessRoadSignStructExt(tmTimeOfTrigger, *pReadSample);
    }

    while (IS_OK(laserDataIn.GetNextSample(pReadSample))) //TODO see what works best
    {
        // update
        //LOG_INFO("Line Data Recivied");
//        if (m_isInitialized == tTrue){

            LaserUpdate(tmTimeOfTrigger, *pReadSample);
//        }
    }

    SendDebugPoints();

    RETURN_NOERROR;
}
