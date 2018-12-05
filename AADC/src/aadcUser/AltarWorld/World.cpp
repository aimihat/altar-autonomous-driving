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
#include "World.h"
#include "ADTF3_OpenCV_helper.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_MAPPING_FILTER,
    "AltarWorld",
    cWorld,
    adtf::filter::pin_trigger({"LaserScanner"}));


cWorld::cWorld()
{

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

    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register output pin
    Register(m_oWriter, "Map", pType);

    /*
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
                                    { return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter); });

*/

    // Lazer Data

    object_ptr<IStreamType> pLSData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pLSData, m_LSStructSampleFactory)) {

        (adtf_ddl::access_element::find_index(m_LSStructSampleFactory,"ui32Size", m_ddlLSDataId.size));
        (adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray));
    }

    else {

        LOG_WARNING("No mediadescription for tTemplateData found!");
    }

    Register(laserDataIn, "LaserScanner" , pLSData);

    //Loop vars

    // Lidar Position
    lidarLocal.x = 250;
    lidarLocal.y = 0;
    lidarLocal.theta = 0 * DEGTORAD;

    // Map Params
    map_res = 50; // in mm
    wall_thickness = 100;
    csv_counter = 0;

    //Inverse Sensor model params
    angular_offset_threshold = 4.0  * DEGTORAD; //degrees
    log_prior = 0;

    // Grid Window
    window_length = 2000;//in mm
    window_width = 2000;

    window_rows = window_length / map_res;
    window_colums = window_width / map_res;

    // Map initlization

    car_initalized = false;

    for (int i = 0; i < OCCMAP_WIDTH; i++) {
        for (int j = 0; j < OCCMAP_LENGTH; j++) {
            occMap[i][j] = log_prior;
        }
    }

    gridMap = cv::Mat(OCCMAP_WIDTH, OCCMAP_LENGTH, CV_8UC1, cv::Scalar::all(128));
}


//implement the Configure function to read ALL Properties
tResult cWorld::Configure()
{
    //-------------------------------
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    //--------------------------------
    RETURN_NOERROR;
}
tResult cWorld::CarInit(){

    LOG_DUMP("Updating Car Position");

    object_ptr<const ISample> pReadSample;
    tFloat32 f32x, f32y, f32heading;

    if (IS_OK(PoseDataIn.GetLastSample(pReadSample)))
    {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(pReadSample);
        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &f32x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &f32y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &f32heading));
        car.theta = f32heading;
        car.x = f32x;
        car.y = f32y;
        LOG_DUMP("Initalized");

        LOG_DUMP(cString::Format("Car Pos: (%f, %f, %f)", car.x, car.y, car.theta));
        car_initalized = true;

        //LOG_DUMP(cString::Format("Car Pos - map Coords: (%f, %f)", car.x/map_res, car.y/map_res));

    }

    RETURN_NOERROR;
}


tResult cWorld::Process(tTimeStamp tmTimeOfTrigger)
{

    if (!car_initalized){
        CarInit();
        LOG_DUMP("Waiting for Init.....");
        RETURN_NOERROR;
    }


    UpdateCarPos();

    // Read Lazer Data

    object_ptr<const ISample> pReadLaserSample;


    if (IS_OK(laserDataIn.GetLastSample(pReadLaserSample)))
    {
        auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pReadLaserSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        tSize numOfScanPoints = 0;

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLSDataId.size, &numOfScanPoints));

        const tPolarCoordiante* pCoordiante = reinterpret_cast<const tPolarCoordiante*>(oDecoder.GetElementAddress(m_ddlLSDataId.scanArray));

        std::vector<tPolarCoordiante> scan;
        tPolarCoordiante scanPoint;

        for (tSize i = 0; i < numOfScanPoints; ++i){
            scanPoint.f32Radius = pCoordiante[i].f32Radius;
            scanPoint.f32Angle = pCoordiante[i].f32Angle;
            if ((scanPoint.f32Angle < 90) || (scanPoint.f32Angle > 280)){

                scan.push_back(scanPoint);

            }
        }

        LOG_DUMP("MAPPING!");
        Map(scan);
    }

    StreamWorld();

    RETURN_NOERROR;
}
void cWorld::UpdateCarUI(){

    tInt16 car_x = tInt16(roundf(car.x/map_res));
    tInt16 car_y = tInt16(roundf(car.y/map_res));
    CordsToMat(car_x,car_y);
    sensorPoseGlobal lidar;
    lidar = SensorToGlobal(car, lidarLocal);

    tInt16 lidar_x = tInt16(roundf(lidar.x/map_res));
    tInt16 lidar_y = tInt16(roundf(lidar.y/map_res));

    CordsToMat(lidar_x,lidar_y);
    /*
    gridMap.at<std::uint8_t>(lidar_y, lidar_x) = static_cast<uint8_t>(0);
    gridMap.at<std::uint8_t>(50, 100) = static_cast<uint8_t>(0);

    gridMap.at<std::uint8_t>(car_y, car_x) = static_cast<uint8_t>(0);
    gridMap.at<std::uint8_t>(car_y+1, car_x) = static_cast<uint8_t>(80);
    gridMap.at<std::uint8_t>(car_y-1, car_x) = static_cast<uint8_t>(80);
    gridMap.at<std::uint8_t>(car_y, car_x+1) = static_cast<uint8_t>(80);
    gridMap.at<std::uint8_t>(car_y, car_x-1) = static_cast<uint8_t>(80);
    gridMap.at<std::uint8_t>(car_y+1, car_x+-1) = static_cast<uint8_t>(80);
    gridMap.at<std::uint8_t>(car_y+1, car_x+1) = static_cast<uint8_t>(80);
    gridMap.at<std::uint8_t>(car_y-1, car_x+1) = static_cast<uint8_t>(80);
    gridMap.at<std::uint8_t>(car_y-1, car_x-1) = static_cast<uint8_t>(80);
    gridMap.at<std::uint8_t>(car_y-1, car_x-1) = static_cast<u int8_t>(80);
     */

    if ((lidar_x >= 0) && (lidar_x < OCCMAP_WIDTH) && (lidar_y >= 0) && (lidar_y < OCCMAP_LENGTH)) {
        gridMap.at<std::uint8_t>(lidar_y, lidar_x) = static_cast<uint8_t>(80);

    }

    if ((car_x >= 0) && (car_x < OCCMAP_WIDTH) && (car_y >= 0) && (car_y < OCCMAP_LENGTH)) {
        gridMap.at<std::uint8_t>(car_y, car_x) = static_cast<uint8_t>(0);

    }






    /*
    LOG_INFO(cString::Format("Car Pos - map coords: (%f, %f, %f)", car.x, car.y, car.theta));
    LOG_INFO(cString::Format("Lidar Pos - map coords: (%f, %f, %f)", lidar.x, lidar.y, lidar.theta));
    LOG_INFO(cString::Format("Car Pos - map coords: (%d, %d)", car_x, car_y));
    LOG_INFO(cString::Format("Lidar Pos - map coords: (%d, %d)", lidar_x, lidar_y));
     */


}
void cWorld::StreamWorld(){

    UpdateCarUI();

    if (csv_counter % 40 == 0){
        UpdateGrayGrid();

        //std::string filename = "/home/aadc/Desktop/Occupancy_Maps/map" + std::to_string(csv_counter) + ".jpg";

        //imwrite(filename, outputImage);
    }
    Mat outputImage = gridMap;
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

    csv_counter++;
}

  tResult cWorld::UpdateCarPos(){

    LOG_DUMP("Updating Car Position");

    object_ptr<const ISample> pReadSample;
      tFloat32 f32x, f32y, f32heading;


      if (IS_OK(PoseDataIn.GetLastSample(pReadSample)))
    {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(pReadSample);
        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &f32x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &f32y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &f32heading));
        car.theta = f32heading;
        car.x = f32x;
        car.y = f32y;

        LOG_DUMP(cString::Format("Car Pos: (%f, %f, %f)", car.x, car.y, car.theta));
        //LOG_DUMP(cString::Format("Car Pos - map Coords: (%f, %f)", car.x/map_res, car.y/map_res));

    }

    RETURN_NOERROR;
}


void cWorld::Map(std::vector<tPolarCoordiante> laser_scan){

        /*
    if (count >= 20){

        count += 1;
    }

    else {
        count = 1;

    }
    */
    sensorPoseGlobal lidarGlobal;

    lidarGlobal = SensorToGlobal(car, lidarLocal);

    std::vector<tPolarCoordiante>::iterator it;
    globalSensor.x = lidarGlobal.x;
    globalSensor.y = lidarGlobal.y;

    for (tUInt16 i = 1; i < laser_scan.size();i+=20) {

        // determine point position in global
        tFloat32 range = laser_scan[i].f32Radius;
        tFloat32 angle = laser_scan[i].f32Angle * DEGTORAD; // MAKE SURE THIS ISN"T CAUSING AN ERROR
        sensorPoint lidar_point;

        lidar_point.x = range * cos(angle);
        lidar_point.y = range * sin(angle) * -1; //need to flip over x_axis

        sensorPoint lidar_point_global = PointToGlobal(lidarGlobal, lidar_point);


        tFloat32 laser_angle_global = SensorRayAngle(lidarGlobal, lidar_point_global);


        tp_y = tInt16(roundf((globalSensor.y / map_res) + (window_rows / 2)));
        bp_y = tInt16(roundf((globalSensor.y / map_res) - (window_rows / 2)));

        tp_x = tInt16(roundf((globalSensor.x / map_res) + (window_colums / 2)));
        bp_x = tInt16(roundf((globalSensor.x / map_res) - (window_colums / 2)));


        //REMEBER IF YOUR IN REAL WORLD COORDS OR GRID COORDS
        for (tInt16 grid_y = bp_y; grid_y < tp_y; grid_y += 1) {
            for (tInt16 grid_x = bp_x; grid_x < tp_x; grid_x += 1) {

                MapPoint center_point;
                center_point.x = (grid_x * map_res) + 0.5 * map_res;
                center_point.y = (grid_y * map_res) + 0.5 * map_res;

                tFloat32 map_distance = sqrt(pow((lidarGlobal.x - center_point.x), 2) +
                                             pow((lidarGlobal.y - center_point.y), 2));

                tFloat32 map_angle = GridAngle(center_point, lidarGlobal);

                if ((abs(laser_angle_global - map_angle)) < angular_offset_threshold) {// if within FOV of lazer

                    //LOG_INFO(cString::Format("Lazer Angle (%f)", laser_angle_global));
                    //LOG_INFO(cString::Format("Angle: (%f)", map_angle));

                    tInt16 i = grid_x;
                    tInt16 j = grid_y;

                    CordsToMat(i, j);

                    if ((i >= 0) && (i <= OCCMAP_WIDTH) && (j >= 0) && (j <= OCCMAP_LENGTH)) {

                        occMap[i][j] =
                                occMap[i][j] + InverseSensorModel(range, laser_angle_global, map_distance, map_angle)
                                + log_prior; //set value

                    }

                }
            }
        }
    }

}

sensorPoseGlobal cWorld::SensorToGlobal(robotPose robot, sensorPoseLocal sensor){

    sensorPoseGlobal sensorGlobal;

    tFloat32 c = cos(robot.theta);
    tFloat32 s = sin(robot.theta);

    sensorGlobal.x = c*sensor.x - s*sensor.y + robot.x;
    sensorGlobal.y = s*sensor.x + c*sensor.y + robot.y;;
    sensorGlobal.theta = sensor.theta + robot.theta;
    return sensorGlobal;

}

void cWorld::CordsToMat(tInt16& i,tInt16& j){

    i = i ;
    j = (j - OCCMAP_WIDTH) * -1;

}

sensorPoint  cWorld::PointToGlobal(sensorPoseGlobal sensor, sensorPoint data){

    sensorPoint dataGlobal;

    float theta = sensor.theta; //because forward is along y-axis
    tFloat32 c = cos(theta);
    tFloat32 s = sin(theta);

    dataGlobal.x = c*data.x + -s*data.y + sensor.x;
    dataGlobal.y = s*data.x + c*data.y + sensor.y;;

    return dataGlobal;
}

tFloat32 cWorld::GridAngle(MapPoint grid_center, sensorPoseGlobal sensor){

    return atan2( (grid_center.y - sensor.y), (grid_center.x - sensor.x) );
}

tFloat32 cWorld::SensorRayAngle(sensorPoseGlobal sensor, sensorPoint data){

    return atan2( (data.y-sensor.y), (data.x - sensor.x) );
}

void cWorld::FreeSpace(GridObjects grid_object){
    /*
    tInt16 tx = grid_object.tp_x;
    tInt16 ty = grid_object.tp_x;

    tInt16 bx = grid_object.bp_x;
    tInt16 by = grid_object.bp_y;

    for (tInt16 x_cell = tx; x_cell < bx; x_cell++) {
        for (tInt16 y_cell = ty; y_cell < by; y_cell++) {

            occMap[x_cell][y_cell] += 0;
        }
    }
     */

}

tFloat32 cWorld::InverseSensorModel(tFloat32 data_range, tFloat32 data_angle, tFloat32 grid_distance, tFloat32 grid_angle) {

    tFloat32 p_m = 0.5;

    if ((grid_distance < data_range - wall_thickness/2) && (grid_distance > 0)){

        p_m = 0.3;

        return std::log( p_m / (1-p_m) );

    }

    else if ((grid_distance > data_range - wall_thickness/2) && (grid_distance < data_range + wall_thickness/2)){

        //LOG_INFO("Updating Occupied Cell");

        p_m = 0.7;

        return std::log( p_m / (1-p_m) );

    }
    return std::log( p_m / (1-p_m) );

}


void cWorld::UpdateGrayGrid(){

    for (tInt16 i = 0; i < OCCMAP_WIDTH; i++) {
        for (tInt16 j = 0; j < OCCMAP_LENGTH; j++) {
            gridMap.at<std::uint8_t>(i, j) = static_cast<uint8_t>(GrayValue(occMap[j][i])); //GRID MAP INDEX IS REVERSED
        }
    }
}

tInt16 cWorld::GrayValue(double occValue) {

    double p = 1 - 1 / (1 + exp(occValue));

    tInt16 grayValue = (tInt16) ((1 - p) * 255);

    return grayValue;
}


void cWorld::writeCSV(string filename, cv::Mat m) {
    std::fstream outputFile;
    outputFile.open(filename, std::ios::out ) ;

    for(tInt16 i=0; i<m.rows; i++) {
        for(tInt16 j=0; j<m.cols; j++)
        {
            outputFile << static_cast<tInt16>(m.at<std::uint8_t>(j, i)) << ", ";
        }
        outputFile << endl;

    }
    outputFile.close( );
}



