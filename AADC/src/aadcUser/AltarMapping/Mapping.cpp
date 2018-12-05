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
#include "Mapping.h"
#include "ADTF3_OpenCV_helper.h"

// Thanks to HTWK Smart Driving 2017 for Functions

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_MAPPING_FILTER,
    "AltarMapping",
    cMapping,
    adtf::filter::pin_trigger({"input"}));


cMapping::cMapping()
{
    /*
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);
    */

    // Lazer Data Media Description
    object_ptr<IStreamType> pLSData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pLSData, m_LSStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_LSStructSampleFactory,"ui32Size", m_ddlLSDataId.size));
        (adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray));

    }

    else
    {
        LOG_WARNING("No mediadescription for tTemplateData found!");
    }

    Register(laserDataIn, "input" , pLSData);

    car.theta = 90 * DEGTORAD;

    // Lidar Position

    lidarLocal.x = 0;
    lidarLocal.y = 100;
    lidarLocal.theta = 0;

    // Map Params
    map_resolution = 100; // in mm
    wall_thickness = 50;
    csv_counter = 0;

    //Inverse Sensor model params
    angular_offset_threshold = 4  * DEGTORAD; //degrees
    log_prior = 0;

    // Grid Window
    sensor_window_length = 1000;//in mm
    sensor_window_width = 1000;

    sensor_window_rows = tInt16((sensor_window_length / map_resolution));
    sensor_window_colums = tInt16((sensor_window_width / map_resolution));



    for (tInt16 i = 0; i < occmap_width; i++) {
        for (tInt16 j = 0; j < occmap_length; j++) {
            occMap[i][j] = log_prior;
        }
    }

    gridMap = cv::Mat(GRID_WIDTH, GRID_LENGTH, CV_8UC1, cv::Scalar::all(128));
}


//implement the Configure function to read ALL Properties
tResult cMapping::Configure()
{
    car.x = 50;
    car.y = 50;
    RETURN_NOERROR;
}

tResult cMapping::Process(tTimeStamp tmTimeOfTrigger)
{

    object_ptr<const ISample> pReadSample;

    tLaserScannerData inputLSData;

    if (IS_OK(laserDataIn.GetLastSample(pReadSample)))
    {
        auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pReadSample);

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
            scan.push_back(scanPoint);

        }
        //TODO Get Most uptodate Positioning Data;

    UpdateCarPos();

    Map(scan);
    }
    RETURN_NOERROR;
}


void cMapping::UpdateCarPos(){
    car.x = 50;
    car.y = 50;
}


void cMapping::Map(std::vector<tPolarCoordiante> laser_scan){

    sensorPoseGlobal lidarGlobal;

    lidarGlobal = SensorToGlobal(car, lidarLocal);

    std::vector<tPolarCoordiante>::iterator it;

    for (tInt16 i = 0; i < laser_scan.size();i+=10) {

        // determine point position in global
        tFloat32 range = laser_scan[i].f32Radius;
        tFloat32 angle = laser_scan[i].f32Angle * DEGTORAD; // MAKE SURE THIS ISN"T CAUSING AN ERROR

        sensorPoint lidar_point;

        lidar_point.x = range * cos(angle);
        lidar_point.y = range * sin(angle);

        sensorPoint lidar_point_global = PointToGlobal(lidarGlobal, lidar_point);

        tFloat32 laser_angle_global = SensorRayAngle(lidarGlobal, lidar_point_global);

        //tInt16 map_length = sizeof(occMap) / sizeof(occMap[0]);
        //tInt16 map_width = sizeof(occMap[0]) / sizeof(occMap[0][0]);
        LOG_INFO(cString::Format("Car Pos: (%f, %f)", car.x, car.y));

        tp_y = car.y + (sensor_window_rows/2);
        bp_y = car.y - (sensor_window_rows/2);
        LOG_INFO(cString::Format("Y_Pivots: (%f, %f)", tp_y, bp_y));


        tp_x = 50 + (sensor_window_colums/2);
        bp_x = 50 - (sensor_window_colums/2);

        // i and j are in global coords s
        for (tInt16 x_cell = tp_x; x_cell < bp_x; x_cell++) {
            for (tInt16 y_cell = tp_y; y_cell < bp_y; y_cell++) {

                mapPoint center_point;

                center_point.x = (x_cell * map_resolution) + 0.5 * map_resolution;
                center_point.y = (y_cell * map_resolution) + 0.5 * map_resolution;


                tFloat32 map_distance = sqrt(pow((lidar_point_global.x - center_point.x), 2) +
                                          pow((lidar_point_global.y - center_point.y), 2));

                tFloat32 map_angle = GridAngle(center_point, lidar_point_global);

		LOG_INFO("Checking FOV");
                if ((abs(laser_angle_global - map_angle)) < angular_offset_threshold) {
			LOG_INFO("Passed Angle Check");

                    if (map_distance < range + (wall_thickness / 2)) { // if within FOV of lazer
                        LOG_INFO("Updating");
                        LOG_INFO(cString::Format("Global Cell: (%f, %f)", x_cell, y_cell));
                        CordsToMat(x_cell,y_cell);
                        LOG_INFO(cString::Format("Mat Cell: (%f, %f)",  x_cell, y_cell));

                        occMap[x_cell][y_cell] =
                                occMap[x_cell][y_cell] + InverseSensorModel(range, laser_angle_global, map_distance, map_angle) +
                                log_prior; //set value

                    }


                }
            }
        }
    }

    /*
    std::string filename = "testGrid" + std::to_string(csv_counter) + ".csv";



    if(LOG_CSV && (csv_counter % 30 == 0)) {
        writeCSV(filename, gridMap);
    }
    csv_counter++;

    }
     */


    if (csv_counter % 100 == 0){
        UpdateGrayGrid();

        Mat outputImage = gridMap;

        std::string filename = "/home/aadc/Desktop/Occupancy_Maps/map" + std::to_string(csv_counter) + ".jpg";


        imwrite(filename, outputImage);
    }

    csv_counter++;

    /*
    if (!outputImage.empty()) {
        //update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize() != m_sImageFormat.m_szMaxByteSize) {
            setTypeFromMat(m_oWriter, outputImage);
        }
        // write to pin
        writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
    }
     */

}

sensorPoseGlobal cMapping::SensorToGlobal(robotPose robot, sensorPoseLocal sensor){

    sensorPoseGlobal sensorGlobal;

    tFloat32 c = cos(robot.theta);
    tFloat32 s = sin(robot.theta);

    sensorGlobal.x = c*sensor.x + s*sensor.y + robot.x;
    sensorGlobal.y = -s*sensor.x + c*sensor.y + robot.y;;
    sensorGlobal.mounting_angle = sensor.theta;
    return sensorGlobal;

}

void cMapping::CordsToMat(tInt16& i,tInt16& j){

    i = i;
    j = (j - occmap_width) * -1;


}
sensorPoint  cMapping::PointToGlobal(sensorPoseGlobal sensor, sensorPoint data){

    sensorPoint dataGlobal;

    tFloat32 c = cos(sensor.mounting_angle);
    tFloat32 s = sin(sensor.mounting_angle);

    dataGlobal.x = c*data.x + s*data.y + sensor.x;
    dataGlobal.y = -s*data.x + c*data.y + sensor.y;;

    return dataGlobal;
}

tFloat32 cMapping::GridAngle(mapPoint grid_center, sensorPoint data){

    return atan2( (grid_center.y - data.y), (grid_center.x - data.x) );
    // change to atan2

}

tFloat32 cMapping::SensorRayAngle(sensorPoseGlobal sensor, sensorPoint data){

    return atan2( (sensor.y - data.y), (sensor.x - data.x) );

}

void cMapping::FreeSpace(GridObjects grid_object){
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

tFloat32 cMapping::InverseSensorModel(tFloat32 data_range, tFloat32 data_angle, tFloat32 grid_distance, tFloat32 grid_angle) {

    tFloat32 p_m = 0;

    if (grid_distance < data_range - wall_thickness/2) {

        p_m = 0.3;

        return std::log( p_m / (1-p_m) );

    }

    else {

        p_m = 0.7;

        return std::log( p_m / (1-p_m) );

    }
}


void cMapping::UpdateGrayGrid(){

    for (tInt16 i = 0; i < GRID_WIDTH; i++) {
        for (tInt16 j = 0; j < GRID_LENGTH; j++) {
            gridMap.at<std::uint8_t>(j, i) = static_cast<uint8_t>(GrayValue(occMap[j][i]));
        }
    }
}

tInt16 cMapping::GrayValue(double occValue) {

    double p = 1 - 1 / (1 + exp(occValue));

    tInt16 grayValue = (tInt16) ((1 - p) * 255);

    return grayValue;
}


void cMapping::writeCSV(string filename, cv::Mat m) {
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


// OPEN CV STUFF TO MOVE TO ANOTHER FLODER AT SOME POINT

