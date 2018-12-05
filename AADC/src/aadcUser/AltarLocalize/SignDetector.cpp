//
// Created by robin on 03/09/18.
//
#include "SignDetector.h"
#include "../AltarUtils/MapStructs.h"


/*! calculates modulus after division */
double mod(double x, double y)
{
    double r;
    double b_x;
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

double normalizeAngle(double alpha, double center)
{
    return mod(alpha - center + static_cast<double>(M_PI), 2.0f*static_cast<double>(M_PI)) + center - static_cast<double>(M_PI);
}


SignDetector::SignDetector(){


    // Intialize Update Variables
    signDistance = 0;
    signBearing = 0;
    lastUpdate = 0;

    // Intialize Detection Variables
    signWidth = 0.117;
    minBufferLength = signWidth - 0.05; // meters - width of sign
    maxBufferLength = signWidth + 0.05; // meters - width of sign
    signBufferLength = 0;
    maxRangeThreshold = 0.1; // meters
    minRangeThreshold = -2; // meters
    bearingThreshold = 15; //* DEGTORAD;
    timeThreshold = 10; // seconds
    regionThreshold = 0.03; //meters difference from expected
}


bool SignDetector::GetRoadSign(std::vector<tPolarCoordiante>& laserScan, double timeStamp, Pose& roadSign){

    bool found = false;
    debugPointList.clear();

    roadSign.x = 0;
    roadSign.y = 0;
    roadSign.theta = 0;

    if (!RecentlyUpdated(timeStamp)){

        LOG_DUMP("LASER ROADSIGN: Not recently Updated");

        return found;
    }

    LOG_DUMP(cString::Format("LASER ROADSIGN SIZE [%d]", laserScan.size()));

    // TODO Make it so you find an iterator to the start of your threshold
    for (uint i = 0; i < laserScan.size(); i++){

        LOG_DUMP("LASER ROADSIGN: Looping");

        tPolarCoordiante currPoint = laserScan.at(i);

        double nextDistance;

        if (InAngluarThreshhold(currPoint) && InRangeThreshhold(currPoint)){ // In Region of Interest

            if (InRegionThreshold(currPoint, laserScan.at(i+1), nextDistance)){ // If part of current Cluster

                signBuffer.push_back(currPoint);
                debugPointList.push_back(PolarToDebug(currPoint, PURPLE));
                signBufferLength += nextDistance;
            }

            else { // Check if there is a sign in the cluster, otherwise start a new cluster

                if(ValidSignWidth(signBufferLength)){ // Sign Found, Break
                    roadSign = ExtractRoadSign(signBuffer);
                    signBuffer.clear();
                    signBufferLength = 0;
                    found = true;
                    LOG_DUMP("LASER ROADSIGN: FOUND THE SIGN");

                    debugPointList.push_back(PolarToDebug(currPoint, GREEN));

//                break;
                    //! ADD THIS BACK IN
                }

                else { // Clear and Start new Cluster
                    signBuffer.clear();
                    signBufferLength = 0;
                    signBuffer.push_back(currPoint);
                    signBufferLength += nextDistance;
                    debugPointList.push_back(PolarToDebug(currPoint, WHITE));
                }

            }

        }

        else {

            if(ValidSignWidth(signBufferLength)){
                roadSign = ExtractRoadSign(signBuffer);
                signBuffer.clear();
                signBufferLength = 0;
                found = true;
//                break;
        //! ADD THIS BACK IN
            }

            else { // Reset Cluster

                signBuffer.clear();
                signBufferLength = 0;
            }


        }
//! OLD BUG WHEN YOU END A CLUSTER
//        if (InAngluarThreshhold(currPoint) && InRangeThreshhold(currPoint) && InRegionThreshold(currPoint, laserScan.at(i+1), nextDistance)){
//
//            LOG_DUMP("LASER ROADSIGN: HIT ROAD SIGN");
//
//            signBuffer.push_back(currPoint);
//            debugPointList.push_back(PolarToDebug(currPoint, WHITE));
//            signBufferLength += nextDistance;
//        }
//
//        else {
//
//            if(ValidSignWidth(signBufferLength)){
//
//                roadSign = ExtractRoadSign(signBuffer);
//                signBuffer.clear();
//                signBufferLength = 0;
//                found = true;
////                break;
//        //! ADD THIS BACK IN
//            }
//
//            else { // Reset Cluster
//
//                signBuffer.clear();
//                signBufferLength = 0;
//            }
//        }


    }
    LOG_DUMP(cString::Format("LASER ROADSIGN STAMP [%f]", timeStamp));

//    std::vector<tPolarCoordiante> cleanedScan = FilterScan(laserScan);

//    if (found){
//        for (uint i = 0; i < signBuffer.size(); i++){

//            debugPointList.push_back(PolarToDebug(signBuffer[i], PURPLE));

//        }
//    }
    return found;
}

void SignDetector::ExpectedSign(double distance, double bearing, double timeStamp, int id){

    LOG_DUMP("LASER ROADSIGN: Updating Expected Values");

    // Update Values
    signDistance = distance;
    signBearing = bearing;
    lastUpdate = timeStamp;
    signInd = id;
}// Values are all given relative to the car

int SignDetector::GetSignInd(){

return signInd;
}


std::vector<tPolarCoordiante> SignDetector::FilterScan(std::vector<tPolarCoordiante> laserScan) {

    // Do this more Intelligently


    return laserScan;
}


bool SignDetector::InAngluarThreshhold(tPolarCoordiante currPoint){

    double dtheta = abs(currPoint.f32Angle - signBearing);

    LOG_DUMP(cString::Format("LASER ROADSIGN: InAngluarThreshhold [%f - %f]", currPoint.f32Angle , signBearing));

    if (dtheta > bearingThreshold){

        LOG_DUMP("LASER ROADSIGN: Failed Bearing Threshold");
//        debugPointList.push_back(PolarToDebug(currPoint, RED));
        return false;
    }

    return true;
}

bool SignDetector::InRangeThreshhold(tPolarCoordiante currPoint){

    double ds = (currPoint.f32Radius - signDistance);

    LOG_DUMP(cString::Format("LASER ROADSIGN: InRangeThreshhold [%f - %f]", currPoint.f32Radius , signDistance));

    if (ds > minRangeThreshold && ds < maxRangeThreshold){

        return true;
    }
    LOG_DUMP("LASER ROADSIGN: Failed Range Threshold");

    debugPointList.push_back(PolarToDebug(currPoint, BLUE));

    return false;

}

bool SignDetector::InRegionThreshold(tPolarCoordiante currPoint, tPolarCoordiante nextPoint, double& nextDistance){

    nextDistance = 0;

    MapPoint cP;
    MapPoint nP;

    cP.x =  currPoint.f32Radius * cos(currPoint.f32Angle * static_cast<tFloat32 >(DEG2RAD));
    cP.y =  currPoint.f32Radius * sin(currPoint.f32Angle * static_cast<tFloat32 >(DEG2RAD));
    nP.x =  nextPoint.f32Radius * cos(nextPoint.f32Angle * static_cast<tFloat32 >(DEG2RAD));
    nP.y =  nextPoint.f32Radius * sin(nextPoint.f32Angle * static_cast<tFloat32 >(DEG2RAD));

    double expectedDist = currPoint.f32Radius * (abs(nextPoint.f32Angle - currPoint.f32Angle) * DEG2RAD); // If perfectly flat;

    nextDistance = sqrt( pow(nP.x - cP.x,2) + pow(nP.y - cP.y, 2) );
    LOG_DUMP(cString::Format("LASER ROADSIGN: RegionThreshhold Points Angles [%f - %f]", currPoint.f32Angle , nextPoint.f32Angle));
    LOG_DUMP(cString::Format("LASER ROADSIGN: RegionThreshhold Points Radius [%f - %f]", currPoint.f32Radius , nextPoint.f32Radius));

    LOG_DUMP(cString::Format("LASER ROADSIGN: RegionThreshhold [%f - %f]", nextDistance , expectedDist));


    if (abs(nextDistance - expectedDist) > regionThreshold){
        LOG_DUMP("LASER ROADSIGN: Failed Region Threshold");
        return false;

    }

    return true;

}

bool SignDetector::ValidSignWidth(double signBufferLength){

    if (signBufferLength > minBufferLength && signBufferLength < maxBufferLength) {

        return true;

    }

    LOG_DUMP("LASER ROADSIGN:Failed Sign Width Threshold");

    return false;

}

Pose SignDetector::ExtractRoadSign(std::vector<tPolarCoordiante> &signBuffer){

    Pose roadSign;

    // Calculate Center of Mass

    roadSign.x = 0;
    roadSign.y = 0;

    int n = signBuffer.size();
    std::vector<double> xPoints;
    std::vector<double> yPoints;

    double tempX;
    double tempY;

    for(uint i = 0; i < n; i++){

        // Calculate Center of Mass
        tempX = signBuffer[i].f32Radius * cos(signBuffer[i].f32Angle * static_cast<tFloat32 >(DEG2RAD));
        tempY = signBuffer[i].f32Radius * sin(signBuffer[i].f32Angle * static_cast<tFloat32 >(DEG2RAD));

        roadSign.x +=  tempX;
        roadSign.y +=  tempY;

        xPoints.push_back(tempX);
        yPoints.push_back(tempY);

    }

    roadSign.x /= n;
    roadSign.y /= n;


    double numerator = 0;
    double denominator = 0;

    for(uint i = 0; i < n; i++){

        numerator += (xPoints[i] - roadSign.x) * (yPoints[i] - roadSign.y);
        denominator += pow((xPoints[i] - roadSign.x),2);


    }

    double a = numerator/denominator;
//    double b = roadSign.y - (a * roadSign.x);
    double theta = atan(a) * RAD2DEG;
    // Fit Line

    // Take Slope of line
    roadSign.theta = (tFloat32)theta;


//! TO TEST SLOPE
//    std::vector<double> xTest = {0, 0.1, -0.1, 0, 0, -0.2, 0.2};
//    std::vector<double> yTest = {5, 10, -20, 1, 4, -2, 5.3};
//    int nTest = yTest.size();
//    double xTestm;
//    double yTestm;
//
//    for(uint i = 0; i < nTest; i++){
//
//        xTestm += xTest[i];
//        yTestm += yTest[i];
//    }
//
//    xTestm /= nTest;
//    yTestm /= nTest;
//
//    for(uint i = 0; i < nTest; i++){
//
//        xTestm += xTest[i];
//        yTestm += yTest[i];
//    }
//
//    xTestm /= nTest;
//    yTestm /= nTest;
//
//    numerator = 0;
//   denominator = 0;
//
//    for(uint i = 0; i < nTest; i++){
//
//        numerator += (xTest[i] - xTestm) * (yTest[i] - yTestm);
//         denominator += pow((xTest[i] - xTestm),2);
//    }
//
//    a = numerator/denominator;
////    double b = roadSign.y - (a * roadSign.x);
//    theta = atan(a) * RAD2DEG;
//    // Fit Line
//
//    // Take Slope of line
//    roadSign.theta = (tFloat32)theta;

    return roadSign;
}


bool SignDetector::RecentlyUpdated(double timeStamp){

    double dt = (timeStamp - lastUpdate) * 1e-6;

    if (dt > timeThreshold){
        return false;
    }

    else{
        return true;
//        return false;
    }
}

DebugPoint SignDetector::PolarToDebug(tPolarCoordiante point, int colour){

    DebugPoint p;

    p.x = point.f32Radius * cos(point.f32Angle * static_cast<float>(DEG2RAD));
    p.y = point.f32Radius * sin(point.f32Angle * static_cast<float>(DEG2RAD));
    p.theta = point.f32Angle;
    p.type = HIT_POINT;
    p.Id = 1;
    p.colour = colour;

    return p;

}
std::vector<DebugPoint> SignDetector::GetDebugPoints(){
//    std::vector<DebugPoint> blank;
//    return blank;
    return debugPointList;
}

//void SignDetector::ClearDebugPoints(){

//    debugPointList.clear();
//}


