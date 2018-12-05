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

#include "openDriveReader.h"
#include "../GraphViz.h"


openDriveReader::openDriveReader(std::string file)
{

    BiDirectional = BIDIRECTIONAL;
    SingleLane = SINGLELANE;
    Altitude = ALTITUDE;
    Scale = MAP_SCALE;
    LaneWidth = (float)LANEWIDTH;
    LaneWidth = LaneWidth;
    LoadMap(file);
}

openDriveReader::openDriveReader()
{
    BiDirectional = BIDIRECTIONAL;
    SingleLane = SINGLELANE;
    Altitude = ALTITUDE;
    Scale = MAP_SCALE;
    LaneWidth = (float)LANEWIDTH;
}
openDriveReader::~openDriveReader()
{

}


void openDriveReader::LoadMap(std::string file, int num)
{
    FileName = file.c_str();

#ifndef WIN32
    std::locale::global(std::locale("en_US.utf8"));
#endif WIN32

    FileReadErr = Doc.LoadFile(FileName);
    if (FileReadErr == 0)
    {
        //Read Vector
        RoadListOrg.clear();
        RoadList.clear();
        ReadFile(num);
    }
    return;


}

void openDriveReader::ReadFile(int num)
{

    
    XMLNode *pRoot = Doc.FirstChildElement("OpenDRIVE");
    if (pRoot == NULL)
    {
        return;
    }
    XMLElement *pRoad = pRoot->FirstChildElement("road");
    while (pRoad != NULL)
    {
        roadElementWrapper curRoadElement;
        curRoadElement.predET = "";
        curRoadElement.succET = "";

        pRoad->QueryIntAttribute("id", &curRoadElement.id);
        pRoad->QueryIntAttribute("junction", &curRoadElement.junction);
        XMLElement *pPredecessor = pRoad->FirstChildElement("link")->FirstChildElement("predecessor");
        while (pPredecessor != NULL)
        {
            curRoadElement.predCP = pPredecessor->Attribute("contactPoint");
            pPredecessor->QueryIntAttribute("elementId", &curRoadElement.predId);
            curRoadElement.predET = pPredecessor->Attribute("elementType");
            pPredecessor = pPredecessor->NextSiblingElement("predecessor");
        }
        XMLElement *pSuccessor = pRoad->FirstChildElement("link")->FirstChildElement("successor");
        while (pSuccessor != NULL)
        {
            curRoadElement.succCP = pSuccessor->Attribute("contactPoint");
            pSuccessor->QueryIntAttribute("elementId", &curRoadElement.succId);
            curRoadElement.succET = pSuccessor->Attribute("elementType");
            pSuccessor = pSuccessor->NextSiblingElement("successor");
        }
        XMLElement *pGeometry = pRoad->FirstChildElement("planView")->FirstChildElement("geometry");
        while (pGeometry != NULL)
        {
            roadGeometryWrapper geometry;
            pGeometry->QueryFloatAttribute("hdg", &geometry.hdg);
            pGeometry->QueryFloatAttribute("length", &geometry.length);
            pGeometry->QueryFloatAttribute("s", &geometry.s);
            pGeometry->QueryFloatAttribute("x", &geometry.x);
            pGeometry->QueryFloatAttribute("y", &geometry.y);
            XMLElement *pParamPoly = pGeometry->FirstChildElement("paramPoly3");
            while (pParamPoly != NULL)
            {
                pParamPoly->QueryFloatAttribute("aU", &geometry.aU);
                pParamPoly->QueryFloatAttribute("aV", &geometry.aV);
                pParamPoly->QueryFloatAttribute("bU", &geometry.bU);
                pParamPoly->QueryFloatAttribute("bV", &geometry.bV);
                pParamPoly->QueryFloatAttribute("cU", &geometry.cU);
                pParamPoly->QueryFloatAttribute("cV", &geometry.cV);
                pParamPoly->QueryFloatAttribute("dU", &geometry.dU);
                pParamPoly->QueryFloatAttribute("dV", &geometry.dV);
                curRoadElement.geometry.push_back(geometry);
                pParamPoly = pParamPoly->NextSiblingElement("paramPoly3");
            }
            pGeometry = pGeometry->NextSiblingElement("geometry");
        }
        curRoadElement.cost = 99999999999;
        curRoadElement.nextRoad = -1;
        curRoadElement.altitude = Altitude;
        curRoadElement.scale = Scale;
        curRoadElement.laneSplit = 0.0;
        RoadListOrg.push_back(curRoadElement);
        pRoad = pRoad->NextSiblingElement("road");
    }
    if (SingleLane == true)
    {
        MergeLanes();
    }
    else {
        SplitLanes();
    }
    GetNodes();
    MapPoints = GetRoadPoints(RoadList, num);
    MapElList = GetMapPoints(roads, 4);
    return;
}

void openDriveReader::MergeLanes()
{
    std::vector<int> removelist;
    for (int i = 0; i < (int)RoadListOrg.size(); i++)
    {
        int removeElement = -1;
        for (int j = i + 1; j < (int)RoadListOrg.size(); j++)
        {
            if (RoadListOrg[i].succId == RoadListOrg[j].succId && RoadListOrg[i].predId == RoadListOrg[j].predId)
            {
                removeElement = j;
                removelist.push_back(j);
            }
        }
        //Parallel lane not found,Add the lane
        if (removeElement == -1)
        {
            for (int k = 0; k < (int)removelist.size(); k++) {
                if (removelist[k] == i)
                {
                    removeElement = i;
               }
            }
            if (removeElement == -1)
            {
                RoadList.push_back(RoadListOrg[i]);
            }
        }
        else {
            //if Parallel lane found average the values and add
            roadGeometryWrapper geo1 = RoadListOrg[i].geometry[0];
            roadGeometryWrapper geo2 = RoadListOrg[removeElement].geometry[0];
            roadElementWrapper mergedRoad = RoadListOrg[i];
            mergedRoad.geometry[0].x = (geo1.x + geo2.x) / 2;
            mergedRoad.geometry[0].y = (geo1.y + geo2.y) / 2;
            mergedRoad.geometry[0].hdg = (geo1.hdg + geo2.hdg) / 2;
            mergedRoad.geometry[0].s = (geo1.s + geo2.s) / 2;
            mergedRoad.geometry[0].length = (geo1.length + geo2.length) / 2;
            mergedRoad.geometry[0].aU = (geo1.aU + geo2.aU) / 2;
            mergedRoad.geometry[0].aV = (geo1.aV + geo2.aV) / 2;
            mergedRoad.geometry[0].bU = (geo1.bU + geo2.bU) / 2;
            mergedRoad.geometry[0].bV = (geo1.bV + geo2.bV) / 2;
            mergedRoad.geometry[0].cU = (geo1.cU + geo2.cU) / 2;
            mergedRoad.geometry[0].cV = (geo1.cV + geo2.cV) / 2;
            mergedRoad.geometry[0].dU = (geo1.dU + geo2.dU) / 2;
            mergedRoad.geometry[0].dV = (geo1.dV + geo2.dV) / 2;
            RoadList.push_back(mergedRoad);
        }
    }
    return;
}

void openDriveReader::LatticePoints(std::map<int, roadElementWrapper> roadNetwork, std::vector<int> segmentId, float lateral_density, float longitudinal_density)
{
    int vect_count = 0;
    float road_width = 0.440;

    for (int i = 0; i < segmentId.size(); i++) {

        int currSegId = segmentId[i];
        roadElementWrapper currEl = roadNetwork[currSegId];
        roadGeometryWrapper geo = currEl.geometry[0];

        float road_length = geo.length;

        int num_lat_points = (int)round(road_width * lateral_density);
        int num_long_points = (int)round(road_length * longitudinal_density);
        num_lat_points = 5; // ODD NUMBER
        num_long_points = 4;

        int points_per_lane = 3; // must be odd
        num_lat_points = (points_per_lane * 2) - 1;

//        float midline_dist = abs(currEl.laneSplit / currEl.scale);
        float midline_dist = abs(LaneWidth);

        float latStep = (midline_dist / (points_per_lane/2)); //lines from road line to road line
        float latScale = 0;

        std::vector<float> tempxs;
        std::vector<float> tempys;


        // Generate points along line segment. Points for straight driving are down the middle of the lane, points for curves are long middle of curve
        for (int z = 0; z < num_long_points; z++) {

            float ds = (float)(z + 0.1) / (num_long_points);

            //Find points from parametric polynomial in local coordinate
            tempxs.push_back(CubicPoly(geo.aU, geo.bU, geo.cU, geo.dU, ds));
            tempys.push_back(CubicPoly(geo.aV, geo.bV, geo.cV, geo.dV, ds));

        }


        bool flag = true;

        //TODO speical case for turns
        // Generate Graph based on the sampled points

        for (int j = 0; j < num_lat_points; j++){ // for each lateral point

            std::vector<Pose3D> list;
            Pose3D pose;

            float curHeading = 0.0, headingChange = 0.0, lastHeading = 0.0;

            for (int k = 0; k < num_long_points; k++)
            {
                if (k == 0){
                    curHeading = 0;
                }
                else if (k == num_long_points - 1) {

                    float x0 = tempxs[k - 1], y0 = tempys[k - 1];
                    float x1 = tempxs[k], y1 = tempys[k];

                    curHeading = atan2(y1 - y0, x1 - x0);
                }

                else
                {
                    float x0 = tempxs[k - 1], y0 = tempys[k - 1];
                    float x1 = tempxs[k + 1], y1 = tempys[k + 1];

                    curHeading = atan2(y1 - y0, x1 - x0);

//                    lastHeading = curHeading;
//                    curHeading = atan2(y1 - y0, x1 - x0);
//                    headingChange += normalizeAngle(curHeading - lastHeading, 0);
//                    headingChange = normalizeAngle(headingChange, 0);
                }

                graphNode currGraphNode;

                // LATERAL TRANSFORMATION

                pose.p.x = (tempxs[k]) * currEl.scale;
                pose.p.y = (tempys[k]) * currEl.scale;
                pose.p.z = currEl.altitude;
                pose.q = toQuaternion(0, 0, geo.hdg);

                float tempLanex = RotateCCWX(0, latScale / currEl.scale, curHeading);
                float tempLaney = RotateCCWY(0, latScale / currEl.scale, curHeading);

                currGraphNode.p.x =  pose.p.x + tempLanex * currEl.scale ;
                currGraphNode.p.y =  pose.p.y  + tempLaney * currEl.scale ;

                // GLOBAL TRANSFORMATION
                float newx = RotateCCWX(currGraphNode.p.x,currGraphNode.p.y, geo.hdg);
                float newy = RotateCCWY(currGraphNode.p.x, currGraphNode.p.y, geo.hdg);
                currGraphNode.p.x = newx + (geo.x* currEl.scale);
                currGraphNode.p.y = newy + (geo.y* currEl.scale);


                // ADD LATERAL OFFSET TO NODE FOR USE IN PATH PLANNING, NEED TO DEAL WITH TURNS SEPERATELY
                if (roadNetwork[currSegId].junction != -1) {

                    currGraphNode.distanceToMid = latScale / currEl.scale;
                }
                else {
                        currGraphNode.distanceToMid = (latScale - roadNetwork[currSegId].laneSplit) / currEl.scale; // Lane split will either be + or minues 10
                }
                // CURRENT NODE ID

                nodeId currId;
                currId.roadId = currSegId;
                currId.uniqueId =  j + k * num_lat_points + 1;

                if (i == 0 && k == 0 && j == 0) {

                    startNodeId = currId;
                }

                if (i == segmentId.size() - 1 && k == num_long_points - 1) {

                    startNodeId = currId;

                    //TODO all for many end nodes
                }

                // ADD CONNECTION INFORMATION

                for (int t = 0; t < num_lat_points; t++) {

                    if (k != 0 && k != num_long_points - 1) { // NORMAL CASE - nodes in the middle

                        nodeId childId;
                        childId.roadId = currSegId;

                        if (!roadNetwork[currSegId].reverse) {
                            childId.uniqueId = t + (k + 1) * num_lat_points + 1;
                            currGraphNode.succ->push_back(childId);

                            childId.uniqueId = t + (k - 1) * num_lat_points + 1;
                            currGraphNode.pred->push_back(childId);
                        }

                        else {
                            childId.uniqueId = t + (k - 1) * num_lat_points + 1;
                            currGraphNode.succ->push_back(childId);

                            childId.uniqueId = t + (k + 1) * num_lat_points + 1;
                            currGraphNode.pred->push_back(childId);
                        }
                    }

                    else if (k == 0) { // START CASE - Node in first index position, can either be the start of end of segment

                        nodeId childId;
                        childId.roadId = currSegId;

                        if (!roadNetwork[currSegId].reverse) {

                            if (t == 0) { // Only assign once

                                if (i != 0) { // if not on the first line segment

                                    int prevSegId = segmentId[i - 1];
                                    currGraphNode.pred = &roadNetwork[prevSegId].endNodes;

                                }

                                roadNetwork[currSegId].startNodes.push_back(currId); // add CURRENT ID to road information so other line segments can query correctly
                            }

                            childId.uniqueId = t + (k + 1) * num_lat_points + 1;
                            currGraphNode.succ->push_back(childId);

                            LOG_INFO("WORKING FOR FIRST ONE");
                        }

                        else {

                            LOG_INFO("NOT WORKING SHOULD NOT BE RESERVESE");


                            if (t == 0) { // Only assign once

                                if (i < segmentId.size() - 1) { // if not on last segment

                                    int nextSegId = segmentId[i + 1];
                                    currGraphNode.succ = &roadNetwork[nextSegId].startNodes;

                                }

                                roadNetwork[currSegId].endNodes.push_back(currId); // add CURRENT ID to road information so other line segments can query correctly

                            }

                            childId.uniqueId = t + (k + 1) * num_lat_points + 1;
                            currGraphNode.pred->push_back(childId);
                        }
                    }


                    else if (k == num_long_points - 1) { // END CASE - Node in last index position, usally at the end, but can be at the start

                        nodeId childId;
                        childId.roadId = currSegId;

                        if (!roadNetwork[currSegId].reverse) {
                            if (t == 0) { // Only assign once

                                if (i < segmentId.size() - 1) { // if not on the first line segment

                                    int nextSegId = segmentId[i + 1];
                                    currGraphNode.succ = &roadNetwork[nextSegId].startNodes;

                                }

                                roadNetwork[currSegId].endNodes.push_back(currId); // add CURRENT ID to road information so other line segments can query correctly

                            }

                            childId.uniqueId = t + (k - 1) * num_lat_points + 1;
                            currGraphNode.pred->push_back(childId);
                        }

                        else {

                            if (t == 0) { // Only assign once

                                if (i != 0) { // if not on the first line segment

                                    int prevSegId = segmentId[i - 1];
                                    currGraphNode.pred = &roadNetwork[prevSegId].endNodes;

                                }

                                roadNetwork[currSegId].startNodes.push_back(currId); // add CURRENT ID to road information so other line segments can query correctly

                            }
                            childId.uniqueId = t + (k - 1) * num_lat_points + 1;
                            currGraphNode.succ->push_back(childId);
                        }
                    }
                }


                lattice[currId] = currGraphNode;

                }


            if (flag){

                latScale = abs(latScale) + latStep;
                flag = false;
            }

            else{

                latScale *= -1;
                flag = true;
            }

        }
    }
}




void openDriveReader::SplitLanes()
{

    int vect_count = 0;

    for (int i = 0; i < (int)RoadListOrg.size(); i++)
    {
        roadElementWrapper el = RoadListOrg[i];
        if (el.junction != -1)
        {
            if (el.id % 2 == 1)
            {
                //el.laneSplit = JunctionWidth*el.scale/2;
            }
            else {
                //el.laneSplit = -LaneWidth + JunctionWidth*el.scale;
            }
            el.uniqueId = el.id;
            roads[el.uniqueId] = el;
            RoadList.push_back(el);
        }

        else
        {
            roadElementWrapper left = el, right = el;
            left.laneSplit = LaneWidth;
            right.laneSplit = -LaneWidth;
            left.predId = el.succId;
            left.succId = el.predId;
            left.predET = el.succET;
            left.succET = el.predET;
            left.predCP = el.succCP;
            left.succCP = el.predCP;


            left.reverse = true; // one will always have to be reversed * Hard code this into the xml?

            right.vectId = vect_count;
            right.uniqueId = el.id;
            roads[right.uniqueId] = right;

            vect_count++;

            left.vectId = vect_count;
            left.uniqueId = el.id + (int)RoadListOrg.size() + 100;
            roads[left.uniqueId] = left;

            vect_count++;

            RoadList.push_back(left);
            RoadList.push_back(right);
        }
    }
}

void openDriveReader::GetNodes()
{

    std::map<int, roadElementWrapper>::iterator it;

}



Quaternion openDriveReader::toQuaternion(double pitch, double roll, double yaw)
{
    Quaternion q;
    double t0 = std::cos(yaw * 0.5);
    double t1 = std::sin(yaw * 0.5);
    double t2 = std::cos(roll * 0.5);
    double t3 = std::sin(roll * 0.5);
    double t4 = std::cos(pitch * 0.5);
    double t5 = std::sin(pitch * 0.5);

    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x = t0 * t3 * t4 - t1 * t2 * t5;
    q.y = t0 * t2 * t5 + t1 * t3 * t4;
    q.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q;
}


Euler openDriveReader::toEulerianAngle(Quaternion q)
{
    Euler a;
    double ysqr = q.y * q.y;

    // roll (x-axis rotation)
    double t0 = +2.0 * (q.w * q.x + q.y * q.z);
    double t1 = +1.0 - 2.0 * (q.x * q.x + ysqr);
    a.roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0 * (q.w * q.y - q.z * q.x);
    t2 = ((t2 > 1.0) ? 1.0 : t2);
    t2 = ((t2 < -1.0) ? -1.0 : t2);
    a.pitch = std::asin(t2);

    // yaw (z-axis rotation)
    double t3 = +2.0 * (q.w * q.z + q.x * q.y);
    double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
    a.yaw = std::atan2(t3, t4);
    return a;
}


float openDriveReader::CubicPoly(float a1, float b1, float c1, float d1, float ds)
{

    return (a1 + b1*ds + c1*pow(ds, 2.0) + d1*pow(ds, 3.0));
}

float openDriveReader::RotateCCWX(float u2, float v2, float hdg2)
{
    return (u2*cos(hdg2) - v2*sin(hdg2));
}

float openDriveReader::RotateCCWY(float u1, float v1, float hdg1)
{
    return (u1*sin(hdg1) + v1*cos(hdg1));
}

float openDriveReader::EuclideanDistance(Pose3D pose1, Pose3D pose2)
{
    float x0 = pose1.p.x;
    float y0 = pose1.p.y;
    float z0 = pose1.p.z;
    float x1 = pose2.p.x;
    float y1 = pose2.p.y;
    float z1 = pose2.p.z;
    float d = sqrt((x0 - x1)*(x0 - x1) + (y0 - y1)*(y0 - y1) + (z0 - z1)*(z0 - z1));
    return d;
}

std::vector<Pose3D> openDriveReader::UpdatePoseHeading(std::vector<Pose3D> path)
{
    std::vector<Pose3D> list;
    Pose3D pose;
    int pathLength = path.size();
    float h = 0.0;
    for (int i = 0; i < pathLength; i++)
    {
        if (i + 1 != pathLength)
        {
            float x0 = path[i].p.x, y0 = path[i].p.y;
            float x1 = path[i + 1].p.x, y1 = path[i + 1].p.y;
            h = atan2(y1 - y0, x1 - x0);
        }
        pose = path[i];
        pose.q = toQuaternion(0, 0, h);
        list.push_back(pose);
    }
    return list;
}


std::vector<Pose3D> openDriveReader::GetRoadPoints(roadElementWrapper el, int num)
{
    std::vector<Pose3D> list;
    Pose3D pose;
    //Finding heading change for lane shifting, Does not affect anything if there is no lane shift
    float curHeading = 0.0, headingChange = 0.0, lastHeading = 0.0;
    for (int i = 0; i < num; i++)
    {
        float ds = (float)(i + 0.1) / (num);
        roadGeometryWrapper geo = el.geometry[0];
        //Find points from parametric polynomial in local coordinate
        float tempx = CubicPoly(geo.aU, geo.bU, geo.cU, geo.dU, ds);
        float tempy = CubicPoly(geo.aV, geo.bV, geo.cV, geo.dV, ds);
        //Split to lane point, use heading change to rotate lane shift
        float tempLanex = RotateCCWX(0, el.laneSplit / el.scale, headingChange);
        float tempLaney = RotateCCWY(0, el.laneSplit / el.scale, headingChange);
        //Rotate to global coordinate
        float newx = RotateCCWX(tempx + tempLanex, tempy + tempLaney, geo.hdg);
        float newy = RotateCCWY(tempx + tempLanex, tempy + tempLaney, geo.hdg);
        //Shift to global coordinate
        pose.p.x = (geo.x + newx) * el.scale;
        pose.p.y = (geo.y + newy) * el.scale;
        pose.p.z = el.altitude;
        pose.q = toQuaternion(0, 0, geo.hdg);
        //Store line
        list.push_back(pose);
        //Get the heading change from last point to current point
        if (i != 0)
        {
            float x0 = list[i - 1].p.x, y0 = list[i - 1].p.y;
            float x1 = pose.p.x, y1 = pose.p.y;
            lastHeading = curHeading;
            curHeading = atan2(y1 - y0, x1 - x0);
            headingChange += normalizeAngle(curHeading - lastHeading, 0);
            headingChange = normalizeAngle(headingChange, 0);
        }
        else
        {
            curHeading = geo.hdg;
        }
    }
    UpdatePoseHeading(list);
    return list;
}



std::vector<Pose3D> openDriveReader::GetRoadPoints(std::vector<roadElementWrapper> el, int num)
{
    std::vector<Pose3D> vect;
    for (int j = 0; j < (int)el.size(); j++)
    {
        std::vector<Pose3D> points3d = GetRoadPoints(el[j], num);
        for (int i = 0; i < (int)points3d.size(); i++) {
            vect.push_back(points3d[i]);
        }
    }
    return vect;
}        std::map<nodeId, graphElement> graph;


std::vector<MapElement> openDriveReader::GetMapPoints(std::map<int, roadElementWrapper> roads, int num)
{

    std::vector<MapElement> vect;
    int numPoints;
    for (auto const& it : roads) { // Loop Through All Roads

        std::vector<Pose3D> list;
        Pose3D pose;
        roadElementWrapper el = it.second;
        roadGeometryWrapper geo = el.geometry[0];

        float curHeading = 0;
        numPoints = 10;
        std::vector<float> tempxs;
        std::vector<float> tempys;

        // Generate points along line segment. Points for straight driving are down the middle of the lane, points for curves are long middle of curve
        for (int z = 0; z < numPoints; z++) {
            float ds = (float)(z + 0.1) / (numPoints);

            //Find points from parametric polynomial in local coordinate
            tempxs.push_back(CubicPoly(geo.aU, geo.bU, geo.cU, geo.dU, ds));
            tempys.push_back(CubicPoly(geo.aV, geo.bV, geo.cV, geo.dV, ds));

        }

        for (int i = 0; i < numPoints; i++) { // Determine Number of points to Place //TODO place points based on desnity vs set number

            graphElement pathEl;
            nodeId currNode;

            currNode.roadId = it.first;

            if (!el.reverse){
                currNode.uniqueId = i;
            }
            else{
                currNode.uniqueId = numPoints - 1 - i;
            }

            if (i == 0){
                curHeading = 0;

            }
            else if (i == numPoints - 1) {

                float x0 = tempxs[i - 1], y0 = tempys[i - 1];
                float x1 = tempxs[i], y1 = tempys[i];
                curHeading = atan2(y1 - y0, x1 - x0);

            }

            else
            {
                float x0 = tempxs[i - 1], y0 = tempys[i - 1];
                float x1 = tempxs[i + 1], y1 = tempys[i + 1];

                curHeading = atan2(y1 - y0, x1 - x0);
            }


            roadGeometryWrapper geo = el.geometry[0];

            pose.p.x = (tempxs[i]) * el.scale;
            pose.p.y = (tempys[i]) * el.scale;
            pose.p.z = el.altitude;
            pose.q = toQuaternion(0, 0, geo.hdg);

            float tempLanex = RotateCCWX(0, el.laneSplit / el.scale, curHeading);
            float tempLaney = RotateCCWY(0, el.laneSplit / el.scale, curHeading);

            pathEl.p.x =  pose.p.x + tempLanex * el.scale ;
            pathEl.p.y =  pose.p.y  + tempLaney * el.scale ;

            // GLOBAL TRANSFORMATION
            float newx = RotateCCWX(pathEl.p.x,pathEl.p.y, geo.hdg);
            float newy = RotateCCWY(pathEl.p.x, pathEl.p.y, geo.hdg);
            pathEl.p.x = newx + (geo.x* el.scale);
            pathEl.p.y = newy + (geo.y* el.scale);


            if (i == 0)
            {
                nodeId succNode;
                succNode.roadId = it.first;
                succNode.uniqueId = i + 1;
                pathEl.succId.push_back(succNode);
                pathEl.succType.push_back(STRAIGHT);
            }
            else if (i + 1 == num) // If last
            {
                for (int k = 0; k < el.succIds.size(); k++){

                    nodeId succNode;
                    succNode.roadId = el.succIds[k];
                    succNode.uniqueId = 0;
                    pathEl.succId.push_back(succNode);
                    pathEl.succType.push_back(el.succType[k]);

                }
            }

            else
            {
                // Middle of path

                nodeId succNode;
                succNode.roadId = it.first;
                succNode.uniqueId = i + 1;
                pathEl.succId.push_back(succNode);
                pathEl.succType.push_back(STRAIGHT);

                //Query George's Function here and add SuccId's
            }
//            vect.push_back(mapEl);

            graph[currNode] = pathEl;

//            LOG_INFO(cString::Format("graph: (%f, %f)", graph[currNode].p.x, graph[currNode].p.x));

        }

    }
    return vect;
}



/*! calculates normalized angle */
tFloat32 openDriveReader::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
    return mod(alpha - center + M_PI, 2.0*M_PI) + center - M_PI;
}

/*! calculates modulus after division */
tFloat32 openDriveReader::mod(tFloat32 x, tFloat32 y)
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
            b_x = ceil(r - 0.5);
        }
        else
        {
            b_x = floor(r + 0.5);
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

//bool nodeId::operator() (const nodeId &left, const nodeId &right) const {
//        return (left.roadId < right.roadId) || (left.roadId == right.roadId && left.uniqueId < right.uniqueId);
//}
