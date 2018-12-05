//
//  audiParser.h
//  maptest
//
//  Created by george kousouris on 05/08/2018.
//  Copyright © 2018 george kousouris. All rights reserved.
//

#ifndef audiParser_h
#define audiParser_h


#include <string>
#include <vector>
#include <map>
#include <string>
#include <adtf3.h>
#include "mapReader/openDriveReader.h"

struct roadGeometryWrapper {
    float x, y, hdg, s, length;
    float aU, aV, bU, bV, cU, cV, dU, dV;
};

struct Quaternion {
    float x;
    float y;
    float z;
    float w;
};


struct nodeIdWrapper {
    int roadId;
    int uniqueId;
    bool const operator <(const nodeIdWrapper& o) const {
        return roadId < o.roadId || (roadId == o.roadId && uniqueId < o.uniqueId);
    }
};

enum turn {
    LEFT,
    STRAIGHT,
    RIGHT
};

struct segmentId {
    int uniqueId=0;
    bool isStart;
};

struct segments {
    segmentId left;
    segmentId center;
    segmentId right;
};

struct mapPoint {
    tFloat32 x;
    tFloat32 y;
};

struct Point {
    float x;
    float y;
    float z;
};
struct Pose3D {
    Point p;
    Quaternion q;
};

struct graphNode {
    float g;
    float f;
    float rhs;
    int key;
    double distanceToMid = 0;
    std::vector<nodeIdWrapper>* succ = new std::vector<nodeIdWrapper>;
    std::vector<nodeIdWrapper>* pred = new std::vector<nodeIdWrapper>;
    std::vector<nodeIdWrapper>* neighbours = new std::vector<nodeIdWrapper>;
    mapPoint p;
    double cost = 0;
    double edgeCost = 0;
    bool inFront = true;
    double intensity = 0;
};


struct graphElement {
    std::vector<nodeIdWrapper> succId;
    std::vector<turn> succType;
    mapPoint p;
};


struct roadElementWrapper {
    int junction;
    std::vector<roadGeometryWrapper>  geometry;

    float cost;
    float altitude;
    float scale;
    float laneSplit;
    bool reverse = false;
    bool reverseId = false;

    int uniqueId = 0;

    // TODO INitalize vectors
    std::vector<nodeIdWrapper> startNodes;
    std::vector<nodeIdWrapper> endNodes;


    // For GraphSearch
    std::vector<int> succIds;
    std::vector<turn> succType;

    segments succs;
};


std::map<int, roadElementWrapper> getMap(std::string file);
std::map<int, ODReader::roadElement> getMapNew(std::string file);


// Ensure this is run
void initialize(std::string file);
segments getNextSegments(int id);

#endif /* audiParser_h */
