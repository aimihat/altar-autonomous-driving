//
// Created by robin on 03/09/18.
//

#ifndef AADC_USER_MAPSTRUCTS_H
#define AADC_USER_MAPSTRUCTS_H


#include <string>
#include <vector>
#include <map>
#include <string>
#include <adtf3.h>

//DATA TYPES
struct MapPoint {
    float x;
    float y;
};

struct segment {
    int id;
    std::vector<MapPoint> Points;
    std::vector<MapPoint> left;
    std::vector<MapPoint> middle;
    std::vector<MapPoint> right;
};


/*! Storage structure for the road sign data */
typedef struct _roadSign
{
    /*! road sign */
    tInt16 u16Id;

    /*! init sign */
    tBool bInit;

    /*! location  x*/
    tFloat32 f32X;
    /*! location  y*/
    tFloat32 f32Y;

    /*! sign search radius */
    tFloat32 f32Radius;

    /*! direction (heading) of the road sign */
    tFloat32 f32Direction;

    /*! Number of 16s */
    tInt u16Cnt;

    /*! measurement ticks*/
    tTimeStamp u32ticks;

} roadSign;


struct RoadLine {

    double a;
    double b;
    bool skip;
    double theta;
    double mid_x;
    double mid_y;

};

struct LineSegment {

    double x1;
    double x2;
    double y1;
    double y2;
    uint id;

};

struct RoadSignPose {

    double x;
    double y;
    double rotation;

};

#pragma pack(push,1)

typedef struct
{
    tFloat32 f32Distance;
    tFloat32 f32FrontDistance;
    tFloat32 f32RoadId;
} tLineCoordiante;

#pragma pack(pop)

#pragma pack(push,1)

enum DebugType{
    POINT,
    ROADSIGN,
    POSITION,
    CAR,
    CAR_DEBUG,
    FRONT_LINE,
    LASER,
    DETECTED_SIGN,
    HIT_POINT,
    CAM_SIGN,
    LIDAR_SIGN
};


enum Colour{
    RED,
    GREEN,
    BLUE,
    WHITE,
    PURPLE
};
#pragma pack(pop)

typedef struct
{
    float x = 0;
    float y = 0;
    float theta = 0;
    int type = POINT;
    int Id = 0;
    int colour = WHITE;

} DebugPoint;

//struct segment {
//    int id;
//    std::vector<MapPoint> Points;
//    std::vector<MapPoint> left;
//    std::vector<MapPoint> middle;
//    std::vector<MapPoint> right;
//};


enum RoadMarking {
    NOT_SURE,
    OUTER_A,
    MIDDLE,
    OUTER_B

};

struct GridObjects {
    // x - y cordinates are relative to the COM of car
    tInt16 top_pivot_x;
    tInt16 top_pivot_y;
    tInt16 bot_pivot_x;
    tInt16 bot_pivot_y;
};

struct Pose {

    tFloat32 x = 0;
    tFloat32 y = 0;
    tFloat32 theta = 0;
};

#pragma pack(push,1)
typedef struct
{
    tInt32 x;
    tInt32 y;
} lanePoint;
#pragma pack(pop)

typedef struct{

    MapPoint center;
    double theta = 0;
    double a = 0;
    double b = 0;
    RoadMarking mark = NOT_SURE;
    uint id = 111;
    bool skip;

}Particle;


typedef struct{

    double x = 0;
    double y = 0;
    double a = 0;
    double b = 0;
    bool skip = false;

}vizParticle;



struct Boundary {

    MapPoint center;
    double half_distance;


    Boundary(){

        center.x = 0;
        center.y = 0;
        half_distance = 10;

    }

    bool ContainsPoint(MapPoint p){

        if((p.x > center.x - half_distance)&&(p.x < center.x + half_distance)){
            if((p.y > center.y - half_distance)&&(p.y < center.y + half_distance)){

                return true;
            }
        }


        return false;
    }

    bool Intersects(Boundary range){

        MapPoint l1;
        MapPoint l2;
        MapPoint r1;
        MapPoint r2;


        l1.x = center.x - half_distance;
        l1.y = center.y - half_distance;
        r1.x = center.x + half_distance;
        r1.y = center.y + half_distance;

        l2.x = range.center.x - range.half_distance;
        l2.y = range.center.y - range.half_distance;
        r2.x = range.center.x + range.half_distance;
        r2.y = range.center.y + range.half_distance;

        if (l1.x > r2.x || l2.x > r1.x)
            return false;

        // If one rectangle is above other
        if (l1.y > r2.y || l2.y > r1.y)
            return  false;

        return true;
    }
};
enum situations {
    EXIT = 0,
    DRIVE_STRAIGHT,
    LEFT_TURN,
    RIGHT_TURN,
    TURN,
    START,
    OBSTACLE
};


#endif //AADC_USER_MAPSTRUCTS_H
