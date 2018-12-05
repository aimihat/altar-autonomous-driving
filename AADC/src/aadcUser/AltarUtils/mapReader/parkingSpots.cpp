#include "parkingSpots.h"
#include <string>
#include <iostream>

std::map<int, MapPoint> getSpots(std::string fileName) {
    std::map<int, MapPoint> map;

    tinyxml2::XMLDocument Doc2;
    const char* FileName = fileName.c_str();

    tinyxml2::XMLError FileReadErr2 = Doc2.LoadFile(FileName);
    if (FileReadErr2 == 0) {
       tinyxml2::XMLNode *pRoot = Doc2.FirstChildElement("configuration");
       if (pRoot == NULL)
       {
           return map;
       }
       tinyxml2::XMLElement *roadSign = pRoot->FirstChildElement("parkingSpace");

       while (roadSign != NULL)
       {
          float x,y;
          int id;
          roadSign->QueryIntAttribute("id", &id);
          roadSign->QueryFloatAttribute("x", &x);
          roadSign->QueryFloatAttribute("y", &y);
          MapPoint p;
          p.x = x;
          p.y = y;
          map[id] = p;
          roadSign = roadSign->NextSiblingElement("parkingSpace");
       }

    }
    return map;
}
