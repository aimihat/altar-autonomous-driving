#include "openDriveReader.h"
#include "../audiParser.h"
#include <iostream>
#include <string>
#include <algorithm>    // std::sort
#include <set>
#include <vector>
#include <map>

using namespace std;

map <int, segments> nextMap;
set <int> junctionIds;
map <int, int> preds;


map<int, roadElementWrapper> roadElements;
map<int, ODReader::roadElement> roadElementsNew;

void printLengths(const ODReader::openDriveReader *m_odReader);
void printPrevsNexts(const ODReader::openDriveReader *m_odReader);
void printForFile(const ODReader::openDriveReader *m_odReader);
void populateMap(ODReader::openDriveReader *m_odReader);
void printAllSegments(ODReader::openDriveReader *m_odReader);
void printSet();
void printLengths();
void printMap();

#define AUDI_RIGHT_17 6
#define AUDI_CENTER_17 10
#define AUDI_LEFT_17 25

#define AUDI_KICKOFF_RIGHT 10
#define AUDI_KICKOFF_LEFT 15.8
#define AUDI_KICKOFF_CENTER 25

string getDirection(float length) {
    if (length < AUDI_KICKOFF_RIGHT)
        return "right";
    if (length < AUDI_KICKOFF_LEFT)
        return "left";
    return "center";
}



segments getNextSegments(int id) {
    return nextMap[id];
}

std::map<int, roadElementWrapper> getMap(std::string file) {
  initialize(file);
  // vector<int> succIds;
  // vector<turn> succType;

  // succIds.push_back(134);
  // succType.push_back(STRAIGHT);

  // roadElementWrapper roadElementWrapper;
  // roadElementWrapper.succIds = succIds;
  // roadElementWrapper.succType = succType;
  // roadElements[135] = roadElementWrapper;
  // roadElements[134] = roadElementWrapper;
  // roadElements[151] = roadElementWrapper;
  // for (int i = -33; i <= 33; i++) {
  //   roadElements
  // }
  printMap();
//    return  roadElementsNew;
  return roadElements;
}

std::map<int, ODReader::roadElement> getMapNew(std::string file) {
//    initialize(file);
    // vector<int> succIds;
    // vector<turn> succType;

    // succIds.push_back(134);
    // succType.push_back(STRAIGHT);

    // roadElementWrapper roadElementWrapper;
    // roadElementWrapper.succIds = succIds;
    // roadElementWrapper.succType = succType;
    // roadElements[135] = roadElementWrapper;
    // roadElements[134] = roadElementWrapper;
    // roadElements[151] = roadElementWrapper;
    // for (int i = -33; i <= 33; i++) {
    //   roadElements
    // }
//    printMap();
    return  roadElementsNew;
}


bool myfunction (ODReader::roadElement i,ODReader::roadElement j) { return (i.id<j.id); }
//
void initialize(string file) {
    ODReader::openDriveReader *m_odReader;
    m_odReader = new ODReader::openDriveReader(file);

    printLengths(m_odReader);



    cout<<"Error: "<<m_odReader->FileReadErr<<endl;
    cout<<"Number of roads: "<<m_odReader->RoadList.size()<<endl;

    for (unsigned int i = 0; i < m_odReader->RoadList.size(); i++) {
        ODReader::roadElement CurrRoadElement;

        ODReader::roadGeometry foo;
        foo.x = m_odReader->RoadList[i].geometry[0].x;
        foo.y = m_odReader->RoadList[i].geometry[0].y;
        foo.hdg = m_odReader->RoadList[i].geometry[0].hdg;
        foo.s = m_odReader->RoadList[i].geometry[0].s;
        foo.length = m_odReader->RoadList[i].geometry[0].length;
        foo.aU = m_odReader->RoadList[i].geometry[0].aU;
        foo.aV = m_odReader->RoadList[i].geometry[0].aV;
        foo.bU = m_odReader->RoadList[i].geometry[0].bU;
        foo.bV = m_odReader->RoadList[i].geometry[0].bV;
        foo.cU = m_odReader->RoadList[i].geometry[0].cU;
        foo.cV = m_odReader->RoadList[i].geometry[0].cV;
        foo.dU = m_odReader->RoadList[i].geometry[0].dU;
        foo.dV = m_odReader->RoadList[i].geometry[0].dV;

        CurrRoadElement.geometry.push_back(foo);
        CurrRoadElement.junction = m_odReader->RoadList[i].junction;
        CurrRoadElement.cost = m_odReader->RoadList[i].cost;
        CurrRoadElement.altitude = m_odReader->RoadList[i].altitude;
        CurrRoadElement.scale = m_odReader->RoadList[i].scale;
        CurrRoadElement.laneSplit = m_odReader->RoadList[i].laneSplit;

        if (CurrRoadElement.laneSplit > 0){
            CurrRoadElement.reverse = true;
        }

        else {

            CurrRoadElement.reverse = false;

        }
//        CurrRoadElement.uniqueId = m_odReader->RoadList[i].id;
//        vector<int> succIds;
//        vector<turn> succType;
//        CurrRoadElement.succIds = succIds;
//        CurrRoadElement.succType = succType;
//        CurrRoadElement.reverse = m_odReader->RoadList[i].reversed;
//
//        CurrRoadElement.uniqueId = m_odReader->RoadList[i].id;

//        roadElementsNew[m_odReader->RoadList[i].id] = m_odReader->RoadList[i];
        roadElementsNew[m_odReader->RoadList[i].id] = CurrRoadElement;

    }

    sort (m_odReader->RoadList.begin(), m_odReader->RoadList.end(), myfunction);

    populateMap(m_odReader);
    printMap();
    cout<<"----\n";
    for (unsigned int i = 0; i < m_odReader->RoadList.size(); i++) {
      roadElementWrapper CurrRoadElement;

      roadGeometryWrapper foo;
        foo.x = m_odReader->RoadList[i].geometry[0].x;
        foo.y = m_odReader->RoadList[i].geometry[0].y;
        foo.hdg = m_odReader->RoadList[i].geometry[0].hdg;
        foo.s = m_odReader->RoadList[i].geometry[0].s;
        foo.length = m_odReader->RoadList[i].geometry[0].length;
        foo.aU = m_odReader->RoadList[i].geometry[0].aU;
        foo.aV = m_odReader->RoadList[i].geometry[0].aV;
        foo.bU = m_odReader->RoadList[i].geometry[0].bU;
        foo.bV = m_odReader->RoadList[i].geometry[0].bV;
        foo.cU = m_odReader->RoadList[i].geometry[0].cU;
        foo.cV = m_odReader->RoadList[i].geometry[0].cV;
        foo.dU = m_odReader->RoadList[i].geometry[0].dU;
        foo.dV = m_odReader->RoadList[i].geometry[0].dV;

        CurrRoadElement.geometry.push_back(foo);

        CurrRoadElement.junction = m_odReader->RoadList[i].junction;
        CurrRoadElement.cost = m_odReader->RoadList[i].cost;
        CurrRoadElement.altitude = m_odReader->RoadList[i].altitude;
        CurrRoadElement.scale = m_odReader->RoadList[i].scale;
        CurrRoadElement.laneSplit = m_odReader->RoadList[i].laneSplit;

        // For Junctions
        if (m_odReader->RoadList[i].isLeft && m_odReader->RoadList[i].junction != -1) {
            CurrRoadElement.reverseId = true;
        }

        // For regular Roads
        else if (CurrRoadElement.laneSplit > 0 && m_odReader->RoadList[i].junction == -1){
            CurrRoadElement.reverseId = true;
        }

        else {

            CurrRoadElement.reverseId = false;

        }

        CurrRoadElement.reverse = m_odReader->RoadList[i].reversed;
//        CurrRoadElement.reverseId = m_odReader->RoadList[i].reversed;

        CurrRoadElement.uniqueId = m_odReader->RoadList[i].id;

      vector<int> succIds;
      vector<turn> succType;
      int left = nextMap[m_odReader->RoadList[i].id].left.uniqueId;
      int center = nextMap[m_odReader->RoadList[i].id].center.uniqueId;
      int right = nextMap[m_odReader->RoadList[i].id].right.uniqueId ;
      if (left) {
        succIds.push_back(left);
        succType.push_back(LEFT);
      }
      if (center) {
        succIds.push_back(center);
        succType.push_back(STRAIGHT);
      }
      if (right) {
        succIds.push_back(right);
        succType.push_back(RIGHT);
      }

//      if (!(m_odReader->RoadList[i].succET[0] == 'j')){
//          CurrRoadElement.succIds = succIds;
//          CurrRoadElement.succType = succType;
//      }
        CurrRoadElement.succIds = succIds;
        CurrRoadElement.succType = succType;
      roadElements[m_odReader->RoadList[i].id] = CurrRoadElement;

      // cout<<m_odReader->RoadList[i].id<<" ";
    }
    cout<<endl;

}

void putElementInMap(int curr, int prev, int next, string dir) {
    // in func:
    segments segments1 = nextMap[prev];


    int val;
    // Assume all right turns  ordered correctly
    if (dir == "left") {
        segments1.left.uniqueId = curr;
        segments1.left.isStart = curr < 0;
    } else if (dir == "center") {
        segments1.center.uniqueId = curr;
        segments1.center.isStart = curr < 0;
    } else {
        segments1.right.uniqueId = curr;
        segments1.right.isStart = curr < 0;
    }
    nextMap[prev] = segments1;

    roadElementWrapper roadElementWrapper1;
    roadElementWrapper1.succs = segments1;
    roadElements[prev] = roadElementWrapper1;

    segments segment2 = nextMap[curr];

    segment2.center.uniqueId = next;
    segment2.center.isStart = segment2.center.uniqueId < 0;

    nextMap[curr] = segment2;

    roadElementWrapper roadElementWrapper2;
    roadElementWrapper2.succs = segment2;
    roadElements[curr] = roadElementWrapper2;

}


// Populate the map
void populateMap(ODReader::openDriveReader *m_odReader) {
    // populate non-junctions
    for (unsigned int i = 0; i < m_odReader->RoadList.size(); i++) {
        if (m_odReader->RoadList[i].junction > -1) continue;

        if (m_odReader->RoadList[i].succET[0] == 'j') {
            junctionIds.insert(m_odReader->RoadList[i].id);
            preds[m_odReader->RoadList[i].id] = m_odReader->RoadList[i].succId;
            continue; // Check for junction successor
        }

        int curr = m_odReader->RoadList[i].id;
        int  next = m_odReader->RoadList[i].succId;

        segmentId segmentId;
        segmentId.uniqueId = next;
        segmentId.isStart = next < 0;

        segments segment;
        segment.center = segmentId;

//        if (m_odReader->RoadList[i].reversed) {
//            int temp = segment.center.uniqueId;
//            segment.center.uniqueId = curr;
//            curr = temp;
//        }

        nextMap[curr] = segment;

        roadElementWrapper roadElementWrapper;
        roadElementWrapper.succs = segment;
        roadElements[curr] = roadElementWrapper;
    }

    // HARD CODE IN SEGMENT CORRECTIONS
    int next = 26;
    int curr = -2;
    segmentId segmentId;
    segments segment;

    segmentId.uniqueId = next;
    segmentId.isStart = next < 0;

    segment.center = segmentId;
    nextMap[curr] = segment;

    next = 2;
    curr = -26;

    segmentId.uniqueId = next;
    segmentId.isStart = next < 0;

    segment.center = segmentId;
    nextMap[curr] = segment;

    // populate junctions
    for (unsigned int i = 0; i < m_odReader->RoadList.size(); i++) {
        if (m_odReader->RoadList[i].junction == -1) continue;

        int prev = m_odReader->RoadList[i].predId;
        int curr = m_odReader->RoadList[i].id;
        int next = m_odReader->RoadList[i].succId;

        // Handle wrong output at turns
        string dir = getDirection(m_odReader->RoadList[i].length);


        if (m_odReader->RoadList[i].isLeft) {
            int temp = prev;
            prev = next;
            next = temp;
        }

//        if (m_odReader->RoadList[i].reversed) {
//            int temp = prev;
//            prev = next;
//            next = temp;
//        }

        int k = 0;
        if (junctionIds.count(next) && preds[next] == m_odReader->RoadList[i].junction) {
            next *= -1;
         }
        if (!(junctionIds.count(prev) && preds[prev] == m_odReader->RoadList[i].junction)) {
            prev *= -1;
        }


        putElementInMap(curr, prev, next, dir);
    }

}

//Draw individual roads one by one
void printAllSegments(ODReader::openDriveReader *m_odReader) {

    for (unsigned int i = 0; i < m_odReader->RoadList.size(); i++)
    {

        //Get the points from each road
        std::vector<ODReader::Pose3D> path = m_odReader->GetRoadPoints(m_odReader->RoadList[i]);

        if (/* DISABLES CODE */ (false) /*m_odReader->RoadList[i].id == YOUR_ID */) {
            cout<<m_odReader->RoadList[i].id<<" "<<m_odReader->RoadList[i].isLeft<<endl;
        } else {
            continue;
        }

        //Draw line for each point
        cout<<"("<<path[0].p.x<<", "<<path[0].p.y<<")";
        for (unsigned int j = 1; j < path.size()-1; j++) {
            cout<<",("<<path[j].p.x<<", "<<path[j].p.y<<")";
        }
        cout<<endl;
    }
}


void printForFile(const ODReader::openDriveReader *m_odReader) {
    for (unsigned int i = 0; i < m_odReader->RoadList.size(); i++) {
        if (m_odReader->RoadList[i].junction > -1) continue;
        cout<<m_odReader->RoadList[i].id << " "<<m_odReader->RoadList[i].predId << " " << m_odReader->RoadList[i].succId<<" "<<m_odReader->RoadList[i].succCP<<endl;
    }
    cout<<"junctions:\n";
    for (unsigned int i = 0; i < m_odReader->RoadList.size(); i++) {
        if (m_odReader->RoadList[i].junction == -1) continue;

        int prev = m_odReader->RoadList[i].predId;
        int curr = m_odReader->RoadList[i].id;
        int next = m_odReader->RoadList[i].succId;
        int junct = m_odReader->RoadList[i].junction;

        // Handle wrong output at turns
        string dir = getDirection(m_odReader->RoadList[i].length);

        if (curr < 0) {
            if (!junctionIds.count(next)) {
                next *= -1;
            }
            if (junctionIds.count(prev)) {
                prev *= -1;
            }
            cout<< junct << ": " << dir<<" "<< curr<<" "<<next << " " << prev<<endl;
        } else {
            if (junctionIds.count(next)) {
                next *= -1;
            }
            if (!junctionIds.count(prev)) {
                prev *= -1;
            }
            cout<< junct << ": " << dir<<" "<< curr<<" "<<prev << " " << next<<endl;
        }
    }
}

void printMap() {
    for(map<int, segments >::const_iterator it = nextMap.begin();
                     it != nextMap.end(); ++it) {
        std::cout << it->first << ": " << it->second.left.uniqueId << " " << it->second.center.uniqueId <<" " << it->second.right.uniqueId<< "\n\n";
    }
}

void printLengths(const ODReader::openDriveReader *m_odReader) {
    for (unsigned int i = 0; i < m_odReader->RoadList.size(); i++)
    {

        if (m_odReader->RoadList[i].junction > -1) {
          cout<<m_odReader->RoadList[i].length<<" "<<m_odReader->RoadList[i].id<<" ";
            cout<<"("<<m_odReader->RoadList[i].length<<",5)"<<endl;
        }
    }
}

void printSet() {
    set<int>::iterator iter;
    for(iter=junctionIds.begin(); iter!=junctionIds.end();++iter) {
        cout<<(*iter)<<endl;
    }
}

void printPrevsNexts(const ODReader::openDriveReader *m_odReader) {
    for (unsigned int i = 0; i < m_odReader->RoadList.size(); i++)
    {
        cout<<m_odReader->RoadList[i].id << ": "<<m_odReader->RoadList[i].predId << " " << m_odReader->RoadList[i].succId<<" "<< (m_odReader->RoadList[i].junction > -1 ? getDirection(m_odReader->RoadList[i].length) : "road")<<endl;
        if (m_odReader->RoadList[i].succET[0] == 'j') {
            cout<<"Junction\n";
        }
    }
}
