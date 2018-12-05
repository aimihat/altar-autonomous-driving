#include "openDriveReader.h"
#include <iostream>

using namespace std;

int main() {
  cout<<"Hello World!\n";
  ODReader::openDriveReader *m_odReader;
  m_odReader = new ODReader::openDriveReader("test.xodr");

  cout<<"Error: "<<m_odReader->FileReadErr;
  cout<<"Number of roads: "<<m_odReader->RoadList.size();

  //Draw individual roads one by one
  for (unsigned int i = 0; i < m_odReader->RoadList.size(); i++)
  {
      // cout<<m_odReader->predID<<endl;;

      //Get the points from each road
      std::vector<ODReader::Pose3D> path = m_odReader->GetRoadPoints(m_odReader->RoadList[i]);
      cout<<m_odReader->RoadList[i].id<<endl;
      //Draw line for each point
      for (unsigned int j = 0; j < path.size() - 1; j++)
      {//
        //Convert map coordinates in -x1 to x2 to Pixel 0 to GRAPHICS_WIDTH
        //Convert map coordinates in y2 to -y1 to Pixel 0 to GRAPHICS_HEIGHT
        //Note Y direction is flipped as map and pixel is opposite in y-direction
        //Pixel coordinates

          cout<<"("<<path[j].p.x<<", "<<path[j].p.y<<")"<<endl;
    }
  }

    cout<<"For 25:"<<endl;
    for (unsigned int i = 0; i < m_odReader->RoadList[25].nodes.size(); i++)
    {
        cout<<m_odReader->RoadList[25].nodes[i].id << endl;
    }

    int val = 25;
    cout<<"For "<<val<<":"<<endl;
    cout<<m_odReader->RoadListOrg[val].id << ": "<<m_odReader->RoadListOrg[val].predId << " " << m_odReader->RoadListOrg[val].succId<<" "<<endl;
    cout<<"---\n";




//    for (unsigned int i = 0; i < m_odReader->RoadList[val].nodes.size(); i++)
////    {
////        cout<<m_odReader->RoadList[val].nodes[i].id << endl;
////    }

    for (unsigned int i = 0; i < m_odReader->RoadListOrg.size(); i++)
    {
        if (
                m_odReader->RoadListOrg[i].id == 1 ||
                m_odReader->RoadListOrg[i].id == 166 ||
                        m_odReader->RoadListOrg[i].id == 73||

                        m_odReader->RoadListOrg[i].id == 161||
                        m_odReader->RoadListOrg[i].id == 25||
                        m_odReader->RoadListOrg[i].id == 54||

                        m_odReader->RoadListOrg[i].id == 21||
                m_odReader->RoadListOrg[i].id == 120 || 0
                )
        cout<<m_odReader->RoadListOrg[i].id << ": "<<m_odReader->RoadListOrg[i].predId << " " << m_odReader->RoadListOrg[i].succId<<" "<<m_odReader->RoadListOrg[i].nextRoad<<" "<<endl;
    }


    // ------------------------------------------------------------------------------------
//    for (std::vector<ODReader::MapElement>::const_iterator i = m_odReader->MapElList.begin(); i != m_odReader->MapElList.end(); ++i) {
//        cout << (*i).roadId<<": ";
//        for (std::vector<int>::const_iterator j = (*i).nodes.begin(); j != (*i).nodes.end(); ++j) {
//            cout<<(*i).nodes.front() << " ";
//        }
//        cout<<endl;
//    }





  cout<<"Success";
  return 0;
}
