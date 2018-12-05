#include <stdio.h>
#include <iostream>
#include <map>
#include <assert.h>

#include "audiParser.h"

using namespace std;

void shouldPopulateStraightRoads( map<int, roadElementWrapper> roads) {
    bool pass = true;
    // cout<<roads[135].succs.center.uniqueId<<endl;
    // cout<<roads[-21].succs.left.uniqueId<<endl<<roads[-21].succs.right.uniqueId<<endl;
    // assert(roads[-21].succs.right.uniqueId == 15);
    // assert(roads[-21].succs.left.uniqueId == 17);

    assert(roads[-21].succIds[0] == 17);
    assert(roads[-21].succType[0] == LEFT);

    assert(roads[-21].succIds[1] == 15);
    assert(roads[-21].succType[1] == RIGHT);


    // assert(roads[134].succIds[0] == 151);
    // assert(roads[134].succType[0] == STRAIGHT);
    //
    // assert(roads[151].succIds[0] == 133);
    // assert(roads[151].succType[0] == STRAIGHT);

    cout<<"All tests passed\n";
}


int main() {
  string path2 = "/Users/georgekousouris/Desktop/audi/AADC/src/aadcUser/AltarUtils/mapReader/test.xodr";
  string path = "/Users/georgekousouris/Desktop/audi/AADC/src/aadcUser/AltarGraph/aadc2018#kickoff.xodr";
  map<int, roadElementWrapper> roads = getMap(path);
  shouldPopulateStraightRoads(roads);
}
