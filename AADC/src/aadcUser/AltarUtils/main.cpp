//
//  main.cpp
//  maptest
//
//  Created by george kousouris on 05/08/2018.
//  Copyright © 2018 george kousouris. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include <string>
#include "audiParser.h"

using namespace std;

int main() {
    // File has to be in absolute path
    string file2 = "/Users/georgekousouris/Desktop/maptest/maptest/test.xodr";
    string file = "/Users/georgekousouris/Desktop/audi/AADC/src/aadcUser/AltarGraph/aadc2018#kickoff.xodr";
    initialize(file);

    int key = -21;
    segments segs = getNextSegments(key);
    cout<<key<<": "<<segs.left.uniqueId<<" "<<segs.center.uniqueId<<" "<<segs.right.uniqueId<<endl;

    // key = 144;
    // segs = getNextSegments(key);
    // cout<<key<<": "<<segs.left.uniqueId<<" "<<segs.center.uniqueId<<" "<<segs.right.uniqueId<<endl;

    return 0;
}
