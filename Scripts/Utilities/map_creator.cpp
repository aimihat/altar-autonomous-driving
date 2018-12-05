#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;


struct point {
    float x;
    float y;
};

struct segment {
    int id;
    vector<point> points;
    vector<point> left;
    vector<point> middle;
    vector<point> orig;
    vector<point> right;
};

vector<segment> segments;


struct point getPoint(point q, point w, float h) {
    float x1 = q.x;
    float x2 = w.x;

    float y1 = q.y;
    float y2 = w.y;

    if (x1 == x2) {
        point f;
        f.y = y1;
        f.x = x1 - h;
        return f;
    }

    if (y1 == y2) {
      point f;
      f.x = x1;
      f.y = y1 - h;
      return f;
    }

    // Find inverse slope
    float m = -1 / ((y2 - y1) / (x2 - x1));
    float b = y1 - m * x1;

    point p;
    p.x = x1 - h / sqrt(1 + pow(m, 2));
    p.y = m * p.x + b;

    return p;
}

void printTerrain();
void addMiddleCircle();

vector<point> invert(vector<point> points) {
    vector<point> newPoints;
    int size = points.size();
    for (int i = 0; i < points.size(); i++) {
        newPoints.push_back(points[size - 1 - i]);
    }
    return newPoints;
}


float distance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void saveSegments(std::vector<segment> &segments) {
    ofstream myfile("map_segments.txt");
    myfile << segments.size();

    for (int i = 0; i < segments.size(); i++) {

        myfile << segments[i].id << " ";
        myfile << segments[i].points.size() << " ";
        for (int j = 0; j < segments[i].points.size(); j++) {
            myfile << segments[i].points[j].x << " " << segments[i].points[j].y << " ";
        }
        myfile << segments[i].left.size() << " ";
        for (int j = 0; j < segments[i].left.size(); j++) {
            myfile << segments[i].left[j].x << " " << segments[i].left[j].y << " ";
        }
        myfile << segments[i].middle.size() << " ";
        for (int j = 0; j < segments[i].middle.size(); j++) {
            myfile << segments[i].middle[j].x << " " << segments[i].middle[j].y << " ";
        }
        myfile << segments[i].right.size() << " ";
        for (int j = 0; j < segments[i].right.size(); j++) {
            myfile << segments[i].right[j].x << " " << segments[i].right[j].y << " ";
        }
    }
}

void loadSegments(std::vector<segment> &loadedSegments) {
    //Use to load segments into any plugin from the text file generated
    ifstream myfile("map_segments.txt");
    int numSegments;
    myfile >> numSegments;
    for (int i = 0; i < numSegments; i++) {
        segment s;
        int sizeLeft, sizeRight, sizeMiddle, sizePoints;
        myfile >> s.id;
        myfile >> sizePoints;
        for (int j = 0; j < sizePoints; j++) {
            point p;
            myfile >> p.x;
            myfile >> p.y;
            s.points.push_back(p);
        }

        myfile >> sizeLeft;
        for (int j = 0; j < sizeLeft; j++) {
            point p;
            myfile >> p.x;
            myfile >> p.y;
            s.left.push_back(p);
        }
        myfile >> sizeMiddle;
        for (int j = 0; j < sizeMiddle; j++) {
            point p;
            myfile >> p.x;
            myfile >> p.y;
            s.middle.push_back(p);
        }
        myfile >> sizeRight;
        for (int j = 0; j < sizeRight; j++) {
            point p;
            myfile >> p.x;
            myfile >> p.y;
            s.right.push_back(p);
        }
        loadedSegments.push_back(s);
    }
}

void plotMap(std::vector<segment> &segments) {
    float METERS_TO_PIXELS = 120.0;
    float OFFSET_DRAW = 200;
    cv::Mat mImageGrid(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));

    for (int i = 0; i < segments.size(); i++) {
       if (i <= 11 || i == 13) {
         cout<<"Q"<<segments[i].left.size()<<endl;
        for (int p = 0; p < segments[i].left.size() - 1; p++) {
            cv::Point line_start_l((int) (segments[i].left[p].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                   (int) (segments[i].left[p].y * METERS_TO_PIXELS + OFFSET_DRAW));
            cv::Point line_end_l((int) (segments[i].left[p + 1].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                 (int) (segments[i].left[p + 1].y * METERS_TO_PIXELS + OFFSET_DRAW));
           if (i == 13) {
             cout<<segments[i].left[p].x<<" "<<segments[i].left[p].y<<endl;
           }
            cv::line(mImageGrid, line_start_l, line_end_l, cv::Scalar(255, 255, 0));
          }
          for (int p = 0; p < segments[i].right.size() - 1; p++) {

            cv::Point line_start_r((int) (segments[i].right[p].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                   (int) (segments[i].right[p].y * METERS_TO_PIXELS + OFFSET_DRAW));
            cv::Point line_end_r((int) (segments[i].right[p + 1].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                 (int) (segments[i].right[p + 1].y * METERS_TO_PIXELS + OFFSET_DRAW));
            cv::line(mImageGrid, line_start_r, line_end_r, cv::Scalar(255, 255, 255));
          }
          for (int p = 0; p < segments[i].middle.size() - 1; p++) {
            cv::Point line_start_m((int) (segments[i].middle[p].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                   (int) (segments[i].middle[p].y * METERS_TO_PIXELS + OFFSET_DRAW));
            cv::Point line_end_m((int) (segments[i].middle[p + 1].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                 (int) (segments[i].middle[p + 1].y * METERS_TO_PIXELS + OFFSET_DRAW));
            cv::line(mImageGrid, line_start_m, line_end_m, cv::Scalar(0, 255, 255));
          }
        }
          for (int p = 0; p < segments[i].points.size() - 1; p++) {

            cv::Point line_start_or((int) (segments[i].points[p].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                   (int) (segments[i].points[p].y * METERS_TO_PIXELS + OFFSET_DRAW));
            cv::Point line_end_or((int) (segments[i].points[p + 1].x * METERS_TO_PIXELS + OFFSET_DRAW),
                                 (int) (segments[i].points[p + 1].y * METERS_TO_PIXELS + OFFSET_DRAW));
            cv::line(mImageGrid, line_start_or, line_end_or, cv::Scalar(0, 135, 155));
        }
        cv::circle(
                mImageGrid,
                cv::Point((segments[i].middle[0].x * METERS_TO_PIXELS + OFFSET_DRAW),
                          (segments[i].middle[0].y * METERS_TO_PIXELS + OFFSET_DRAW)),
                5,
                cv::Scalar(255, 255, 255),
                -1);

        std::ostringstream ss;
        ss << segments[i].id;
        putText(mImageGrid, ss.str(), cv::Point(
                ((segments[i].middle[0].x + segments[i].middle[segments[i].middle.size() - 1].x) / 2.0 *
                 METERS_TO_PIXELS + OFFSET_DRAW),
                ((segments[i].middle[0].y + segments[i].middle[segments[i].middle.size() - 1].y) / 2.0 *
                 METERS_TO_PIXELS + OFFSET_DRAW)), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 255));

        cv::circle(
                mImageGrid,
                cv::Point((segments[i].middle[segments[i].middle.size() - 1].x * METERS_TO_PIXELS + OFFSET_DRAW),
                          (segments[i].middle[segments[i].middle.size() - 1].y * METERS_TO_PIXELS + OFFSET_DRAW)), 5,
                cv::Scalar(255, 255, 255),
                -1);
    }
    cv::imshow("window", mImageGrid);
    cv::waitKey(0);
}


int main() {
    // printTerrain() does some hacky stuff to generate the points and create a "vector<segment> segments".
    // This vector holds the id of the segment with its points.
    // The original terrain had only 3 segments.

    int LOAD = 0; //1 to load, 0 to generate
    int SAVE = 1;
    int PLOT = 1;

    if (!LOAD) {
        printTerrain();
        if (SAVE)
            saveSegments(segments);
    } else {
        cout << "LOADING SEGMENTS" << endl;
        loadSegments(segments);
        cout << "LOADED!" << endl;
    }
    if (PLOT)
        plotMap(segments);

    return 0;
}


void printTerrain() {
    ifstream fin("./audiroom.in");
    int id;
    fin >> id;

    for (int k = 0; k < 3; k++) {
        segment s;
        vector<point> points;

        while (true) {
            point p;
            fin >> p.x;
            //cout<<p.x<<endl;
            if (p.x < 0) {
                s.id = id;
                s.points = points;
                segments.push_back(s);
                id = p.x;
                break;
            }
            fin >> p.y;
            points.push_back(p);
        }
    }

    int currSegments = segments.size();
    int k = -4;

    for (int i = 0; i < currSegments; i++) {
        segment s;
        vector<point> newPoints;
        for (int j = 0; j < segments[i].points.size(); j++) {
            vector<point> points = segments[i].points;
            point p;
            float diff = 1.99 - points[j].x;

            p.x = 1.99 + diff;
            p.y = points[j].y;

            newPoints.push_back(p);
        }
        s.id = k--;

        newPoints = invert(newPoints);

        s.points = newPoints;
        segments.push_back(s);
    }
    //

    currSegments = segments.size();


    for (int i = 0; i < currSegments; i++) {
        segment s;
        vector<point> newPoints;
        for (int j = 0; j < segments[i].points.size(); j++) {
            vector<point> points = segments[i].points;
            point p;
            float diff = points[j].y - 10.077;
            p.y = 10.077 - diff;
            p.x = points[j].x;

            newPoints.push_back(p);
        }
        s.id = k--;
        if (i > 3) {
            s.points = newPoints;
        } else {
            s.points = newPoints;
        }
        segments.push_back(s);
    }

    currSegments = segments.size();
    // ----- Transpose -----
    float bottomPointY = 6.918;   // the bottom coordinate of the current system
    float yTranspose = bottomPointY - 0.29; //Increase the length of the straight line to 2m by adding 0.29
    for (int i = 0; i < currSegments; i++) {
        for (int j = 0; j < segments[i].points.size(); j++) {
            if (segments[i].points[j].y < 10) {
                segments[i].points[j].y -= 0.5;
            }
        }
    }

    // transpose towards bottom
    for (int i = 0; i < currSegments; i++) {
        for (int j = 0; j < segments[i].points.size(); j++) {
            segments[i].points[j].y -= yTranspose;
        }
    }

    //Add middle segments.
    addMiddleCircle();

    int order[15] = {2, 1, 3, 9, 7, 8, 11, 10, 12, 6, 4, 5, 13, 14, 15};

    // This for loop generates the 9 rest segments by inverting, flipping and transposing coordinates relative to axes.
    cout<<segments.size();
    for (int ii = 0; ii < segments.size(); ii++) {
        int i = order[ii] - 1;
        int offset = 1;
        if (segments[i].id < -3) {
            offset *= -1;
            if (segments[i].id >= -9 && segments[i].id < -6) {
                offset *= -1;
            }
        }
        if (segments[i].id == -14) {
            offset *= -1;
        }

        vector<point> points = segments[i].points;
        if (segments[i].id == -7 ||
            segments[i].id == -8 ||
            segments[i].id == -10 ||
            segments[i].id == -11) {
            points = invert(points);
        }

        // IMPORTANT THINGS START HERE:
        // points[j] are the points of the trajectory.
        // To get the left - mid - right lanes, you call getPoint() with the correct parameters.
        // `int offset` is a hack to get the correct direction of lanes.
        cout<<ii<<endl;
        for (int j = 1; j < points.size(); j++) {
            point a1 = getPoint(points[j - 1], points[j], 0.25 * offset);
            point a2 = getPoint(points[j - 1], points[j], -0.25 * offset);
            point a3 = getPoint(points[j - 1], points[j], -0.75 * offset);
            segments[i].left.push_back(a1);
            segments[i].middle.push_back(a2);
            segments[i].right.push_back(a3);
            // segments[i].orig.push_back(points[j]);

        }
    }
}

void addMiddleCircle() {
    ifstream ffin("./points.in");

    for (int i = 0; i < 3; i++) {
        segment s;
        vector <point> newPoints;
        s.id = -13 - i;
        int limit = (i == 1 ? 11 : 9);

        for (int j = 0; j < limit; j++) {
            point p;
            float x, y;
            ffin >> x >> y;
            p.x = x;
            if (i == 1) cout<<y<<" ";
            p.y = y;
            newPoints.push_back(p);
        }
        s.points = newPoints;
        segments.push_back(s);
    }
}
