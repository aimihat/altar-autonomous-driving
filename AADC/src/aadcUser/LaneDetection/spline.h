#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

class lin_spline
{
public:
    lin_spline() {}
    lin_spline(int x[], int y[], int size){
        this->x_coord = x;
        this->y_coord = y;
        this->size = size;
    }

    int * x_coord;
    int * y_coord;
    int size;

    vector<vector<double>> spline_points;

    void create_spline()
    {
        double gradient;
        double y_int;

        for(int i = 0; i < size-1; i++)
        {
            spline_points.push_back({x_coord[i], y_coord[i]});
            if ((x_coord[i] != x_coord[i + 1]))
            {
                gradient = (y_coord[i + 1] - y_coord[i]) / double(x_coord[i + 1] - x_coord[i]);
                y_int = y_coord[i] - x_coord[i] * gradient;


                if (x_coord[i] < x_coord[i + 1]) {
                    for (int j = x_coord[i]; j < x_coord[i + 1]; j++) {
                        spline_points.push_back({j, j * gradient + y_int});
                    }
                } else {
                    for (int j = x_coord[i]; j > x_coord[i + 1]; j--) {
                        spline_points.push_back({j, j * gradient + y_int});
                    }
                }
            }
            else
            {
                if (y_coord[i] < y_coord[i + 1]) {
                    for (int j = y_coord[i]; j < y_coord[i + 1]; j++) {
                        spline_points.push_back({x_coord[i], j});
                    }
                } else {
                    for (int j = y_coord[i]; j > y_coord[i + 1]; j--) {
                        spline_points.push_back({x_coord[i], j});
                    }
                }
            }

        }
    }





};
