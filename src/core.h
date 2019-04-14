#ifndef VEHICLE_H
#define VEHICLE_H

#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>

using namespace std;


class Core
{
public:

    Core(int lane, double speed, int debugOn);
    void updateData(int lane, double speed);
    void predictState(vector<vector<double>> sensor_fusion, double s, int path_size);
    // virtual ~ Core();
    int current_lane = 1;
    double current_speed = 0;
    bool debug = true;
    double my_s = 0;

    struct updatedResults
    {
        double intendedSpeed = 0;
        int intendedLane = 0;
        bool tooClose = false;
    } updatedResults;
};

#endif