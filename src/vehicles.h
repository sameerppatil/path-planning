#ifndef VEHICLES_H
#define VEHICLES_H
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>

using namespace std;

class Vehicles {
    public:
        int my_lane = 1;
 //       double my_speed = 0;
        int my_cyles_since_lane_change = 0;
        string state;
        double my_x = 0;
        double my_y = 0;
        double my_s = 0;
        double my_d = 0;
        double my_yaw = 0;
        double my_speed = 0;
        double check_speed_mph = 0;
        int path_size = 0;
        bool is_car_too_close = false;

        void Update_localization(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double check_speed_mph, int path_size, bool is_car_too_close, string state);

        Vehicles(int lane, double velocity, int cycles_since_lane_change);
        void NextLane(vector<vector<double>> sensor_fusion);

};



#endif
