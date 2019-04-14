#include "core.h"
// #include "helpers.h"
#include <vector>


Core::Core(int lane, double speed, int debugOn)
{
    current_lane = lane;
    current_speed = speed;
    if (debugOn)
    {
        cout << "Debug is On" << endl;
        debug = true;
    }
}

void Core::updateData(int lane, double speed)
{
    current_lane = lane;
    current_speed = speed;
}

void Core::predictState(vector<vector<double>> sensor_fusion, double s, int path_size)
{
    double laneSpeed = 0;
    bool too_close = false;

    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        //car is in my lane
        float d = sensor_fusion[i][6];
        int bound_a = 2 + 4*current_lane + 2;
        int bound_b = 2 + 4*current_lane - 2;
        // if (debug)
        // {
        //     cout << "d: " << d << " current_lane: " << current_lane << " a: " << bound_a << " b: " << bound_b << endl;
        // }
        if (d < (bound_a) && d > (bound_b))
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)path_size*.02*check_speed); // using previous points to project s value outwards in time

            //check s values greater than mine and s gap is smaller than 30 meters (arbitrary value)
            if ((check_car_s > s) && ((check_car_s - s) < 30))
            {
                // do some logic here, e.g. lower ref velocity so we don't crash into the car in front of us,
                // could also set the flag to try to change lane
                // ref_vel = 29.5; // mph
                cout << "Detected car in my lane, slowing down to " << current_speed << endl;
                too_close = true;
                laneSpeed = check_speed * 2.24;
                // generate trajectories for alternate lanes
                generateTrajectories(sensor_fusion, s, path_size);
            }
        }
        else
        {
            // keep lane, do nothing
        }
    }

    if (too_close && current_speed > laneSpeed)
    {
        // decrease acc. by 5ms2
        current_speed -= .225;
    }
    else if (current_speed < 49.5 && laneSpeed == 0)
    {
        current_speed += .225;
    }
    updatedResults.intendedLane = current_lane;
    updatedResults.intendedSpeed = current_speed;
}

void Core::generateTrajectories(vector <vector <double>> sensor_fusion, double curr_s, int path_size)
{
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        //car is in my lane
        float d = sensor_fusion[i][6];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx+vy*vy);
        double check_car_s = sensor_fusion[i][5];

        check_car_s += ((double)path_size*.02*check_speed); // using previous points to project s value outwards in time

        //check s values greater than mine and s gap is smaller than 30 meters (arbitrary value)
        if ((check_car_s > curr_s) && ((check_car_s - curr_s) < 50))
        {
            int test = 1;
        }

    }

}