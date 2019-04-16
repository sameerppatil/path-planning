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
                generateTrajectories(sensor_fusion, s, path_size, laneSpeed);
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
        cout << "Detected car in my lane, slowing down to " << current_speed << endl;
    }
    else if (current_speed < 49.5 && laneSpeed == 0)
    {
        current_speed += .225;
    }
    updatedResults.intendedLane = current_lane;
    updatedResults.intendedSpeed = current_speed;
}

void Core::updateSpeedCost(double laneSpeed, float curr_d)
{
    double currentSpeedCost, laneSpeed_mph;
    laneSpeed_mph = laneSpeed * 2.24;
    currentSpeedCost = (99.0 * (49.5 - laneSpeed_mph) / 49.5);

    if ((curr_d < 4 && curr_d > 0) && (speedCost.lane0 < currentSpeedCost))
    {
        speedCost.lane0 = currentSpeedCost;
    }
    if ((curr_d < 8 && curr_d > 4) && (speedCost.lane1 < currentSpeedCost))
    {
        speedCost.lane1 = currentSpeedCost;
    }
    if ((curr_d < 12 && curr_d > 8) && (speedCost.lane2 < currentSpeedCost))
    {
        speedCost.lane2 = currentSpeedCost;
    }
    cout << "\t\tLane 0\t" << "Lane 1\t" << "Lane 2" << endl;
    cout << "Speed cost:\t" << speedCost.lane0 << "\t" << speedCost.lane1 << "\t" << speedCost.lane2 << endl;
}

void Core::updateCollisionCost(float curr_d)
{
    double overallCollisionCost = 999.0;
    for (int i = 0; i < 12; i=i+4)
    {
        if ((curr_d < (i+4)) && (curr_d > i))
        {
            if (!(curr_d < (2+4*current_lane+2) && curr_d > (2+4*current_lane-2)))
            {
                int temp = i/4;
                collCost[temp] = overallCollisionCost;
                if (debug)
                {
                    cout << "currd: " << curr_d << " i:" << i << " temp:" << temp << endl;
                }
            }
        }
        /* code */
    }
    cout << "Total cost:\t" << collCost[0] << "\t" << collCost[1] << "\t" << collCost[2] << endl;

}
void Core::initCosts(void)
{
    speedCost.lane0 = 0;
    speedCost.lane1 = 0;
    speedCost.lane2 = 0;
    collCost[0] = 0;
    collCost[1] = 0;
    collCost[2] = 0;
}

void Core::generateTrajectories(vector <vector <double>> sensor_fusion, double curr_s,
    int path_size, double laneSpeed)
{
    int tempLane = current_lane;
    vector<double> costs = {0.0, 0.0, 0.0};

    // init all costs to zero
    initCosts();
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        //car is in my lane
        float d = sensor_fusion[i][6];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx+vy*vy);
        double check_car_s = sensor_fusion[i][5];

        check_car_s += ((double)path_size*.02*check_speed);

        if ((check_car_s > curr_s) && ((check_car_s - curr_s) < 50))
        {
            updateSpeedCost(check_speed, d);
        }

        if (((check_car_s > curr_s) && ((check_car_s - curr_s) < 15))
            || ((check_car_s < curr_s) && ((check_car_s - curr_s) > - 9)))
        {
            updateCollisionCost(d);
        }

    }

    costs[0] = costs[0] + speedCost.lane0 + collCost[0];
    costs[1] = costs[1] + speedCost.lane1 + collCost[1];
    costs[2] = costs[2] + speedCost.lane2 + collCost[2];


    costs[0] += 0.5;
    costs[2] += 0.5;

    if (debug)
    {
        cout << "Total cost:\t" << costs[0] << "\t" << costs[1] << "\t" << costs[2] << endl;
    }

    //choose lane with lowest cost
    vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);

    if (keepCurrLane > 0)
    {
        keepCurrLane--;
    }
    if (abs(current_lane - best_idx) == 2)
    {
        if (collCost[1] == 0)
        {
            // when trying to switch between lane 0 and 2, try to do a smooth transition
            // like 0 -> 1 -> 2 rather than rapid one from 0 -> 2
            // Keep same lane for 22 cycles atleast
            current_lane = 1;
            cout << "\t\t\tGoing to middle lane for now" << endl;
            keepCurrLane = 20;
        }
    }
    else
    {
        if (current_lane != best_idx && keepCurrLane == 0)
        {
            current_lane = best_idx;
        }
    }
    if (tempLane != current_lane)
    {
        cout << "Changing lane to " << current_lane << " from " << tempLane << "." << endl;
    }

}