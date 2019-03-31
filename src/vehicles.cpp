#include "vehicles.h"

Vehicles::Vehicles(int lane, double velocity, int cycles_since_lane_change)
{
    my_speed = velocity;
    my_lane = lane;
    my_cyles_since_lane_change = cycles_since_lane_change;
}

void Vehicles::Update_localization(double car_x, double car_y, double car_s, double car_d,
        double car_yaw, double car_speed, double check_speed_mph, int path_size, bool is_car_too_close, string state)
{
    this->my_x = car_x;
    this->my_y = car_y;
    this->my_s = car_s;
    this->my_d = car_d;
    this->my_yaw = car_yaw;
    this->my_speed = car_speed;
    this->state = state;
}

double Vehicles::Calculate_speed(double vx, double vy)
{
    return sqrt(vx*vx + vy*vy);
}

double Vehicles::Predict_s(double neighbour_speed, int size, double neighbour_s)
{
    return neighbour_s += ((double)size * 0.2 * neighbour_speed);
}

void Vehicles::Generate_costs(vector<vector <double>> sensor_fusion)
{

}

void Vehicles::NextLane(vector<vector <double>> sensor_fusion)
{
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        //car is in my lane
        float d = sensor_fusion[i][6];
        if (d < (2+4*my_lane+2) && d > (2+4*my_lane-2))
        {
            double neighbour_speed = Calculate_speed(sensor_fusion[i][3], sensor_fusion[i][4]);
            double neighbour_s = Predict_s(neighbour_speed, this->path_size, sensor_fusion[i][5]);

            if ((neighbour_s > my_s) && ((neighbour_s - my_s) < 30))
            {
                is_car_too_close = true;
                check_speed_mph = neighbour_speed * 2.24; // set ref speed for slowing down & convert from m/s to mph

                // ********* BEHAVIOR PLANNER ******

                // 1. My FSM states will simply be target lanes, i.e. 0, 1, 2
                cout << "current lane = " << my_lane << endl;
                cout << "cycles elapsed since last lane change = " << my_cyles_since_lane_change << endl;

                // 2. Calculate cost for each FSM state (each lane)
                vector<double> costs = {0.0, 0.0, 0.0};
                double speed_penalty, speed_penalty_lane0 = 0.0, speed_penalty_lane1 = 0.0, speed_penalty_lane2 = 0.0; // implemented
                // if all else is equal, the cost component below will send the car into the middle lane
                double lane_change_opportunities;
                double collision_penalty, collision_penalty_lane0 = 0.0, collision_penalty_lane1 = 0.0, collision_penalty_lane2 = 0.0; // implemented

                for (int i = 0; i < sensor_fusion.size(); i++){
                    d = sensor_fusion[i][6];
                    vx = sensor_fusion[i][3];
                    vy = sensor_fusion[i][4];
                    check_speed = sqrt(vx*vx+vy*vy);
                    check_car_s = sensor_fusion[i][5];

                    check_car_s += ((double)path_size*.02*check_speed);

                    // speed penalty - check cars within 50 meters ahead
                    if ((check_car_s > my_s) && ((check_car_s - my_s) < 50)){
                        speed_penalty = 99.0*((49.5 - check_speed_mph) / 49.5);
                        // lane 0
                        if (d < 4 && d > 0){
                            //don't add speed penalty of multiple cars together, choose the highest one:
                            if (speed_penalty_lane0 < speed_penalty) {
                                speed_penalty_lane0 = speed_penalty;
                            }
                        }
                        // lane 1
                        if (d < 8 && d > 4){
                            if (speed_penalty_lane1 < speed_penalty) {
                                speed_penalty_lane1 = speed_penalty;
                            }
                        }
                        // lane 2
                        if (d < 12 && d > 8){
                            if (speed_penalty_lane2 < speed_penalty) {
                                speed_penalty_lane2 = speed_penalty;
                            }
                        }
                    }

                    // collision detection & penalty
                    // check for cars within 15 meters in front and 7 behind the car
                    if ( ((check_car_s > my_s) && ((check_car_s - my_s) < 15))
                            || ((check_car_s < my_s) && ((check_car_s - my_s) > - 7)) ){
                        // set collision penalty
                        collision_penalty = 999.0;
                        if (d < 4 && d > 0){
                            // do not set collision penalty for the lane our car is in
                            if (!(d < (2+4*my_lane+2) && d > (2+4*my_lane-2))){
                                if (collision_penalty_lane0 < collision_penalty){
                                    collision_penalty_lane0 = collision_penalty;
                                }
                            }
                        }
                        if (d < 8 && d > 4){
                            if (!(d < (2+4*my_lane+2) && d > (2+4*my_lane-2))){
                                if (collision_penalty_lane1 < collision_penalty){
                                     collision_penalty_lane1 = collision_penalty;
                                }
                            }
                        }
                        if (d < 12 && d > 8){
                            if (!(d < (2+4*my_lane+2) && d > (2+4*my_lane-2))){
                                if (collision_penalty_lane2 < collision_penalty){
                                     collision_penalty_lane2 = collision_penalty;
                                }
                            }
                        }
                    }
                }

                    //add speed penalties to the costs vector
                    costs[0] += speed_penalty_lane0;
                    cout << "speed penalty lane 0 = " << speed_penalty_lane0 << endl;
                    costs[1] += speed_penalty_lane1;
                    cout << "speed penalty lane 1 = " << speed_penalty_lane1 << endl;
                    costs[2] += speed_penalty_lane2;
                    cout << "speed penalty lane 2 = " << speed_penalty_lane2 << endl;

                    //add collision penalties to the costs vector
                    costs[0] += collision_penalty_lane0;
                    cout << "collision penalty lane 0 = " << collision_penalty_lane0 << endl;
                    costs[1] += collision_penalty_lane1;
                    cout << "collision penalty lane 1 = " << collision_penalty_lane1 << endl;
                    costs[2] += collision_penalty_lane2;
                    cout << "collision penalty lane 2 = " << collision_penalty_lane2 << endl;

                    // lane changing opportunities
                    // all else equal, it's better to be in the middle lane
                    // than in the left or right lane
                    lane_change_opportunities = 1.0;
                    costs[0] += lane_change_opportunities;
                    costs[2] += lane_change_opportunities;
                    cout << "lane change opportunities cost lane 0 = " << lane_change_opportunities << endl;
                    cout << "lane change opportunities cost lane 2 = " << lane_change_opportunities << endl;

                    //output total costs
                    for (int i = 0; i < costs.size(); i++) {
                        cout << "total cost for lane " << i << " = " << costs[i] << endl;
                    }

                    //choose lane with lowest cost
                    vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
                    int best_idx = distance(begin(costs), best_cost);

                    // lane selection logic (= FSM state selection logic)
                    // change lane only if there have been at least 25 cycles since last lane change
                    if (my_cyles_since_lane_change > 25){
                        if ((my_lane == 0 && best_idx == 2) || (my_lane == 2 && best_idx == 0)) {
                            if (collision_penalty_lane1 == 0){
                                my_lane = 1;
                                cout << "going to middle my_lane as a preparatory step" << endl;
                                my_cyles_since_lane_change = 0;
                            }
                        }
                        else {
                            if (my_lane != best_idx){
                                my_cyles_since_lane_change = 0;
                                my_lane = best_idx;
                            }
                        }
                    }

                    cout << "selected my_lane #:" << my_lane << endl;
                    cout << endl;

                    // ********* EO BEH. PLANNER ******

                }
            }
        }

}
