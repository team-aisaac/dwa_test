#include<dwa_test/dwa_planner.hpp>

DWAPlanner::DWAPlanner(){
    planTrajectory();
}

DWAPlanner::~DWAPlanner(){}

// x, y, r
// x1, y1, x2, y2
void DWAPlanner::setObstacles(){
    Obstacle obs1;
    obs1.shape = "circle";
    obs1.circle.x = 5;
    obs1.circle.y = 0;
    obs1.circle.r = 0.3;
    obstacles.push_back(obs1);

    Obstacle obs2;
    obs2.shape = "circle";
    obs2.circle.x = 5;
    obs2.circle.y = 3;
    obs2.circle.r = 0.3;
    obstacles.push_back(obs2);

    Obstacle obs3;
    obs3.shape = "circle";
    obs3.circle.x = 5;
    obs3.circle.y = -3;
    obs3.circle.r = 0.3;
    obstacles.push_back(obs3);

    Obstacle obs4;
    obs4.shape = "circle";
    obs4.circle.x = 5;
    obs4.circle.y = -1;
    obs4.circle.r = 0.3;
    obstacles.push_back(obs4);

    Obstacle obs5;
    obs5.shape = "circle";
    obs5.circle.x = 7;
    obs5.circle.y = 1;
    obs5.circle.r = 0.3;
    obstacles.push_back(obs5);

}

DynamicWindow DWAPlanner::calclateDynamicWindow(){
    //Dynamic window from robot specification
    DynamicWindow vs;
    vs.vx_max = MAX_VEL;
    vs.vx_min = MIN_VEL;
    vs.vy_max = MAX_VEL;
    vs.vy_min = MIN_VEL;

    //Dynamic window from motion model
    DynamicWindow vd;
    vd.vx_max = start_velocity.vx + MAX_ACC * DT;
    vd.vx_min = start_velocity.vx - MAX_ACC * DT;
    vd.vy_max = start_velocity.vy + MAX_ACC * DT;
    vd.vy_min = start_velocity.vy - MAX_ACC * DT;

    DynamicWindow dw;
    dw.vx_max = min(vs.vx_max, vd.vx_max);
    dw.vx_min = max(vs.vx_min, vd.vx_min);
    dw.vy_max = min(vs.vy_max, vd.vy_max);
    dw.vy_min = max(vs.vy_min, vd.vy_min);
    return dw;
}

Trajectory DWAPlanner::calclateTrajectory(DynamicWindow dw, Velocity &control_value){
    float best_cost = numeric_limits<float>::infinity();
    Velocity best_control_value;
    Trajectory best_trajectory;

    for(float vx=dw.vx_min; vx<=dw.vx_max; vx+=dw.RESOLUTION){
        for(float vy=dw.vy_min; vy<=dw.vy_max; vy+=dw.RESOLUTION){
            Trajectory trajectory = predictTrajectory(Velocity(vx, vy));

            float to_goal_cost = TO_GOAL_COST_GAIN * calclateToGoalCost(trajectory);
            if(hypot(vx, vy) > MAX_VEL) continue;
            if(hypot(vx, vy) < 0.01) continue;
            float speed_cost = SPEED_COST_GAIN * (MAX_VEL - hypot(vx, vy));
            float obstacle_cost = OBSTACLE_COST_GAIN * calclateObstacleCost(trajectory);
            float cost = to_goal_cost + speed_cost + obstacle_cost;

            if (cost <= best_cost){
                best_cost = cost;
                best_trajectory = trajectory;
                control_value = Velocity(vx, vy);
                cout << calclateToGoalCost(trajectory) << endl;
                cout << to_goal_cost << " " << speed_cost << " " << obstacle_cost << endl;
                cout << vx << " " << vy << endl;
            }
        }
    }

    return best_trajectory;
}

Trajectory DWAPlanner::predictTrajectory(Velocity control_value){
    Trajectory trajectory;

    Point waypoint = start_point;

    float t = 0;
    while(t <= PREDICT_TIME){
        waypoint.x += control_value.vx * DT;
        waypoint.y += control_value.vy * DT;
        trajectory.waypoints.push_back(waypoint);

        t += DT;
    }
    return trajectory;
}

float DWAPlanner::calclateToGoalCost(Trajectory trajectory){
    int wp_num = trajectory.waypoints.size();
    float dx = goal_point.x - trajectory.waypoints[wp_num-1].x;
    float dy = goal_point.y - trajectory.waypoints[wp_num-1].y;
    //float error_angle = atan2(dy, dx);

    //dx = trajectory.waypoints[wp_num-1].x - trajectory.waypoints[wp_num-2].x;
    //dy = trajectory.waypoints[wp_num-1].y - trajectory.waypoints[wp_num-2].y;
    //float cost_angle = error_angle - atan2(dy, dx);

    //float cost = abs(atan2(sin(cost_angle), cos(cost_angle)));
    float cost = hypot(dx, dy);
    return cost;
}

float DWAPlanner::calclateObstacleCost(Trajectory trajectory){
    float dist_min = numeric_limits<float>::infinity();
    for(int wp_id=0; wp_id<trajectory.waypoints.size(); wp_id++){
        Point wp = trajectory.waypoints[wp_id];
        float dist = calclateMinDistanceFromObstacles(wp);
        dist_min = min(dist, dist_min);
    }

    if(dist_min <= 0)
        return numeric_limits<float>::infinity();
    else
        return 1.0 / dist_min;
}

float DWAPlanner::calclateMinDistanceFromObstacles(Point pt){
    float dist_min = numeric_limits<float>::infinity();
    for(int ob_id=0; ob_id<obstacles.size(); ob_id++){
        if(obstacles[ob_id].shape == "circle"){
            float dx = obstacles[ob_id].circle.x - pt.x;
            float dy = obstacles[ob_id].circle.y - pt.y;
            float dist = hypot(dx, dy) - obstacles[ob_id].circle.r - ROBOT_RADIUS;
            dist_min = min(dist, dist_min);
        }
        if(obstacles[ob_id].shape == "rectangle"){
            float dist = calclateDistanceFromRectangle(pt, obstacles[ob_id].rectangle) - ROBOT_RADIUS;
            dist_min = min(dist, dist_min);
        }
    }

    return dist_min;
}

float DWAPlanner::calclateDistanceFromRectangle(Point pt, ObstacleRectangle rect){
    if(pt.x < rect.x_min){
        if(pt.y < rect.y_min) return hypot(rect.x_min - pt.x, rect.y_min - pt.y);
        if(pt.y <= rect.y_max) return rect.x_min - pt.x;
        return hypot(rect.x_min - pt.x, rect.y_max - pt.y);
    }else if(pt.x <= rect.x_max){
        if(pt.y < rect.y_min) return rect.y_min - pt.y;
        if(pt.y <= rect.y_max) return 0;
        return pt.y - rect.y_max;
    }else{
        if(pt.y < rect.y_min) return hypot(rect.x_max - pt.x, rect.y_min - pt.y);
        if(pt.y <= rect.y_max) return pt.x - rect.x_max;
        return hypot(rect.x_max - pt.x, rect.y_max - pt.y);
    }
}

Trajectory DWAPlanner::planTrajectory(){
    setObstacles();
    start_point.x = 0;
    start_point.y = 0;
    start_velocity = Velocity(0, 0);
    goal_point.x = 10;
    goal_point.y = 0;
    Velocity control_value = Velocity(0, 0);

    while(1){
        start_point.x += control_value.vx * DT;
        start_point.y += control_value.vy * DT;
        start_velocity = control_value;
        auto dw = calclateDynamicWindow();
        auto trajectory = calclateTrajectory(dw, control_value);

        cout << control_value.vx << " " << control_value.vy << endl;
        visualizeTrajectory(trajectory, control_value);
    }
}

Trajectory DWAPlanner::visualizeTrajectory(Trajectory trajectory,
                                           Velocity control_value){

    for(int ob_id=0; ob_id<obstacles.size(); ob_id++){
        if(obstacles[ob_id].shape == "circle"){
            vector<float> obs_x;
            vector<float> obs_y;
            float obs_r;

            obs_r = obstacles[ob_id].circle.r;

            for(float theta=0; theta<=2*M_PI; theta+=0.1){
                obs_x.push_back(obstacles[ob_id].circle.x + obs_r * cos(theta));
                obs_y.push_back(obstacles[ob_id].circle.y + obs_r * sin(theta));
            }

            plt::plot(obs_x, obs_y,  ".-r");
        }
    }

    vector<float> x;
    vector<float> y;
    for(int wp_id=0; wp_id<trajectory.waypoints.size(); wp_id++){
        x.push_back(trajectory.waypoints[wp_id].x);
        y.push_back(trajectory.waypoints[wp_id].y);
    }
    plt::plot(x, y,  ".-b");

    vector<float> goal_x(1);
    vector<float> goal_y(1);
    goal_x.at(0) = goal_point.x;
    goal_y.at(0) = goal_point.y;
    plt::plot(goal_x, goal_y,  "og");

    vector<float> start_x;
    vector<float> start_y;
    for(float theta=0; theta<=2*M_PI; theta+=0.1){
        start_x.push_back(start_point.x + ROBOT_RADIUS * cos(theta));
        start_y.push_back(start_point.y + ROBOT_RADIUS * sin(theta));
    }
    plt::plot(start_x, start_y,  ".-g");

    //plt::axis("equal");
    //plt::axis("scaled");
    plt::pause(0.1);
    plt::clf();
}
