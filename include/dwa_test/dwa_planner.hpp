#include<ros/ros.h>
#include<dwa_test/obstacle.hpp>
#include"matplotlib-cpp/matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

class Point{
    public:
        float x;
        float y;
};

class Velocity{
    public:
        float vx;
        float vy;

        Velocity(){
        }

        Velocity(float set_vx, float set_vy){
            vx = set_vx;
            vy = set_vy;
        }
};

class Trajectory{
    public:
        std::vector<Point> waypoints;
};

class DynamicWindow{
    public:
        constexpr static float RESOLUTION = 0.01;
        float vx_max;
        float vx_min;
        float vy_max;
        float vy_min;
};

class DWAPlanner{
    public:
        constexpr static float DT = 0.1;
        constexpr static float PREDICT_TIME = 3.0;
        constexpr static float TO_GOAL_COST_GAIN = 1.0;
        constexpr static float SPEED_COST_GAIN = 1.0;
        constexpr static float OBSTACLE_COST_GAIN = 1.0;
        constexpr static float MAX_VEL = 1;
        constexpr static float MIN_VEL = -1;
        constexpr static float MAX_ACC = 0.5;
        constexpr static float MAX_ANG_VEL = 40.0 * M_PI / 180.0;
        constexpr static float MAX_ANG_ACC = 40.0 * M_PI / 180.0;
        constexpr static float ROBOT_RADIUS = 0.3;

        DWAPlanner();
        ~DWAPlanner();
    private:
        void setObstacles();
        DynamicWindow calclateDynamicWindow();
        Trajectory calclateTrajectory(DynamicWindow, Velocity&);
        Trajectory predictTrajectory(Velocity);
        float calclateToGoalCost(Trajectory);
        float calclateObstacleCost(Trajectory);
        float calclateMinDistanceFromObstacles(Point);
        float calclateDistanceFromRectangle(Point, ObstacleRectangle);
        Trajectory planTrajectory();
        Trajectory visualizeTrajectory(Trajectory, Velocity);

        Point start_point;
        Velocity start_velocity;
        Point goal_point;

        vector<Obstacle> obstacles;
};
