class ObstacleCircle{
    public:
        float x;
        float y;
        float r;
};

class ObstacleRectangle{
    public:
        float x_max;
        float x_min;
        float y_max;
        float y_min;
};

class Obstacle{
    public:
        std::string shape;
        ObstacleCircle circle;
        ObstacleRectangle rectangle;
};
