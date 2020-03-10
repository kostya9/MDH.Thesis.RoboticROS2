#define MAP_DISTANCE 10
#define SQUARES_PER_METER 100
#define SQUARES MAP_DISTANCE * SQUARES_PER_METER

#include <limits>
#include <cmath>
#include <vector>

class Path
{
private:
    double xPos;
    double yPos;
    double curYaw;
    bool map[SQUARES][SQUARES];
public:
    Path();
    void AddCollision(double distance, double dYaw);
    void SetPosition(double x, double y, double curYaw);

    std::vector<std::pair<double, double>> GetCollisions() 
    {
        std::vector<std::pair<double, double>> collisions;
        for(auto i = 0; i < SQUARES; i++) 
        {
            for(auto j = 0; j < SQUARES; j++) 
            {
                if(map[i][j]) {
                    auto x = i / (double)SQUARES_PER_METER;
                    auto y = j / (double)SQUARES_PER_METER;
                    collisions.push_back(std::pair<double, double>(x, y));
                }
            }   
        }
        return collisions;
    }
};

Path::Path()
{
    for(auto i = 0; i < SQUARES; i++) 
        {
            for(auto j = 0; j < SQUARES; j++) 
            {
                map[i][j] = false;
            }   
        }
}

void Path::SetPosition(double x, double y, double curYaw) 
{
    this->xPos = x;
    this->yPos = y;
    this->curYaw = curYaw;
}

void Path::AddCollision(double distance, double dYaw) 
{
    auto yaw = curYaw + dYaw;
    auto targetX = this->xPos + distance * cos(yaw);
    auto targetY = this->yPos + distance * sin(yaw);

    auto targetXIndex = (int)(targetX * SQUARES_PER_METER);
    auto targetYIndex = (int)(targetY * SQUARES_PER_METER);

    if(targetXIndex < 0 || targetXIndex >= SQUARES || 
        targetYIndex < 0 || targetYIndex >= SQUARES) 
    {
        return;
    }

    map[targetXIndex][targetYIndex] = true;
}
