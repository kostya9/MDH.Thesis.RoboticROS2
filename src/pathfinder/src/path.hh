#include <limits>
#include <cmath>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#define MAP_DISTANCE 20
#define SQUARES_PER_METER 10
#define SQUARES MAP_DISTANCE * SQUARES_PER_METER

using Coordinate = std::pair<double, double>;
using CoordinateInternal = std::pair<int, int>;

class Pathfinder
{
private:
    CoordinateInternal curPos;
    CoordinateInternal targetPos;
    double curYaw;
    double robotWidth;
    bool changed;
    bool map[SQUARES][SQUARES];
    rclcpp::Logger logger;
    std::shared_ptr<std::vector<CoordinateInternal>> prevPath;
public:
    Pathfinder(rclcpp::Logger logger, double robotWidth = 0.5);
    void SetTarget(int targetX, int targetY);
    bool AddCollision(double distance, double dYaw);
    void SetPosition(double x, double y, double curYaw);
    std::shared_ptr<std::vector<Coordinate>> GetPathToTarget();
    std::vector<std::pair<double, double>> GetCollisions();
};

Pathfinder::Pathfinder(rclcpp::Logger logger, double robotWidth) : logger(logger)
{
    this->robotWidth = robotWidth;
    for(auto i = 0; i < SQUARES; i++) 
    {
        for(auto j = 0; j < SQUARES; j++) 
        {
            map[i][j] = false;
        }   
    }
    this->prevPath = std::make_shared<std::vector<CoordinateInternal>>();
}

CoordinateInternal ToInternal(Coordinate ex)
{
    return std::make_pair((int)(ex.first * SQUARES_PER_METER), (int)(ex.second * SQUARES_PER_METER));
}

Coordinate FromInternal(CoordinateInternal in)
{
    return std::make_pair(in.first / (double)SQUARES_PER_METER, in.second / (double)SQUARES_PER_METER);
}

std::vector<std::pair<double, double>> Pathfinder::GetCollisions() 
{
    std::vector<std::pair<double, double>> collisions;
    for(auto i = 0; i < SQUARES; i++) 
    {
        for(auto j = 0; j < SQUARES; j++) 
        {
            if(map[i][j]) {
                auto external = FromInternal(std::pair<double, double>(i, j));
                collisions.push_back(external);
            }
        }   
    }
    return collisions;
}

double DistTo(CoordinateInternal pos, CoordinateInternal target) {
    double dy = pos.second - target.second;
    double dx = pos.first - target.first;
    return sqrt(dy * dy + dx * dx);
}

double AStarDist(CoordinateInternal pos, CoordinateInternal target) {
    double dy = pos.second - target.second;
    double dx = pos.first - target.first;
    return dy * dy + dx * dx;
}

void AdvancePath(std::shared_ptr<std::vector<CoordinateInternal>> path, CoordinateInternal curPos)
{
    while(true)
    {
        if(path->size() == 0)
        {
            return;
        }

        auto first = path->at(0);
        if(DistTo(first, curPos) <= SQUARES_PER_METER / 2)
        {
            path->erase(path->begin());
            continue;
        }
        
        return;
    }
}

void Pathfinder::SetPosition(double x, double y, double curYaw) 
{
    this->curPos = ToInternal(std::make_pair(x, y));
    this->curYaw = curYaw;
    AdvancePath(prevPath, curPos);
}

bool Pathfinder::AddCollision(double distance, double dYaw) 
{
    auto yaw = curYaw + dYaw;
    auto curPosEx = FromInternal(curPos);
    auto targetX = curPosEx.first + distance * cos(yaw);
    auto targetY = curPosEx.second + distance * sin(yaw);

    auto internal = ToInternal(std::make_pair(targetX, targetY));

    bool addedCollision = false;
    // Assume that +- robotWidth around collision is inacessible too
    for (int dx = -robotWidth * SQUARES_PER_METER; dx < robotWidth * SQUARES_PER_METER; dx++)
    {
        for (int dy = -robotWidth * SQUARES_PER_METER; dy < robotWidth * SQUARES_PER_METER; dy++)
        {
            int x = internal.first + dx;
            int y = internal.second + dy;
            if(x < 0 || x >= SQUARES || 
                y < 0 || y >= SQUARES) 
            {
                continue;
            }

            if(!map[x][y])
            {
                this->map[x][y] = true;
                this->changed = true;
                addedCollision = true;
            }
        }
    }

    return addedCollision;
}

void Pathfinder::SetTarget(int targetX, int targetY)
{
    this->targetPos = ToInternal(std::make_pair(targetX, targetY));
    this->changed = true;
}

class Node
{
    public:
    Node(CoordinateInternal pos)
    {
        this->pos = pos;
    }

    CoordinateInternal pos;
    double g;
    double f;
    double h;
    std::shared_ptr<Node> parent;
};

int IdxOfSmallestF(std::vector<std::shared_ptr<Node>> & openList)
{
    int smallestIdx = 0;
    double smallestF = openList.at(0)->f;
    for(size_t i = 1; i < openList.size(); i++) {
        auto cur = openList.at(i);
        double curF = cur->f;
        if(curF < smallestF) {
            smallestF = curF;
            smallestIdx = i;
        }
    }
    
    return smallestIdx;
}

std::shared_ptr<std::vector<CoordinateInternal>> GeneratePathFromParents(std::shared_ptr<Node> target)
{
    std::shared_ptr<std::vector<CoordinateInternal>> path = std::make_shared<std::vector<CoordinateInternal>>();
    
    std::shared_ptr<Node> parent = target;
    while(true) {
        if(parent == nullptr) {
            break;
        }
        
        // insert to the beginning
        path->insert(path->begin(), parent->pos);    
        parent = parent->parent;
    }

    path->erase(path->begin());
    
    return path;
}

std::shared_ptr<Node> FindEqualPos(std::vector<std::shared_ptr<Node>>& list, CoordinateInternal target)
{
    for(size_t i = 0; i < list.size(); i++)
    {
        auto cur = list.at(i);

        if(cur->pos == target)
        {
            return cur;
        } 
    }

    return nullptr;
}

std::shared_ptr<std::vector<Coordinate>> FromInternalPath(std::shared_ptr<std::vector<CoordinateInternal>> inPath)
{
    std::vector<Coordinate> exPath;
    std::transform(inPath->begin(), inPath->end(),
                   std::back_inserter(exPath),
                   [](const CoordinateInternal& elem) { return FromInternal(elem); });

    return std::make_shared<std::vector<Coordinate>>(exPath);
}

std::vector<std::tuple<int, int>> GetNeighbors(double yaw) {
    std::vector<std::tuple<int, int>> neighbors;

    neighbors.push_back(std::make_tuple(1, 0));
    neighbors.push_back(std::make_tuple(1, 1));
    neighbors.push_back(std::make_tuple(0, 1));
    neighbors.push_back(std::make_tuple(-1, 1));
    neighbors.push_back(std::make_tuple(-1, 0));
    neighbors.push_back(std::make_tuple(-1, -1));
    neighbors.push_back(std::make_tuple(0, -1));
    neighbors.push_back(std::make_tuple(1, -1));

    int startIdx = 0;
    if(yaw > M_PI_4 && yaw <= 3 * M_PI_4) {
        startIdx = 2;
    } else if(yaw > 3 * M_PI_4 || yaw < - 3 * M_PI_4) {
        startIdx = 4;
    } else if(yaw > - 3 * M_PI_4 && yaw < - M_PI_4) {
        startIdx = 6;
    }

    std::rotate(neighbors.begin(), neighbors.begin() + startIdx ,neighbors.end());

    return neighbors;
}

std::shared_ptr<std::vector<Coordinate>> Pathfinder::GetPathToTarget()
{
    if(!this->changed)
    {
        return FromInternalPath(prevPath);
    }

    RCLCPP_INFO(logger, "Calculating new path");

    this->changed = false;

    std::vector<std::shared_ptr<Node>> openList;
    std::vector<std::shared_ptr<Node>> closedList;
    auto firstNode = std::make_shared<Node>(this->curPos);
    openList.push_back(firstNode);
    
    while(openList.size() != 0) {
        int curOpenListIdx = IdxOfSmallestF(openList);
        auto curNode = openList.at(curOpenListIdx);
        openList.erase(openList.begin() + curOpenListIdx);
        closedList.push_back(curNode);
        
        int x = curNode->pos.first;
        int y = curNode->pos.second;
        if(x == targetPos.first && y == targetPos.second) {
            auto path = GeneratePathFromParents(curNode);
            AdvancePath(path, curPos);

            this->prevPath = path;

            RCLCPP_INFO(logger, "Done calculating new path");
            return FromInternalPath(prevPath);
        }

        auto neighbors = GetNeighbors(curYaw);
        
        // for each neighbor
        for (auto neighbor: neighbors) {
            auto dx = std::get<0>(neighbor);
            auto dy = std::get<1>(neighbor);
            
            // IGNORE DIAGONALS
            // TODO: HANDLE DIAGONALS
            if(dx == 0 && dy == 0) {
                continue;
            }
            
            int neighborX = x + dx;
            int neighborY = y + dy;
            auto neighborPos = std::make_pair(neighborX, neighborY);
            
            if(neighborX < 0 || neighborX >= SQUARES ||
                neighborY < 0 || neighborY >= SQUARES) {
                continue;
            }
            
            bool collision = map[neighborX][neighborY];
            if(collision) {
                continue;
            }
            
            auto alreadyInClosedList = FindEqualPos(closedList, neighborPos);
            if(alreadyInClosedList != nullptr) {
                continue;
            }
            
            double g = curNode->g + sqrt(dx * dx + dy * dy); // TODO: handle diagonals
            double h = AStarDist(neighborPos, targetPos);
            double f = g + h;
            
            auto alreadyInOpenList = FindEqualPos(openList, neighborPos);
            if(alreadyInOpenList != nullptr) {
                if(g > alreadyInOpenList->g) {
                    continue;
                }
            } else {
                alreadyInOpenList = std::make_shared<Node>(neighborPos);
            }
            
            alreadyInOpenList->parent = curNode;
            alreadyInOpenList->g = g;
            alreadyInOpenList->h = h;
            alreadyInOpenList->f = f;
            openList.push_back(alreadyInOpenList);
        }
    }

    RCLCPP_ERROR(logger ,"Could not build path");

    return nullptr;
}