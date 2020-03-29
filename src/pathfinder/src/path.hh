#include <limits>
#include <cmath>
#include <vector>

#define MAP_DISTANCE 10
#define SQUARES_PER_METER 100
#define SQUARES MAP_DISTANCE * SQUARES_PER_METER

using Coordinate = std::pair<double, double>;
using CoordinateInternal = std::pair<int, int>;

class Path
{
private:
    CoordinateInternal curPos;
    CoordinateInternal targetPos;
    double curYaw;
    double robotWidth;
    bool changed;
    bool map[SQUARES][SQUARES];
    std::shared_ptr<std::vector<Coordinate>> prevPath;
public:
    Path(double robotWidth = 0.5);
    void SetTarget(int targetX, int targetY);
    void AddCollision(double distance, double dYaw);
    void SetPosition(double x, double y, double curYaw);
    std::shared_ptr<std::vector<Coordinate>> GetPathToTarget();

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

Path::Path(double robotWidth = 0.5)
{
    this->robotWidth = robotWidth;
    for(auto i = 0; i < SQUARES; i++) 
    {
        for(auto j = 0; j < SQUARES; j++) 
        {
            map[i][j] = false;
        }   
    }
    this->prevPath = std::make_shared<std::vector<Coordinate>>();
}

void Path::SetPosition(double x, double y, double curYaw) 
{
    this->curPos = ToInternal(std::make_pair(x, y));
    this->curYaw = curYaw;

    if(prevPath->size() > 0)
    {
        auto first = prevPath->at(0);
        while(DistTo(first, curPos) < 0.1)
        {
            prevPath->erase(prevPath->begin());
            first = prevPath->at(0);
        }
    }
}

void Path::AddCollision(double distance, double dYaw) 
{
    auto yaw = curYaw + dYaw;
    auto targetX = this->curPos.first + distance * cos(yaw);
    auto targetY = this->curPos.second + distance * sin(yaw);

    auto internal = ToInternal(std::make_pair(targetX, targetY));

    if(internal.first < 0 || internal.first >= SQUARES || 
        internal.second < 0 || internal.second >= SQUARES) 
    {
        return;
    }

    if(!map[internal.first][internal.second])
    {
        this->map[internal.first][internal.second] = true;
        this->changed = true;
    }
}

CoordinateInternal ToInternal(Coordinate ex)
{
    return std::make_pair(ex.first * SQUARES_PER_METER, ex.second * SQUARES_PER_METER);
}

Coordinate FromInternal(CoordinateInternal in)
{
    return std::make_pair(in.first / SQUARES_PER_METER, in.second / SQUARES_PER_METER);
}

void Path::SetTarget(int targetX, int targetY)
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

std::shared_ptr<std::vector<Coordinate>> GeneratePathFromParents(std::shared_ptr<Node> target)
{
    std::shared_ptr<std::vector<Coordinate>> path = std::make_shared<std::vector<Coordinate>>();
    
    std::shared_ptr<Node> parent = target;
    while(true) {
        if(parent == nullptr) {
            break;
        }
        
        // insert to the beginning
        path->insert(path->begin(), parent->pos);    
        parent = parent->parent;
    }
    
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

double DistTo(CoordinateInternal pos, CoordinateInternal target) {
    double dy = pos.second - target.second;
    double dx = pos.first - target.first;
    return dy * dy + dx * dx;
}

std::shared_ptr<std::vector<Coordinate>> Path::GetPathToTarget()
{
    if(!this->changed)
    {
        return prevPath;
    }

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

            this->prevPath = path;
            
            return path;
        }
        
        // for each neighbor
        for(int dx = -1; dx <= 1; dx++) {
            for(int dy = -1; dy <= 1; dy++) {
                
                // IGNORE DIAGONALS
                // TODO: HANDLE DIAGONALS
                if(dx + dy != 1 && dx + dy != -1) {
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
                
                double g = curNode->g + 1; // TODO: handle diagonals
                double h = DistTo(neighborPos, targetPos);
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
    }
}