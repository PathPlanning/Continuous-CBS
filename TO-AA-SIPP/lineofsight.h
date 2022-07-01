/* (c) 2017. Andreychuk A.
 * This class implements line-of-sight function for a variable size of agent.
 * It also has a method for checking cell's traversability.
 * For its work is needed the size of agent and a map container that has 'cell_is_obstacle' and 'cell_on_grid' methods.
 * If it is not possible to give the permission to access the grid, the one can use 'getCellsCrossedByLine' method.
 * It doesn't use grid and returns a set of all cells(as pairs of coordinates) that are crossed by an agent moving along a line.
 */

#ifndef LINEOFSIGHT_H
#define LINEOFSIGHT_H
#include "const.h"

#define CN_OBSTACLE 1

#include <vector>
#include <math.h>
#include <algorithm>

class LineOfSight
{
public:
    LineOfSight(double agentSize = 0.5)
    {
        this->agentSize = agentSize;
        int add_x, add_y, num = agentSize + 0.5 - CN_EPSILON;
        for(int x = -num; x <= +num; x++)
            for(int y = -num; y <= +num; y++)
            {
                add_x = x != 0 ? 1 : 0;
                add_y = y != 0 ? 1 : 0;
                if((pow(2*abs(x) - add_x, 2) + pow(2*abs(y) - add_y, 2)) < pow(2*agentSize, 2))
                    cells.push_back({x, y});
            }
        if(cells.empty())
            cells.push_back({0,0});
    }

    void setSize(double agentSize)
    {
        this->agentSize = agentSize;
        int add_x, add_y, num = agentSize + 0.5 - CN_EPSILON;
        cells.clear();
        for(int x = -num; x <= +num; x++)
            for(int y = -num; y <= +num; y++)
            {
                add_x = x != 0 ? 1 : 0;
                add_y = y != 0 ? 1 : 0;
                if((pow(2*abs(x) - add_x, 2) + pow(2*abs(y) - add_y, 2)) < pow(2*agentSize, 2))
                    cells.push_back({x, y});
            }
        if(cells.empty())
            cells.push_back({0,0});
    }

    template <class T>
    std::vector<std::pair<int, int>> getCellsCrossedByLine(int x1, int y1, int x2, int y2, const T &map)
    {
        std::vector<std::pair<int, int>> lineCells(0);
        if(x1 == x2 && y1 == y2)
        {
            for(auto cell:cells)
                lineCells.push_back({x1+cell.first, y1+cell.second});
            return lineCells;
        }
        int delta_x = std::abs(x1 - x2);
        int delta_y = std::abs(y1 - y2);
        if((delta_x >= delta_y && x1 > x2) || (delta_y > delta_x && y1 > y2))
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        int step_x = (x1 < x2 ? 1 : -1);
        int step_y = (y1 < y2 ? 1 : -1);
        int error = 0, x = x1, y = y1;
        int k, num;
        std::pair<int, int> add;
        int gap = agentSize*sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y)/2 - CN_EPSILON;

        if(delta_x >= delta_y)
        {
            int extraCheck = agentSize*delta_y/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_y;
                num = (gap - error)/delta_x;
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x1 - n*step_x, y1 + k*step_y});
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x2 + n*step_x, y2 - k*step_y});
            }
            error = 0;
            for(x = x1; x != x2 + step_x; x+=step_x)
            {
                lineCells.push_back({x, y});
                if(x < x2 - extraCheck)
                {
                    num = (gap + error)/delta_x;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x, y + k*step_y});
                }
                if(x > x1 + extraCheck)
                {
                    num = (gap - error)/delta_x;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x, y - k*step_y});
                }
                error += delta_y;
                if((error<<1) > delta_x)
                {
                    y += step_y;
                    error -= delta_x;
                }
            }
        }
        else
        {
            int extraCheck = agentSize*delta_x/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_x;
                num = (gap - error)/delta_y;
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x1 + k*step_x, y1 - n*step_y});
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x2 - k*step_x, y2 + n*step_y});
            }
            error = 0;
            for(y = y1; y != y2 + step_y; y += step_y)
            {
                lineCells.push_back({x, y});
                if(y < y2 - extraCheck)
                {
                    num = (gap + error)/delta_y;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x + k*step_x, y});
                }
                if(y > y1 + extraCheck)
                {
                    num = (gap - error)/delta_y;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x - k*step_x, y});
                }
                error += delta_x;
                if((error<<1) > delta_y)
                {
                    x += step_x;
                    error -= delta_y;
                }
            }
        }
        for(k = 0; k < cells.size(); k++)
        {
            add = {x1 + cells[k].first, y1 + cells[k].second};
            if(std::find(lineCells.begin(), lineCells.end(), add) == lineCells.end())
                lineCells.push_back(add);
            add = {x2 + cells[k].first, y2 + cells[k].second};
            if(std::find(lineCells.begin(), lineCells.end(), add) == lineCells.end())
                lineCells.push_back(add);
        }

        for(auto it = lineCells.begin(); it != lineCells.end(); it++)
            if(!map.cell_on_grid(it->first, it->second))
            {
                lineCells.erase(it);
                it = lineCells.begin();
            }
        return lineCells;
    }
    //returns all cells that are affected by agent during moving along a line

    template <class T>
    bool checkTraversability(int x, int y, const T &map)
    {
        for(int k = 0; k < cells.size(); k++)
            if(!map.cell_on_grid(x + cells[k].first, y + cells[k].second) || map.cell_is_obstacle(x + cells[k].first, y + cells[k].second))
                return false;
        return true;
    }
    //checks traversability of all cells affected by agent's body

    template <class T>
    bool checkLine(int x1, int y1, int x2, int y2, const T &map)
    {
        //if(!checkTraversability(x1, y1) || !checkTraversability(x2, y2)) //additional check of start and goal traversability,
        //    return false;                                                //it can be removed if they are already checked

        int delta_x = std::abs(x1 - x2);
        int delta_y = std::abs(y1 - y2);
        if((delta_x > delta_y && x1 > x2) || (delta_y >= delta_x && y1 > y2))
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        int step_x = (x1 < x2 ? 1 : -1);
        int step_y = (y1 < y2 ? 1 : -1);
        int error = 0, x = x1, y = y1;
        int gap = agentSize*sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y)/2 - CN_EPSILON;
        int k, num;

        if(delta_x > delta_y)
        {
            int extraCheck = agentSize*delta_y/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_y;
                num = (gap - error)/delta_x;
                for(k = 1; k <= num; k++)
                    if(map.cell_on_grid(x1 - n*step_x, y1 + k*step_y))
                        if(map.cell_is_obstacle(x1 - n*step_x, y1 + k*step_y))
                            return false;
                for(k = 1; k <= num; k++)
                    if(map.cell_on_grid(x2 + n*step_x, y2 - k*step_y))
                        if(map.cell_is_obstacle(x2 + n*step_x, y2 - k*step_y))
                            return false;
            }
            error = 0;
            for(x = x1; x != x2 + step_x; x+=step_x)
            {
                if(map.cell_is_obstacle(x, y))
                    return false;
                if(x < x2 - extraCheck)
                {
                    num = (gap + error)/delta_x;
                    for(k = 1; k <= num; k++)
                        if(map.cell_is_obstacle(x, y + k*step_y))
                            return false;
                }
                if(x > x1 + extraCheck)
                {
                    num = (gap - error)/delta_x;
                    for(k = 1; k <= num; k++)
                        if(map.cell_is_obstacle(x, y - k*step_y))
                            return false;
                }
                error += delta_y;
                if((error<<1) > delta_x)
                {
                    y += step_y;
                    error -= delta_x;
                }
            }
        }
        else
        {
            int extraCheck = agentSize*delta_x/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_x;
                num = (gap - error)/delta_y;
                for(k = 1; k <= num; k++)
                    if(map.cell_on_grid(x1 + k*step_x, y1 - n*step_y))
                        if(map.cell_is_obstacle(x1 + k*step_x, y1 - n*step_y))
                            return false;
                for(k = 1; k <= num; k++)
                    if(map.cell_on_grid(x2 - k*step_x, y2 + n*step_y))
                        if(map.cell_is_obstacle(x2 - k*step_x, y2 + n*step_y))
                            return false;
            }
            error = 0;
            for(y = y1; y != y2 + step_y; y += step_y)
            {
                if(map.cell_is_obstacle(x, y))
                    return false;
                if(y < y2 - extraCheck)
                {
                    num = (gap + error)/delta_y;
                    for(k = 1; k <= num; k++)
                        if(map.cell_is_obstacle(x + k*step_x, y))
                            return false;
                }
                if(y > y1 + extraCheck)
                {
                    num = (gap - error)/delta_y;
                    for(k = 1; k <= num; k++)
                        if(map.cell_is_obstacle(x - k*step_x, y))
                            return false;
                }
                error += delta_x;
                if((error<<1) > delta_y)
                {
                    x += step_x;
                    error -= delta_y;
                }
            }
        }
        return true;
    }
    //checks line-of-sight between a line
    std::vector<std::pair<int, int>> getCells(int i, int j)
    {
        std::vector<std::pair<int, int>> cells;
        for(int k=0; k<this->cells.size(); k++)
            cells.push_back({i+this->cells[k].first,j+this->cells[k].second});
        return cells;
    }
private:
    double agentSize;
    std::vector<std::pair<int, int>> cells; //cells that are affected by agent's body
};

#endif // LINEOFSIGHT_H
