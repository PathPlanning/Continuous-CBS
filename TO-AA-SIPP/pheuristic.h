#ifndef PHEURISTIC_H
#define PHEURISTIC_H

#include "structs.h"
#include "const.h"
#include <vector>
#include <unordered_map>
#include "map.h"
#include "lineofsight.h"
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/composite_key.hpp>
using boost::multi_index_container;
using namespace boost::multi_index;

struct hNode
{
    int i, j, id;
    double g;
    bool operator<(const hNode& n) const
    {
        if(this->i == n.i)
            return this->j < n.j;
        return this->i < n.i;
    }
    hNode(int _i, int _j, int _id, double _g=-1):i(_i), j(_j), id(_id), g(_g){}
};

typedef multi_index_container<
  hNode,
  indexed_by<
    ordered_non_unique<member<hNode, double, &hNode::g>>,
    hashed_unique<member<hNode, int, &hNode::id>>
  >
> H_Container;

class PHeuristic
{
    std::vector<std::vector<double>> h_values;
    H_Container open;
    int goal_i, goal_j;
    double dist(const hNode& a, const hNode& b){ return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2)); }
public:
    PHeuristic(){}
    void init(unsigned int width, unsigned int height)
    {
        h_values.clear();
        h_values.resize(height);
        for(unsigned int i = 0; i < height; i++)
            h_values[i].resize(width, CN_INFINITY);
    }
    void count(const Map &map, Agent agent)
    {
        init(map.get_width(), map.get_height());
        LineOfSight los;
        los.setSize(0.353553);
        hNode curNode(agent.goal_i, agent.goal_j, 0);
        open.clear();
        open.insert(curNode);
        int k=0;
        while(!open.empty())
        {
            curNode = *open.get<0>().begin();
            open.get<0>().erase(open.get<0>().begin());
            k++;
            h_values[curNode.i][curNode.j] = curNode.g;
            int h = map.get_height(), w = map.get_width();
            for(int i1=-h; i1<h; i1++)
                for(int j1=-w; j1<w; j1++)
                {
                    hNode newNode(curNode.i + i1, curNode.j + j1, (curNode.i + i1)*map.get_width() + curNode.j + j1);
                    newNode.g = curNode.g + dist(curNode, newNode);
                    if(map.cell_on_grid(newNode.i, newNode.j))
                        if(!map.cell_is_obstacle(newNode.i, newNode.j))
                            if(h_values[newNode.i][newNode.j] > newNode.g)
                                if(los.checkLine(curNode.i, curNode.j, newNode.i, newNode.j, map))
                                {
                                    h_values[newNode.i][newNode.j] = newNode.g;
                                    auto it = open.get<1>().find(newNode.id);
                                    if(it != open.get<1>().end())
                                    {
                                        if(it->g > newNode.g)
                                            open.get<1>().erase(it);
                                        else
                                            continue;
                                    }
                                    open.insert(newNode);
                                }
                }
        }
    }
    void set_goal(int i, int j) { goal_i = i; goal_j = j;}
    unsigned int get_size() const { return h_values[0].size(); }
    double get_value(int i, int j) { if(h_values.empty())
                                        return sqrt(pow(i - goal_i,2) + pow(j - goal_j,2));
                                     return h_values[i][j]; }
};

#endif // PHEURISTIC_H
