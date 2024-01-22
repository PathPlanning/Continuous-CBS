#ifndef PHEURISTIC_H
#define PHEURISTIC_H

#include "structs.h"
#include "const.h"
#include <vector>
#include <unordered_map>
#include "map.h"
#include <iomanip>
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
    double dist(const hNode& a, const hNode& b){ return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2)); }
    std::vector<std::vector<int>> edges;
    LineOfSight los;
public:
    PHeuristic()
    {
        los.setSize(CN_AGENT_SIZE);
    }
    void init(unsigned int width, unsigned int height)
    {
        h_values.clear();
        edges.clear();
        int size = height*width;
        h_values.resize(size);
        for(unsigned int i = 0; i < size; i++)
            h_values[i].resize(size, CN_INFINITY);
        edges.resize(size);
        for(unsigned int i = 0; i < size; i++)
            edges[i].resize(size, -1);
    }
    bool get_los(int i1, int j1, int i2, int j2, const Map &map)
    {
        int id1 = map.get_id(i1, j1);
        int id2 = map.get_id(i2, j2);

        if(edges[id1][id2] == -1)
        {
            bool has_edge = los.checkLine(i1, j1, i2, j2, map);
            edges[id1][id2] = has_edge;
            edges[id2][id1] = has_edge;
        }
        return edges[id1][id2];
    }
    void load(std::string filename, const Map &map)
    {
        std::ifstream input(filename+".heuristic");
        if(input.is_open())
        {
            for(int i = 0; i < map.get_height(); i++)
            {
                for(int j = 0; j < map.get_width(); j++)
                    for(int i2 = 0; i2 < map.get_height(); i2++)
                        for(int j2 = 0; j2 < map.get_width(); j2++)
                            input>>h_values[map.get_id(i,j)][map.get_id(i2,j2)];
                std::cout<<i<<" ";
            }
        }
        else
            std::cout<<"HEURISTIC ERROR!\n";
        input.close();
        input.open(filename+".los");
        if(input.is_open())
        {
            for(int i = 0; i < map.get_height(); i++)
            {
                for(int j = 0; j < map.get_width(); j++)
                    for(int i2 = 0; i2 < map.get_height(); i2++)
                        for(int j2 = 0; j2 < map.get_width(); j2++)
                            input>>edges[map.get_id(i,j)][map.get_id(i2,j2)];
                std::cout<<i<<" ";
            }
        }
        else
            std::cout<<"LOS ERROR!\n";
        std::cout<<"Heuristic loaded\n";
    }
    void count(const Map &map)
    {
        for(int i = 0; i < map.get_height(); i++)
            for(int j = 0; j < map.get_width(); j++)
            {
                if(map.cell_is_obstacle(i,j))
                    continue;
                //std::cout<<i<<" "<<j<<" cur cell\n";
                hNode curNode(i, j, map.get_id(i,j), 0);
                open.clear();
                open.insert(curNode);
                int id = map.get_id(i,j);
                while(!open.empty())
                {
                    curNode = *open.get<0>().begin();
                    open.get<0>().erase(open.get<0>().begin());
                    h_values[id][curNode.id] = curNode.g;
                    int h = map.get_height(), w = map.get_width();
                    for(int i1 = -h; i1 < h; i1++)
                        for(int j1 = -w; j1 < w; j1++)
                        {
                            hNode newNode(curNode.i + i1, curNode.j + j1, map.get_id(curNode.i + i1, curNode.j + j1));
                            newNode.g = curNode.g + dist(curNode, newNode);
                            if(map.cell_on_grid(newNode.i, newNode.j))
                                if(!map.cell_is_obstacle(newNode.i, newNode.j))
                                    if(h_values[id][newNode.id] > newNode.g)
                                        if(get_los(curNode.i, curNode.j, newNode.i, newNode.j, map))
                                        {
                                            h_values[id][newNode.id] = newNode.g;
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
        std::ofstream out;
        out.open("heuristic.txt");
        for(int i = 0; i < map.get_height(); i++)
            for(int j = 0; j < map.get_width(); j++)
                for(int i2 = 0; i2 < map.get_height(); i2++)
                    for(int j2 = 0; j2 < map.get_width(); j2++)
                        out<<std::setprecision(8)<<h_values[map.get_id(i,j)][map.get_id(i2,j2)]<<" ";
        out.close();
        out.open("los.txt");
        for(int i = 0; i < map.get_height(); i++)
            for(int j = 0; j < map.get_width(); j++)
                for(int i2 = 0; i2 < map.get_height(); i2++)
                    for(int j2 = 0; j2 < map.get_width(); j2++)
                        out<<edges[map.get_id(i,j)][map.get_id(i2,j2)]<<" ";
    }
    double get_value(int id1, int id2)
    {
        return h_values[id1][id2];
    }
};

#endif // PHEURISTIC_H
