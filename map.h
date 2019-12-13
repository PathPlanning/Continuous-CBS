#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <vector>
#include "tinyxml2.h"
#include "const.h"
#include "structs.h"

class Map
{
private:
    std::vector<std::vector<int>> grid;
    std::vector<gNode> nodes;
    std::vector<std::vector<Node>> valid_moves;
    int  height, width, size;
    int  connectedness;
    double agent_size;
    bool map_is_roadmap;
    bool check_line(int x1, int y1, int x2, int y2);
    bool get_grid(const char* FileName);
    bool get_roadmap(const char* FileName);
public:
    Map(double size, int k){ agent_size = size; connectedness = k; }
    ~Map(){}
    int  get_size() const { return size; }
    bool get_map(const char* FileName);
    bool is_roadmap() const {return map_is_roadmap;}
    bool cell_is_obstacle(int i, int j) const;
    int  get_width() const {return width;}
    gNode get_gNode(int id) const {if(id < nodes.size()) return nodes[id]; return gNode();}
    int  get_id(int i, int j) const;
    std::vector<Node> get_valid_moves(int id) const;
    void print_map();
    void printPPM();
};

#endif // MAP_H
