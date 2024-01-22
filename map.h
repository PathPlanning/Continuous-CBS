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
    int  height, width, size;
    double agent_size;
    bool check_line(int x1, int y1, int x2, int y2);
    bool get_grid(const char* FileName);
    bool get_roadmap(const char* FileName);
public:
    Map(double size){ agent_size = size; }
    ~Map(){}
    int     get_size() const { return size; }
    bool    get_map(const char* FileName);
    bool    cell_on_grid(int i, int j) const;
    bool    cell_is_obstacle(int i, int j) const;
    int     get_width() const {return width;}
    int     get_height() const {return height;}
    gNode   get_gNode(int id) const {if(id < int(nodes.size())) return nodes[id]; return gNode();}
    int     get_id(int i, int j) const;
    double  get_i (int id) const;
    double  get_j (int id) const;
    void    print_map();
    void    printPPM();
};

#endif // MAP_H
