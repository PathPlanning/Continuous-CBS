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
    int height, width;
    std::vector<std::vector<std::vector<Step>>> valid_moves;
    bool check_line(int x1, int y1, int x2, int y2);
public:
    Map(){}
    ~Map(){}
    int get_height() const { return height; }
    int get_width()  const { return width;  }
    bool get_map(const char* FileName);
    bool cell_is_traversable (int i, int j) const;
    bool cell_on_grid (int i, int j) const;
    bool cell_is_obstacle(int i, int j) const;
    int  get_value(int i, int j) const;
    std::vector<Step> get_valid_moves(int i, int j) const;
    void generate_moves();
    void print_map();
    void printPPM();
    void generate_map();
};

#endif // MAP_H
