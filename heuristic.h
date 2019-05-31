#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "structs.h"
#include "const.h"
#include <vector>
#include <unordered_map>
#include "map.h"

class Heuristic
{
    std::vector<std::vector<std::vector<double>>> h_values;
    std::vector<std::pair<int, int>> moves_2k;
    unsigned int openSize;
    std::vector<std::list<Node>> open;
    Node find_min(int size);
    void add_open(Node newNode);
    double dist(const Node& a, const Node& b){ return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2)); }
public:
    Heuristic(){}
    void init(int height, int width, int agents);
    void count(const Map &map, Agent agent);
    double get_value(int i, int j, int id) { return h_values[i][j][id]; }


};

#endif // HEURISTIC_H
