#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "structs.h"
#include "const.h"
#include <vector>
#include <unordered_map>
#include "graph.h"

class Heuristic
{
    std::vector<std::vector<double>> h_values;
    unsigned int openSize;
    std::list<Node> open;
    Node find_min();
    void add_open(Node newNode);
    double dist(const Node& a, const Node& b){ return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2)); }
public:
    Heuristic(){}
    void init(int size, int agents);
    void count(const Graph &map, Agent agent);
    double get_value(int id_node, int id_agent) { return h_values[id_node][id_agent]; }


};

#endif // HEURISTIC_H
