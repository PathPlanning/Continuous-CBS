#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "structs.h"
#include "const.h"
#include <vector>
#include <unordered_map>
#include "map.h"

typedef multi_index_container<
        Node,
        indexed_by<
                    //ordered_non_unique<tag<cost>, BOOST_MULTI_INDEX_MEMBER(Open_Elem, double, cost)>,
                    ordered_non_unique<BOOST_MULTI_INDEX_MEMBER(Node, double, g)>,
                    hashed_non_unique<BOOST_MULTI_INDEX_MEMBER(Node, int, id)>
        >
> Open_Container;


class Heuristic
{
    std::vector<std::vector<double>> h_values;
    Open_Container open;
    bool planforturns;
    Node find_min();
    Agent agent;
    double dist(const Node& a, const Node& b){ return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2)); }
    double getRCost(double headingA, double headingB);
    double calcHeading(const Node &node, const Node &son);
    void add_open(Node newNode);
public:
    Heuristic(){}
    void init(int size, int agents, bool pft);
    void count(const Map &map, Agent agent);
    double get_value(int id_node, int id_agent) { return h_values[id_node][id_agent]; }
};

#endif // HEURISTIC_H
