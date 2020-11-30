#ifndef SIPP_H
#define SIPP_H
#include "structs.h"
#include "map.h"
#include "heuristic.h"
#include <unordered_map>
#include <map>
#include <set>
class SIPP
{
public:

    SIPP()  {}
    ~SIPP() {}
    Path find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values, bool pft);

private:
    Agent agent;
    double getRCost(double headingA, double headingB);
    double calcHeading(const Node &node, const Node &son);
    void find_successors(Node curNode, const Map &map, std::list<Node> &succs, Heuristic &h_values);
    void add_open(Node newNode);
    Node find_min();
    double dist(const Node& a, const Node& b);
    void reconstruct_path(Node curNode);
    void make_constraints(std::list<Constraint> &cons);
    void clear();
    bool stopCriterion(const Node &curNode, Node &goalNode, bool pft);

    bool planforturns;
    std::unordered_multimap<int, Node> close;
    OPEN_container open;
    Path path;
    std::map<std::pair<int, int>, std::vector<Move>> constraints;//stores sets of constraints associated with moves
    std::map<int, std::vector<std::pair<double, double>>> collision_intervals;//stores sets of collision intervals associated with cells
};

#endif // SIPP_H
