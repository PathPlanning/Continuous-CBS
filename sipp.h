#ifndef SIPP_H
#define SIPP_H
#include "structs.h"
#include "heuristic.h"
#include <unordered_map>
#include <map>
#include <set>
#include <graph.h>
class SIPP
{
public:

    SIPP()  {}
    ~SIPP() {}
    Path find_path(Agent agent, const Graph &map, std::list<Constraint> cons, Heuristic &h_values);

private:
    Agent agent;
    void find_successors(Node curNode, const Graph &map, std::list<Node> &succs, Heuristic &h_values);
    void add_open(Node newNode);
    double dist(const Node& a, const Node &b);
    void reconstruct_path(Node curNode);
    void make_constraints(std::list<Constraint> &cons);
    void clear();

    unsigned int openSize;
    std::unordered_multimap<int, Node> close;
    std::list<Node> open;
    Path path;
    std::map<std::pair<int, int>, std::vector<Move>> constraints;//stores sets of constraints associated with moves
    std::map<int, std::vector<std::pair<double, double>>> collision_intervals;//stores sets of collision intervals associated with cells
};

#endif // SIPP_H
