#ifndef TO_AA_SIPP_H
#define TO_AA_SIPP_H
#include "states_container.h"
#include "structs.h"
#include <iomanip>
#include "heuristic.h"

class TO_AA_SIPP
{

public:
    TO_AA_SIPP();
    ~TO_AA_SIPP();
    Path findPath(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values_);

private:

    double calculateDistanceFromCellToCell(double start_i, double start_j, double fin_i, double fin_j);
    //bool lineOfSight(int i1, int j1, int i2, int j2, const Map &map);
    void findSuccessors(const oNode &curNode, const Map &Map, std::list<oNode> &succs, int numOfCurAgent){}
    void makePrimaryPath(oNode curNode);
    void initStates(Agent agent, const Map &Map, const std::list<Constraint> &constraints);
    void make_constraints(const std::list<Constraint> &cons);
    void clear();
    void add_collision_interval(int id, std::pair<double, double> interval);
    void add_move_constraint(Move move);
    int constraints_type;
    unsigned int closeSize, openSize;
    Path path;
    StatesContainer states;
    LineOfSight los;
    Heuristic h_values;
    std::map<std::pair<int, int>, std::vector<Move>> constraints;//stores sets of constraints associated with moves
    std::unordered_map<int, std::vector<std::pair<double, double>>> collision_intervals;//stores sets of collision intervals associated with cells
    double findEAT(oNode node);
};

#endif // TO_AA_SIPP_H
