#ifndef TO_AA_SIPP_H
#define TO_AA_SIPP_H
#include "states_container.h"
#include "structs.h"
#include <iomanip>
#include "pheuristic.h"

class TO_AA_SIPP
{

public:
    TO_AA_SIPP();
    ~TO_AA_SIPP();
    //std::vector<Path> find_multiple_paths(Agent agent, const Map &map, std::list<Constraint> cons, PHeuristic &h_values_, std::pair<Node, Node> colliding_edge);
    Path find_path(Agent agent, const Map &map, std::list<Multiconstraint> cons, PHeuristic &h_values_);
    Path check_all_paths(Agent agent, const Map &map, std::list<Multiconstraint> cons, PHeuristic &h_values_, int id=-1);
    int node_id;
private:
    double check_endpoint(Node start, Node goal);
    Path add_part(Path result, Path part);
    std::vector<Path> find_partial_path(Agent agent, const Map &map, double max_f = CN_INFINITY);
    std::vector<Node> get_endpoints(int node_id, double node_i, double node_j, double t1, double t2);
    double calculateDistanceFromCellToCell(double start_i, double start_j, double fin_i, double fin_j);
    //bool lineOfSight(int i1, int j1, int i2, int j2, const Map &map);
    void findSuccessors(const oNode &curNode, const Map &Map, std::list<oNode> &succs, int numOfCurAgent){}
    void makePrimaryPath(oNode curNode);
    void initStates(Agent agent, const Map &Map);
    void make_constraints(const std::list<Multiconstraint> &cons);
    void clear();
    void add_collision_interval(int id, std::pair<double, double> interval);
    void add_move_constraint(Move move);
    std::vector<Path> execute_positive_actions(const std::vector<Move> &actions, const Map &map, const Node &start);
    int constraints_type;
    //std::vector<Move> landmarks;
    std::vector<std::vector<Move>> multilandmarks;
    unsigned int closeSize, openSize;
    Path path;
    StatesContainer states;
    LineOfSight los;
    PHeuristic* h_values;
    std::map<std::pair<int, int>, std::vector<Move>> constraints;//stores sets of constraints associated with moves
    std::unordered_map<int, std::vector<std::pair<double, double>>> collision_intervals;//stores sets of collision intervals associated with cells
    double findEAT(oNode node);
    const Map* map;
    double dist(int id1, int id2);
};

#endif // TO_AA_SIPP_H
