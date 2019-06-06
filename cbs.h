#ifndef CBS_H
#define CBS_H
#include <chrono>
#include "structs.h"
#include "task.h"
#include "sipp.h"
#include "heuristic.h"

class CBS
{
public:
    CBS() {}
    Solution find_solution(const Graph &map, const Task &task);
private:
    bool init_root(const Graph &map, const Task &task);
    std::list<Constraint> get_constraints(CBS_Node *node, int agent_id = -1);
    std::list<Constraint> get_altconstraints(CBS_Node *node, int agent_id = -1);
    Conflict check_conflicts(std::vector<Path> &paths, std::vector<int> conflicting_agents, std::vector<std::pair<int, int> > conflicting_pairs);
    Conflict check_paths(Path pathA, Path pathB);
    bool check_conflict(Move move1, Move move2);
    std::vector<Conflict> get_all_conflicts(std::vector<Path> &paths);
    Constraint get_constraint(int agent, Move move1, Move move2);
    Constraint get_wait_constraint(int agent, Move move1, Move move2);
    double get_cost(CBS_Node node, int agent_id);
    std::vector<Path> get_paths(CBS_Node *node, int agents_size);
    CBS_Tree tree;
    SIPP planner;
    Solution solution;
    Heuristic h_values;
};

#endif // CBS_H
