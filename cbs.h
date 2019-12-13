#ifndef CBS_H
#define CBS_H
#include <chrono>
#include "structs.h"
#include "map.h"
#include "task.h"
#include "config.h"
#include "sipp.h"
#include "heuristic.h"

class CBS
{
public:
    CBS() {}
    Solution find_solution(const Map &map, const Task &task, const Config &cfg);
private:
    bool init_root(const Map &map, const Task &task);
    std::list<Constraint> get_constraints(CBS_Node *node, int agent_id = -1);
    Conflict check_conflicts(std::vector<Path> &paths, std::vector<int> conflicting_agents, std::vector<std::pair<int, int> > conflicting_pairs);
    Conflict check_paths(Path pathA, Path pathB);
    bool check_conflict(Move move1, Move move2);
    std::vector<Conflict> get_all_conflicts(std::vector<Path> &paths, int id);
    Constraint get_constraint(int agent, Move move1, Move move2);
    Constraint get_wait_constraint(int agent, Move move1, Move move2);
    void find_new_conflicts(const Map &map, const Task &task, CBS_Node &node, std::vector<Path> paths, Path path,
                            std::list<Conflict> conflicts, std::list<Conflict> semicard_conflicts, std::list<Conflict> cardinal_conflicts,
                            int &low_level_searches, int &low_level_expanded);
    double get_cost(CBS_Node node, int agent_id);
    std::vector<Path> get_paths(CBS_Node *node, int agents_size);
    Conflict get_conflict(std::list<Conflict> &conflicts);
    CBS_Tree tree;
    SIPP planner;
    Solution solution;
    Heuristic h_values;
    Config config;

};

#endif // CBS_H
