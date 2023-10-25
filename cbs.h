#ifndef CBS_H
#define CBS_H
#include <chrono>
#include "structs.h"
#include "map.h"
#include "task.h"
#include "config.h"
#include "sipp.h"
#include "heuristic.h"
#include "simplex/simplex.h"
#include "simplex/pilal.h"
#include "TO-AA-SIPP/to_aa_sipp.h"
class CBS
{
public:
    CBS(CBS_Tree *_tree=nullptr) {tree = _tree; if(tree == nullptr) tree = new CBS_Tree(); low_level_searches = 0; low_level_expanded = 0;}
    Solution find_solution(const Map &map, const Task &task, const Config &cfg);
    Solution find_solution_new(const Map &map, const Task &task, const Config &cfg);
    bool init_root(const Map &map, const Task &task);
    std::list<Constraint> get_constraints(CBS_Node *node, int agent_id = -1);
    //std::list<Constraint> merge_constraints(std::list<Constraint> constraints);
    bool validate_constraints(std::list<Constraint> constraints, int agent);
    bool check_positive_constraints(std::list<Constraint> constraints, Constraint constraint);
    Conflict check_paths(const sPath &pathA, const sPath &pathB);
    bool check_conflict(Move move1, Move move2);
    double get_hl_heuristic(const std::list<Conflict> &conflicts);
    double get_h(const std::list<Conflict> &conflicts, const std::map<int, double> &base_costs);
    std::vector<Conflict> get_all_conflicts(const std::map<int, sPath> &paths, int id);
    Constraint get_constraint(int agent, Move move1, Move move2);
    Constraint get_wait_constraint(int agent, Move move1, Move move2);
    void find_new_conflicts(const Map &map, const Task &task, CBS_Node &node, std::map<int, sPath> &paths, const sPath &path,
                            const std::list<Conflict> &conflicts);
    bool validate_cons(CBS_Node* node, const Task &task);
    bool validate_paths(CBS_Node* node, const Task &task);
    CBS_Node *expand(CBS_Tree *tree, const Task &task);
    void add_agent(CBS_Tree *tree, Task &task, int agent_id, double cost);
    double get_cost(CBS_Node node, int agent_id);
    std::map<int, sPath> get_paths(CBS_Node *node, std::vector<int> ids);
    double dist(int id1, int id2);
    Conflict get_conflict(std::list<Conflict> &conflicts);
    CBS_Tree *tree;
    SIPP planner;
    TO_AA_SIPP aa_planner;
    Solution solution;
    Heuristic h_values;
    std::vector<PHeuristic> aa_h_values;
    Config config;
    const Map* map;
    int low_level_searches;
    int low_level_expanded;
    std::list<CBS_Node> all_nodes;
    std::vector<Constraint> v_cons;
    std::vector<std::vector<sNode>> v_paths;
    std::pair<std::vector<Move>, std::vector<Move> > find_similar_actions(Move a, Move b);
    std::pair<std::vector<Move>,std::vector<Move>> find_similar_actions(const Task &task, Conflict conflict, std::list<Constraint> consA, std::list<Constraint> consB);
    std::vector<Constraint> get_multiconstraint(int agent, std::vector<Move> moves_a, std::vector<Move> moves_b);
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> get_similar_actions(int i1, int j1, int i2, int j2);
};

#endif // CBS_H
