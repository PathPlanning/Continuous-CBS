#ifndef CBS_H
#define CBS_H
#include <chrono>
#include "structs.h"
#include "map.h"
#include "task.h"
#include "config.h"
#include "to_aa_sipp.h"
class CBS
{
public:
    CBS(CBS_Tree *_tree=nullptr) {tree = _tree; if(tree == nullptr) tree = new CBS_Tree(); low_level_searches = 0; low_level_expanded = 0;}
    Solution find_solution(const Map &map, const Task &task, const Config &cfg, PHeuristic &pheuristic);
    bool init_root(const Map &map, const Task &task);
    std::list<Multiconstraint> get_constraints(CBS_Node *node, int agent_id = -1);
    bool validate_constraints(std::list<Multiconstraint> constraints, int agent);
    bool check_positive_constraints(std::list<Multiconstraint> constraints, Multiconstraint constraint);
    Conflict check_paths(const sPath &pathA, const sPath &pathB);
    bool check_conflict(Move move1, Move move2);
    std::vector<Conflict> get_all_conflicts(const std::map<int, sPath> &paths, int id);
    Constraint get_constraint(int agent, Move move1, Move move2);
    Constraint get_wait_constraint(int agent, Move move1, Move move2);
    void find_new_conflicts(const Map &map, const Task &task, CBS_Node &node, std::map<int, sPath> &paths, const sPath &path,
                            const std::list<Conflict> &conflicts);
    bool validate_cons(CBS_Node* node, const Task &task);
    bool validate_paths(CBS_Node* node, const Task &task);
    CBS_Node *expand(CBS_Tree *tree, const Task &task);
    double get_cost(CBS_Node node, int agent_id);
    std::map<int, sPath> get_paths(CBS_Node *node, std::vector<int> ids);
    double dist(int id1, int id2);
    Conflict get_conflict(std::list<Conflict> &conflicts);
    CBS_Tree *tree;
    TO_AA_SIPP aa_planner;
    Solution solution;
    PHeuristic* aa_h_values;
    Config config;
    const Map* map;
    int low_level_searches;
    int low_level_expanded;
    std::list<CBS_Node> all_nodes;
    std::vector<Constraint> v_cons;
    std::vector<std::vector<sNode>> v_paths;
    std::pair<std::vector<Move>, std::vector<Move> > find_similar_actions(Move a, Move b);
    std::pair<std::vector<Move>,std::vector<Move>> find_similar_actions(const Task &task, Conflict conflict, std::list<Constraint> consA, std::list<Constraint> consB);
    std::pair<std::vector<Move>,std::vector<Move>> get_all_similar_actions(Move a, Move b);
    Multiconstraint get_multiconstraint(int agent, std::vector<Move> moves_a, std::vector<Move> moves_b);
    std::vector<std::pair<int, int>> get_cells_in_spiral(int start_i, int start_j);
    std::vector<std::pair<int, int>> get_cells(int x1, int y1, int x2, int y2);
};

#endif // CBS_H
