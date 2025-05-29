#ifndef STRUCTS_H
#define STRUCTS_H
#include <math.h>
#include <vector>
#include <list>
#include <iostream>
#include <chrono>
#include "const.h"
#include <memory>
#include <set>
#include <iterator>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/composite_key.hpp>
using boost::multi_index_container;
using namespace boost::multi_index;

struct Agent
{
    double start_i, start_j, goal_i, goal_j;
    int start_id, goal_id;
    int id;
    double size;
    Agent(int s_id = -1, int g_id = -1, int _id = -1)
        :start_id(s_id), goal_id(g_id), id(_id) {}
};

struct gNode
{
    double i;
    double j;
    std::vector<int> neighbors;
    gNode(double i_ = -1, double j_ = -1):i(i_), j(j_) {}
    ~gNode() { neighbors.clear(); }
};

struct Node
{
    int     id;
    double  f, g, i, j;
    Node*   parent;
    std::pair<double, double> interval;
    int interval_id;
    Node(int _id = -1, double _f = -1, double _g = -1, double _i = -1, double _j = -1, Node* _parent = nullptr, double begin = -1, double end = -1)
        :id(_id), f(_f), g(_g), i(_i), j(_j), parent(_parent), interval(std::make_pair(begin, end)) {interval_id = 0;}
    bool operator <(const Node& other) const //required for heuristic calculation
    {
        return this->g < other.g;
    }
};

struct Position
{
    double  i, j, t;
    Position(double _i = -1, double _j = -1, double _t = -1)
        :i(_i), j(_j), t(_t) {}
    Position(const Node& node): i(node.i), j(node.j), t(node.g) {}
};

struct Path
{
    std::vector<Node> nodes;
    double cost;
    int agentID;
    int expanded;
    Path(std::vector<Node> _nodes = std::vector<Node>(0), double _cost = -1, int _agentID = -1)
        : nodes(_nodes), cost(_cost), agentID(_agentID) {expanded = 0;}
};

struct sNode
{
    int id;
    double g;
    sNode(int id_=-1, double g_=-1):id(id_),g(g_){}
    sNode(const Node &n)
    {
        id = n.id;
        g = n.g;
    }
};

struct sPath
{
    std::vector<sNode> nodes;
    double cost;
    int agentID;
    int expanded;
    sPath(std::vector<sNode> _nodes = std::vector<sNode>(0), double _cost = -1, int _agentID = -1)
        : nodes(_nodes), cost(_cost), agentID(_agentID) {expanded = 0;}
    sPath operator= (const Path &path)
    {
        cost = path.cost;
        agentID = path.agentID;
        expanded = path.expanded;
        nodes.clear();
        for(auto n:path.nodes)
            nodes.push_back(sNode(n));
        return *this;
    }
};

struct Constraint
{
    int agent;
    double t1, t2; // prohibited to start moving from (i1, j1) to (i2, j2) during interval (t1, t2)
    //double i1, j1, i2, j2; // in case of node constraint i1==i2, j1==j2.
    int id1, id2;
    bool positive;
    Constraint(int _agent = -1, double _t1 = -1, double _t2 = -1, int _id1 = -1, int _id2 = -1, bool _positive = false)
        : agent(_agent), t1(_t1), t2(_t2), id1(_id1), id2(_id2), positive(_positive) {}
    friend std::ostream& operator <<(std::ostream& os, const Constraint& con)
    {
        os<<con.agent<<" "<<con.t1<<" "<<con.t2<<" \n";//<<con.i1<<" "<<con.j1<<" "<<con.i2<<" "<<con.j2<<"\n";
        return os;
    }
    bool operator <(const Constraint& other) const
    {
        return t1 < other.t1;
    }
};

struct Move
{
    double t1, t2; // t2 is required for wait action
    //double i1, j1, i2, j2; // in case of wait action i1==i2, j1==j2
    int id1, id2;
    Move(double _t1 = -1, double _t2 = -1, int _id1 = -1, int _id2 = -1)
        : t1(_t1), t2(_t2), id1(_id1), id2(_id2) {}
    Move(const Move& move) : t1(move.t1), t2(move.t2), id1(move.id1), id2(move.id2) {}
    Move(const Constraint& con) : t1(con.t1), t2(con.t2), id1(con.id1), id2(con.id2) {}
    Move(Node a, Node b) : t1(a.g), t2(b.g), id1(a.id), id2(b.id) {}
    Move(sNode a, sNode b): t1(a.g), t2(b.g), id1(a.id), id2(b.id) {}
    bool operator <(const Move& other) const
    {
        if     (id1 < other.id1) return true;
        else if(id1 > other.id1) return false;
        else if(id2 < other.id2) return true;
        else return false;
    }
};

struct Step
{
    int i;
    int j;
    int id;
    double cost;
    //Step(const Node& node): i(node.i), j(node.j), id(node.id), cost(node.g) {}
    Step(int _i = 0, int _j = 0, int _id = 0, double _cost = -1.0): i(_i), j(_j), id(_id), cost(_cost) {}
};

struct Conflict
{
    int agent1, agent2;
    double t;
    Move move1, move2;
    double overcost;
    //sPath path1, path2;
    Conflict(int _agent1 = -1, int _agent2 = -1, Move _move1 = Move(), Move _move2 = Move(), double _t = CN_INFINITY)
        : agent1(_agent1), agent2(_agent2), t(_t), move1(_move1), move2(_move2) {overcost = 0;}
    bool operator < (const Conflict& other)
    {
        return this->overcost < other.overcost;
    }
};

struct CBS_Node
{
    std::vector<sPath> paths;
    CBS_Node* parent;
    Constraint constraint;
    Constraint positive_constraint;
    int id;
    std::string id_str;
    double cost;
    double h;
    unsigned int conflicts_num;
    unsigned int total_cons;
    unsigned int low_level_expanded;
    std::list<Conflict> conflicts;
    std::list<Conflict> semicard_conflicts;
    std::list<Conflict> cardinal_conflicts;
    CBS_Node(std::vector<sPath> _paths = {}, CBS_Node* _parent = nullptr, Constraint _constraint = Constraint(), double _cost = 0, int _conflicts_num = 0, int total_cons_ = 0)
        :paths(_paths), parent(_parent), constraint(_constraint), cost(_cost), conflicts_num(_conflicts_num), total_cons(total_cons_)
    {
        low_level_expanded = 0;
        h = 0;
        conflicts.clear();
        semicard_conflicts.clear();
        cardinal_conflicts.clear();
    }
    ~CBS_Node()
    {
        parent = nullptr;
        paths.clear();
        conflicts.clear();
        semicard_conflicts.clear();
        cardinal_conflicts.clear();
    }

};

struct Open_Elem
{
    CBS_Node* tree_pointer;
    int id;
    double cost;
    unsigned int cons_num;
    unsigned int conflicts_num;

    Open_Elem(CBS_Node* _tree_pointer = nullptr, int _id = -1, double _cost = -1, unsigned int _cons_num = 0, unsigned int _conflicts_num = 0)
        : tree_pointer(_tree_pointer), id(_id), cost(_cost), cons_num(_cons_num), conflicts_num(_conflicts_num) {}
    ~Open_Elem()
    {
        tree_pointer = nullptr;
    }
};

struct cost{};
struct id{};

typedef multi_index_container<
        Open_Elem,
        indexed_by<
                    //ordered_non_unique<tag<cost>, BOOST_MULTI_INDEX_MEMBER(Open_Elem, double, cost)>,
                    ordered_non_unique<composite_key<Open_Elem, BOOST_MULTI_INDEX_MEMBER(Open_Elem, double, cost), BOOST_MULTI_INDEX_MEMBER(Open_Elem, unsigned int, conflicts_num), BOOST_MULTI_INDEX_MEMBER(Open_Elem, unsigned int, cons_num)>,
                    composite_key_compare<std::less<double>, std::less<int>, std::greater<int>>>,
                    hashed_unique<tag<id>, BOOST_MULTI_INDEX_MEMBER(Open_Elem, int, id)>
        >
> CT_container;

struct Focal_Elem
{
    int id;
    unsigned int conflicts_num;
    unsigned int constraints;
    double cost;
    Focal_Elem(int id_=-1, unsigned int conflicts_num_ = 0, unsigned int constraints_ = 0, double cost_ = 0):id(id_), conflicts_num(conflicts_num_), constraints(constraints_), cost(cost_){}
    bool operator <(const Focal_Elem& other) const
    {
        if(this->conflicts_num < other.conflicts_num)
            return true;
        else if(this->conflicts_num > other.conflicts_num)
            return false;
        else if(this->constraints > other.constraints)
            return true;
        else if(this->constraints < other.constraints)
            return false;
        else if(this->cost < other.cost)
            return true;
        else
            return false;
    }
};
struct conflicts_num{};
struct constraints_num{};

typedef multi_index_container<
        Focal_Elem,
        indexed_by<
            ordered_non_unique<tag<conflicts_num>, identity<Focal_Elem>>,
            //ordered_non_unique<tag<constraints_num>, BOOST_MULTI_INDEX_MEMBER(Focal_Elem, int, constraints)>,
            hashed_unique<tag<id>, BOOST_MULTI_INDEX_MEMBER(Focal_Elem, int, id)>
        >
> Focal_container;

typedef multi_index_container<
        Node,
        indexed_by<
                    //ordered_non_unique<tag<cost>, BOOST_MULTI_INDEX_MEMBER(Open_Elem, double, cost)>,
                    ordered_non_unique<BOOST_MULTI_INDEX_MEMBER(Node, double, g)>,
                    hashed_non_unique<BOOST_MULTI_INDEX_MEMBER(Node, int, id)>
        >
> Open_List;

class CBS_Tree
{
    std::list<CBS_Node> tree;
    Focal_container focal;
    CT_container container;
    double focal_weight;
    int open_size;
    std::set<int> closed;
public:
    CBS_Tree() { open_size = 0; focal_weight = 1.0; }
    unsigned int get_size()
    {
        return tree.size();
    }

    void set_focal_weight(double weight)
    {
        focal_weight = weight;
    }

    int get_open_size()
    {
        return open_size;
    }

    void add_node(CBS_Node node)
    {
        tree.push_back(node);
        container.insert(Open_Elem(&tree.back(), node.id, node.cost, node.total_cons, node.conflicts_num));
        open_size++;
        if(focal_weight > 1.0)
            if(container.get<0>().begin()->cost*focal_weight > node.cost)
                focal.insert(Focal_Elem(node.id, node.conflicts_num, node.total_cons, node.cost));
    }

    CBS_Node* get_front()
    {
        open_size--;
        if(focal_weight > 1.0)
        {
            double cost = container.get<0>().begin()->cost;
            if(focal.empty())
            {
                update_focal(cost);
            }
            auto min = container.get<1>().find(focal.get<0>().begin()->id);
            focal.get<0>().erase(focal.get<0>().begin());
            auto pointer = min->tree_pointer;
            container.get<1>().erase(min);
            if(container.get<0>().begin()->cost > cost + CN_EPSILON)
                update_focal(cost);
            return pointer;
        }
        else
        {
            auto pointer = container.get<0>().begin()->tree_pointer;
            container.get<0>().erase(container.get<0>().begin());
            return pointer;
        }
    }

    void update_focal(double cost)
    {
        auto it0 = container.get<0>().begin();
        auto it1 = container.get<0>().upper_bound(cost*focal_weight + CN_EPSILON);
        for(auto it = it0; it != it1; it++)
            focal.insert(Focal_Elem(it->id, it->conflicts_num, it->cons_num, it->cost));
    }

    std::vector<sPath> get_paths(CBS_Node node, int size)
    {
        std::vector<sPath> paths(size);
        while(node.parent != nullptr)
        {
            if(paths.at(node.paths.begin()->agentID).nodes.empty())
                paths.at(node.paths.begin()->agentID) = *node.paths.begin();
            node = *node.parent;
        }
        for(unsigned int i = 0; i < node.paths.size(); i++)
            if(paths.at(i).nodes.empty())
                paths.at(i) = node.paths.at(i);
        return paths;
    }

};

struct Solution
{
    bool found;
    double flowtime;
    double makespan;
    double check_time;
    double init_cost;
    int constraints_num;
    int max_constraints;
    int high_level_expanded;
    int high_level_generated;
    int low_level_expansions;
    double low_level_expanded;
    int cardinal_solved;
    int semicardinal_solved;
    std::chrono::duration<double> time;
    std::chrono::duration<double> init_time;
    std::vector<sPath> paths;
    Solution(double _flowtime = -1, double _makespan = -1, std::vector<sPath> _paths = {})
        : flowtime(_flowtime), makespan(_makespan), paths(_paths) { init_cost = -1; constraints_num = 0; low_level_expanded = 0; low_level_expansions = 0; cardinal_solved = 0; semicardinal_solved = 0; max_constraints = 0;}
    ~Solution() { paths.clear(); found = false;}
};

class Vector2D {
  public:
    Vector2D(double _i = 0.0, double _j = 0.0):i(_i),j(_j){}
    double i, j;

    inline Vector2D operator +(const Vector2D &vec) { return Vector2D(i + vec.i, j + vec.j); }
    inline Vector2D operator -(const Vector2D &vec) { return Vector2D(i - vec.i, j - vec.j); }
    inline Vector2D operator -() { return Vector2D(-i,-j); }
    inline Vector2D operator /(const double &num) { return Vector2D(i/num, j/num); }
    inline Vector2D operator *(const double &num) { return Vector2D(i*num, j*num); }
    inline double operator *(const Vector2D &vec){ return i*vec.i + j*vec.j; }
    inline void operator +=(const Vector2D &vec) { i += vec.i; j += vec.j; }
    inline void operator -=(const Vector2D &vec) { i -= vec.i; j -= vec.j; }
};

class Point {
public:
    double i;
    double j;

    Point(double _i = 0.0, double _j = 0.0):i (_i), j (_j){}
    Point operator-(Point &p){return Point(i - p.i, j - p.j);}
    int operator== (Point &p){return (i == p.i) && (j == p.j);}
    int classify(Point &pO, Point &p1)
    {
        Point p2 = *this;
        Point a = p1 - pO;
        Point b = p2 - pO;
        double sa = a.i * b.j - b.i * a.j;
        if (sa > 0.0)
            return 1;//LEFT;
        if (sa < 0.0)
            return 2;//RIGHT;
        if ((a.i * b.i < 0.0) || (a.j * b.j < 0.0))
            return 3;//BEHIND;
        if ((a.i*a.i + a.j*a.j) < (b.i*b.i + b.j*b.j))
            return 4;//BEYOND;
        if (pO == p2)
            return 5;//ORIGIN;
        if (p1 == p2)
            return 6;//DESTINATION;
        return 7;//BETWEEN;
    }
};

#endif // STRUCTS_H
