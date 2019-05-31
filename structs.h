#ifndef STRUCTS_H
#define STRUCTS_H
#include <math.h>
#include <vector>
#include <list>
#include <iostream>
#include <chrono>
#include "const.h"

struct Agent
{
    int start_id, goal_id, id;
    double size;
    Agent(int s_id = -1, int g_id = -1, int _id = -1)
        :start_id(s_id), goal_id(g_id), id(_id) {}
};

struct gNode
{
    double i;
    double j;
    std::vector<int> neighbors;
    gNode(double i_ = -1, double j_ = -1):i(i_), j(j_) { neighbors.clear(); }
    ~gNode() { neighbors.clear(); }
};

struct Node
{
    double  i, j;
    double  f, g;
    int id;
    Node*   parent;
    std::pair<double, double> interval;

    Node(double _i = -1, double _j = -1, double _f = -1, double _g = -1, Node* _parent = nullptr, double begin = -1, double end = -1)
        :i(_i), j(_j), f(_f), g(_g), parent(_parent), interval(std::make_pair(begin, end)) {}
    Node(const gNode &gnode, double _f = -1, double _g = -1, Node* _parent = nullptr, double begin = -1, double end = -1)
    :i(gnode.i), j(gnode.j), f(_f), g(_g), parent(_parent), interval(std::make_pair(begin, end)) {}
    ~Node() { parent = nullptr; }
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
        : nodes(_nodes), cost(_cost), agentID(_agentID) {}
};

struct Constraint
{
    int agent;
    double t1, t2; // prohibited to start moving from (i1, j1) to (i2, j2) during interval (t1, t2)
    double i1, j1, i2, j2; // in case of node constraint i1==i2, j1==j2.
    int id1, id2;
    int num;
    int agent2, m2id1, m2id2;
    double m2t1, m2t2, m2i1, m2j1, m2i2, m2j2;
    Constraint(int _agent = -1, double _t1 = -1, double _t2 = -1, double _i1 = -1, double _j1 = -1, double _i2 = -1, double _j2 = -1, int _id1 = -1, int _id2 = -1)
        : agent(_agent), t1(_t1), t2(_t2), i1(_i1), j1(_j1), i2(_i2), j2(_j2), id1(_id1), id2(_id2){}
    friend std::ostream& operator <<(std::ostream& os, const Constraint& con)
    {
        os<<con.agent<<" "<<con.t1<<" "<<con.t2<<" "<<con.i1<<" "<<con.j1<<" "<<con.i2<<" "<<con.j2<<"\n";
        return os;
    }
};

struct Move
{
    double t1, t2; // t2 is required for wait action
    double i1, j1, i2, j2; // in case of wait action i1==i2, j1==j2
    int id1, id2;
    Move(double _t1 = -1, double _t2 = -1, double _i1 = -1, double _j1 = -1, double _i2 = -1, double _j2 = -1, int _id1 = -1, int _id2 = -1)
        : t1(_t1), t2(_t2), i1(_i1), j1(_j1), i2(_i2), j2(_j2), id1(_id1), id2(_id2) {}
    Move(const Move& move) : t1(move.t1), t2(move.t2), i1(move.i1), j1(move.j1), i2(move.i2), j2(move.j2), id1(move.id1), id2(move.id2) {}
    Move(const Constraint& con) : t1(con.t1), t2(con.t2), i1(con.i1), j1(con.j1), i2(con.i2), j2(con.j2), id1(con.id1), id2(con.id2) {}
    Move(Node a, Node b) : t1(a.g), t2(b.g), i1(a.i), j1(a.j), i2(b.i), j2(b.j), id1(a.id), id2(b.id) {}
    bool operator <(const Move& other) const
    {
        if     (id1 < other.id1) return true;
        else if(id1 == other.id1 && id2 < other.id2) return true;
        else return false;
    }
};

struct Step
{
    int i;
    int j;
    double cost;
    Step(const Node& node): i(node.i), j(node.j), cost(node.g) {}
    Step(int _i = 0, int _j = 0, double _cost = -1.0): i(_i), j(_j), cost(_cost) {}
};

struct Conflict
{
    int agent1, agent2;
    double t;
    Move move1, move2;
    Conflict(int _agent1 = -1, int _agent2 = -1, Move _move1 = Move(), Move _move2 = Move(), double _t = -1)
        : agent1(_agent1), agent2(_agent2), t(_t), move1(_move1), move2(_move2) {}
};

struct CBS_Node
{
    std::vector<Path> paths;
    CBS_Node* parent;
    Constraint constraint;
    Constraint altconstraint;
    long long int id;
    double cost;
    int cons_num;
    int conflicts_num;
    bool look_for_cardinal;
    int low_level_expanded;
    CBS_Node(std::vector<Path> _paths = {}, CBS_Node* _parent = nullptr, Constraint _constraint = Constraint(), double _cost = 0, int _cons_num = 0, int _conflicts_num = 0, bool _look_for_cardinal = true)
        :paths(_paths), parent(_parent), constraint(_constraint), cost(_cost), cons_num(_cons_num), conflicts_num(_conflicts_num), look_for_cardinal(_look_for_cardinal)
    {
        low_level_expanded = 0;
    }
    ~CBS_Node()
    {
        parent = nullptr;
        paths.clear();
    }

};

struct Open_Elem
{
    CBS_Node* tree_pointer;
    double cost;
    int cons_num;
    int conflicts_num;

    Open_Elem(CBS_Node* _tree_pointer = nullptr, double _cost = -1, int _cons_num = -1, int _conflicts_num = -1)
        : tree_pointer(_tree_pointer), cost(_cost), cons_num(_cons_num), conflicts_num(_conflicts_num) {}
    ~Open_Elem()
    {
        tree_pointer = nullptr;
    }
};

class CBS_Tree
{
    std::list<CBS_Node> tree;
    std::vector<std::list<Open_Elem>> open;
    int open_size;
public:
    CBS_Tree() { open_size = 0; open.clear(); }
    int get_size()
    {
        return tree.size();
    }

    int get_open_size()
    {
        return open_size;
    }

    void add_node(CBS_Node node)
    {
        tree.push_back(node);
        bool inserted = false;
        for(int k = 0; k < open.size(); k++)
        {
            if(open[k].begin()->cost + CN_EPSILON > node.cost)
            {
                if(fabs(open[k].begin()->cost - node.cost) < CN_EPSILON)
                {
                    open[k].push_back(Open_Elem(&tree.back(), node.cost, node.cons_num, node.conflicts_num));
                    inserted = true;
                    break;
                }
                else
                {
                    std::list<Open_Elem> open_elem = {Open_Elem(&tree.back(), node.cost, node.cons_num, node.conflicts_num)};
                    open.insert(open.begin() + k, open_elem);
                    inserted = true;
                    break;
                }
            }
        }
        if(!inserted)
        {
            std::list<Open_Elem> open_elem = {Open_Elem(&tree.back(), node.cost, node.cons_num, node.conflicts_num)};
            open.push_back(open_elem);
        }
        open_size++;
        /*for(auto it = open.begin(); it != open.end(); it++)
        {
            if(it->cost > node.cost || (it->cost == node.cost && node.cons_num < it->cons_num))
            {
                open.emplace(it, Open_Elem(&tree.back(), node.cost, node.cons_num, node.conflicts_num));
                inserted = true;
                break;
            }
        }
        if(!inserted)
            open.emplace_back(Open_Elem(&tree.back(), node.cost, node.cons_num));*/
    }

    CBS_Node* get_front()
    {
        return open[0].begin()->tree_pointer;
    }

    void pop_front()
    {
        open[0].pop_front();
        if(open[0].empty())
            open.erase(open.begin());
        open_size--;
        return;
    }

    std::vector<Path> get_paths(CBS_Node node, int size)
    {
        std::vector<Path> paths(size);
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
    double flowtime;
    double makespan;
    double check_time;
    double init_cost;
    int constraints_num;
    int high_level_expanded;
    int low_level_expansions;
    double low_level_expanded;
    std::chrono::duration<double> time;
    std::chrono::duration<double> init_time;
    std::vector<Path> paths;
    Solution(double _flowtime = -1, double _makespan = -1, std::vector<Path> _paths = {})
        : flowtime(_flowtime), makespan(_makespan), paths(_paths) { init_cost = -1; constraints_num = 0; }
    ~Solution() { paths.clear(); }
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
