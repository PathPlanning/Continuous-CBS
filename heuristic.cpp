#include "heuristic.h"

void Heuristic::init(int size, int agents, bool pft)
{
    h_values.clear();
    h_values.resize(size);
    for(int i = 0; i < size; i++)
        h_values[i].resize(agents, -1);
    planforturns = pft;
}

double Heuristic::getRCost(double headingA, double headingB)
{
    if(planforturns)
        return std::min(360 - fabs(headingA - headingB), fabs(headingA - headingB))/(agent.rspeed*180.0);
    else
        return 0;
}

double Heuristic::calcHeading(const Node &node, const Node &son)
{
    double heading = acos((son.j - node.j)/dist(node,son))*180/M_PI;
    if(node.i < son.i)
        heading = 360 - heading;
    return heading;
}

void Heuristic::add_open(Node newNode)
{
    auto range = open.get<1>().equal_range(newNode.id);
    auto it = range.first;
    bool dominated = false;
    while(it != range.second)
    {
        if((it->g - newNode.g + getRCost(it->heading, newNode.heading)) < CN_EPSILON)//if existing state dominates new one
        {
            dominated = true;
        }
        else if((newNode.g + getRCost(it->heading, newNode.heading) - it->g) < CN_EPSILON)//if new state dominates the existing one
        {
            auto iter(it);
            it++;
            open.get<1>().erase(iter);
            continue;
        }
        it++;
    }
    if(!dominated)
        open.insert(newNode);
    return;
}

void Heuristic::count(const Map &map, Agent agent)
{
    Node curNode(agent.goal_id, 0, 0, agent.goal_i, agent.goal_j), newNode;
    curNode.heading = agent.goal_heading;
    this->agent = agent;
    open.clear();
    open.insert(curNode);
    int k=0;
    while(!open.empty())
    {
        curNode = find_min();
        if(h_values[curNode.id][agent.id] < 0 || h_values[curNode.id][agent.id] > curNode.g)
            h_values[curNode.id][agent.id] = curNode.g;
        k++;
        std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
        for(auto move: valid_moves)
        {
            newNode.i = move.i;
            newNode.j = move.j;
            newNode.id = move.id;
            newNode.heading = calcHeading(curNode, newNode);
            if(curNode.i == agent.goal_i && curNode.j == agent.goal_j)
                newNode.g = curNode.g + dist(curNode, newNode)/agent.mspeed;
            else
                newNode.g = curNode.g + getRCost(curNode.heading, newNode.heading) + dist(curNode, newNode)/agent.mspeed;
            if(h_values[newNode.id][agent.id] < 0)
                add_open(newNode);
        }
    }

}

Node Heuristic::find_min()
{
    Node min = *open.get<0>().begin();
    open.get<0>().erase(open.get<0>().begin());
    return min;
}

