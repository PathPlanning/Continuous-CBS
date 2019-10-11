#include "heuristic.h"

void Heuristic::init(int size, int agents)
{
    h_values.clear();
    h_values.resize(size);
    for(int i = 0; i < size; i++)
        h_values[i].resize(agents, -1);
}

void Heuristic::count(const Map &map, Agent agent)
{
    open.clear();
    Node curNode(agent.goal_id, 0, 0, agent.goal_i, agent.goal_j), newNode;
    add_open(curNode);
    while(!open.empty())
    {
        do curNode = find_min();
        while(h_values[curNode.id][agent.id] >= 0 && open.size() > 0);
        if(h_values[curNode.id][agent.id] <= 0)
            h_values[curNode.id][agent.id] = curNode.g;
        else
            break;
        std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
        for(auto move: valid_moves)
        {
            newNode.i = move.i;
            newNode.j = move.j;
            newNode.id = move.id;
            newNode.g = curNode.g + dist(curNode, newNode);
            if(h_values[newNode.id][agent.id] < 0)
                add_open(newNode);
        }
    }

}

void Heuristic::add_open(Node newNode)
{
    for(auto iter = open.begin(); iter != open.end(); iter++)
    {
        if (iter->g + CN_EPSILON > newNode.g)
        {
            open.insert(iter, newNode);
            return;
        }
        if (iter->id == newNode.id)
            return;
    }
    open.push_back(newNode);
    return;
}

Node Heuristic::find_min()
{
    Node min = *open.begin();
    open.pop_front();
    return min;
}

