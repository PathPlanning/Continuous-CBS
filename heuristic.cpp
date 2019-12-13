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
    Node curNode(agent.goal_id, 0, 0, agent.goal_i, agent.goal_j), newNode;
    open.clear();
    open.insert(curNode);
    while(!open.empty())
    {
        curNode = find_min();
        h_values[curNode.id][agent.id] = curNode.g;
        std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
        for(auto move: valid_moves)
        {
            newNode.i = move.i;
            newNode.j = move.j;
            newNode.id = move.id;
            newNode.g = curNode.g + dist(curNode, newNode);
            if(h_values[newNode.id][agent.id] < 0)
            {
                auto it = open.get<1>().find(newNode.id);
                if(it != open.get<1>().end())
                {
                    if(it->g > newNode.g)
                        open.get<1>().erase(it);
                    else
                        continue;
                }
                open.insert(newNode);
            }
        }
    }

}

Node Heuristic::find_min()
{
    Node min = *open.begin();
    open.erase(open.begin());
    return min;
}

