#include "heuristic.h"

void Heuristic::init(int size, int agents)
{
    h_values.clear();
    h_values.resize(size);
    for(int i = 0; i < size; i++)
        h_values[i].resize(agents, -1);
}

void Heuristic::count(const Graph &map, Agent agent)
{
    open.clear();
    openSize = 0;

    Node curNode(map.get_gnode(agent.goal_id), 0, 0), newNode;
    curNode.id = agent.goal_id;
    add_open(curNode);
    int k=0;
    while(openSize > 0)
    {
        do curNode = find_min();
        while(h_values[curNode.id][agent.id] >= 0 && openSize > 0);
        k++;
        if(h_values[curNode.id][agent.id] < 0)
            h_values[curNode.id][agent.id] = curNode.g;
        std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
        for(auto move: valid_moves)
        {
            newNode.i = move.i;
            newNode.j = move.j;
            newNode.id = move.id;
            newNode.g = curNode.g + dist(curNode, move);
            if(h_values[newNode.id][agent.id] < 0)
                add_open(newNode);
        }
    }
    if(k<10)
        for(int i=0; i<map.get_nodes_size(); i++)
            if(h_values[i][agent.id]>=0)
                std::cout<<i<<" ";

}

void Heuristic::add_open(Node newNode)
{
    for(auto iter = open.begin(); iter != open.end(); ++iter)
    {
        if (iter->g + CN_EPSILON > newNode.g)
        {
            openSize++;
            open.insert(iter, newNode);
            return;
        }
        if (iter->j == newNode.j)
            return;
    }
    openSize++;
    open.push_back(newNode);
    return;
}

Node Heuristic::find_min()
{
    Node min;
    min = *open.begin();
    open.pop_front();
    openSize--;
    return min;
}

