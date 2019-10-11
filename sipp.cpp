#include "sipp.h"

void SIPP::clear()
{
    open.clear();
    close.clear();
    collision_intervals.clear();
    constraints.clear();
    path.cost = -1;
}

double SIPP::dist(const Node& a, const Node& b)
{
    return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2));
}

void SIPP::find_successors(Node curNode, const Map &map, std::list<Node> &succs, Heuristic &h_values)
{
    Node newNode;
    std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
    for(auto move : valid_moves)
    {
        newNode.i = move.i;
        newNode.j = move.j;
        newNode.id = move.id;
        double cost = dist(curNode, newNode);
        newNode.g = curNode.g + cost;
        std::vector<std::pair<double, double>> intervals(0);
        auto colls_it = collision_intervals.find(newNode.id);
        if(colls_it != collision_intervals.end())
        {
            std::pair<double, double> interval = {0, CN_INFINITY};
            for(unsigned int i = 0; i < colls_it->second.size(); i++)
            {
                interval.second = colls_it->second[i].first;
                intervals.push_back(interval);
                interval.first = colls_it->second[i].second;
            }
            interval.second = CN_INFINITY;
            intervals.push_back(interval);
        }
        else
            intervals.push_back({newNode.g, CN_INFINITY});
        auto cons_it = constraints.find({curNode.id, newNode.id});
        for(auto interval: intervals)
        {
            if(interval.second < newNode.g)
                continue;
            if(interval.first > newNode.g)
                newNode.g = interval.first;
            if(cons_it != constraints.end())
                for(unsigned int i = 0; i < cons_it->second.size(); i++)
                    if(newNode.g - cost + CN_EPSILON > cons_it->second[i].t1 && newNode.g - cost < cons_it->second[i].t2)
                        newNode.g = cons_it->second[i].t2 + cost;
            newNode.interval = interval;
            if(newNode.g - cost > curNode.interval.second || newNode.g > newNode.interval.second)
                continue;
            newNode.f = newNode.g + h_values.get_value(newNode.id, agent.id);
            succs.push_back(newNode);
        }
    }
}

Node SIPP::find_min()
{
    Node min = *open.begin();
    open.pop_front();
    return min;
}

void SIPP::add_open(Node newNode)
{
    std::list<Node>::iterator iter, pos;
    bool pos_found = false;
    if (open.empty())
    {
        open.push_back(newNode);
        return;
    }
    for(iter = open.begin(); iter != open.end(); ++iter)
    {
        if (!pos_found)
        {
            if(iter->f > newNode.f + CN_EPSILON) // if newNode.f has lower f-value
            {
                pos = iter;
                pos_found = true;
            }
            else if(fabs(iter->f - newNode.f) < CN_EPSILON && newNode.g + CN_EPSILON > iter->g) // if f-values are equal, compare g-values
            {
                pos = iter;
                pos_found = true;
            }
        }
        if (iter->id == newNode.id && fabs(iter->interval.second - newNode.interval.second) < CN_EPSILON)
        {
            if(newNode.f > iter->f - CN_EPSILON)
                return;
            if(pos == iter)
            {
                iter->f = newNode.f;
                iter->g = newNode.g;
                iter->interval = newNode.interval;
                iter->parent = newNode.parent;
                return;
            }
            open.erase(iter);
            break;
        }
    }
    if(pos_found)
        open.insert(pos, newNode);
    else
        open.push_back(newNode);
    return;
}

void SIPP::reconstruct_path(Node curNode)
{
    path.nodes.clear();
    if(curNode.parent != nullptr)
    do
    {
        path.nodes.insert(path.nodes.begin(), curNode);
        curNode = *curNode.parent;
    }
    while(curNode.parent != nullptr);
    path.nodes.insert(path.nodes.begin(), curNode);
    for(unsigned int i = 0; i < path.nodes.size(); i++)
    {
        unsigned int j = i + 1;
        if(j == path.nodes.size())
            break;
        if(fabs(path.nodes[j].g - path.nodes[i].g - dist(path.nodes[j], path.nodes[i])) > CN_EPSILON)
        {
            Node add = path.nodes[i];
            add.g = path.nodes[j].g - dist(path.nodes[j], path.nodes[i]);
            path.nodes.emplace(path.nodes.begin() + j, add);
        }
    }
}

void SIPP::make_constraints(std::list<Constraint> &cons)
{
    for(auto con : cons)
    {
        if(con.id1 == con.id2) // wait consatraint
        {
            std::pair<double, double> interval = {con.t1, con.t2};
            std::vector<std::pair<double, double>> intervals(0);
            if(collision_intervals.count(con.id1) == 0)
                collision_intervals.insert({con.id1, {interval}});
            else
            {
                intervals = collision_intervals.at(con.id1);
                bool inserted(false);
                for(unsigned int i = 0; i < intervals.size(); i++)
                {
                    if(inserted)
                        break;
                    if(interval.first < intervals[i].first + CN_EPSILON)
                    {
                        if(interval.second + CN_EPSILON > intervals[i].first)
                        {
                            intervals[i].first = interval.first;
                            if(interval.second + CN_EPSILON > intervals[i].second)
                                intervals[i].second = interval.second;
                            inserted = true;
                            if(i != 0)
                                if(intervals[i-1].second + CN_EPSILON > interval.first && intervals[i-1].second < interval.second + CN_EPSILON)
                                {
                                    intervals[i-1].second = interval.second;
                                    if(intervals[i-1].second < intervals[i].second + CN_EPSILON)
                                    {
                                        intervals[i-1].second = intervals[i].second;
                                        intervals.erase(intervals.begin() + i);
                                    }
                                    inserted = true;
                                }
                        }
                        else
                        {
                            if(i != 0)
                                if(intervals[i-1].second + CN_EPSILON > interval.first && intervals[i-1].second < interval.second + CN_EPSILON)
                                {
                                    intervals[i-1].second = interval.second;
                                    inserted = true;
                                    break;
                                }
                            intervals.insert(intervals.begin() + i, interval);
                            inserted = true;
                        }
                    }
                }
                if(intervals.back().second + CN_EPSILON > interval.first && intervals.back().second < interval.second + CN_EPSILON)
                    intervals.back().second = interval.second;
                else if(!inserted)
                    intervals.push_back(interval);
                collision_intervals.at(con.id1) = intervals;
            }
        }
        else //move constraint
        {
            Move move(con);
            std::vector<Move> m_cons(0);
            if(constraints.count({move.id1, move.id2}) == 0)
                constraints.insert({{move.id1, move.id2}, {move}});
            else
            {
                m_cons = constraints.at({move.id1, move.id2});
                bool inserted(false);
                for(unsigned int i = 0; i < m_cons.size(); i++)
                {
                    if(inserted)
                        break;
                    if(m_cons[i].t1 > move.t1)
                    {
                        if(m_cons[i].t1 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i].t1 = move.t1;
                            if(move.t2 + CN_EPSILON > m_cons[i].t2)
                                m_cons[i].t2 = move.t2;
                            inserted = true;
                            if(i != 0)
                                if(m_cons[i-1].t2 + CN_EPSILON > move.t1 && m_cons[i-1].t2 < move.t2 + CN_EPSILON)
                                {
                                    m_cons[i-1].t2 = move.t2;
                                    if(m_cons[i-1].t2 + CN_EPSILON > m_cons[i].t1 && m_cons[i-1].t2 < m_cons[i].t2 + CN_EPSILON)
                                    {
                                        m_cons[i-1].t2 = m_cons[i].t2;
                                        m_cons.erase(m_cons.begin() + i);
                                    }
                                    inserted = true;
                                }
                        }
                        else
                        {
                            if(i != 0)
                                if(m_cons[i-1].t2 + CN_EPSILON > move.t1 && m_cons[i-1].t2 < move.t2 + CN_EPSILON)
                                {
                                    m_cons[i-1].t2 = move.t2;
                                    inserted = true;
                                    break;
                                }
                            m_cons.insert(m_cons.begin() + i, move);
                            inserted = true;
                        }
                    }
                }
                if(m_cons.back().t2 + CN_EPSILON > move.t1 && m_cons.back().t2 < move.t2 + CN_EPSILON)
                    m_cons.back().t2 = move.t2;
                else if(!inserted)
                    m_cons.push_back(move);
                constraints.at({move.id1, move.id2}) = m_cons;
            }
        }
    }
}

Path SIPP::find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values)
{
    this->clear();
    make_constraints(cons);
    this->agent = agent;
    Node curNode(agent.start_id, 0, 0, agent.start_i, agent.start_j);
    if(collision_intervals.count(curNode.id) > 0)
    {
        auto intervals = collision_intervals.at(curNode.id);
        curNode.interval = {0, intervals[0].first};
    }
    else
        curNode.interval = {0, CN_INFINITY};

    bool pathFound = false;
    open.push_back(curNode);
    while(!open.empty())
    {
        curNode = find_min();
        close.insert({curNode.id, curNode});
        if(curNode.id == agent.goal_id && curNode.interval.second == CN_INFINITY)
        {
            pathFound = true;
            break;
        }
        std::list<Node> succs;
        succs.clear();
        find_successors(curNode, map, succs, h_values);
        std::list<Node>::iterator it = succs.begin();
        auto parent = &(close.find(curNode.id)->second);
        while(it != succs.end())
        {
            bool has = false;
            it->parent = parent;
            auto range = close.equal_range(it->id);
            for(auto i = range.first; i != range.second; i++)
                if(i->second.interval.first - CN_EPSILON < it->interval.first && i->second.interval.second + CN_EPSILON > it->interval.second)
                {
                    has = true;
                    break;
                }
            if(!has)
                add_open(*it);
            it++;
        }
    }
    if (pathFound)
    {
        reconstruct_path(curNode);
        path.cost = curNode.g;
    }
    path.agentID = agent.id;
    path.expanded = close.size();
    return path;
}
