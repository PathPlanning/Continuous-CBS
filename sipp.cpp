#include "sipp.h"

void SIPP::clear()
{
    open.clear();
    close.clear();
    collision_intervals.clear();
    constraints.clear();
    openSize = 0;
    path.cost = -1;
}

double SIPP::dist(const Node& a, const Node& b)
{
    return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2));
}

void SIPP::find_successors(Node curNode, const Map &map, std::list<Node> &succs, Heuristic &h_values)
{
    Node newNode;
    std::vector<Step> valid_moves = map.get_valid_moves(curNode.i, curNode.j);
    for(auto move : valid_moves)
    {
        newNode.i = curNode.i + move.i;
        newNode.j = curNode.j + move.j;
        newNode.g = curNode.g + move.cost;
        std::vector<std::pair<double, double>> intervals(0);
        auto colls_it = collision_intervals.find(std::make_pair(newNode.i, newNode.j));
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
        auto cons_it = constraints.find(Move(curNode.g, newNode.g, curNode.i, curNode.j, newNode.i, newNode.j));
        for(auto interval: intervals)
        {
            if(interval.second < newNode.g)
                continue;
            if(interval.first > newNode.g)
                newNode.g = interval.first;
            if(cons_it != constraints.end())
                for(unsigned int i = 0; i < cons_it->second.size(); i++)
                    if(newNode.g - move.cost + CN_EPSILON > cons_it->second[i].t1 && newNode.g - move.cost < cons_it->second[i].t2)
                        newNode.g = cons_it->second[i].t2 + move.cost;
            newNode.interval = interval;
            if(newNode.g - move.cost > curNode.interval.second || newNode.g > newNode.interval.second)
                continue;
            newNode.f = newNode.g + h_values.get_value(newNode.i, newNode.j, agent.id);
            succs.push_back(newNode);
        }
    }
}

Node SIPP::find_min(int size)
{
    Node min;
    min.f = CN_INFINITY;
    for(int i = 0; i < size; i++)
    {
        if(!open[i].empty())
        {
            if(open[i].begin()->f + CN_EPSILON < min.f) // if min.f has higher value
                min = *open[i].begin();
            else if(fabs(open[i].begin()->f - min.f) < CN_EPSILON && open[i].begin()->g + CN_EPSILON > min.g) // if f-values are equal, compare g-values
                min = *open[i].begin();
        }
    }
    open[min.i].pop_front();
    openSize--;
    return min;
}

void SIPP::add_open(Node newNode)
{
    std::list<Node>::iterator iter, pos;
    bool pos_found = false;
    pos = open[newNode.i].end();
    if (open[newNode.i].size() == 0)
    {
        open[newNode.i].push_back(newNode);
        openSize++;
        return;
    }
    for(iter = open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter)
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
        if (iter->j == newNode.j && fabs(iter->interval.second - newNode.interval.second) < CN_EPSILON)
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
            open[newNode.i].erase(iter);
            openSize--;
            break;
        }
    }
    openSize++;
    open[newNode.i].insert(pos, newNode);
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
    for(int i = 0; i < path.nodes.size(); i++)
    {
        int j = i + 1;
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
        if(con.i1 == con.i2 && con.j1 == con.j2)
        {
            std::pair<double, double> interval = {con.t1, con.t2};
            std::vector<std::pair<double, double>> intervals(0);
            if(collision_intervals.count(std::make_pair(con.i1, con.j1)) == 0)
                collision_intervals.insert({std::make_pair(con.i1, con.j1), {interval}});
            else
            {
                intervals = collision_intervals.at(std::make_pair(con.i1, con.j1));
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
                collision_intervals.at(std::make_pair(con.i1, con.j1)) = intervals;
            }
        }
        else
        {
            Move move(con);
            std::vector<Move> m_cons(0);
            if(constraints.count(move) == 0)
                constraints.insert({move, {move}});
            else
            {
                m_cons = constraints.at(move);
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
                constraints.at(move) = m_cons;
            }
        }
    }
}

Path SIPP::find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values)
{
    this->clear();
    make_constraints(cons);
    this->agent = agent;
    open.resize(map.get_height());
    Node curNode(agent.start_i, agent.start_j, 0, 0);
    curNode.g = 0;
    if(collision_intervals.count(std::make_pair(curNode.i, curNode.j)) > 0)
    {
        auto intervals = collision_intervals.at(std::make_pair(curNode.i, curNode.j));
        curNode.interval = {0, intervals[0].first};
    }
    else
        curNode.interval = {0, CN_INFINITY};

    bool pathFound = false;
    open[curNode.i].push_back(curNode);
    openSize++;
    while(openSize > 0)
    {
        curNode = find_min(map.get_height());
        close.insert({curNode.i * map.get_width() + curNode.j, curNode});
        if(curNode.i == agent.goal_i && curNode.j == agent.goal_j && curNode.interval.second == CN_INFINITY)
        {
            pathFound = true;
            break;
        }
        std::list<Node> succs;
        succs.clear();
        find_successors(curNode, map, succs, h_values);
        std::list<Node>::iterator it = succs.begin();
        auto parent = &(close.find(curNode.i * map.get_width() + curNode.j)->second);
        while(it != succs.end())
        {
            bool has = false;
            it->parent = parent;
            auto range = close.equal_range(it->i * map.get_width() + it->j);
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
