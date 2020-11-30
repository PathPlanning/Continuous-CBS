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

double SIPP::getRCost(double headingA, double headingB)
{
    if(planforturns)
        return std::min(360 - fabs(headingA - headingB), fabs(headingA - headingB))/(agent.rspeed*180.0);
    else
        return 0;
}

double SIPP::calcHeading(const Node &node, const Node &son)
{
    double heading = acos((son.j - node.j)/dist(node,son))*180/M_PI;
    if(node.i < son.i)
        heading = 360 - heading;
    return heading;
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
        newNode.heading = calcHeading(curNode, newNode);
        Node angleNode = curNode; //the same state, but with extended g-value
        angleNode.g += getRCost(angleNode.heading, newNode.heading);
        if(angleNode.g > angleNode.interval.second - CN_EPSILON)
            continue;
        double cost = dist(curNode, newNode)/agent.mspeed;
        newNode.g = angleNode.g + cost;
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
            {
                for(unsigned int i = 0; i < cons_it->second.size(); i++)
                    if(newNode.g - cost + CN_EPSILON > cons_it->second[i].t1 && newNode.g - cost < cons_it->second[i].t2)
                        newNode.g = cons_it->second[i].t2 + cost;
            }
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
    Node min = *open.get<0>().begin();
    open.get<0>().erase(open.get<0>().begin());
    return min;
}

void SIPP::add_open(Node newNode)
{
    auto range = open.get<1>().equal_range(newNode.id);
    auto it = range.first;
    bool dominated = false;
    while(it != range.second)
    {
        if(fabs(it->interval.second - newNode.interval.second) > CN_EPSILON)
        {
            it++;
            continue;
        }
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

void SIPP::reconstruct_path(Node curNode)
{
    Node current = curNode;
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
        if(fabs(path.nodes[j].g - path.nodes[i].g - dist(path.nodes[j], path.nodes[i])/agent.mspeed) > CN_EPSILON)
        {
            Node add = path.nodes[i];
            add.heading = path.nodes[j].heading;
            add.g = path.nodes[j].g - dist(path.nodes[j], path.nodes[i])/agent.mspeed;
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

bool SIPP::stopCriterion(const Node &curNode, Node &goalNode, bool pft)
{
    if(curNode.id == agent.goal_id && fabs(curNode.interval.second - CN_INFINITY) < CN_EPSILON)
    {
        if(!pft || agent.goal_heading == CN_HEADING_WHATEVER)
            goalNode = curNode;
        else if(goalNode.g > curNode.g + getRCost(curNode.heading, agent.goal_heading))
        {
            goalNode = curNode;
            goalNode.g = curNode.g + getRCost(curNode.heading, agent.goal_heading);
            goalNode.f = curNode.f + getRCost(curNode.heading, agent.goal_heading);
        }
    }
    if(open.size() == 0)
    {
        std::cout << "OPEN list is empty! "<<close.size();
        return true;
    }
    if(goalNode.f - CN_EPSILON < curNode.f)
        return true;
    return false;
}

Path SIPP::find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values, bool pft)
{
    this->clear();
    planforturns = pft;
    make_constraints(cons);
    this->agent = agent;
    Node curNode(agent.start_id, 0, 0, agent.start_i, agent.start_j), goalNode(agent.goal_id, CN_INFINITY, CN_INFINITY);
    curNode.heading = agent.start_heading;
    if(collision_intervals.count(curNode.id) > 0)
    {
        auto intervals = collision_intervals.at(curNode.id);
        curNode.interval = {0, intervals[0].first};
    }
    else
        curNode.interval = {0, CN_INFINITY};

    bool pathFound = false;
    add_open(curNode);
    while(!stopCriterion(curNode, goalNode, pft))
    {
        curNode = find_min();
        close.insert({curNode.id, curNode});
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
                if(i->second.interval.first - CN_EPSILON < it->interval.first && i->second.interval.second + CN_EPSILON > it->interval.second && i->second.g + getRCost(i->second.heading, it->heading) <= it->g)
                {
                    has = true;
                    break;
                }

            if(!has)
                add_open(*it);
            it++;
        }
    }
    if (goalNode.f < CN_INFINITY)
    {
        reconstruct_path(curNode);
        path.cost = curNode.g;
    }
    path.agentID = agent.id;
    path.expanded = close.size();
    return path;
}
