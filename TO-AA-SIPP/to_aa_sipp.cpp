#include "to_aa_sipp.h"

TO_AA_SIPP::TO_AA_SIPP()
{
    closeSize = 0;
    openSize = 0;
}

TO_AA_SIPP::~TO_AA_SIPP()
{
}

double TO_AA_SIPP::calculateDistanceFromCellToCell(double start_i, double start_j, double fin_i, double fin_j)
{
    return sqrt(double((start_i - fin_i)*(start_i - fin_i) + (start_j - fin_j)*(start_j - fin_j)));
}

void TO_AA_SIPP::initStates(Agent agent, const Map &Map, const std::list<Constraint> &constraints)
{
    states.map = &Map;
    double g, h = h_values.get_value(agent.start_i, agent.start_j);
    oNode start(agent.start_i, agent.start_j,0,h);
    make_constraints(constraints);
    start.h = h;
    start.expanded = true;
    start.best_g = 0;
    start.consistent = 1;
    start.interval.begin = 0;
    start.interval.end = CN_INFINITY;
    start.id = start.i*Map.get_width() + start.j;
    if(collision_intervals.find(start.id) != collision_intervals.end())
        start.interval.end = collision_intervals[start.id][0].first;
    start.interval.id = 0;
    start.interval_id = 0;
    states.insert(start);
    auto parent = states.getParentPtr();
    for(int i = 0; i < Map.get_height(); i++)
        for(int j = 0; j < Map.get_width(); j++)
        {
            if(Map.cell_is_obstacle(i,j))
                continue;
            std::vector<std::pair<double, double>> intervals(0);
            auto colls_it = collision_intervals.find(i*Map.get_width() + j);
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
                intervals.push_back({0, CN_INFINITY});

            g = calculateDistanceFromCellToCell(i, j, agent.start_i, agent.start_j);
            h = h_values.get_value(i,j);
            oNode n = oNode(i,j,g,g+h);
            n.h = h;
            n.Parent = parent;

            for(int k = 0; k < intervals.size(); k++)
            {
                if(i == agent.start_i && j == agent.start_j && k==0)
                    continue;
                n.interval.begin = intervals[k].first;
                n.interval.end = intervals[k].second;
                n.interval.id = k;
                n.interval_id = k;
                n.id = n.i*Map.get_width() + n.j;
                if(n.interval.begin > n.g)
                {
                    n.g = n.interval.begin;
                    n.F = n.g + n.h;
                }
                n.parents.clear();
                n.parents.push_back({parent, n.g});
                if(n.g <= n.interval.end)
                {
                    states.insert(n);
                }
            }
        }
    states.expand(start);
}

void TO_AA_SIPP::add_collision_interval(int id, std::pair<double, double> interval)
{
    std::vector<std::pair<double, double>> intervals(0);
    if(collision_intervals.count(id) == 0)
        collision_intervals.insert({id, {interval}});
    else
        collision_intervals[id].push_back(interval);
    std::sort(collision_intervals[id].begin(), collision_intervals[id].end());
    for(unsigned int i = 0; i + 1 < collision_intervals[id].size(); i++)
        if(collision_intervals[id][i].second + CN_EPSILON > collision_intervals[id][i+1].first)
        {
            collision_intervals[id][i].second = collision_intervals[id][i+1].second;
            collision_intervals[id].erase(collision_intervals[id].begin() + i + 1);
            i--;
        }
}

void TO_AA_SIPP::add_move_constraint(Move move)
{
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

void TO_AA_SIPP::clear()
{
    states.clear();
    collision_intervals.clear();
    constraints.clear();
    path.cost = -1;
}

void TO_AA_SIPP::make_constraints(const std::list<Constraint> &cons)
{
    for(auto con : cons)
    {
        if(con.positive == false)
        {
            if(con.id1 == con.id2) // wait consatraint
                add_collision_interval(con.id1, std::make_pair(con.t1, con.t2));
            else
                add_move_constraint(Move(con));
        }
        else
        {
            /*bool inserted = false;
            for(unsigned int i = 0; i < landmarks.size(); i++)
                if(landmarks[i].t1 > con.t1)
                {
                    landmarks.insert(landmarks.begin() + i, Move(con.t1, con.t2, con.id1, con.id2));
                    inserted = true;
                    break;
                }
            if(!inserted)
                landmarks.push_back(Move(con.t1, con.t2, con.id1, con.id2));*/
            continue;
        }
    }
}

double TO_AA_SIPP::findEAT(oNode node)
{
    auto cons = constraints.find(std::make_pair(node.Parent->id, node.id));
    double cost = calculateDistanceFromCellToCell(node.Parent->i, node.Parent->j, node.i, node.j);
    if(cons != constraints.end())
        for(auto c:cons->second)
            if(node.g - cost + CN_EPSILON > c.t1 && node.g - cost < c.t2)
                node.g = c.t2 + cost;
    return node.g;
}

Path TO_AA_SIPP::findPath(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values_)
{

    h_values = h_values_;
    //h_values.set_goal(agent.goal_i, agent.goal_j);
    Path resultPath;
    states.clear();
    initStates(agent, map, cons);
    oNode curNode(states.getMin()), newNode;
    bool pathFound(false);
    int expanded(0);
    states.printStats();
    while(curNode.g < CN_INFINITY)//if curNode.g=CN_INFINITY => there are only unreachable non-consistent states => path cannot be found
    {
        newNode = curNode;
        if(newNode.consistent == 0)
            if(!los.checkLine(newNode.i, newNode.j, newNode.Parent->i, newNode.Parent->j, map))
            {
                states.update(newNode, false);
                curNode = states.getMin();
                continue;
            }
        newNode.g = findEAT(newNode);
        if(newNode.g < newNode.best_g)
        {
            newNode.best_g = newNode.g;
            newNode.best_Parent = newNode.Parent;
            states.update(newNode, true);
        }
        else
            states.update(newNode, false);

        curNode = states.getMin();
        if((newNode.best_g + newNode.h - curNode.F) < CN_EPSILON)
        {
            expanded++;
            states.expand(newNode);
            states.updateNonCons(newNode);
            if(newNode.i == agent.goal_i && newNode.j == agent.goal_j && newNode.interval.end == CN_INFINITY)
            {
                newNode.g = newNode.best_g;
                newNode.Parent = newNode.best_Parent;
                pathFound = true;
                break;
            }
            curNode = states.getMin();
        }
    }
    if(pathFound)
    {
        makePrimaryPath(newNode);
        path.cost = newNode.g;
        path.agentID = agent.id;
        path.expanded = expanded;
        return path;
    }
    else
    {
        return Path();
    }
}

/*std::vector<conflict> AA_SIPP_O::CheckConflicts()
{
    std::vector<conflict> conflicts(0);
    conflict conf;
    Node cur, check;
    std::vector<std::vector<conflict>> positions;
    positions.resize(sresult.agents);
    for(int i = 0; i < sresult.agents; i++)
    {
        if(!sresult.pathInfo[i].pathfound)
            continue;
        positions[i].resize(0);
        int k = 0;
        double part = 1;
        for(int j = 1; j<sresult.pathInfo[i].sections.size(); j++)
        {
            cur = sresult.pathInfo[i].sections[j];
            check = sresult.pathInfo[i].sections[j-1];
            int di = cur.i - check.i;
            int dj = cur.j - check.j;
            double dist = (cur.g - check.g)*10;
            int steps = (cur.g - check.g)*10;
            if(dist - steps + part >= 1)
            {
                steps++;
                part = dist - steps;
            }
            else
                part += dist - steps;
            double stepi = double(di)/dist;
            double stepj = double(dj)/dist;
            double curg = double(k)*0.1;
            double curi = check.i + (curg - check.g)*di/(cur.g - check.g);
            double curj = check.j + (curg - check.g)*dj/(cur.g - check.g);
            conf.i = curi;
            conf.j = curj;
            conf.g = curg;
            if(curg <= cur.g)
            {
                positions[i].push_back(conf);
                k++;
            }
            while(curg <= cur.g)
            {
                if(curg + 0.1 > cur.g)
                    break;
                curi += stepi;
                curj += stepj;
                curg += 0.1;
                conf.i = curi;
                conf.j = curj;
                conf.g = curg;
                positions[i].push_back(conf);
                k++;
            }
        }
        if(double(k - 1)*0.1 < sresult.pathInfo[i].sections.back().g)
        {
            conf.i = sresult.pathInfo[i].sections.back().i;
            conf.j = sresult.pathInfo[i].sections.back().j;
            conf.g = sresult.pathInfo[i].sections.back().g;
            positions[i].push_back(conf);
        }
    }
    int max = 0;
    for(int i = 0; i < positions.size(); i++)
        if(positions[i].size() > max)
            max = positions[i].size();
    for(int i = 0; i < sresult.agents; i++)
    {
        for(int k = 0; k < max; k++)
        {
            for(int j = i + 1; j < sresult.agents; j++)
            {
                if(!sresult.pathInfo[j].pathfound || !sresult.pathInfo[i].pathfound)
                    continue;
                conflict a, b;
                if(positions[i].size() > k)
                    a = positions[i][k];
                else
                    a = positions[i].back();
                if(positions[j].size() > k)
                    b = positions[j][k];
                else
                    b = positions[j].back();
                if(sqrt((a.i - b.i)*(a.i - b.i) + (a.j - b.j)*(a.j - b.j)) + CN_EPSILON < 1.0)
                {
                   // std::cout<<a.i<<" "<<a.j<<" "<<b.i<<" "<<b.j<<" "<<sqrt((a.i - b.i)*(a.i - b.i) + (a.j - b.j)*(a.j - b.j))<<"\n";
                    conf.i = b.i;
                    conf.j = b.j;
                    conf.agent1 = i;
                    conf.agent2 = j;
                    conf.g = b.g;
                    conflicts.push_back(conf);
                }
            }
        }
    }
    return conflicts;
}*/

void TO_AA_SIPP::makePrimaryPath(oNode curNode)
{
    std::list<oNode> nodes;
    oNode n(curNode.i, curNode.j, curNode.g, curNode.F);
    nodes.push_front(n);
    if(curNode.Parent != nullptr)
    {
        curNode = *curNode.Parent;
        if(curNode.Parent != nullptr)
        {
            do
            {
                oNode n(curNode.i, curNode.j, curNode.g, curNode.F);
                nodes.push_front(n);
                curNode = *curNode.Parent;
            }
            while(curNode.Parent != nullptr);
        }
        oNode n(curNode.i, curNode.j, curNode.g, curNode.F);
        nodes.push_front(n);
    }
    for(auto it = nodes.begin(); it != nodes.end(); it++)
        this->path.nodes.push_back(Node(0, it->F, it->g, it->i, it->j));
    return;
}
