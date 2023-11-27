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

void TO_AA_SIPP::initStates(Agent agent, const Map &Map)
{
    states.clear();
    states.map = &Map;
    std::set<int> start_cells;
    if(!agent.starts.empty())
    {
        for(auto s: agent.starts)
        {
            double h = h_values.get_value(s.i, s.j);//calculateDistanceFromCellToCell(agent.start_i, agent.start_j, agent.goal_i, agent.goal_j);
            oNode start(s.i, s.j, s.g, h);
            start.h = h;
            start.expanded = true;
            start.best_g = s.g;
            start.consistent = 1;
            start.interval = SafeInterval(s.interval.first, s.interval.second, s.interval_id);
            start.interval_id = s.interval_id;
            start.id = s.id;
            states.insert(start);
            start_cells.insert(s.id);
            //std::cout<<start.i<<" "<<start.j<<" "<<start.g<<" start state\n";
        }
    }
    else
    {
        double h = h_values.get_value(agent.start_i, agent.start_j);//calculateDistanceFromCellToCell(agent.start_i, agent.start_j, agent.goal_i, agent.goal_j);
        oNode start(agent.start_i, agent.start_j,0,h);
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
        start_cells.insert(start.id);
    }
    auto parent = states.getParentPtr();
    for(int i = 0; i < Map.get_height(); i++)
        for(int j = 0; j < Map.get_width(); j++)
        {
            if(Map.cell_is_obstacle(i, j))// || (i == agent.start_i && j == agent.start_j))
                continue;
            if(start_cells.count(Map.get_id(i,j)) > 0)
                continue;
            std::vector<std::pair<double, double>> intervals(0);
            auto colls_it = collision_intervals.find(i*Map.get_width() + j);
            if(colls_it != collision_intervals.end())
            {
                std::pair<double, double> interval = {0, CN_INFINITY};
                for(size_t k = 0; k < colls_it->second.size(); k++)
                {
                    interval.second = colls_it->second[k].first;
                    intervals.push_back(interval);
                    interval.first = colls_it->second[k].second;
                }
                interval.second = CN_INFINITY;
                intervals.push_back(interval);
            }
            else
                intervals.push_back({0, CN_INFINITY});

            double g = calculateDistanceFromCellToCell(i, j, agent.start_i, agent.start_j);
            double h = h_values.get_value(i,j);//calculateDistanceFromCellToCell(i, j, agent.goal_i, agent.goal_j);
            oNode n = oNode(i,j,g,g+h);
            n.h = h;
            n.Parent = parent;

            for(size_t k = 0; k < intervals.size(); k++)
            {
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
}

Path TO_AA_SIPP::add_part(Path result, Path part)
{
    part.nodes.erase(part.nodes.begin());
    for(auto n: part.nodes)
        result.nodes.push_back(n);
    return result;
}

double TO_AA_SIPP::check_endpoint(Node start, Node goal)
{
    double cost = sqrt(pow(start.i - goal.i, 2) + pow(start.j - goal.j, 2));
    if(start.g + cost < goal.interval.first)
        start.g = goal.interval.first - cost;
    if(constraints.count({start.id, goal.id}) != 0)
    {
        auto it = constraints.find({start.id, goal.id});
        for(unsigned int i = 0; i < it->second.size(); i++)
            if(start.g + CN_EPSILON > it->second[i].t1 && start.g < it->second[i].t2)
                start.g = it->second[i].t2;
    }
    if(start.g > start.interval.second || start.g + cost > goal.interval.second)
        return CN_INFINITY;
    else
        return start.g + cost;
}

void TO_AA_SIPP::add_collision_interval(int id, std::pair<double, double> interval)
{
    //std::cout<<id<<" "<<interval.first<<" "<<interval.second<<" CONSTRAINT\n";
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

void TO_AA_SIPP::make_constraints(const std::list<Multiconstraint> &cons)
{
    multilandmarks.clear();
    for(auto con : cons)
    {
        if(con.positive == false)
        {
            for(auto c: con.constraints)
                if(c.id1 == c.id2) // wait consatraint
                    add_collision_interval(c.id1, std::make_pair(c.t1, c.t2));
                else
                    add_move_constraint(Move(c));
        }
        else
        {
            std::vector<Move> marks;
            for(auto c: con.constraints)
                marks.emplace_back(c.t1, c.t2, c.id1, c.id2);
            bool inserted = false;
            for(unsigned int i = 0; i < multilandmarks.size(); i++)
            {
                if(multilandmarks[i][0].id1 == marks[0].id1)
                {
                    for(auto m:marks)
                    {
                        bool new_action = true;
                        for(auto& m2:multilandmarks[i])
                        {
                            if(m2.id1 == m.id1 && m2.id2 == m.id2)
                            {
                                m2.t1 = std::max(m2.t1, m.t1);
                                m2.t2 = std::min(m2.t2, m.t2);
                                new_action = false;
                                break;
                            }
                        }
                        if(new_action)
                            multilandmarks[i].push_back(m);
                    }
                    inserted = true;
                    break;
                }
                else if(multilandmarks[i][0].t1 > con.constraints[0].t1)
                {

                    multilandmarks.insert(multilandmarks.begin() + i, marks);
                    inserted = true;
                    break;
                }
            }
            if(!inserted)
                multilandmarks.push_back(marks);
        }
    }
}

double TO_AA_SIPP::findEAT(oNode node)
{
    auto cons = constraints.find(std::make_pair(node.Parent->id, node.id));
    double cost = calculateDistanceFromCellToCell(node.Parent->i, node.Parent->j, node.i, node.j);
    node.g = std::max(node.Parent->g + cost, node.interval.begin);
    if(cons != constraints.end()) {
        for(const auto &c:cons->second)
        {
            //std::cout<<node.Parent->i<<" "<<node.Parent->j<<" "<<node.i<<" "<<node.j<<" this move is constrained for "<<c.t1<<" "<<c.t2<<" period\n";
            if(node.g - cost + 1e-6 > c.t1 && node.g - cost < c.t2)
                node.g = c.t2 + cost;
            //std::cout<<"new node.g="<<node.g<<" start="<<node.g - cost<<"\n";
        }
    }
    if(node.Parent->interval.end + cost < node.g)
        node.g = CN_INFINITY;
    return node.g;
}

std::vector<Path> TO_AA_SIPP::find_partial_path(Agent agent, const Map &map, double max_f)
{
    initStates(agent, map);
    oNode curNode(states.getMin()), newNode;
    bool pathFound(false);
    int expanded(0);
    if(curNode.F < 0)
        return {Path()};
    //states.printStats();
    //std::cout<<"find path for "<<agent.id<<" "<<agent.start_id<<" "<<agent.goal_id<<" cons="<<cons.size()<<"\n";
    while(curNode.g < CN_INFINITY)//if curNode.g=CN_INFINITY => there are only unreachable non-consistent states => path cannot be found
    {
        newNode = curNode;
        std::cout<<newNode.i<<" "<<newNode.j<<" "<<newNode.g<<" "<<newNode.Parent->g<<" node\n";
        if(newNode.Parent == nullptr)
        {
            //std::cout<<"WTF?!\n";
            return {Path()};
        }
        if(newNode.consistent == 0)
        {
            if(!h_values.get_los(newNode.i, newNode.j, newNode.Parent->i, newNode.Parent->j, map))
            {
                states.update(newNode, false, h_values);
                curNode = states.getMin();
                continue;
            }
        }
        newNode.g = findEAT(newNode);
        if(newNode.g < newNode.best_g)
        {
            newNode.best_g = newNode.g;
            newNode.best_Parent = newNode.Parent;
            states.update(newNode, true, h_values);
        }
        else
            states.update(newNode, false, h_values);

        curNode = states.getMin();
        std::cout<<newNode.best_g<<" "<<newNode.h<<" "<<newNode.F<<" "<<curNode.g<<" "<<curNode.best_g<<" "<<curNode.h<<" "<<curNode.F<<" "<<expanded<<" values\n";
        if(newNode.best_g + newNode.h < curNode.F + CN_EPSILON)
        {

            expanded++;
            states.expand(newNode);
            states.updateNonCons(newNode, h_values);
            if((agent.goals.empty() && newNode.id == agent.goal_id && newNode.interval.end == CN_INFINITY) ||
                (!agent.goals.empty() && newNode.id == agent.goals[0].id && newNode.interval.end == CN_INFINITY))
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
        if(agent.goals.size() > 0)
        {
            std::vector<Path> paths;
            auto goals = states.get_nodes_by_ij(agent.goals[0].i, agent.goals[0].j);
            for(auto g: goals)
                if(g.F < CN_INFINITY)
                {
                    makePrimaryPath(g);
                    path.cost = g.F;
                    paths.push_back(path);
                }
            return paths;
        }
        else
            return {path};
    }
    else
    {
        return {Path()};
    }
}

std::vector<Node> TO_AA_SIPP::get_endpoints(int node_id, double node_i, double node_j, double t1, double t2)
{
    std::vector<Node> nodes;
    nodes = {Node(node_id, 0, 0, node_i, node_j, nullptr, t1, t2)};
    if(collision_intervals[node_id].empty())
        return nodes;
    else
        for(unsigned int k = 0; k < collision_intervals[node_id].size(); k++)
        {
            unsigned int i(0);
            while(i < nodes.size())
            {
                Node n = nodes[i];
                auto c = collision_intervals[node_id][k];
                bool changed = false;
                if(c.first - CN_EPSILON < n.interval.first && c.second + CN_EPSILON > n.interval.second)
                {
                    nodes.erase(nodes.begin() + i);
                    changed = true;
                }
                else if(c.first - CN_EPSILON < n.interval.first && c.second > n.interval.first)
                {
                    nodes[i].interval.first = c.second;
                    changed = true;
                }
                else if(c.first - CN_EPSILON > n.interval.first && c.second + CN_EPSILON < n.interval.second)
                {
                    nodes[i].interval.second = c.first;
                    nodes.insert(nodes.begin() + i + 1, Node(node_id, 0, 0, node_i, node_j, nullptr, c.second, n.interval.second));
                    changed = true;
                }
                else if(c.first < n.interval.second && c.second + CN_EPSILON > n.interval.second)
                {
                    nodes[i].interval.second = c.first;
                    changed = true;
                }
                if(changed)
                {
                    i = -1;
                    k = 0;
                }
                i++;
            }
        }
    return nodes;
}

Path TO_AA_SIPP::find_path(Agent agent, const Map &map, std::list<Multiconstraint> cons, PHeuristic &h_values_)
{
    //std::cout<<"find path for "<<agent.id<<" "<<agent.start_i<<" "<<agent.start_j<<" "<<agent.goal_i<<" "<<agent.goal_j<<"\n";
    clear();
    make_constraints(cons);
    h_values = h_values_;
    h_values.set_goal(agent.goal_i, agent.goal_j);
    Path resultPath;
    path = Path();

    std::vector<Node> starts, goals;
    std::vector<Path> parts, results, new_results;
    Path part, result;
    int expanded(0);
    if(!multilandmarks.empty())
    {
        //std::cout<<"has multilandmark\n";
        for(unsigned int i = 0; i <= multilandmarks.size(); i++)
        {
            if(i == 0)
            {
                starts = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY).at(0)};
                goals = get_endpoints(multilandmarks[i][0].id1, map.get_i(multilandmarks[i][0].id1), map.get_j(multilandmarks[i][0].id1), multilandmarks[i][0].t1, multilandmarks[i][0].t2);
            }
            else
            {
                starts.clear();
                for(auto p:results)
                    starts.push_back(p.nodes.back());
                if(i == multilandmarks.size())
                    goals = {get_endpoints(agent.goal_id, agent.goal_i, agent.goal_j, 0, CN_INFINITY).back()};
                else
                    goals = get_endpoints(multilandmarks[i][0].id1, map.get_i(multilandmarks[i][0].id1), map.get_j(multilandmarks[i][0].id1), multilandmarks[i][0].t1, multilandmarks[i][0].t2);
            }
            //std::cout<<starts.size()<<" "<<goals.size()<<" starts and goals sizes\n";
            if(goals.empty())
            {
                //std::cout<<" no goal endpoints\n";
                return Path();
            }
            agent.starts = starts;
            agent.goals = goals;
            //for(auto s: agent.starts)
            //    std::cout<<s.i<<" "<<s.j<<" "<<s.g<<" "<<s.interval_id<<" start\n";
            //for(auto s: agent.goals)
            //    std::cout<<s.i<<" "<<s.j<<" "<<s.interval_id<<" goal\n";
            parts = find_partial_path(agent, map, goals.back().interval.second);
            new_results.clear();
            if(i == 0)
                for(unsigned int k = 0; k < parts.size(); k++)
                {
                    if(parts[k].nodes.empty())
                        continue;
                    new_results.push_back(parts[k]);
                }

            /*for(unsigned int k = 0; k < parts.size(); k++)
            {
                std::cout<<k<<" part "<<parts[k].nodes.size()<<" "<<parts[k].nodes[0].interval_id<<"\n";
                for(auto n:parts[k].nodes)
                    std::cout<<n.i<<" "<<n.j<<" "<<n.interval_id<<" "<<n.g<<"\n";
            }
            for(unsigned int j = 0; j < results.size(); j++)
            {
                std::cout<<j<<" result "<<results[j].nodes.size()<<" "<<results[j].nodes.back().interval_id<<"\n";
                for(auto n:results[j].nodes)
                    std::cout<<n.i<<" "<<n.j<<" "<<n.interval_id<<" "<<n.g<<"\n";
            }*/

            for(unsigned int k = 0; k < parts.size(); k++)
                for(unsigned int j = 0; j < results.size(); j++)
                {
                    if(parts[k].nodes.empty())
                        continue;
                    if(parts[k].nodes[0].interval_id == results[j].nodes.back().interval_id)
                    {
                        new_results.push_back(results[j]);
                        new_results.back() = add_part(new_results.back(), parts[k]);
                    }
                }
            results = new_results;
            if(results.empty())
            {
                //std::cout<<" empty results\n";
                return Path();
            }
            if(i < multilandmarks.size())
            {
                starts.clear();
                for(auto p:results)
                    starts.push_back(p.nodes.back());
                for(auto c: multilandmarks[i])
                {
                    double offset = sqrt(pow(map.get_i(c.id1) - map.get_i(c.id2), 2) + pow(map.get_j(c.id1) - map.get_j(c.id2), 2));
                    goals = get_endpoints(c.id2, map.get_i(c.id2), map.get_j(c.id2), c.t1 + offset, c.t2 + offset);
                    if(goals.empty())
                    {
                        //std::cout<<" no goal endpoints 2\n";
                        return Path();
                    }
                    new_results.clear();
                    for(unsigned int k = 0; k < goals.size(); k++)
                    {
                        double best_g(CN_INFINITY);
                        int best_start_id = -1;
                        for(unsigned int j = 0; j < starts.size(); j++)
                        {
                            double g = check_endpoint(starts[j], goals[k]);
                            if(g < best_g)
                            {
                                best_start_id = j;
                                best_g = g;
                            }
                            //std::cout<<check_endpoint(starts[j], goals[k])<<" "<<best_g<<" "<<best_start_id<<" check best g\n";
                        }
                        if(best_start_id >= 0)
                        {
                            goals[k].g = best_g;
                            if(collision_intervals[goals[k].id].empty())
                                goals[k].interval.second = CN_INFINITY;
                            else
                            {
                                for(auto c:collision_intervals[goals[k].id])
                                    if(goals[k].g < c.first)
                                    {
                                        goals[k].interval.second = c.first;
                                        break;
                                    }
                            }
                            new_results.push_back(results[best_start_id]);
                            if(goals[k].g - starts[best_start_id].g > offset + CN_EPSILON)
                            {
                                new_results.back().nodes.push_back(new_results.back().nodes.back());
                                new_results.back().nodes.back().g = goals[k].g - offset;
                            }
                            new_results.back().nodes.push_back(goals[k]);
                        }
                    }
                }

                results = new_results;
                if(results.empty())
                {
                    //std::cout<<" empty results 2\n";
                    return Path();
                }
            }
        }
        result = results.front();
    }
    else
    {
        /*starts = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY).at(0)};
        goals = {get_endpoints(agent.goal_id, agent.goal_i, agent.goal_j, 0, CN_INFINITY).back()};
        agent.start_intervals.clear();
        agent.goal_intervals.clear();
        for(auto s: starts)
            agent.start_intervals.push_back(SafeInterval(s.interval.first, s.interval.second));
        for(auto g: goals)
            agent.goal_intervals.push_back(SafeInterval(g.interval.first, g.interval.second));*/
        parts = find_partial_path(agent, map);
        if(parts.front().cost < 0)
            return Path();
        result = parts.front();
    }
    result.nodes.shrink_to_fit();
    result.cost = result.nodes.back().g;
    result.agentID = agent.id;
    result.expanded = expanded;
    return result;

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
    n.id = curNode.id;
    nodes.push_front(n);
    this->path = Path();
    if(curNode.Parent != nullptr)
    {
        curNode = *curNode.Parent;
        if(curNode.Parent != nullptr)
        {
            do
            {
                nodes.push_front(curNode);
                curNode = *curNode.Parent;
            }
            while(curNode.Parent != nullptr);
        }
        nodes.push_front(curNode);
    }
    for(auto it = nodes.begin(); it != nodes.end(); it++)
    {
        //std::cout<<it->id<<" "<<it->i<<" "<<it->j<<" "<<it->g<<" path node\n";
        Node n = Node(it->id, it->F, it->g, it->i, it->j, nullptr, it->interval.begin, it->interval.end);
        n.interval_id = it->interval_id;
        this->path.nodes.push_back(n);
    }
    for(unsigned int i = 0; i < path.nodes.size(); i++)
    {
        unsigned int j = i + 1;
        if(j == path.nodes.size())
            break;
        if(fabs(path.nodes[j].g - path.nodes[i].g - calculateDistanceFromCellToCell(path.nodes[j].i, path.nodes[j].j, path.nodes[i].i, path.nodes[i].j)) > CN_EPSILON)
        {
            Node add = path.nodes[i];
            add.g = path.nodes[j].g - calculateDistanceFromCellToCell(path.nodes[j].i, path.nodes[j].j, path.nodes[i].i, path.nodes[i].j);
            path.nodes.emplace(path.nodes.begin() + j, add);
        }
    }
    return;
}
