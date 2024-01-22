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
    if(!agent.starts.empty())
    {
        for(auto s: agent.starts)
        {
            double h = h_values->get_value(agent.goals[0].id, s.id);
            oNode start(s.i, s.j, s.g, h);
            start.h = h;
            start.expanded = true;
            start.best_g = s.g;
            start.consistent = 1;
            start.interval_id = -s.interval_id - 1;
            start.interval = SafeInterval(s.interval.first, s.interval.second, start.interval_id);
            start.id = s.id;
            states.insert(start);
        }
    }
    else
    {
        double h = h_values->get_value(agent.goals[0].id, agent.start_id);
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
    }
    auto parents = states.getAllParentsPtr();
    for(int i = 0; i < Map.get_height(); i++)
        for(int j = 0; j < Map.get_width(); j++)
        {
            if(Map.cell_is_obstacle(i, j))
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

            const oNode* parent;
            double min_g(CN_INFINITY), new_g;
            for(auto p: parents)
            {
                new_g = p->g + calculateDistanceFromCellToCell(i, j, p->i, p->j);
                if(new_g < min_g)
                {
                    parent = p;
                    min_g = new_g;
                }
            }
            double h = h_values->get_value(agent.goals[0].id, map->get_id(i,j));//calculateDistanceFromCellToCell(i, j, agent.goal_i, agent.goal_j);
            oNode n = oNode(i, j, min_g, min_g+h);
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
    std::vector<std::pair<double, double>> intervals(0);
    if(collision_intervals.count(id) == 0)
        collision_intervals.insert({id, {interval}});
    else
        collision_intervals[id].push_back(interval);
    std::sort(collision_intervals[id].begin(), collision_intervals[id].end());
    for(int i = 0; i + 1 < collision_intervals[id].size(); i++)
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

double TO_AA_SIPP::dist(int id1, int id2)
{
    int i1 = map->get_i(id1);
    int j1 = map->get_j(id1);
    int i2 = map->get_i(id2);
    int j2 = map->get_j(id2);
    return std::sqrt((i1-i2)*(i1-i2)+(j1-j2)*(j1-j2));
}

void TO_AA_SIPP::make_constraints(const std::list<Multiconstraint> &cons)
{
    multilandmarks.clear();
    std::multimap<int, Multiconstraint> positives;

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
            positives.insert(std::pair{con.constraints[0].id1, con});
    }
    std::set<int> already_merged;
    for(auto it = positives.begin(); it != positives.end(); it++)
    {
        std::vector<Move> marks;
        if(positives.count(it->first) == 1)
        {
            for(auto c: it->second.constraints)
                marks.emplace_back(c.t1, c.t2, c.id1, c.id2);
        }
        else
        {
            if(already_merged.count(it->first))
                continue;
            already_merged.insert(it->first);
            auto range = positives.equal_range(it->first);
            auto all_cons = range.first->second.constraints;
            for(auto c: all_cons)
            {
                int total_positives(0), contains(0);
                double max_t1(0), min_t2(CN_INFINITY);
                for(auto it = range.first; it != range.second; it++)
                {
                    total_positives++;
                    for(auto n:it->second.constraints)
                        if(c.id2 == n.id2)
                        {
                            contains++;
                            max_t1 = std::fmax(max_t1, n.t1);
                            min_t2 = std::fmin(min_t2, n.t2);
                        }
                }
                if(contains == total_positives)
                    marks.emplace_back(max_t1, min_t2, c.id1, c.id2);
            }

        }
        if(marks.empty())
            continue;
        bool inserted = false;
        for(unsigned int i = 0; i < multilandmarks.size(); i++)
        {
            if(multilandmarks[i][0].t1 > it->second.constraints[0].t1)
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

double TO_AA_SIPP::findEAT(oNode node)
{
    auto cons = constraints.find(std::make_pair(node.Parent->id, node.id));
    double cost = calculateDistanceFromCellToCell(node.Parent->i, node.Parent->j, node.i, node.j);
    node.g = std::max(node.Parent->g + cost, node.interval.begin);
    if(cons != constraints.end()) {
        for(const auto &c:cons->second)
        {
            if(node.g - cost + CN_EPSILON > c.t1 && node.g - cost < c.t2)
                node.g = c.t2 + cost;
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
    while(curNode.g < CN_INFINITY)//if curNode.g=CN_INFINITY => there are only unreachable non-consistent states => path cannot be found
    {
        newNode = curNode;
        if(newNode.Parent == nullptr)
        {
            return {Path()};
        }
        if(newNode.consistent == 0)
        {
            if(!h_values->get_los(newNode.i, newNode.j, newNode.Parent->i, newNode.Parent->j, map))
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
        if(newNode.best_g + newNode.h < curNode.F + CN_EPSILON)
        {

            expanded++;
            states.expand(newNode);
            states.updateNonCons(newNode, h_values);
            if(newNode.id == agent.goals[0].id && newNode.interval.end == CN_INFINITY)
            {
                newNode.g = newNode.best_g;
                newNode.Parent = newNode.best_Parent;
                pathFound = true;
                break;
            }
            curNode = states.getMin();
        }
        if(curNode.expanded)
        {
            pathFound = true;
            newNode = curNode;
            break;
        }
    }
    if(pathFound)
    {
        makePrimaryPath(newNode);
        path.cost = newNode.g;
        path.agentID = agent.id;
        path.expanded = expanded;
        if(agent.goals.size() > 1)
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
    nodes = {Node(node_id, 0, 0, node_i, node_j, nullptr, 0, t2)};
    if(collision_intervals.find(node_id) == collision_intervals.end())
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

std::vector<Path> TO_AA_SIPP::execute_positive_actions(const std::vector<Move>& actions, const Map &map, const Node& start)
{
    std::vector<Path> result;
    for(auto a: actions)
    {
        Path p;
        double t_start(start.g);
        if(constraints.count(std::make_pair(a.id1, a.id2)) > 0)
            for(auto c: constraints[std::make_pair(a.id1, a.id2)])
            {
                if(c.t1 - CN_EPSILON <= t_start && c.t2 > t_start)
                {
                    t_start = c.t2;
                }
            }
        if(t_start < a.t1)
            t_start = a.t1;
        if(t_start > start.interval.second || t_start > a.t2) //unable to wait in start
            continue;
        p.nodes.push_back(start);
        if(t_start > start.g)
        {
            Node s_wait = start;
            s_wait.g = t_start;
            p.nodes.push_back(s_wait);
        }
        bool bad_start_collision_interval = false;
        if(collision_intervals.find(a.id1) != collision_intervals.end())
        {
            auto intervals = collision_intervals[a.id1];
            for(auto c: intervals)
                if(start.g - CN_EPSILON < c.first && t_start + CN_EPSILON > c.first)
                {
                    bad_start_collision_interval = true;
                    break;
                }
        }
        if(bad_start_collision_interval)
            continue;
        double offset = sqrt(pow(map.get_i(a.id1) - map.get_i(a.id2), 2) + pow(map.get_j(a.id1) - map.get_j(a.id2), 2));
        if(collision_intervals.find(a.id2) != collision_intervals.end())
        {
            auto goals = get_endpoints(a.id2, map.get_i(a.id2), map.get_j(a.id2), t_start + offset, a.t2 + offset);
            if(goals.empty())
                continue;
            for(auto g: goals)
            {
                Path p2 = p;
                if(g.interval.first <= t_start + offset <= g.interval.second)
                {
                    p2.nodes.push_back(Node(g.id, t_start + offset, t_start + offset, g.i, g.j, nullptr, g.interval.first, g.interval.second));
                    result.push_back(p2);
                }
                else if(t_start + offset < g.interval.first && g.interval.first - offset < start.interval.second)
                {
                    p2.nodes.push_back(p2.nodes.back());
                    p2.nodes.back().g = g.interval.first - offset;
                    p2.nodes.push_back(Node(g.id, g.interval.first, g.interval.first, g.i, g.j, nullptr, g.interval.first, g.interval.second));
                    result.push_back(p2);
                }
            }
        }
        else
        {
            p.nodes.push_back(Node(a.id2, t_start + offset, t_start + offset, map.get_i(a.id2), map.get_j(a.id2), nullptr, 0, CN_INFINITY));
            result.push_back(p);
        }
    }
    return result;
}

Path TO_AA_SIPP::check_all_paths(Agent agent, const Map &map, std::list<Multiconstraint> cons, PHeuristic &h_values_, int id)
{
    node_id = id;
    try
    {
        auto path = find_path(agent, map, cons, h_values_);
        return path;
    }
    catch(const char* error_message)
    {
        std::cout << error_message << std::endl;
    }

    std::vector<Multiconstraint> positives;
    std::list<Multiconstraint> negatives;
    for(auto c: cons)
    {
        if(c.positive)
            positives.push_back(c);
        else
            negatives.push_back(c);
    }
    auto true_path = find_path(agent, map, cons, h_values_);
    Path min_path = true_path;
    for(int i=0; i < positives.size(); i++)
        for(auto c: positives[i].constraints)
        {
            auto constraints = negatives;
            if(positives.size() > 1)
                for(int k=0; k < positives.size(); k++)
                    if(i != k)
                        constraints.push_back(positives[k]);
            Multiconstraint m;
            m.agent = c.agent;
            m.constraints = {c};
            m.positive = true;
            constraints.push_back(m);
            auto path = find_path(agent, map, constraints, h_values_);
            if(true_path.cost - path.cost > CN_EPSILON && path.cost > 0)
            {
                std::cout<<"CHECK FAILED! "<<true_path.cost<<" "<<path.cost<<"\n";
                for(auto n:true_path.nodes)
                    std::cout<<n.id<<" "<<n.g<<" => ";
                std::cout<<"\n";
                for(auto n:path.nodes)
                    std::cout<<n.id<<" "<<n.g<<" => ";
                std::cout<<"\n";
                std::cout<<agent.start_id<<" "<<agent.goal_id<<" "<<agent.start_i<<" "<<agent.start_j<<" "<<agent.goal_i<<" "<<agent.goal_j<<" agent\n";
                for(auto s: agent.starts)
                    std::cout<<s.id<<" "<<s.g<<" "<<s.f<<' '<<s.interval_id<<" s in starts\n";
                for(auto s: agent.goals)
                    std::cout<<s.id<<" "<<s.g<<" "<<s.f<<' '<<s.interval_id<<" g in goals\n";
                std::cout<<positives.size()<<" different positive constraints\n";
                int k=0;
                for(auto p:positives)
                {
                    for(auto c: p.constraints)
                        std::cout<<k<<" all.constraints.push_back(Constraint(0, "<<c.t1<<", "<<c.t2<<", "<<c.id1<<", "<<c.id2<<", true));\n";
                    k++;
                }
                for(auto c: positives[i].constraints)
                    std::cout<<"m.constraints.push_back(Constraint(0, "<<c.t1<<", "<<c.t2<<", "<<c.id1<<", "<<c.id2<<", true));\n";
                for(auto n: negatives)
                    for(auto c:n.constraints)
                        std::cout<<"m2.constraints.push_back(Constraint(0, "<<c.t1<<", "<<c.t2<<", "<<c.id1<<", "<<c.id2<<", false));\n";
                find_path(agent, map, cons, h_values_);

            }
            if(min_path.cost < 0 || min_path.cost > path.cost)
                min_path = path;
        }
    return true_path;
}

Path TO_AA_SIPP::find_path(Agent agent, const Map &map, std::list<Multiconstraint> cons, PHeuristic &h_values_)
{

    this->map = &map;
    clear();
    make_constraints(cons);
    h_values = &h_values_;
    Path resultPath;
    path = Path();

    std::vector<Node> starts, goals;
    std::vector<Path> parts, results, new_results;
    Path part, result;
    int expanded(0);
    if(!multilandmarks.empty())
    {
        for(unsigned int i = 0; i <= multilandmarks.size(); i++)
        {
            if(multilandmarks[i].size() == 0)//no available actions after merging multiple positive multiconstraints
                return Path();
            if(i == 0)
            {
                starts = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY).at(0)};
                goals = get_endpoints(multilandmarks[i][0].id1, map.get_i(multilandmarks[i][0].id1), map.get_j(multilandmarks[i][0].id1), multilandmarks[i][0].t1, multilandmarks[i][0].t2);
            }
            else
            {
                starts.clear();
                for(auto p:results)
                    if(p.nodes.back().id != agent.goal_id)
                        starts.push_back(p.nodes.back());
                if(i == multilandmarks.size())
                    goals = {get_endpoints(agent.goal_id, agent.goal_i, agent.goal_j, 0, CN_INFINITY).back()};
                else
                    goals = get_endpoints(multilandmarks[i][0].id1, map.get_i(multilandmarks[i][0].id1), map.get_j(multilandmarks[i][0].id1), multilandmarks[i][0].t1, multilandmarks[i][0].t2);
            }
            if(goals.empty())
            {
                return Path();
            }
            agent.starts = starts;
            agent.goals = goals;
            if(i == 0 && multilandmarks[0][0].id1 == agent.start_id)
            {
                results = execute_positive_actions(multilandmarks[0], map, starts.front());
                continue;
            }
            parts = find_partial_path(agent, map, goals.back().interval.second);
            new_results.clear();
            if(i == 0)
                for(unsigned int k = 0; k < parts.size(); k++)
                {
                    if(parts[k].nodes.empty())
                        continue;
                    new_results.push_back(parts[k]);
                }

            for(unsigned int k = 0; k < parts.size(); k++)
                for(unsigned int j = 0; j < results.size(); j++)
                {
                    if(parts[k].nodes.empty())
                        continue;
                    if(parts[k].nodes[0].id == results[j].nodes.back().id && parts[k].nodes[0].interval_id == results[j].nodes.back().interval_id)
                    {
                        new_results.push_back(results[j]);
                        new_results.back() = add_part(new_results.back(), parts[k]);
                    }
                }
            results = new_results;
            if(results.empty())
            {
                return Path();
            }
            if(i < multilandmarks.size())
            {
                starts.clear();
                new_results.clear();
                for(auto p:results)
                    if(p.nodes.back().id != agent.goal_id)
                    {
                        starts.push_back(p.nodes.back());
                        auto executed_actions = execute_positive_actions(multilandmarks[i], map, p.nodes.back());
                        for(auto e: executed_actions)
                        {
                            new_results.push_back(p);
                            new_results.back() = add_part(new_results.back(), e);
                        }
                    }
                results = new_results;
                if(results.empty())
                {
                    return Path();
                }
            }
        }
        result = results.front();
    }
    else
    {
        agent.goals = {Node(agent.goal_id, -1, -1, agent.goal_i, agent.goal_j)};
        parts = find_partial_path(agent, map);
        if(parts.front().cost < 0)
            return Path();
        result = parts.front();
    }
    result.nodes.shrink_to_fit();
    result.cost = result.nodes.back().g;
    result.agentID = agent.id;
    result.expanded = expanded;
    auto p = result;
    for(int i=0; i + 1 < p.nodes.size(); i++)
        if(p.nodes[i+1].id != p.nodes[i].id)
            if(std::fabs(p.nodes[i+1].g - p.nodes[i].g - std::sqrt(std::pow(p.nodes[i+1].i - p.nodes[i].i,2) + std::pow(p.nodes[i+1].j - p.nodes[i].j,2))) > CN_EPSILON)
            {
                std::cout<<"BAD PATH\n";
                std::cout<<p.nodes[i+1].g <<" "<< p.nodes[i].g<<" "<< std::sqrt(std::pow(p.nodes[i+1].i - p.nodes[i].i,2) + std::pow(p.nodes[i+1].j - p.nodes[i].j,2))<<"\n";
                for(auto n:p.nodes)
                    std::cout<<n.i<<" "<<n.j<<" "<<n.g<<" "<<n.f<<"\n";
                std::cout<<"\n";
            }

    return result;

}

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
        Node n = Node(it->id, it->F, it->g, it->i, it->j, nullptr, it->interval.begin, it->interval.end);
        n.interval_id = it->interval_id;
        if(n.interval_id < 0)
            n.interval_id = -n.interval_id - 1;
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
