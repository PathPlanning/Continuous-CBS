#include "cbs.h"

bool CBS::init_root(const Map &map, const Task &task)
{
    CBS_Node root;
    tree->set_focal_weight(config.focal_weight);
    sPath path;
    for(int id:task.get_ids())
    {
        Agent agent = task.get_agent(id);
        if(config.connectdness > 0)
            path = planner.find_path(agent, map, {}, h_values);
        else
            path = aa_planner.find_path(agent, map, {}, aa_h_values[agent.id]);
        if(path.cost < 0)
            return false;
        std::cout<<path.agentID<<" "<<path.cost<<"\n";
        root.paths[id] = path;
        root.cost += path.cost;
    }
    root.low_level_expanded = 0;
    root.parent = nullptr;
    root.id = 0;
    root.id_str = "0";
    auto conflicts = get_all_conflicts(root.paths, -1);
    root.conflicts_num = conflicts.size();

    for(auto conflict: conflicts)
    {
        std::cout<<conflict.agent1<<" "<<conflict.agent2<<"    "<<config.use_cardinal<<" conflict\n";
        if(!config.use_cardinal)
            root.conflicts.push_back(conflict);
        else
        {
            sPath pathA, pathB;
            if(config.connectdness > 0)
            {
                pathA = planner.find_path(task.get_agent(conflict.agent1), map, {get_constraint(conflict.agent1, conflict.move1, conflict.move2)}, h_values);
                pathB = planner.find_path(task.get_agent(conflict.agent2), map, {get_constraint(conflict.agent2, conflict.move2, conflict.move1)}, h_values);
            }
            else
            {
                pathA = aa_planner.find_path(task.get_agent(conflict.agent1), map, {get_constraint(conflict.agent1, conflict.move1, conflict.move2)}, aa_h_values[conflict.agent1]);
                pathB = aa_planner.find_path(task.get_agent(conflict.agent2), map, {get_constraint(conflict.agent2, conflict.move2, conflict.move1)}, aa_h_values[conflict.agent2]);
            }
            //conflict.path1 = pathA;
            //conflict.path2 = pathB;
            if(pathA.cost > root.paths[conflict.agent1].cost && pathB.cost > root.paths[conflict.agent2].cost)
            {
                conflict.overcost = std::min(pathA.cost - root.paths[conflict.agent1].cost, pathB.cost - root.paths[conflict.agent2].cost);
                root.cardinal_conflicts.push_back(conflict);
            }
            else if(pathA.cost > root.paths[conflict.agent1].cost || pathB.cost > root.paths[conflict.agent2].cost)
                root.semicard_conflicts.push_back(conflict);
            else
                root.conflicts.push_back(conflict);
        }
    }
    //std::cout<<conflicts.size()<<" init conflicts\n";
    solution.init_cost = root.cost;
    solution.initial_conflicts = conflicts.size();
    all_nodes.push_back(root);
    tree->add_node(&all_nodes.back());
    return true;
}

bool CBS::check_conflict(Move move1, Move move2)
{
    double startTimeA(move1.t1), endTimeA(move1.t2), startTimeB(move2.t1), endTimeB(move2.t2);
    double m1i1(map->get_i(move1.id1)), m1i2(map->get_i(move1.id2)), m1j1(map->get_j(move1.id1)), m1j2(map->get_j(move1.id2));
    double m2i1(map->get_i(move2.id1)), m2i2(map->get_i(move2.id2)), m2j1(map->get_j(move2.id1)), m2j2(map->get_j(move2.id2));
    Vector2D A(m1i1, m1j1);
    Vector2D B(m2i1, m2j1);
    Vector2D VA((m1i2 - m1i1)/(move1.t2 - move1.t1), (m1j2 - m1j1)/(move1.t2 - move1.t1));
    Vector2D VB((m2i2 - m2i1)/(move2.t2 - move2.t1), (m2j2 - m2j1)/(move2.t2 - move2.t1));
    if(startTimeB > startTimeA)
    {
        A += VA*(startTimeB-startTimeA);
        startTimeA = startTimeB;
    }
    else if(startTimeB < startTimeA)
    {
        B += VB*(startTimeA - startTimeB);
        startTimeB = startTimeA;
    }
    double r(2*CN_AGENT_SIZE);
    Vector2D w(B - A);
    double c(w*w - r*r);
    if(c < 0)
        return true;

    Vector2D v(VA - VB);
    double a(v*v);
    double b(w*v);
    double dscr(b*b - a*c);
    if(dscr - CN_EPSILON < 0)
        return false;
    double ctime = (b - sqrt(dscr))/a;
    if(ctime > -CN_EPSILON && ctime < std::min(endTimeB,endTimeA) - startTimeA + CN_EPSILON)
        return true;
    return false;
}

double CBS::dist(int id1, int id2)
{
    int i1 = map->get_i(id1);
    int j1 = map->get_j(id1);
    int i2 = map->get_i(id2);
    int j2 = map->get_j(id2);
    return std::sqrt((i1-i2)*(i1-i2)+(j1-j2)*(j1-j2));
}



/*std::pair<std::vector<Move>,std::vector<Move>> CBS::find_similar_actions(Move a, Move b)
{
    std::vector<Move> moves_a = {a};
    std::vector<Move> moves_b = {b};
    auto na = map->get_valid_moves(a.id2);
    auto nb = map->get_valid_moves(b.id2);
    std::ofstream out;
    out.open("log_moves.xml", std::ios::app);
    out<<"\n\n";
    for(int i=0; i < std::max(na.size(), nb.size()); i++)
    {
        if(i < na.size())
        {
            bool no_col = false;
            Move new_move = Move(a.t1, a.t1+dist(a.id1, na[i].id), a.id1, na[i].id);
            out<<"\t\t\t<constraint i1=\""<<map->get_i(a.id1)<<"\" j1=\""<<map->get_j(a.id1)<<"\" i2=\""<<map->get_i(na[i].id)<<"\" j2=\""<<map->get_j(na[i].id)<<"\" t1=\""<<a.t1<<"\" c.t2=\""<<a.t2<<"\" positive=\""<<false<<"\" agent_id=\""<<0<<"\"/>\n";
            //std::cout<<map->get_i(a.id1)<<" "<<map->get_j(a.id1)<<" "<<map->get_i(a.id2)<<" "<<map->get_j(a.id2)<<" "<<map->get_i(na[i].id)<<" "<<map->get_j(na[i].id)<<" check coords\n";
            for(auto c:moves_b)
                if(!check_conflict(new_move, c))
                {
                    std::cout<<"no collision "<<moves_a.size()<<" "<<moves_b.size()<<"\n";
                    no_col = true;
                    break;
                }
            if(!no_col)
                moves_a.push_back(new_move);
        }
        if(i < nb.size())
        {
            bool no_col = false;
            Move new_move = Move(b.t1, b.t1+dist(b.id1, nb[i].id), b.id1, nb[i].id);
            out<<"\t\t\t<constraint i1=\""<<map->get_i(b.id1)<<"\" j1=\""<<map->get_j(b.id1)<<"\" i2=\""<<map->get_i(nb[i].id)<<"\" j2=\""<<map->get_j(nb[i].id)<<"\" t1=\""<<a.t1<<"\" c.t2=\""<<a.t2<<"\" positive=\""<<false<<"\" agent_id=\""<<0<<"\"/>\n";
            for(auto c:moves_a)
                if(!check_conflict(new_move, c))
                {
                    std::cout<<"no collision "<<moves_a.size()<<" "<<moves_b.size()<<"\n";
                    no_col = true;
                    break;
                }
            if(!no_col)
                moves_b.push_back(new_move);
        }
    }
    out.close();
    return {moves_a, moves_b};
}*/

std::vector<Constraint> CBS::get_multiconstraint(int agent, std::vector<Move> moves_a, std::vector<Move> moves_b)
{
    std::vector<Constraint> result;
    for(auto a:moves_a)
    {
        std::vector<Constraint> cons;
        double min_t = CN_INFINITY;
        for(auto b: moves_b)
        {
            auto c = get_constraint(agent, a, b);
            cons.push_back(c);
            if(c.t2 < min_t)
                min_t = c.t2;
        }
        result.emplace_back(Constraint(agent, a.t1, min_t, a.id1, a.id2));
    }
    return result;
}

Constraint CBS::get_wait_constraint(int agent, Move move1, Move move2)
{
    double radius = 2*config.agent_size;
    double i0(map->get_i(move2.id1)), j0(map->get_j(move2.id1)), i1(map->get_i(move2.id2)), j1(map->get_j(move2.id2)), i2(map->get_i(move1.id1)), j2(map->get_j(move1.id1));
    std::pair<double,double> interval;
    Point point(i2,j2), p0(i0,j0), p1(i1,j1);
    int cls = point.classify(p0, p1);
    double dist = fabs((i0 - i1)*j2 + (j1 - j0)*i2 + (j0*i1 - i0*j1))/sqrt(pow(i0 - i1, 2) + pow(j0 - j1, 2));
    double da = (i0 - i2)*(i0 - i2) + (j0 - j2)*(j0 - j2);
    double db = (i1 - i2)*(i1 - i2) + (j1 - j2)*(j1 - j2);
    double ha = sqrt(da - dist*dist);
    double size = sqrt(radius*radius - dist*dist);
    if(cls == 3)
    {
        interval.first = move2.t1;
        interval.second = move2.t1 + (sqrt(radius*radius - dist*dist) - ha);
    }
    else if(cls == 4)
    {
        interval.first = move2.t2 - sqrt(radius*radius - dist*dist) + sqrt(db - dist*dist);
        interval.second = move2.t2;
    }
    else if(da < radius*radius)
    {
        if(db < radius*radius)
        {
            interval.first = move2.t1;
            interval.second = move2.t2;
        }
        else
        {
            double hb = sqrt(db - dist*dist);
            interval.first = move2.t1;
            interval.second = move2.t2 - hb + size;
        }
    }
    else
    {
        if(db < radius*radius)
        {
            interval.first = move2.t1 + ha - size;
            interval.second = move2.t2;
        }
        else
        {
            interval.first = move2.t1 + ha - size;
            interval.second = move2.t1 + ha + size;
        }
    }
    return Constraint(agent, interval.first, interval.second, move1.id1, move1.id2);
}

double CBS::get_h(const std::list<Conflict> &conflicts, const std::map<int, double> &base_costs = {})
{
    optimization::Simplex simplex("simplex");
    std::map<int, int> colliding_agents;
    for(auto c: conflicts)
    {
        colliding_agents.insert({c.agent1, colliding_agents.size()});
        colliding_agents.insert({c.agent2, colliding_agents.size()});
    }

    pilal::Matrix coefficients(conflicts.size()+base_costs.size(), base_costs.size(), 0);
    std::vector<double> overcosts(conflicts.size()+base_costs.size());
    int i(0);
    for(auto c:conflicts)
    {
        coefficients.at(i, colliding_agents.at(c.agent1)) = 1;
        coefficients.at(i, colliding_agents.at(c.agent2)) = 1;
        overcosts[i] = c.overcost;
        i++;
    }
    for(auto c:base_costs)
    {
         coefficients.at(i, c.first) = 1;
         overcosts[i] = c.second;
         i++;
    }
    simplex.set_problem(coefficients, overcosts);
    simplex.solve();
    simplex.print_solution();
    return simplex.get_solution();
}

double CBS::get_hl_heuristic(const std::list<Conflict> &conflicts)
{
    if(conflicts.empty() || config.hlh_type == 0)
        return 0;
    else if (config.hlh_type == 1)
    {
        optimization::Simplex simplex("simplex");
        std::map<int, int> colliding_agents;
        for(auto c: conflicts)
        {
            colliding_agents.insert({c.agent1, colliding_agents.size()});
            colliding_agents.insert({c.agent2, colliding_agents.size()});
        }

        pilal::Matrix coefficients(conflicts.size(), colliding_agents.size(), 0);
        std::vector<double> overcosts(conflicts.size());
        int i(0);
        for(auto c:conflicts)
        {
            coefficients.at(i, colliding_agents.at(c.agent1)) = 1;
            coefficients.at(i, colliding_agents.at(c.agent2)) = 1;
            overcosts[i] = c.overcost;
            i++;
        }
        simplex.set_problem(coefficients, overcosts);
        simplex.solve();
        return simplex.get_solution();
    }
    else
    {
        double h_value(0);
        std::vector<std::tuple<double, int, int>> values;
        values.reserve(conflicts.size());
        std::set<int> used;
        for(auto c:conflicts)
            values.push_back(std::make_tuple(c.overcost, c.agent1, c.agent2));
        std::sort(values.begin(), values.end(), std::greater<std::tuple<double, int, int>>());
        for(auto v: values)
        {
            if(used.find(get<1>(v)) != used.end() || used.find(get<2>(v)) != used.end())
                continue;
            h_value += get<0>(v);
            used.insert(get<1>(v));
            used.insert(get<2>(v));
        }
        return h_value;
    }
}

Constraint CBS::get_constraint(int agent, Move move1, Move move2)
{
    if(move1.id1 == move1.id2)
        return get_wait_constraint(agent, move1, move2);
    double startTimeA(move1.t1), endTimeA(move1.t2);
    Vector2D A(map->get_i(move1.id1), map->get_j(move1.id1)), A2(map->get_i(move1.id2), map->get_j(move1.id2)),
             B(map->get_i(move2.id1), map->get_j(move2.id1)), B2(map->get_i(move2.id2), map->get_j(move2.id2));
    if(move2.t2 == CN_INFINITY)
        return Constraint(agent, move1.t1, CN_INFINITY, move1.id1, move1.id2);
    double delta = move2.t2 - move1.t1;
    while(delta > config.precision/2.0)
    {
        if(check_conflict(move1, move2))
        {
            move1.t1 += delta;
            move1.t2 += delta;
        }
        else
        {
            move1.t1 -= delta;
            move1.t2 -= delta;
        }
        if(move1.t1 > move2.t2 + CN_EPSILON)
        {
            move1.t1 = move2.t2;
            move1.t2 = move1.t1 + endTimeA - startTimeA;
            break;
        }
        delta /= 2.0;
    }
    if(delta < config.precision/2.0 + CN_EPSILON && check_conflict(move1, move2))
    {
        move1.t1 = fmin(move1.t1 + delta*2, move2.t2);
        move1.t2 = move1.t1 + endTimeA - startTimeA;
    }
    return Constraint(agent, startTimeA, move1.t1, move1.id1, move1.id2);
}

Conflict CBS::get_conflict(std::list<Conflict> &conflicts)
{
    auto best_it = conflicts.begin();
    for(auto it = conflicts.begin(); it != conflicts.end(); it++)
    {
        if(it->overcost > 0)
        {
            if(best_it->overcost < it->overcost || (fabs(best_it->overcost - it->overcost) < CN_EPSILON && best_it->t < it->t))
                best_it = it;
        }
        else if(best_it->t < it->t)
            best_it = it;
    }

    Conflict conflict = *best_it;
    conflicts.erase(best_it);
    return conflict;
}

Solution CBS::find_solution(const Map &map, const Task &task, const Config &cfg)
{
    config = cfg;
    config.connectdness = -1;
    config.use_multicons = true;
    config.use_disjoint_splitting = false;
    config.timelimit = 100;
    this->map = &map;
    if(config.connectdness > 0)
    {
        h_values.init(map.get_size(), task.get_agents_size());
        for(int i = 0; i < int(task.get_agents_size()); i++)
        {
            Agent agent = task.get_agent(i);
            h_values.count(map, agent);
        }
    }
    else //any-angle case
    {
        aa_h_values.resize(task.get_agents_size());
        for(int i = 0; i < int(task.get_agents_size()); i++)
        {
            Agent agent = task.get_agent(i);
            aa_h_values[i].init(map.get_width(), map.get_height());
            aa_h_values[i].count(map, agent);
        }
    }
    auto t = std::chrono::high_resolution_clock::now();
    int cardinal_solved = 0, semicardinal_solved = 0;
    std::cout<<"ROOT\n";
    if(!this->init_root(map, task))
        return solution;
    std::cout<<"ROOT INITIALIZED\n";
    solution.init_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    solution.found = true;
    CBS_Node *goal;
    int expanded(1);
    double time(0);
    while (true)
    {
        goal = expand(tree, task);
        if(goal != nullptr)
            break;
        auto time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
        //std::cout<<expanded<<" "<<time_spent.count()<<"\n";
        if(time_spent.count() > config.timelimit)
        {
            solution.found = false;
            solution.low_level_expansions = low_level_searches;
            solution.low_level_expanded = double(low_level_expanded)/std::max(low_level_searches, 1);
            solution.high_level_expanded = expanded;
            solution.high_level_generated = int(tree->get_size());
            solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
            solution.check_time = time;
            return solution;
        }
        expanded++;
    }
    //validate_paths(goal, task);
    //std::cout<<"check all rest nodes\n";
    //std::cout<<tree->get_open_size()<<"\n";
    /*while(tree->get_open_size() > 0)
    {
        auto parent = tree->get_front();
        std::cout<<parent->id<<" "<<parent->cost<<" check node\n";
        validate_paths(parent, task);
        tree->pop_front();
    }*/
    for(int i=0; i<task.get_agents_size(); i++)
    {
        auto cons = get_constraints(goal, i);
        for(auto c:cons)
            std::cout<<c.agent<<" "<<c.id1<<" "<<c.id2<<" "<<c.t1<<" "<<c.t2<<"\n";
    }
    solution.paths = get_paths(goal, task.get_ids());
    for(auto path:solution.paths)
    {
        for(auto p:path.second.nodes)
            std::cout<<"{"<<p.id<<","<<p.g<<"},";
        std::cout<<std::endl;
    }
    auto n = goal;
    while(n)
    {
        std::cout<<n->id<<" ";
        n = n->parent;
    }
    std::cout<<" ids of solution sequence\n";
    solution.flowtime = goal->cost;
    solution.low_level_expansions = low_level_searches;
    solution.low_level_expanded = double(low_level_expanded)/std::max(low_level_searches, 1);
    solution.high_level_expanded = expanded;
    solution.high_level_generated = int(tree->get_size());
    for(auto path:solution.paths)
        solution.makespan = (solution.makespan > path.second.cost) ? solution.makespan : path.second.cost;
    solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    solution.check_time = time;
    solution.cardinal_solved = cardinal_solved;
    solution.semicardinal_solved = semicardinal_solved;
    std::ofstream out;
    out.open("log.txt", std::ios::app);
    out<<task.get_agents_size()<<" "<<solution.found<<" "<<solution.time.count()<<" "<<solution.makespan<<" "<<solution.flowtime<<" "<<solution.init_cost<<" "<<solution.initial_conflicts<<" "<<solution.check_time<<" "<<solution.high_level_expanded<<" "<<solution.low_level_expansions<<" "<<solution.low_level_expanded<<" "<<goal->total_cons<<"\n";
    out.close();
    return solution;
}

bool CBS::check_positive_constraints(std::list<Constraint> constraints, Constraint constraint)
{
    std::list<Constraint> positives;
    for(auto c: constraints)
        if(c.positive && c.agent == constraint.agent)
            positives.push_back(c);

    for(auto p: positives)
    {
        if(p.id1 == constraint.id1 && p.id2 == constraint.id2 && p.t1 - CN_EPSILON < constraint.t1 && p.t2 + CN_EPSILON > constraint.t2) // agent needs to perform two equal actions simultaneously => it's impossible
            return false;
        if(p.id1 == constraint.id1 && p.id2 == constraint.id2 && constraint.t1 - CN_EPSILON < p.t1 && constraint.t2 + CN_EPSILON > p.t2)
            return false;
    }
    return true;
}

bool CBS::validate_constraints(std::list<Constraint> constraints, int agent)
{
    std::list<Constraint> positives;
    for(auto c: constraints)
        if(c.positive && c.agent == agent)
            positives.push_back(c);
    for(auto p: positives)
        for(auto c: constraints)
        {
            if(c.positive)
                continue;
            if(p.agent == c.agent && p.id1 == c.id1 && p.id2 == c.id2) //if the same action
                if(p.t1 > c.t1 - CN_EPSILON && p.t2 < c.t2 + CN_EPSILON) //if the whole positive interval is inside collision interval
                    return false;
        }
    return true;
}

void CBS::find_new_conflicts(const Map &map, const Task &task, CBS_Node &node, std::map<int,sPath> &paths, const sPath &path,
                             const std::list<Conflict> &conflicts)
{
    //std::cout<<"find conflicts "<<paths.size()<<" "<<path.agentID<<" "<<conflicts.size()<<" "<<config.use_cardinal<<"\n";
    auto oldpath = paths[path.agentID];
    paths[path.agentID] = path;
    auto new_conflicts = get_all_conflicts(paths, path.agentID);
    paths[path.agentID] = oldpath;
    std::list<Conflict> conflictsA({}), semicard_conflictsA({}), cardinal_conflictsA({});
    for(auto c: conflicts)
        if(c.agent1 != path.agentID && c.agent2 != path.agentID)
            conflictsA.push_back(c);
    node.conflicts = conflictsA;
    for(auto n:new_conflicts)
        node.conflicts.push_back(n);
    node.cardinal_conflicts.clear();
    node.semicard_conflicts.clear();
    node.conflicts_num = node.conflicts.size();
    return;
}

std::list<Constraint> CBS::get_constraints(CBS_Node *node, int agent_id)
{
    CBS_Node* curNode = node;
    std::list<Constraint> constraints(0);
    while(curNode->parent != nullptr)
    {
        if(agent_id < 0 || curNode->constraint.agent == agent_id)
        {
            if(curNode->constraints.empty())
                constraints.push_back(curNode->constraint);
            else
                for(auto c: curNode->constraints)
                    constraints.push_back(c);
        }
        if(curNode->positive_constraint.agent == agent_id)
            constraints.push_back(curNode->positive_constraint);
        curNode = curNode->parent;
    }
    return constraints;
}


Conflict CBS::check_paths(const sPath &pathA, const sPath &pathB)
{
    unsigned int a(0), b(0);
    auto nodesA = pathA.nodes;
    auto nodesB = pathB.nodes;
    if(pathA.nodes.size()*pathB.nodes.size() == 0)
        return Conflict();
    while(a < nodesA.size() - 1 || b < nodesB.size() - 1)
    {
        double dist = sqrt(pow(map->get_i(nodesA[a].id) - map->get_i(nodesB[b].id), 2) + pow(map->get_j(nodesA[a].id) - map->get_j(nodesB[b].id), 2)) - CN_EPSILON;
        if(a < nodesA.size() - 1 && b < nodesB.size() - 1) // if both agents have not reached their goals yet
        {
            if(dist < (nodesA[a+1].g - nodesA[a].g) + (nodesB[b+1].g - nodesB[b].g) + CN_AGENT_SIZE*2)
                if(check_conflict(Move(nodesA[a], nodesA[a+1]), Move(nodesB[b], nodesB[b+1])))
                    return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a+1]), Move(nodesB[b], nodesB[b+1]), std::min(nodesA[a].g, nodesB[b].g));
        }
        else if(a == nodesA.size() - 1) // if agent A has already reached the goal
        {
            if(dist < (nodesB[b+1].g - nodesB[b].g) + CN_AGENT_SIZE*2)
                if(check_conflict(Move(nodesA[a].g, CN_INFINITY, nodesA[a].id, nodesA[a].id), Move(nodesB[b], nodesB[b+1])))
                    return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a].g, CN_INFINITY, nodesA[a].id, nodesA[a].id), Move(nodesB[b], nodesB[b+1]), std::min(nodesA[a].g, nodesB[b].g));
        }
        else if(b == nodesB.size() - 1) // if agent B has already reached the goal
        {
            if(dist < (nodesA[a+1].g - nodesA[a].g) + CN_AGENT_SIZE*2)
                if(check_conflict(Move(nodesA[a], nodesA[a+1]), Move(nodesB[b].g, CN_INFINITY, nodesB[b].id, nodesB[b].id)))
                    return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a+1]), Move(nodesB[b].g, CN_INFINITY, nodesB[b].id, nodesB[b].id), std::min(nodesA[a].g, nodesB[b].g));
        }
        if(a == nodesA.size() - 1)
            b++;
        else if(b == nodesB.size() - 1)
            a++;
        else if(fabs(nodesA[a+1].g - nodesB[b+1].g) < CN_EPSILON)
        {
            a++;
            b++;
        }
        else if(nodesA[a+1].g < nodesB[b+1].g)
            a++;
        else if(nodesB[b+1].g - CN_EPSILON < nodesA[a+1].g)
            b++;
    }
    return Conflict();
}

std::vector<Conflict> CBS::get_all_conflicts(const std::map<int, sPath> &paths, int id)
{
    std::vector<Conflict> conflicts;
    //check all agents
    if(id < 0)
        for(auto p1:paths)
            for(auto p2:paths)
            {
                if(p1.first >= p2.first)
                    continue;
                Conflict conflict = check_paths(p1.second, p2.second);
                if(conflict.agent1 >= 0)
                    conflicts.push_back(conflict);
            }
    else
    {
        for(auto p1:paths)
        {
            if(int(p1.first) == id)
                continue;
            Conflict conflict = check_paths(p1.second, paths.at(id));
            if(conflict.agent1 >= 0)
                conflicts.push_back(conflict);
        }
    }
    return conflicts;
}

double CBS::get_cost(CBS_Node node, int agent_id)
{
    while(node.parent != nullptr)
    {
        if(node.paths.begin()->second.agentID == agent_id)
            return node.paths.begin()->second.cost;
        node = *node.parent;
    }
    return node.paths.at(agent_id).cost;
}

std::map<int, sPath> CBS::get_paths(CBS_Node *node, std::vector<int> ids)
{
    CBS_Node* curNode = node;
    std::map<int, sPath> paths;
    while(curNode->parent != nullptr)
    {
        if(paths.find(curNode->paths.begin()->second.agentID) == paths.end())
        {
            paths.insert({curNode->paths.begin()->second.agentID, curNode->paths.begin()->second});
        }
        curNode = curNode->parent;
    }
    for(int id:ids)
    {
        if(paths.find(id) == paths.end())
            paths[id] = curNode->paths[id];
    }
    return paths;
}

void CBS::add_agent(CBS_Tree *tree, Task &task, int agent_id, double cost)
{
    sPath path = tree->tree.begin()->paths.at(agent_id);
    std::cout<<agent_id<<" "<<tree->tree.size()<<" "<<tree->get_size()<<" add agent\n";
    CT_container new_open;
    for(auto it:tree->tree)
    {
        std::cout<<it.id<<" "<<it.cost<<" "<<it.paths.size()<<" "<<it.conflicts.size()<<" "<<it.cardinal_conflicts.size()<<" "<<it.semicard_conflicts.size()<<"\n";
        if(it.cardinal_conflicts.size() > 0)
            std::cout<<it.cardinal_conflicts.front().agent1<<" "<<it.cardinal_conflicts.front().agent2<<" "<<it.cardinal_conflicts.front().overcost<<" CARDINAL SHIT?\n";
    }
    for(auto it = tree->container.get<0>().begin(); it != tree->container.get<0>().end(); it++)
    {
        //std::cout<<" modify element of open\n";
        auto node = it->tree_pointer;
        std::cout<<"get paths "<<node->cost<<" "<<node->id<<" "<<node->paths.size()<<"\n";
        auto paths = get_paths(node, task.get_ids());
        std::cout<<"find new conflicts\n";
        find_new_conflicts(*map, task, *node, paths, path, node->conflicts);
        std::cout<<node->conflicts.size()<<" "<<node->cardinal_conflicts.size()<<" "<<node->semicard_conflicts.size()<<" "<<config.use_cardinal<<" conflicts found\n";
        //for(auto c: node->conflicts)
        //    std::cout<<c.agent1<<" vs "<<c.agent2<<"\n";
        node->conflicts_num = node->conflicts.size();
        node->h = fmax(0, cost - it->tree_pointer->cost);
        node->cost = fmax(cost, node->cost) + path.cost;
        std::cout<<node->id<<" "<<node->cost<<" "<<cost<<" "<<path.cost<<" "<<node->h<<" costs\n";
        new_open.insert(Open_Elem(node, node->id, node->cost, node->total_cons, node->conflicts_num));
    }
    tree->container.clear();
    tree->container = new_open;
}

bool CBS::validate_cons(CBS_Node* node, const Task &task)
{
    std::cout<<"Check node "<<node->id<<" with cost "<<node->cost<<"\n";
    auto paths = get_paths(node, task.get_ids());
    for(auto c:v_cons)
        for(auto p:paths)
        {
            if(p.second.agentID != c.agent)
                continue;
            for(int i=0; i + 1 < p.second.nodes.size(); i++)
            {
                auto n1 = p.second.nodes[i];
                auto n2 = p.second.nodes[i+1];
                if(n1.id == c.id1 && n2.id == c.id2)
                    if(n1.g + CN_EPSILON > c.t1 && n1.g - CN_EPSILON < c.t2)
                    {
                        std::cout<<"Node contains a forbidden move of agent"<<i<<": ("<<n1.id<<","<<n2.id<<") which is performed in t="<<n1.g<<" that belongs to interval ["<<c.t1<<","<<c.t2<<")\n";
                        return false;
                    }
            }
        }
    return true;
}

bool CBS::validate_paths(CBS_Node *node, const Task &task)
{
    for(int i=0; i < task.get_agents_size(); i++)
        for(auto c:get_constraints(node, i))
            for(int j=0; j < v_paths[i].size(); j++)
            {
                auto n1 = v_paths[i][j];
                auto n2 = v_paths[i][j+1];

                if(n1.id == c.id1 && n2.id == c.id2)
                    if(n1.g + 1e-4 > c.t1 && n1.g - 1e-4 < c.t2)
                    {
                        std::cout<<"Node contains a forbidden move of agent"<<i<<": ("<<n1.id<<","<<n2.id<<") which is performed in t="<<n1.g<<" that belongs to interval ["<<c.t1<<","<<c.t2<<")\n";
                        return false;
                    }
            }
    return true;
}

CBS_Node* CBS::expand(CBS_Tree *tree, const Task &task)
{
    auto parent = tree->get_front();
    auto node = *parent;
    node.cost -= node.h;
    //std::cout<<all_nodes.size()<<" "<<node.id<<" "<<node.cost<<" "<<node.conflicts_num<<" "<<node.conflicts.size()<<" "<<node.cardinal_conflicts.size()<<" "<<node.semicard_conflicts.size()<<" "<<node.total_cons<<" NODE\n";
    parent->conflicts.clear();
    parent->cardinal_conflicts.clear();
    parent->semicard_conflicts.clear();
    auto paths = get_paths(&node, task.get_ids());
    //for(auto p:paths)
    //    std::cout<<p.second.cost<<" ";
    //std::cout<<'\n';
    /*if(validate_paths(parent, task) && paths[3].cost > 10.05)
    {
        auto cons = get_constraints(&node, 3);
        for(auto c: cons)
            std::cout<<c.t1<<","<<c.t2<<","<<c.id1<<","<<c.id2<<"\n";
    }*/
    auto time_now = std::chrono::high_resolution_clock::now();
    auto conflicts = node.conflicts;
    auto cardinal_conflicts = node.cardinal_conflicts;
    auto semicard_conflicts = node.semicard_conflicts;

    //expanded++;
    //std::cout<<"conflict between "<<conflict.move1.id1<<" "<<conflict.move2.id2<<" and "<<conflict.move2.id1<<" "<<conflict.move2.id2<<"\n";
    if(conflicts.empty() || node.id < 100)
    {
        std::ofstream out;
        out.open("log_multi.xml", std::ios::app);
        out<<"\t\t<node id=\""<<node.id<<"\" cost=\""<<node.cost<<"\" constraints_num=\""<<node.total_cons<<"\">\n";
        for(auto p:paths)
        {
            out<<"\t\t\t<path id=\""<<p.first<<"\">\n";
            for(int i=0; i<p.second.nodes.size()-1; i++)
            {
                auto n1 = p.second.nodes[i];
                auto n2 = p.second.nodes[i+1];
                out<<"\t\t\t\t<section i1=\""<<map->get_i(n1.id)<<"\" j1=\""<<map->get_j(n1.id)<<"\" i2=\""<<map->get_i(n2.id)<<"\" j2=\""<<map->get_j(n2.id)<<"\" duration=\""<<n2.g-n1.g<<"\"/>\n";
            }
            out<<"\t\t\t</path>\n";
        }
        for(int i=0; i<task.get_agents_size(); i++)
        {
            auto cons = get_constraints(&node, i);
            for(auto c: cons)
                out<<"\t\t\t<constraint i1=\""<<map->get_i(c.id1)<<"\" j1=\""<<map->get_j(c.id1)<<"\" i2=\""<<map->get_i(c.id2)<<"\" j2=\""<<map->get_j(c.id2)<<"\" t1=\""<<c.t1<<"\" c.t2=\""<<c.t2<<"\" positive=\""<<c.positive<<"\" agent_id=\""<<i<<"\"/>\n";
        }
        out<<"\t\t</node>\n";
        out.close();
    }
    Conflict conflict;
    if(conflicts.empty() && semicard_conflicts.empty() && cardinal_conflicts.empty())
    {
        std::cout<<node.id<<' '<<node.cost<<" "<<node.conflicts_num<<" "<<node.total_cons<<" GOAL\n";
        for(auto p:paths)
            std::cout<<p.second.agentID<<" "<<p.second.cost<<"\n";
        std::cout<<"solution without conflicts found\n";
        return parent; //i.e. no conflicts => solution found
    }
    tree->pop_front();
    if(!cardinal_conflicts.empty())
    {
        conflict = get_conflict(cardinal_conflicts);
        //cardinal_solved++;
    }
    else if(!semicard_conflicts.empty())
    {
        conflict = get_conflict(semicard_conflicts);
        //semicardinal_solved++;
    }
    else
        conflict = get_conflict(conflicts);
    //std::cout<<"conflict "<<conflict.move1.id1 <<" "<<conflict.move1.t1<<" "<<conflict.move1.id2<<" "<<conflict.move1.t2<<" vs "<<conflict.move2.id1 <<" "<<conflict.move2.t1<<" "<<conflict.move2.id2<<" "<<conflict.move2.t2<<"\n";
    std::list<Constraint> constraintsA = get_constraints(&node, conflict.agent1);
    Constraint constraintA(get_constraint(conflict.agent1, conflict.move1, conflict.move2));
    std::vector<Constraint> multiconstraintA, multiconstraintB;
    std::list<Constraint> constraintsB = get_constraints(&node, conflict.agent2);
    Constraint constraintB(get_constraint(conflict.agent2, conflict.move2, conflict.move1));
    if(config.use_multicons)
    {
        auto actions = find_similar_actions(conflict.move1, conflict.move2);
        //auto actions = find_similar_actions(task, conflict, constraintsA, constraintsB);
        //for(auto s:actions.first)
        //    std::cout<<s.id1<<" "<<s.id2<<" "<<s.t1<<" "<<s.t2<<" similar action\n";
        multiconstraintA = get_multiconstraint(conflict.agent1, actions.first, actions.second);
        for(auto c: multiconstraintA)
            constraintsA.push_back(c);
        multiconstraintB = get_multiconstraint(conflict.agent2, actions.second, actions.first);
        for(auto c: multiconstraintB)
            constraintsB.push_back(c);
        //std::cout<<constraintsA.size()<<" "<<constraintsB.size()<<" consA and consB size\n";
    }
    else
    {
        //auto actions = find_similar_actions(conflict.move1, conflict.move2);
        //multiconstraintA = get_multiconstraint(conflict.agent1, actions.first, actions.second);
        //for(Constraint c: multiconstraintA)
        //    std::cout<<c.agent<<" "<<c.id1<<" "<<c.id2<<" "<<c.t1<<" "<<c.t2<<" multicons\n";
        constraintsA.push_back(constraintA);
        constraintsB.push_back(constraintB);
    }
    //for(auto id:task.get_ids())
    //{
    //    auto constraints = get_constraints(&node, id);
        //for(auto c: constraints)
        //    std::cout<<c.agent<<" "<<c.id1<<" "<<c.t1<<" "<<c.id2<<" "<<c.t2<<" old c\n";
    //}
    //std::cout<<conflict.agent1<<' '<<constraintsA.size()<<" constraints "<<constraintA.id1<<" "<<constraintA.id2<<" constraint A\n";
    sPath pathA;
    if(config.connectdness > 0)
        pathA = planner.find_path(task.get_agent(conflict.agent1), *map, constraintsA, h_values);
    else
        pathA = aa_planner.find_path(task.get_agent(conflict.agent1), *map, constraintsA, aa_h_values[conflict.agent1]);
    low_level_searches++;
    low_level_expanded += pathA.expanded;
    //std::cout<<constraintA.agent<<" "<<constraintA.id1<<" "<<constraintA.t1<<" "<<constraintA.id2<<" "<<constraintA.t2<<" new c\n";
    //std::cout<<tree->get_size()<<" "<<pathA.agentID<<" "<<pathA.expanded<<" "<<pathA.cost<<" expanded\n";
    //std::cout<<conflict.agent2<<' '<<constraintsB.size()<<" constraints "<<constraintB.id1<<" "<<constraintB.id2<<" constraint B\n";
    sPath pathB;
    if(config.connectdness > 0)
        pathB = planner.find_path(task.get_agent(conflict.agent2), *map, constraintsB, h_values);
    else
        pathB = aa_planner.find_path(task.get_agent(conflict.agent2), *map, constraintsB, aa_h_values[conflict.agent2]);
    /*for(auto p:pathB.nodes)
        std::cout<<p.id<<" "<<p.g<<"->";
    std::cout<<"\n";
    for(auto p:pathA.nodes)
        std::cout<<p.id<<" "<<p.g<<"->";
    std::cout<<"\n";*/
    //for(auto c: constraintsB)
    //    std::cout<<c.id1<<" "<<c.t1<<"  "<<c.id2<<" "<<c.t2<<"\n";
    //std::cout<<"\n";
    low_level_searches++;
    low_level_expanded += pathB.expanded;
    //std::cout<<constraintB.agent<<" "<<constraintB.id1<<" "<<constraintB.t1<<" "<<constraintB.id2<<" "<<constraintB.t2<<" new c\n";
    //std::cout<<tree->get_size()+1<<" "<<pathB.agentID<<" "<<pathB.expanded<<" "<<pathB.cost<<" expanded\n";

    std::map<int, sPath> pathsA; pathsA[pathA.agentID] = pathA;
    CBS_Node right(pathsA, parent, constraintA, node.cost + pathA.cost - get_cost(node, conflict.agent1), 0, node.total_cons + 1);
    std::map<int, sPath> pathsB; pathsB[pathB.agentID] = pathB;
    CBS_Node left(pathsB, parent, constraintB, node.cost + pathB.cost - get_cost(node, conflict.agent2), 0, node.total_cons + 1);
    Constraint positive;
    if(config.use_multicons)
    {
        right.constraints = multiconstraintA;
        left.constraints = multiconstraintB;
        right.total_cons += multiconstraintA.size() - 1;
        left.total_cons += multiconstraintB.size() - 1;
    }
    bool inserted = false;
    bool left_ok = true, right_ok = true;
    if(config.use_disjoint_splitting)
    {
        int agent1positives(0), agent2positives(0);
        for(auto c: constraintsA)
            if(c.positive)
                agent1positives++;
        for(auto c: constraintsB)
            if(c.positive)
                agent2positives++;
        if(conflict.move1.id1 != conflict.move1.id2 && agent2positives > agent1positives && pathA.cost > 0)
        {
            positive = Constraint(conflict.agent1, constraintA.t1, constraintA.t2, conflict.move1.id1, conflict.move1.id2, true);
            if(check_positive_constraints(constraintsA, positive))
            {
                left.positive_constraint = positive;
                left.total_cons++;
                constraintsB.push_back(left.positive_constraint);
                inserted = true;
                //std::cout<<"added positive to "<<positive.agent<<"\n\n";
            }
            //else
            //    right_ok = false;
        }
        if(conflict.move2.id1 != conflict.move2.id2 && !inserted && pathB.cost > 0)
        {
            positive = Constraint(conflict.agent2, constraintB.t1, constraintB.t2, conflict.move2.id1, conflict.move2.id2, true);
            if(check_positive_constraints(constraintsB, positive))
            {
                right.positive_constraint = positive;
                right.total_cons++;
                constraintsA.push_back(right.positive_constraint);
                inserted = true;
            }
            //else
            //    left_ok = false;
        }
        if(conflict.move1.id1 != conflict.move1.id2 && !inserted && pathA.cost > 0)
        {
            positive = Constraint(conflict.agent1, constraintA.t1, constraintA.t2, conflict.move1.id1, conflict.move1.id2, true);
            if(check_positive_constraints(constraintsA, positive))
            {
                inserted = true;
                left.positive_constraint = positive;
                left.total_cons++;
                constraintsB.push_back(left.positive_constraint);
            }
            //else
            //    right_ok = false;
        }
    }
    right.id_str = node.id_str + "0";
    left.id_str = node.id_str + "1";
    right.id = tree->get_size();
    left.id = tree->get_size()+1;
    if(right_ok && pathA.cost > 0 && validate_constraints(constraintsA, pathA.agentID))
    {
        find_new_conflicts(*map, task, right, paths, pathA, conflicts);
        if(right.cost > 0)
        {
            right.h = get_hl_heuristic(right.cardinal_conflicts);
            right.cost += right.h;
            right.id = all_nodes.size();
            all_nodes.push_back(right);
            //validate_cons(&right, task);
            tree->add_node(&all_nodes.back());
        }
    }
    if(left_ok && pathB.cost > 0 && validate_constraints(constraintsB, pathB.agentID))
    {
        find_new_conflicts(*map, task, left, paths, pathB, conflicts);
        if(left.cost > 0)
        {
            left.h = get_hl_heuristic(left.cardinal_conflicts);
            left.cost += left.h;
            left.id = all_nodes.size();
            all_nodes.push_back(left);
            //validate_cons(&left, task);
            tree->add_node(&all_nodes.back());
        }
    }
    return nullptr;
}


Solution CBS::find_solution_new(const Map &map, const Task &task, const Config &cfg)
{
    config = cfg;
    config.use_cardinal = false;
    this->map = &map;
    h_values.init(map.get_size(), task.get_agents_size());
    //h_values.resize(task.get_agents_size());
    for(int i = 0; i < int(task.get_agents_size()); i++)
    {
        Agent agent = task.get_agent(i);
        h_values.count(map, agent);
    }
    auto t = std::chrono::high_resolution_clock::now();
    int cardinal_solved = 0, semicardinal_solved = 0;
    std::cout<<"ROOT\n";
    if(!this->init_root(map, task))
        return solution;
    std::cout<<"ROOT INITIALIZED\n";
    solution.init_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    solution.found = true;
    CBS_Node node = *tree->get_front();
    CBS_Node *goal = nullptr;
    CBS_Tree *best_tree = nullptr;
    Conflict best_conflict;
    Solution best_solution;
    std::list<Conflict> conflicts;
    for(auto c: node.conflicts)
    {
        Task new_task;
        new_task.add_agent(task.get_agent(c.agent1));
        new_task.add_agent(task.get_agent(c.agent2));
        CBS cbs;
        cbs.h_values = h_values;
        auto solution = cbs.find_solution(map, new_task, config);
        //std::cout<<c.agent1<<" "<<c.agent2<<" "<<solution.init_cost<<" "<<solution.flowtime<<"\n";
        if(best_tree == nullptr || cbs.tree->get_size() > best_tree->get_size())
        {
            best_tree = cbs.tree;
            best_conflict = c;
            best_solution = solution;
        }
        c.overcost = solution.flowtime;
        //std::cout<<c.agent1<<" "<<c.agent2<<" "<<c.overcost<<" count H\n";
        conflicts.push_back(c);
    }

    std::map<int, double> base_costs;
    for(auto p: node.paths)
        base_costs[p.first] = p.second.cost;
    //std::cout<<"check "<<conflicts.size()<<" "<<base_costs.size()<<"\n";
    //std::cout<<get_h(conflicts, base_costs)<<" "<<best_solution.flowtime<<" summary minimal cost\n";
    //std::cout<<"best tree found\n";
    Task new_task;
    new_task.add_agent(task.get_agent(best_conflict.agent1));
    new_task.add_agent(task.get_agent(best_conflict.agent2));
    for(auto id: task.get_ids())
    {
        auto ids = new_task.get_ids();
        if(std::find(ids.begin(), ids.end(), id) == ids.end())
        {
            std::cout<<"conflict found\n";
            best_tree->tree.begin()->paths[id] = tree->tree.begin()->paths.at(id); //костыль
            std::cout<<id<<" add new agent\n";
            new_task.add_agent(task.get_agent(id));
            std::cout<<"modify tree\n";
            add_agent(best_tree, new_task, id, best_tree->container.get<0>().begin()->cost);
        }
        while (true)
        {
            goal = expand(best_tree, new_task);
            if(goal != nullptr)
                break;
            auto time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
            if(time_spent.count() > config.timelimit)
            {
                solution.found = false;
                break;
            }
        }
    }

    if(goal != nullptr)
    {

        std::cout<<goal->id<<" goal\n";
        auto paths = get_paths(goal, new_task.get_ids());
        std::cout<<paths.size()<<" paths resolved\n";
        auto all_paths = get_paths(&node, task.get_ids());
        for(auto p:paths)
        {
            std::cout<<p.second.agentID<<" "<<p.second.cost<<"\n";
            all_paths[p.first] = p.second;
        }
        solution.paths = all_paths;
        double cost = 0;
        for(auto p:all_paths)
            cost += p.second.cost;
        solution.flowtime = cost;
    }
    solution.low_level_expansions = low_level_searches;
    solution.low_level_expanded = double(low_level_expanded)/std::max(low_level_searches, 1);
    solution.high_level_expanded = best_tree->get_size() - best_tree->get_open_size();
    solution.high_level_generated = int(tree->get_size());
    for(auto path:solution.paths)
        solution.makespan = (solution.makespan > path.second.cost) ? solution.makespan : path.second.cost;
    solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    solution.check_time = 0;
    solution.cardinal_solved = cardinal_solved;
    solution.semicardinal_solved = semicardinal_solved;

    return solution;
}

std::pair<std::vector<Move>,std::vector<Move>> CBS::find_similar_actions(Move a, Move b)
{
    std::vector<Move> moves_a = {a};
    std::vector<Move> moves_b = {b};
    LineOfSight los;
    auto cells_a = los.getCellsCrossedByLine(map->get_i(a.id1), map->get_j(a.id1), map->get_i(a.id2), map->get_j(a.id2), *map);
    auto cells_b = los.getCellsCrossedByLine(map->get_i(b.id1), map->get_j(b.id1), map->get_i(b.id2), map->get_j(b.id2), *map);
    int i(1);
    std::ofstream out;
    out.open("log_moves.xml", std::ios::app);
    out<<"\n\n";
    Constraint initA = get_constraint(0, a, b);
    Constraint initB = get_constraint(0, b, a);
    while(i < cells_a.size() or i < cells_b.size())
    {
        if(i < cells_a.size())
        {
            int id_a = map->get_id(cells_a[cells_a.size() - i].first, cells_a[cells_a.size() - i].second);
            Move new_move = Move(a.t1, a.t1+dist(a.id1, id_a), a.id1, id_a);
            bool has_col = true;
            Constraint con;
            for(auto c:moves_b)
                if(!check_conflict(new_move, c))
                {
                    has_col = false;
                    break;
                }
            if(has_col)
            {
                Constraint newB = get_constraint(0, b, new_move);
                if(newB.t2 >= initB.t2)
                    moves_a.push_back(new_move);
                out<<"\t\t\t<constraint i1=\""<<map->get_i(a.id1)<<"\" j1=\""<<map->get_j(a.id1)<<"\" i2=\""<<map->get_i(id_a)<<"\" j2=\""<<map->get_j(id_a)<<"\" t1=\""<<a.t1<<"\" c.t2=\""<<a.t2<<"\" positive=\""<<false<<"\" agent_id=\""<<0<<"\"/>\n";
            }
        }
        if(i < cells_b.size())
        {
            int id_b = map->get_id(cells_b[cells_b.size() - i].first, cells_b[cells_b.size() - i].second);
            Move new_move = Move(b.t1, b.t1+dist(b.id1, id_b), b.id1, id_b);
            bool has_col = true;
            for(auto c:moves_a)
                if(!check_conflict(new_move, c))
                {
                    has_col = false;
                    break;
                }
            if(has_col)
            {
                Constraint newA = get_constraint(0, a, new_move);
                if(newA.t2 >= initA.t2)
                    moves_b.push_back(new_move);
                out<<"\t\t\t<constraint i1=\""<<map->get_i(b.id1)<<"\" j1=\""<<map->get_j(b.id1)<<"\" i2=\""<<map->get_i(id_b)<<"\" j2=\""<<map->get_j(id_b)<<"\" t1=\""<<a.t1<<"\" c.t2=\""<<a.t2<<"\" positive=\""<<false<<"\" agent_id=\""<<0<<"\"/>\n";
            }
        }
        i++;
    }
    i = 1;
    while(i < cells_a.size() or i < cells_b.size())
    {
        if(i < cells_a.size())
        {
            int id_a = map->get_id(cells_a[cells_a.size() - i].first, cells_a[cells_a.size() - i].second);
            Move new_move = Move(a.t1 + dist(a.id1, id_a), a.t1+dist(a.id1, id_a)+dist(a.id2, id_a), id_a, a.id2);
            bool has_col = true;
            Constraint con;
            for(auto c:moves_b)
                if(!check_conflict(new_move, c))
                {
                    has_col = false;
                    break;
                }
            if(has_col)
            {
                Constraint newB = get_constraint(0, b, new_move);
                if(newB.t2 >= initB.t2)
                    moves_a.push_back(new_move);
                out<<"\t\t\t<constraint i1=\""<<map->get_i(a.id1)<<"\" j1=\""<<map->get_j(a.id1)<<"\" i2=\""<<map->get_i(id_a)<<"\" j2=\""<<map->get_j(id_a)<<"\" t1=\""<<a.t1<<"\" c.t2=\""<<a.t2<<"\" positive=\""<<false<<"\" agent_id=\""<<0<<"\"/>\n";
            }
        }
        if(i < cells_b.size())
        {
            int id_b = map->get_id(cells_b[cells_b.size() - i].first, cells_b[cells_b.size() - i].second);
            Move new_move = Move(b.t1+dist(b.id1, id_b), b.t1+dist(b.id1, id_b)+dist(b.id2, id_b), id_b, b.id2);
            bool has_col = true;
            for(auto c:moves_a)
                if(!check_conflict(new_move, c))
                {
                    has_col = false;
                    break;
                }
            if(has_col)
            {
                Constraint newA = get_constraint(0, a, new_move);
                if(newA.t2 >= initA.t2)
                    moves_b.push_back(new_move);
                out<<"\t\t\t<constraint i1=\""<<map->get_i(b.id1)<<"\" j1=\""<<map->get_j(b.id1)<<"\" i2=\""<<map->get_i(id_b)<<"\" j2=\""<<map->get_j(id_b)<<"\" t1=\""<<a.t1<<"\" c.t2=\""<<a.t2<<"\" positive=\""<<false<<"\" agent_id=\""<<0<<"\"/>\n";
            }
        }
        i++;
    }
    out.close();
    //std::cout<<moves_a.size()<<" "<<moves_b.size()<<" moves found\n";
    return {moves_a, moves_b};
}

std::pair<std::vector<Move>,std::vector<Move>> CBS::find_similar_actions(const Task &task, Conflict conflict, std::list<Constraint> consA, std::list<Constraint> consB)
{
    std::vector<Move> moves_a = {conflict.move1};
    std::vector<Move> moves_b = {conflict.move2};
    Constraint constraintA(get_constraint(conflict.agent1, conflict.move1, conflict.move2));
    Constraint constraintB(get_constraint(conflict.agent2, conflict.move2, conflict.move1));
    double durA = constraintA.t2 - constraintA.t1;
    double durB = constraintB.t2 - constraintB.t1;
    while(true)
    {
        consA.push_back(constraintA);
        consB.push_back(constraintB);
        sPath pathA = sPath(aa_planner.find_path(task.get_agent(conflict.agent1), *map, consA, aa_h_values[conflict.agent1]));
        sPath pathB = sPath(aa_planner.find_path(task.get_agent(conflict.agent2), *map, consA, aa_h_values[conflict.agent2]));
        Conflict new_c = check_paths(pathA, pathB);
        //std::cout<<new_c.move1.id1<<" "<<new_c.move1.id2<<" vs "<<new_c.move2.id1<<" "<<new_c.move2.id2<<"\n";
        if(new_c.agent1 >= 0)
        {
            Move new_move = new_c.move1;
            bool has_col = true;
            Constraint con;
            double least_durA(durA), least_durB(durB);
            for(auto c:moves_b)
                if(!check_conflict(new_move, c))
                {
                    has_col = false;
                    break;
                }
                else
                {
                    con = get_constraint(0, new_move, c);
                    least_durA = std::fmin(least_durA, con.t2 - con.t1);
                }
            if(has_col && least_durA > durA/2)
                moves_a.push_back(new_move);
            new_move = new_c.move2;
            has_col = true;
            for(auto c:moves_a)
                if(!check_conflict(new_move, c))
                {
                    has_col = false;
                    break;
                }
                else
                {
                    con = get_constraint(0, new_move, c);
                }
            if(has_col && con.t2 - con.t1 > durB/2)
                moves_b.push_back(new_move);
            constraintA = Constraint(conflict.agent1, 0, CN_INFINITY, new_c.move1.id1, new_c.move1.id2);
            constraintB = Constraint(conflict.agent2, 0, CN_INFINITY, new_c.move2.id1, new_c.move2.id2);
        }
        else
            break;
    }
    return {moves_a, moves_b};
}

