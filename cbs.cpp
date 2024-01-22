#include "cbs.h"

bool CBS::init_root(const Map &map, const Task &task)
{
    CBS_Node root;
    tree->set_focal_weight(config.focal_weight);
    sPath path;
    for(int id:task.get_ids())
    {
        Agent agent = task.get_agent(id);
        path = aa_planner.find_path(agent, map, {}, *aa_h_values);
        if(path.cost < 0)
            return false;
        //std::cout<<path.agentID<<" "<<path.cost<<" "<<path.nodes.front().id<<" "<<path.nodes.back().id<<"\n";
        //std::cout<<agent.start_id<<" "<<agent.goal_id<<"\n";
        root.paths[id] = path;
        root.cost += path.cost;
    }
    root.low_level_expanded = 0;
    root.parent = nullptr;
    root.id = 0;
    root.id_str = "0";
    auto conflicts = get_all_conflicts(root.paths, -1);
    for(const Conflict& conflict: conflicts)
        root.conflicts.push_back(conflict);
    root.conflicts_num = conflicts.size();
    solution.init_cost = root.cost;
    solution.initial_conflicts = root.conflicts_num;
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

Multiconstraint CBS::get_multiconstraint(int agent, std::vector<Move> moves_a, std::vector<Move> moves_b)
{
    Multiconstraint result;
    if(moves_a.front().id1 == moves_a.front().id2)
        return get_wait_constraint(agent, moves_a.front(), moves_b.front());
    for(auto a:moves_a)
    {
        double min_t = CN_INFINITY;
        for(auto b: moves_b)
        {
            auto c = get_constraint(agent, a, b);
            if(c.t2 < min_t)
                min_t = c.t2;
        }
        result.constraints.emplace_back(Constraint(agent, a.t1, min_t, a.id1, a.id2));
    }
    result.agent = agent;
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

Solution CBS::find_solution(const Map &map, const Task &task, const Config &cfg, PHeuristic &pheuristic)
{
    config = cfg;
    config.connectdness = -1;
    aa_h_values = &pheuristic;
    this->map = &map;
    auto t = std::chrono::high_resolution_clock::now();
    int cardinal_solved = 0, semicardinal_solved = 0;
    if(!this->init_root(map, task))
        return solution;
    solution.init_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    solution.found = true;
    CBS_Node *goal;
    int expanded(1);
    double time(0);
    while(true)
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

    solution.paths = get_paths(goal, task.get_ids());
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
    return solution;
}

bool CBS::validate_constraints(std::list<Multiconstraint> constraints, int agent)
{
    std::list<Multiconstraint> positives;
    for(auto mc: constraints)
        if(mc.positive && mc.agent == agent)
        {
            int overlapped = 0;
            for(auto c: mc.constraints)
            {
                for(auto opposite_cons:constraints)
                {
                    if(opposite_cons.positive)
                        continue;
                    for(auto oc: opposite_cons.constraints)
                        if(oc.agent == mc.agent and c.id1 == oc.id1 and c.id2 == oc.id2)
                            if(oc.t1 < c.t1 - CN_EPSILON and oc.t2 > c.t2 + CN_EPSILON)
                            {
                                overlapped++;
                                break;
                            }
                }
            }
            if(overlapped == mc.constraints.size())
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
    node.conflicts_num = node.conflicts.size();
    return;
}

std::list<Multiconstraint> CBS::get_constraints(CBS_Node *node, int agent_id)
{
    CBS_Node* curNode = node;
    std::list<Multiconstraint> constraints(0);
    while(curNode->parent != nullptr)
    {
        if(agent_id < 0 || curNode->constraint.agent == agent_id)
            constraints.push_back(curNode->constraint);
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
        for(auto cons:get_constraints(node, i))
            for(auto c: cons.constraints)
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
    if(parent == nullptr)
        return parent;
    auto node = *parent;
    node.cost -= node.h;
    parent->conflicts.clear();
    auto paths = get_paths(&node, task.get_ids());
    auto conflicts = node.conflicts;

    if(conflicts.empty())
    {
        /*std::cout<<node.id<<' '<<node.cost<<" "<<node.conflicts_num<<" "<<node.total_cons<<" GOAL\n";
        for(auto p:paths)
            std::cout<<p.second.agentID<<" "<<p.second.cost<<"\n";
        std::cout<<"solution without conflicts found\n";*/
        return parent; //i.e. no conflicts => solution found
    }
    tree->pop_front();
    Conflict conflict = get_conflict(conflicts);
    //std::cout<<"conflict "<<conflict.move1.id1 <<" "<<conflict.move1.t1<<" "<<conflict.move1.id2<<" "<<conflict.move1.t2<<" vs "<<conflict.move2.id1 <<" "<<conflict.move2.t1<<" "<<conflict.move2.id2<<" "<<conflict.move2.t2<<"\n";
    auto constraintsA = get_constraints(&node, conflict.agent1);
    auto constraintsB = get_constraints(&node, conflict.agent2);
    Multiconstraint multiconstraintA, multiconstraintB;
    if(config.mc_type == 2 || config.mc_type == 3)
    {
        auto actions = find_similar_actions(conflict.move1, conflict.move2);
        multiconstraintA = get_multiconstraint(conflict.agent1, actions.first, actions.second);
        multiconstraintB = get_multiconstraint(conflict.agent2, actions.second, actions.first);
    }
    else if(config.mc_type == 1)
    {
        auto actions = get_all_similar_actions(conflict.move1, conflict.move2);
        multiconstraintA = get_multiconstraint(conflict.agent1, actions.first, actions.second);
        multiconstraintB = get_multiconstraint(conflict.agent2, actions.second, actions.first);
    }
    else
    {
        multiconstraintA = get_constraint(conflict.agent1, conflict.move1, conflict.move2);
        multiconstraintB = get_constraint(conflict.agent2, conflict.move2, conflict.move1);
    }
    constraintsA.push_back(multiconstraintA);
    constraintsB.push_back(multiconstraintB);

    sPath pathA;
    pathA = aa_planner.check_all_paths(task.get_agent(conflict.agent1), *map, constraintsA, *aa_h_values, parent->id);
    low_level_searches++;
    low_level_expanded += pathA.expanded;

    sPath pathB;
    pathB = aa_planner.check_all_paths(task.get_agent(conflict.agent2), *map, constraintsB, *aa_h_values);
    low_level_searches++;
    low_level_expanded += pathB.expanded;

    std::map<int, sPath> pathsA; pathsA[pathA.agentID] = pathA;
    CBS_Node right(pathsA, parent, multiconstraintA, node.cost + pathA.cost - get_cost(node, conflict.agent1), 0, node.total_cons + multiconstraintA.constraints.size());
    std::map<int, sPath> pathsB; pathsB[pathB.agentID] = pathB;
    CBS_Node left(pathsB, parent, multiconstraintB, node.cost + pathB.cost - get_cost(node, conflict.agent2), 0, node.total_cons + multiconstraintB.constraints.size());
    Multiconstraint positive;
    if(config.use_disjoint_splitting)
    {
        int agent1positives(0), agent2positives(0);
        for(const auto &c: constraintsA)
            if(c.positive)
                agent1positives++;
        for(const auto &c: constraintsB)
            if(c.positive)
                agent2positives++;
        if(agent2positives > agent1positives && conflict.move1.id1 != conflict.move1.id2 && pathA.cost > 0)
        {
            positive = multiconstraintA;
            positive.positive = true;
            left.positive_constraint = positive;
            left.total_cons += positive.constraints.size();
            constraintsB.push_back(left.positive_constraint);
        }
        else if(conflict.move2.id1 != conflict.move2.id2 && pathB.cost > 0)
        {
            positive = multiconstraintB;
            positive.positive = true;
            right.positive_constraint = positive;
            right.total_cons += positive.constraints.size();
            constraintsA.push_back(right.positive_constraint);
        }
        else if(conflict.move1.id1 != conflict.move1.id2 && pathA.cost > 0)
        {
            positive = multiconstraintA;
            positive.positive = true;
            left.positive_constraint = positive;
            left.total_cons += positive.constraints.size();
            constraintsB.push_back(left.positive_constraint);
        }
    }
    if(pathA.cost > 0)
    {
        right.id = all_nodes.size();
        right.id_str = node.id_str + "0";
        find_new_conflicts(*map, task, right, paths, pathA, conflicts);
        all_nodes.push_back(right);
        tree->add_node(&all_nodes.back());
    }
    if(pathB.cost > 0)
    {
        left.id = all_nodes.size();
        left.id_str = node.id_str + "1";
        find_new_conflicts(*map, task, left, paths, pathB, conflicts);
        all_nodes.push_back(left);
        tree->add_node(&all_nodes.back());
    }
    return nullptr;
}

std::vector<std::pair<int, int>> CBS::get_cells_in_spiral(int start_i, int start_j)
{
    int di(0), dj(0), len(1);
    std::vector<std::pair<int, int>> cells;
    while (true) {
        auto added = cells.size();
        for (int k = 0; k < len; k++)
            if(map->cell_on_grid(start_i + k + di, start_j + dj))
                if(!map->cell_is_obstacle(start_i + k + di, start_j + dj))
                    cells.emplace_back(start_i + k + di, start_j + dj);
        di += len;
        for (int k = 0; k < len; k++)
            if(map->cell_on_grid(start_i + di, start_j + dj + k))
                if(!map->cell_is_obstacle(start_i + di, start_j + dj + k))
                    cells.emplace_back(start_i + di, start_j + dj + k);
        dj += len;
        len++;
        for (int k = 0; k < len; k++)
            if(map->cell_on_grid(start_i - k + di, start_j + dj))
                if(!map->cell_is_obstacle(start_i - k + di, start_j + dj))
                    cells.emplace_back(start_i - k + di, start_j + dj);
        di -= len;
        for (int k = 0; k < len; k++)
            if(map->cell_on_grid(start_i + di, start_j + dj - k))
                if(!map->cell_is_obstacle(start_i + di, start_j + dj - k))
                    cells.emplace_back(start_i + di, start_j + dj - k);
        dj -= len;
        len++;
        if(added == cells.size())
            break;
    }
    return cells;
}

std::pair<std::vector<Move>,std::vector<Move>> CBS::get_all_similar_actions(Move a, Move b)
{
    std::vector<Move> moves_a = {a};
    std::vector<Move> moves_b = {b};
    auto cells_a = get_cells_in_spiral(map->get_i(a.id2), map->get_j(a.id2));
    auto cells_b = get_cells_in_spiral(map->get_i(b.id2), map->get_j(b.id2));
    size_t i(1);
    Constraint initA = get_constraint(0, a, b);
    Constraint initB = get_constraint(0, b, a);
    while(i < cells_a.size() or i < cells_b.size())
    {
        if(i < cells_a.size())
        {
            int id_a = map->get_id(cells_a[i].first, cells_a[i].second);
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
                //if(newB.t2 >= initB.t2)
                    moves_a.push_back(new_move);
            }
        }

        if(i < cells_b.size())
        {
            int id_b = map->get_id(cells_b[i].first, cells_b[i].second);
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
                //if(newA.t2 >= initA.t2)
                    moves_b.push_back(new_move);
            }
        }
        i++;
    }
    return {moves_a, moves_b};
}

std::vector<std::pair<int, int>> CBS::get_cells(int x1, int y1, int x2, int y2)
{
    std::vector<std::pair<int, int>> lineCells(0);
    if(x1 == x2 && y1 == y2)
        return {{x1, y1}};
    int delta_x = std::abs(x1 - x2);
    int delta_y = std::abs(y1 - y2);
    if((delta_x >= delta_y && x1 > x2) || (delta_y > delta_x && y1 > y2))
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }
    int step_x = (x1 < x2 ? 1 : -1);
    int step_y = (y1 < y2 ? 1 : -1);
    int error = 0, x = x1, y = y1;
    int k, num;
    float d = std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)), d1, d2;
    std::pair<int, int> add;

    error = 0;
    if(delta_x >= delta_y)
    {
        for(x = x1; x != x2 + step_x; x+=step_x)
        {
            lineCells.emplace_back(x, y);
            for(int dy = y+1; ; dy++)
            {
                d1 = std::sqrt((x1-x)*(x1-x) + (y1-dy)*(y1-dy));
                d2 = std::sqrt((x2-x)*(x2-x) + (y2-dy)*(y2-dy));
                if(d1 + d2 - d < 1.0 and map->cell_on_grid(x, dy))
                    lineCells.emplace_back(x,dy);
                else
                    break;

            }
            for(int dy = y-1; ; dy--)
            {
                d1 = std::sqrt((x1-x)*(x1-x) + (y1-dy)*(y1-dy));
                d2 = std::sqrt((x2-x)*(x2-x) + (y2-dy)*(y2-dy));
                if(d1 + d2 - d < 1.0 and map->cell_on_grid(x, dy))
                    lineCells.emplace_back(x,dy);
                else
                    break;

            }
            error += delta_y;
            if((error<<1) > delta_x)
            {
                y += step_y;
                error -= delta_x;
            }
        }
    }
    else
    {
        for(y = y1; y != y2 + step_y; y += step_y)
        {
            lineCells.emplace_back(x, y);
            for(int dx = x+1; ; dx++)
            {
                d1 = std::sqrt((x1-dx)*(x1-dx) + (y1-y)*(y1-y));
                d2 = std::sqrt((x2-dx)*(x2-dx) + (y2-y)*(y2-y));
                if(d1 + d2 - d < 1.0 and map->cell_on_grid(dx, y))
                    lineCells.emplace_back(dx,y);
                else
                    break;

            }
            for(int dx = x-1; ; dx--)
            {
                d1 = std::sqrt((x1-dx)*(x1-dx) + (y1-y)*(y1-y));
                d2 = std::sqrt((x2-dx)*(x2-dx) + (y2-y)*(y2-y));
                if(d1 + d2 - d < 1.0 and map->cell_on_grid(dx, y))
                    lineCells.emplace_back(dx,y);
                else
                    break;

            }
            error += delta_x;
            if((error<<1) > delta_y)
            {
                    x += step_x;
                    error -= delta_y;
            }
        }
    }
    return lineCells;
}

std::pair<std::vector<Move>,std::vector<Move>> CBS::find_similar_actions(Move a, Move b)
{
    std::vector<Move> moves_a = {a};
    std::vector<Move> moves_b = {b};
    LineOfSight los;
    auto cells_a = los.getCellsCrossedByLine(map->get_i(a.id1), map->get_j(a.id1), map->get_i(a.id2), map->get_j(a.id2), *map);
    auto cells_b = los.getCellsCrossedByLine(map->get_i(b.id1), map->get_j(b.id1), map->get_i(b.id2), map->get_j(b.id2), *map);

    size_t i(1);
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
            for(auto &c:moves_b)
                if(!check_conflict(new_move, c))
                {
                    has_col = false;
                    break;
                }
            if(has_col)
            {
                Constraint newB = get_constraint(0, b, new_move);
                if(newB.t2 >= initB.t2)
                    if(new_move.id2 != a.id2)
                        moves_a.push_back(new_move);
            }
        }

        if(i < cells_b.size())
        {
            int id_b = map->get_id(cells_b[cells_b.size() - i].first, cells_b[cells_b.size() - i].second);
            Move new_move = Move(b.t1, b.t1+dist(b.id1, id_b), b.id1, id_b);
            bool has_col = true;
            for(auto &c:moves_a)
                if(!check_conflict(new_move, c))
                {
                    has_col = false;
                    break;
                }
            if(has_col)
            {
                Constraint newA = get_constraint(0, a, new_move);
                if(newA.t2 >= initA.t2)
                    if(new_move.id2 != b.id2)
                        moves_b.push_back(new_move);
            }
        }
        i++;
    }
    if(config.mc_type == 3)
    {
        i = 1;
        while(i < cells_a.size() or i < cells_b.size())
        {
            if(i < cells_a.size())
            {
                int id_a = map->get_id(cells_a[cells_a.size() - i].first, cells_a[cells_a.size() - i].second);
                Move new_move = Move(a.t1 + dist(a.id1, id_a), a.t1+dist(a.id1, id_a)+dist(a.id2, id_a), id_a, a.id2);
                bool has_col = true;
                Constraint con;
                for(auto &c:moves_b)
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
                }
            }
            if(i < cells_b.size())
            {
                int id_b = map->get_id(cells_b[cells_b.size() - i].first, cells_b[cells_b.size() - i].second);
                Move new_move = Move(b.t1+dist(b.id1, id_b), b.t1+dist(b.id1, id_b)+dist(b.id2, id_b), id_b, b.id2);
                bool has_col = true;
                for(auto &c:moves_a)
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
                }
            }
            i++;
        }
    }
    return {moves_a, moves_b};
}
