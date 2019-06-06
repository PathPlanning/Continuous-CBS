#include "cbs.h"

bool CBS::init_root(const Graph &map, const Task &task)
{
    CBS_Node root;
    Path path;
    h_values.init(map.get_nodes_size(), task.get_agents_size());
    for(int i = 0; i < task.get_agents_size(); i++)
    {
        Agent agent = task.get_agent(i);
        h_values.count(map, agent);
        path = planner.find_path(agent, map, {}, h_values);
        if(path.cost < 0)
            return false;
        root.paths.push_back(path);
        root.cost += path.cost;
    }
    root.low_level_expanded = 0;
    root.parent = nullptr;
    root.cons_num = 0;
    root.id = 1;
    solution.init_cost = root.cost;
    tree.add_node(root);
    return true;
}

bool CBS::check_conflict(Move move1, Move move2)
{
    double startTimeA(move1.t1), endTimeA(move1.t2), startTimeB(move2.t1), endTimeB(move2.t2);
    Vector2D A(move1.i1, move1.j1);
    Vector2D B(move2.i1, move2.j1);
    Vector2D VA((move1.i2 - move1.i1)/(move1.t2 - move1.t1), (move1.j2 - move1.j1)/(move1.t2 - move1.t1));
    Vector2D VB((move2.i2 - move2.i1)/(move2.t2 - move2.t1), (move2.j2 - move2.j1)/(move2.t2 - move2.t1));
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

Constraint CBS::get_wait_constraint(int agent, Move move1, Move move2)
{
    double radius = 2*CN_AGENT_SIZE;
    double i0(move2.i1), j0(move2.j1), i1(move2.i2), j1(move2.j2), i2(move1.i1), j2(move1.j1);
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
    return Constraint(agent, interval.first, interval.second, move1.i1, move1.j1, move1.i2, move1.j2, move1.id1, move1.id2);
}

Constraint CBS::get_constraint(int agent, Move move1, Move move2)
{
    if(move1.id1 == move1.id2)
        return get_wait_constraint(agent, move1, move2);
    double begin(move1.t1), startTimeA(move1.t1), endTimeA(move1.t2), startTimeB(move2.t1), endTimeB(move2.t2);
    Vector2D A(move1.i1, move1.j1);
    Vector2D B(move2.i1, move2.j1);
    Vector2D VA((move1.i2 - move1.i1)/(move1.t2 - move1.t1), (move1.j2 - move1.j1)/(move1.t2 - move1.t1));
    Vector2D VB((move2.i2 - move2.i1)/(move2.t2 - move2.t1), (move2.j2 - move2.j1)/(move2.t2 - move2.t1));
    while(startTimeA < endTimeB)
    {
        A = Vector2D(move1.i1, move1.j1);
        startTimeA = move1.t1;
        endTimeA = move1.t2;
        if(startTimeA + CN_EPSILON > endTimeB)
        {
            move1.t2 = move1.t2 - (move1.t1 - endTimeB);
            move1.t1 = endTimeB;
            break;
        }
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
        {
            if(move2.t2 == CN_INFINITY)
            {
                return Constraint(agent, begin, CN_INFINITY, move1.i1, move1.j1, move1.i2, move1.j2, move1.id1, move1.id2);
            }
            move1.t1 += CN_DELTA;
            move1.t2 += CN_DELTA;
            continue;
        }

        Vector2D v(VA - VB);
        double a(v*v);
        double b(w*v);
        double dscr(b*b - a*c);
        if(dscr - CN_EPSILON < 0)
        {
            break;
        }

        double ctime = (b - sqrt(dscr))/a;
        if(ctime > -CN_EPSILON && ctime < std::min(endTimeB,endTimeA) - startTimeA + CN_EPSILON)
        {
            if(move2.t2 == CN_INFINITY)
            {
                return Constraint(agent, begin, CN_INFINITY, move1.i1, move1.j1, move1.i2, move1.j2, move1.id1, move1.id2);
            }
            move1.t1 += CN_DELTA;
            move1.t2 += CN_DELTA;
            continue;
        }
        else
        {
            break;
        }
    }
    return Constraint(agent, begin, move1.t1, move1.i1, move1.j1, move1.i2, move1.j2, move1.id1, move1.id2);
}


Solution CBS::find_solution(const Graph &map, const Task &task)
{
    auto t = std::chrono::high_resolution_clock::now();
    if(!this->init_root(map, task))
    {
        return solution;
    }
    solution.init_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    CBS_Node node;
    std::chrono::duration<double> time_spent;
    int expanded(1);
    double time(0);
    std::vector<Conflict> conflicts;
    Conflict conflict;
    std::vector<int> conflicting_agents;
    std::vector<std::pair<int, int>> conflicting_pairs;
    int low_level_searches(0);
    double low_level_expanded(0);
    int nodeid(0);
    do
    {
        auto parent = tree.get_front();
        node = *parent;
        if(!CN_CARDINAL)
            node.look_for_cardinal = false;
        tree.pop_front();
        auto paths = get_paths(&node, task.get_agents_size());
        auto begin = std::chrono::high_resolution_clock::now();
        if(node.look_for_cardinal)
            conflicts = get_all_conflicts(paths);
        else
            conflict = check_conflicts(paths, conflicting_agents, conflicting_pairs);
        time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - begin).count();
        if((node.look_for_cardinal && conflicts.empty()) || (!node.look_for_cardinal && conflict.agent1 < 0))
            break; //i.e. no conflicts => solution found
        expanded++;
        if(!node.look_for_cardinal && CN_HISTORY)
        {
            if(std::find(conflicting_agents.begin(), conflicting_agents.end(), conflict.agent1) == conflicting_agents.end())
                conflicting_agents.push_back(conflict.agent1);
            if(std::find(conflicting_agents.begin(), conflicting_agents.end(), conflict.agent2) == conflicting_agents.end())
                conflicting_agents.push_back(conflict.agent2);
            if(std::find(conflicting_pairs.begin(), conflicting_pairs.end(), std::make_pair(conflict.agent1, conflict.agent2)) == conflicting_pairs.end())
                conflicting_pairs.push_back({conflict.agent1, conflict.agent2});
        }
        bool cardinal_found = false;
        Conflict semi_cardinal;
        if(node.look_for_cardinal)
        {
            for(auto conflict:conflicts)
            {
                std::list<Constraint> constraints = get_constraints(&node, conflict.agent1);
                Constraint constraintA(get_constraint(conflict.agent1, conflict.move1, conflict.move2));
                constraintA.num = node.cons_num;
                constraints.push_back(constraintA);
                Path pathA = planner.find_path(task.get_agent(conflict.agent1), map, constraints, h_values);
                low_level_expanded += pathA.expanded;
                low_level_searches++;
                if(pathA.cost > get_cost(node, conflict.agent1))
                    semi_cardinal = conflict;
                constraints = get_constraints(&node, conflict.agent2);
                Constraint constraintB = get_constraint(conflict.agent2, conflict.move2, conflict.move1);
                constraints.push_back(constraintB);
                Path pathB = planner.find_path(task.get_agent(conflict.agent2), map, constraints, h_values);
                low_level_searches++;
                low_level_expanded += pathB.expanded;
                if(pathB.cost > get_cost(node, conflict.agent2))
                    semi_cardinal = conflict;
                if(pathA.cost > get_cost(node, conflict.agent1) && pathB.cost > get_cost(node, conflict.agent2))
                {
                    CBS_Node right({pathA}, parent, constraintA, node.cost + pathA.cost - get_cost(node, conflict.agent1), node.cons_num + 1, 0, node.look_for_cardinal);
                    CBS_Node left({pathB}, parent, constraintB, node.cost + pathB.cost - get_cost(node, conflict.agent2), node.cons_num + 1, 0, node.look_for_cardinal);
                    right.altconstraint = constraintB;
                    left.altconstraint = constraintA;
                    right.id = parent->id*10;
                    tree.add_node(right);
                    left.id = parent->id*10+1;
                    tree.add_node(left);
                    cardinal_found = true;
                    break;
                }
            }
        }
        if(!cardinal_found || !node.look_for_cardinal)
        {
            if(node.look_for_cardinal)
            {
                conflict = conflicts.at(0);
                if(semi_cardinal.agent1 >= 0)
                {
                    conflict = semi_cardinal;
                }
                else if(CN_STOP_CARDINAL)
                    node.look_for_cardinal = false;
            }
            std::list<Constraint> constraints = get_constraints(&node, conflict.agent1);

            Constraint constraintA(get_constraint(conflict.agent1, conflict.move1, conflict.move2));
            constraints.push_back(constraintA);
            Path pathA = planner.find_path(task.get_agent(conflict.agent1), map, constraints, h_values);
            low_level_searches++;
            low_level_expanded += pathA.expanded;
            CBS_Node right({pathA}, parent, constraintA, node.cost + pathA.cost - get_cost(node, conflict.agent1), node.cons_num + 1, 0, node.look_for_cardinal);
            right.id = nodeid++;
            if(pathA.cost > 0)
                tree.add_node(right);

            constraints = get_constraints(&node, conflict.agent2);;
            Constraint constraintB = get_constraint(conflict.agent2, conflict.move2, conflict.move1);
            constraints.push_back(constraintB);
            Path pathB = planner.find_path(task.get_agent(conflict.agent2), map, constraints, h_values);
            low_level_searches++;
            low_level_expanded += pathB.expanded;
            CBS_Node left({pathB}, parent, constraintB, node.cost + pathB.cost - get_cost(node, conflict.agent2), node.cons_num + 1, 0, node.look_for_cardinal);
            left.id = nodeid++;
            if(pathB.cost > 0)
                tree.add_node(left);
        }
        time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
        if(time_spent.count() > CN_TIMELIMIT)
            break;
    }
    while(tree.get_open_size() > 0);
    solution.paths = get_paths(&node, task.get_agents_size());
    solution.flowtime = node.cost;
    solution.constraints_num = node.cons_num;
    solution.low_level_expansions = low_level_searches;
    solution.low_level_expanded = low_level_expanded/low_level_searches;
    solution.high_level_expanded = expanded;
    for(auto path:solution.paths)
        solution.makespan = (solution.makespan > path.cost) ? solution.makespan : path.cost;
    solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    solution.check_time = time;
    return solution;
}

std::list<Constraint> CBS::get_constraints(CBS_Node *node, int agent_id)
{
    CBS_Node* curNode = node;
    std::list<Constraint> constraints(0);
    while(curNode->parent != nullptr)
    {
        if(agent_id < 0 || curNode->constraint.agent == agent_id)
            constraints.push_back(curNode->constraint);
        curNode = curNode->parent;
    }
    return constraints;
}

std::list<Constraint> CBS::get_altconstraints(CBS_Node *node, int agent_id)
{
    CBS_Node* curNode = node;
    std::list<Constraint> constraints(0);
    while(curNode->parent != nullptr)
    {
        if(agent_id < 0 || curNode->constraint.agent == agent_id)
            constraints.push_back(curNode->altconstraint);
        curNode = curNode->parent;
    }
    return constraints;
}

Conflict CBS::check_paths(Path pathA, Path pathB)
{
    int a(0), b(0);
    std::vector<Node> nodesA = pathA.nodes;
    std::vector<Node> nodesB = pathB.nodes;
    while(a < nodesA.size() - 1 || b < nodesB.size() - 1)
    {
        if(a < nodesA.size() - 1 && b < nodesB.size() - 1) // if both agents have not reached their goals yet
        {
            if(sqrt(pow(nodesA[a].i - nodesB[b].i, 2) + pow(nodesA[a].j - nodesB[b].j, 2) - CN_EPSILON)
                    < (nodesA[a+1].g - nodesA[a].g) + (nodesB[b+1].g - nodesB[b].g))
                if(check_conflict(Move(nodesA[a], nodesA[a+1]), Move(nodesB[b], nodesB[b+1])))
                    return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a+1]), Move(nodesB[b], nodesB[b+1]));

        }
        else if(a == nodesA.size() - 1) // if agent A has already reached the goal
        {
            if(sqrt(pow(nodesA[a].i - nodesB[b].i, 2) + pow(nodesA[a].j - nodesB[b].j, 2)) - CN_EPSILON < (nodesB[b+1].g - nodesB[b].g))
                if(check_conflict(Move(nodesA[a].g, CN_INFINITY, nodesA[a].i, nodesA[a].j, nodesA[a].i, nodesA[a].j), Move(nodesB[b], nodesB[b+1])))
                    return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a].g, CN_INFINITY, nodesA[a].i, nodesA[a].j, nodesA[a].i, nodesA[a].j, nodesA[a].id, nodesA[a].id), Move(nodesB[b], nodesB[b+1]));
        }
        else if(b == nodesB.size() - 1) // if agent B has already reached the goal
        {
            if(sqrt(pow(nodesA[a].i - nodesB[b].i, 2) + pow(nodesA[a].j - nodesB[b].j, 2)) - CN_EPSILON < (nodesA[a+1].g - nodesA[a].g))
                if(check_conflict(Move(nodesA[a], nodesA[a+1]), Move(nodesB[b].g, CN_INFINITY, nodesB[b].i, nodesB[b].j, nodesB[b].i, nodesB[b].j)))
                    return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a+1]), Move(nodesB[b].g, CN_INFINITY, nodesB[b].i, nodesB[b].j, nodesB[b].i, nodesB[b].j, nodesB[b].id, nodesB[b].id));
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

Conflict CBS::check_conflicts(std::vector<Path> &paths, std::vector<int> conflicting_agents, std::vector<std::pair<int,int>> conflicting_pairs)
{
    Conflict conflict;
    //first of all check the pairs of agents that already have collisions
    for(int i = 0; i < conflicting_pairs.size(); i++)
    {
        conflict = check_paths(paths[conflicting_pairs[i].first], paths[conflicting_pairs[i].second]);
        if(conflict.agent1 >= 0)
            return conflict;
    }
    //check other possible combinations of agents that already have conflicts
    for(int i = 0; i < conflicting_agents.size(); i++)
        for(int j = i + 1; j < conflicting_agents.size(); j++)
            if(std::find(conflicting_pairs.begin(), conflicting_pairs.end(), std::make_pair(conflicting_agents[i], conflicting_agents[j])) == conflicting_pairs.end())
            {
                conflict = check_paths(paths[conflicting_agents[i]], paths[conflicting_agents[j]]);
                if(conflict.agent1 >= 0)
                    return conflict;
            }
    //check all other pairs of agents
    for(int i = 0; i < paths.size(); i++)
        for(int j = i + 1; j < paths.size(); j++)
            if(std::find(conflicting_agents.begin(), conflicting_agents.end(), i) == conflicting_agents.end()
               || std::find(conflicting_agents.begin(), conflicting_agents.end(), j) == conflicting_agents.end())
            {
                conflict = check_paths(paths[i], paths[j]);
                if(conflict.agent1 >= 0)
                    return conflict;
            }
    //no conflicts were found
    return Conflict();
}

std::vector<Conflict> CBS::get_all_conflicts(std::vector<Path> &paths)
{
    std::vector<Conflict> conflicts;
    //check all agents
    for(int i = 0; i < paths.size(); i++)
        for(int j = i + 1; j < paths.size(); j++)
        {
            Conflict conflict = check_paths(paths[i], paths[j]);
            if(conflict.agent1 >= 0)
                conflicts.push_back(conflict);
        }
    return conflicts;
}

double CBS::get_cost(CBS_Node node, int agent_id)
{
    while(node.parent != nullptr)
    {
        if(node.paths.begin()->agentID == agent_id)
            return node.paths.begin()->cost;
        node = *node.parent;
    }
    return node.paths.at(agent_id).cost;
}

std::vector<Path> CBS::get_paths(CBS_Node *node, int agents_size)
{
    CBS_Node* curNode = node;
    std::vector<Path> paths(agents_size);
    while(curNode->parent != nullptr)
    {
        if(paths.at(curNode->paths.begin()->agentID).cost < 0)
            paths.at(curNode->paths.begin()->agentID) = *curNode->paths.begin();
        curNode = curNode->parent;
    }
    for(int i = 0; i < agents_size; i++)
        if(paths.at(i).cost < 0)
            paths.at(i) = curNode->paths.at(i);
    return paths;
}
