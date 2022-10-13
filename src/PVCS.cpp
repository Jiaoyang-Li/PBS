#include <PVCS.h>
#include <ilcplex/ilocplex.h>

PVCS::PVCS(const Instance& instance, bool sipp, int screen, bool use_tr): 
    PBS(instance, sipp, screen), use_tr(use_tr) {}

bool PVCS::solve(double time_limit)
{
    this->time_limit = time_limit;

    if (screen > 0) // 1 or 2
    {
        string name = getSolverName();
        name.resize(35, ' ');
        cout << name << ": ";
    }
    start = clock();  // set timer

    while (solution_cost == -2)  // Not yet find a solution
    {
        generateRoot();
        while (!open_list.empty())
        {
            PBSNode* curr = selectNode();

            if (terminate(curr)) break;

            clock_t t1;
            if (!curr->is_expanded)
            {
                curr->is_expanded = true;
                if (screen > 1)
                    cout << "	Expand " << *curr << endl;

                t1 = clock();
                generateChild(0, curr);
                runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;

                if (curr->children[0] != nullptr)
                {
                    pushNode(curr->children[0]);
                }
                else  // For now, we just restart if there is no solution
                {
                    clear();
                    stack<PBSNode*>().swap(open_list);  // clear the open_list
                    num_restart ++;
                    break;  // leave the while loop of open_list.empty
                }
            }
        }  // end of while loop
    }
    return solution_found;
}

bool PVCS::generateRoot()
{
	PBSNode* root = new PBSNode();
	root->cost = 0;
    root->ag_weights = vector<int>(num_of_agents, 1);
    paths.assign(num_of_agents, nullptr);
    priority_graph.assign(num_of_agents, vector<bool>(num_of_agents, false));
    std::random_shuffle(init_agents.begin(), init_agents.end());

    set<int> higher_agents;  // pseudo agents, always empty set
    for (int i = 0; i < num_of_agents; i++)
    {
        int _ag_ = init_agents[i];
        Path new_path = search_engines[_ag_]->findOptimalPath(higher_agents, paths, _ag_);
        num_LL_expanded += search_engines[_ag_]->num_expanded;
        num_LL_generated += search_engines[_ag_]->num_generated;
        if (new_path.empty())
        {
            cout << "No path exists for agent " << _ag_ << endl;
            return false;
        }
        root->paths.emplace_back(_ag_, new_path);
        paths[_ag_] = &root->paths.back().second;
        root->makespan = max(root->makespan, new_path.size() - 1);
        root->cost += (int)new_path.size() - 1;
    }
    clock_t t = clock();
	root->depth = 0;
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            if (use_tr)
            {
                int priority = hasConflicts(a1, a2);
                if(priority == 1)
                    root->conflicts.emplace_front(new Conflict(a1, a2, priority));
                else if (priority == 2)  // target conflict
                {
                    if (paths[a1]->size() < paths[a2]->size())  // a1 is at its goal location
                        root->conflicts.emplace_back(new Conflict(a1, a2, priority));
                    else  // a2 is at its goal location
                        root->conflicts.emplace_back(new Conflict(a2, a1, priority));
                }
            }
            else if (PBS::hasConflicts(a1, a2))  // not using target reasoning
            {
                root->conflicts.emplace_back(new Conflict(a1, a2));
            }
        }
    }
    runtime_detect_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
    num_HL_generated++;
    root->time_generated = num_HL_generated;
    if (screen > 1)
        cout << "Generate " << *root << endl;
	pushNode(root);
	dummy_start = root;
	if (screen >= 2) // print start and goals
		printPaths();

	return true;
}

bool PVCS::generateChild(int child_id, PBSNode* parent)
{
    assert(child_id == 0 or child_id == 1);
    parent->children[child_id] = new PBSNode(*parent);
    PBSNode* node = parent->children[child_id];
    node->ag_weights = vector<int>(num_of_agents, 1);

    // Start MVC
    clock_t mvc_start = clock();
    IloEnv env = IloEnv();  // Initialize the environment for CPLEX
    IloNumVarArray var(env); // variable for each agent
    IloRangeArray con(env);  // Numerical constraints for each agent
    IloExpr sum_obj = IloExpr(env);  // objective is the sum of varialbes of all agents
    IloModel model = IloModel(env);  // Initialize the model for CPLEX

    vector<int> mvc_agents(num_of_agents, -1);  // map the agent id to var id in mvc problem
    vector<bool> need_replan_agents(num_of_agents, false);
    uint64_t counter = 0;


    #ifndef NDEBUG
    if (screen > 1)
        cout << "---------- Current conflicts ----------" << endl;
    #endif
    // Declare the range of variables and the objective function
    for (const auto& conf : boost::adaptors::reverse(parent->conflicts))
    {
        #ifndef NDEBUG
        if (screen > 1)
            cout << *conf << endl;
        #endif
        // The targeting agent will not be added to CG!
        // Since we sort the conflicts with their priorities (1 in the front and 2 in the back),
        // the conflicts with priorities being 2 will be checked first!
        // Otherwise, we might need an extra for loop to filter out targeting agents
        // before adding variables to ILP model.
        if (conf->priority == 2)
        {
            need_replan_agents[conf->a1] = true;

            #ifndef NDEBUG
            if (screen > 1)
                cout << "\ttarget conflict: need to replan agent " << conf->a1 << endl;
            #endif
        }
        else if (conf->priority == 1 or conf->priority == -1)  // Non-TR conflicts
        {
            if (!need_replan_agents[conf->a1])
            {
                var.add(IloNumVar(env, 0, 1, ILOBOOL));
                sum_obj += var[counter] * parent->ag_weights[conf->a1];
                mvc_agents[conf->a1] = counter;
                need_replan_agents[conf->a1] = true;
                counter ++;

                #ifndef NDEBUG
                if (screen > 1)
                    cout << "\tAdd to replan agent " << conf->a1 << " to CG" << endl;
                #endif
            }

            if (!need_replan_agents[conf->a2])
            {
                var.add(IloNumVar(env, 0, 1, ILOBOOL));
                sum_obj += var[counter] * parent->ag_weights[conf->a2];
                mvc_agents[conf->a2] = counter;
                need_replan_agents[conf->a2] = true;
                counter ++;

                #ifndef NDEBUG
                if (screen > 1)
                    cout << "\tAdd to replan agent " << conf->a2 << " to CG" << endl;
                #endif
            }

            con.add(var[mvc_agents[conf->a1]] + var[mvc_agents[conf->a2]] >= 1);
        }
    }

    #ifndef NDEBUG
    if (screen > 1)
        cout << "---------- End current conflicts ----------" << endl;
    #endif

    model.add(con);
    model.add(IloMinimize(env, sum_obj));

    IloCplex cplex(env);
    double tmp_runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
    cplex.setParam(IloCplex::Param::TimeLimit, time_limit - tmp_runtime);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());

    if (cplex.solve())
    {
        for (auto& conf :parent->conflicts)
        {
            if (conf->priority != 2)  // we ignore the target conflicts
            {
                bool is_a1 = cplex.getValue(var[mvc_agents[conf->a1]]);
                bool is_a2 = cplex.getValue(var[mvc_agents[conf->a2]]);

                // if we need to replan a2, then we swap agents in the conflict or
                // if we need to replan both agents, then only replan the one with a shorter path
                if ((!is_a1 and is_a2) or
                    (is_a1 and is_a2 and paths[conf->a1] > paths[conf->a2]))
                {
                    std::swap(conf->a1, conf->a2);
                }
            }
        }
        env.end();
        runtime_run_mvc += (double)(clock()-mvc_start) / CLOCKS_PER_SEC;
    }
    else
    {
        std::cerr << "ERROR" << endl;
        cplex.exportModel("error.lp");

        delete node;
        parent->children[child_id] = nullptr;
        env.end();
        runtime_run_mvc += (double)(clock()-mvc_start) / CLOCKS_PER_SEC;
        return false;
    }
    // End MVC

    for (const auto& conf : parent->conflicts)
    {
        // When generating conflicts, we already set priority a1 < a2
        priority_graph[conf->a2][conf->a1] = false;
        priority_graph[conf->a1][conf->a2] = true;
    }
    if (screen > 2)
        printPriorityGraph();

    if (topologicalSort(ordered_agents))  // true if there is at least one cycle
    {
        if (screen > 2)
            cout << "There is a cycle in the current priority graph!" << endl;

        delete node;
        parent->children[child_id] = nullptr;
        return false;
    }

    if (screen > 2)
    {
        cout << "Ordered agents: ";
        for (int i : ordered_agents)
            cout << i << ",";
        cout << endl;
    }
    vector<int> topological_orders(num_of_agents); // map agent i to its position in ordered_agents
    int i = num_of_agents - 1;
    for (const int & a : ordered_agents)
    {
        topological_orders[a] = i;
        i--;
    }

    std::priority_queue<pair<int, int>> to_replan; // <position in ordered_agents, agent id>
    vector<bool> lookup_table(num_of_agents, false);  // true if the agent is in to_replan
    for (const auto& conf : parent->conflicts)
    {
        if (!lookup_table[conf->a1])
        {
            to_replan.emplace(topological_orders[conf->a1], conf->a1);
            lookup_table[conf->a1] = true;
        }
    }

    while (!to_replan.empty())
    {
        int a, rank;
        tie(rank, a) = to_replan.top();
        to_replan.pop();
        lookup_table[a] = false;
        if (screen > 2) cout << "Replan agent " << a << endl;
        // Re-plan path
        set<int> higher_agents;
        auto p = ordered_agents.rbegin();
        std::advance(p, rank);
        assert(*p == a);
        getHigherPriorityAgents(p, higher_agents);
        assert(!higher_agents.empty());
        if (screen > 2)
        {
            cout << "Higher agents: ";
            for (auto i : higher_agents)
                cout << i << ",";
            cout << endl;
        }
        Path new_path;
        if(!findPathForSingleAgent(*node, higher_agents, a, new_path))
        {
            delete node;
            parent->children[child_id] = nullptr;
            return false;
        }

        // Delete old conflicts
        for (auto c = node->conflicts.begin(); c != node->conflicts.end();)
        {
            if ((*c)->a1 == a or (*c)->a2 == a)
                c = node->conflicts.erase(c);
            else
                ++c;
        }

        // Update conflicts and to_replan
        set<int> lower_agents;
        auto p2 = ordered_agents.begin();
        std::advance(p2, num_of_agents - 1 - rank);
        assert(*p2 == a);
        getLowerPriorityAgents(p2, lower_agents);
        node->ag_weights[a] = lower_agents.size();  // This is for MVC in new conflicts
        if (screen > 2 and !lower_agents.empty())
        {
            cout << "Lower agents: ";
            for (auto i : lower_agents)
                cout << i << ",";
            cout << endl;
        }

        // Find new conflicts
        for (auto a2 = 0; a2 < num_of_agents; a2++)
        {
            if (a2 == a or lookup_table[a2] or higher_agents.count(a2) > 0)
                continue;  // already in to_replan or has higher priority
            clock_t t = clock();

            if (use_tr)
            {
                int priority = hasConflicts(a, a2);
                if (priority > 0)
                {
                    if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                    {
                        if (screen > 1)
                            cout << "\tshould replan " << a2 << " for colliding with " << a << endl;

                        // no need to put this conflict into PT node
                        to_replan.emplace(topological_orders[a2], a2);
                        lookup_table[a2] = true;
                    }
                    else if (priority == 1)
                    {
                        node->conflicts.emplace_front(new Conflict(a, a2, priority));
                    }
                    else if (priority == 2)  // target conflict
                    {
                        if (paths[a]->size() < paths[a2]->size())
                        {
                            need_replan_agents[a] = true;
                            node->conflicts.emplace_back(new Conflict(a, a2, priority));
                        }
                        else
                        {
                            need_replan_agents[a2] = true;
                            node->conflicts.emplace_back(new Conflict(a2, a, priority));
                        }
                    }
                }
            }
            else if (PBS::hasConflicts(a, a2))
            {
                node->conflicts.emplace_back(new Conflict(a, a2));
                if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                {
                    if (screen > 1)
                        cout << "\tshould replan " << a2 << " for colliding with " << a << endl;

                    // no need to put this conflict into PT node
                    to_replan.emplace(topological_orders[a2], a2);
                    lookup_table[a2] = true;
                }
            }
            runtime_detect_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
        }
    }

    num_HL_generated++;
    node->time_generated = num_HL_generated;
    if (screen > 1)
        cout << "Generate " << *node << endl;
    return true;
}

PBSNode* PVCS::selectNode()
{
	PBSNode* curr = open_list.top();
    open_list.pop();
    num_HL_expanded++;
    curr->time_expanded = num_HL_expanded;
	if (screen > 1)
		cout << endl << "Pop " << *curr << endl;
	return curr;
}

string PVCS::getSolverName() const
{
	return "PVCS with " + search_engines[0]->getName();
}

bool PVCS::topologicalSort(list<int>& stack)
{
    stack.clear();
    vector<bool> visited(num_of_agents, false);
    vector<bool> onstack(num_of_agents, false);

    // Call the recursive helper function to store Topological
    // Sort starting from all vertices one by one
    for (int i = 0; i < num_of_agents; i++)
    {
        if (!visited[i])
            if (topologicalSortUtil(i, visited, stack, onstack))
                return true;  // returns true if there is a cycle
    }
    return false;
}

bool PVCS::topologicalSortUtil(int v, vector<bool>& visited, 
    list<int>& stack, vector<bool>& onstack)
{
    // Mark the current node as visited.
    visited[v] = true;
    onstack[v] = true;

    // Recur for all the vertices adjacent to this vertex
    assert(!priority_graph.empty());
    for (int i = 0; i < num_of_agents; i++)
    {
        if (priority_graph[v][i])  // i is a neighbor of v
        {
            if (visited[i] and onstack[i])
                return true;
            else if (!visited[i] and topologicalSortUtil(i, visited, stack, onstack))
                return true;
        }
    }
    onstack[v] = false;
    stack.push_back(v);  // Push current vertex to stack which stores result
    return false;
}