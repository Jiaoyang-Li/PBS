#include <algorithm>
#include <random>
#include <chrono>
#include "PBS2.h"
#include "SIPP.h"
#include "SpaceTimeAStar.h"

PBS2::PBS2(const Instance& instance, bool sipp, int screen) :
    PBS(instance, sipp, screen) {}

bool PBS2::solve(double time_limit)
{
    this->time_limit = time_limit;

    if (screen > 0) // 1 or 2
    {
        string name = getSolverName();
        name.resize(35, ' ');
        cout << name << ": ";
    }
    // set timer
    start = clock();

    generateRoot();

    while (!open_list.empty())
    {
        auto curr = selectNode();

        if (terminate(curr)) break;

        clock_t t1;
        if (!curr->is_expanded)  // This is not a back-tracking
        {
            curr->conflict = chooseConflict(*curr);
            // curr->pri_conflicts.pop();
            curr->is_expanded = true;
            if (screen > 1)
                cout << "	Expand " << *curr << "	on " << *(curr->conflict) << endl;

            assert(!hasHigherPriority(curr->conflict->a1, curr->conflict->a2) and 
                !hasHigherPriority(curr->conflict->a2, curr->conflict->a1) );

            t1 = clock();
            generateChild(0, curr, curr->conflict->a1, curr->conflict->a2);
            runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;

            if (curr->children[0] != nullptr)
                pushNode(curr->children[0]);
        }
        else  // We only generate another child node if back-tracking happens
        {
            if (screen > 1)
                cout << "	Expand " << *curr << "	on " << *(curr->conflict) << endl;

            open_list.pop();
            t1 = clock();    
            generateChild(1, curr, curr->conflict->a2, curr->conflict->a1);
            runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;

            if (curr->children[1] != nullptr)
                pushNode(curr->children[1]);
            else
                num_backtrack ++;
            curr->clear();
        }
    }  // end of while loop
    return solution_found;
}

PBSNode* PBS2::selectNode()
{
	PBSNode* curr = open_list.top();
    update(curr);
    if (!curr->is_expanded)
    {
        num_HL_expanded++;
        curr->time_expanded = num_HL_expanded;
    }
	if (screen > 1)
		cout << endl << "Select " << *curr << endl;
	return curr;
}

string PBS2::getSolverName() const
{
	return "PBS2 with " + search_engines[0]->getName();
}

// shared_ptr<Conflict> PBS2::chooseConflict(const PBSNode &node) const
// {
//     if (screen == 3)
//         printConflicts(node);
//     if (node.pri_conflicts.empty())
//         return nullptr;
//     return node.pri_conflicts.top();
// }

bool PBS2::generateRoot()
{
	auto root = new PBSNode();
	root->cost = 0;
	paths.reserve(num_of_agents);

    set<int> higher_agents;
    for (auto i = 0; i < num_of_agents; i++)
    {
        //CAT cat(dummy_start->makespan + 1);  // initialized to false
        //updateReservationTable(cat, i, *dummy_start);
        auto new_path = search_engines[i]->findOptimalPath(higher_agents, paths, i);
        num_LL_expanded += search_engines[i]->num_expanded;
        num_LL_generated += search_engines[i]->num_generated;
        if (new_path.empty())
        {
            cout << "No path exists for agent " << i << endl;
            return false;
        }
        root->paths.emplace_back(i, new_path);
        paths.emplace_back(&root->paths.back().second);
        root->makespan = max(root->makespan, new_path.size() - 1);
        root->cost += (int)new_path.size() - 1;
    }
    auto t = clock();
	root->depth = 0;
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            int priority = hasConflicts(a1, a2);
            if(priority == 1)
                root->conflicts.emplace_front(new Conflict(a1, a2, priority));
            else if (priority == 2)
            {
                if (paths[a1]->size() < paths[a2]->size())
                    root->conflicts.emplace_back(new Conflict(a1, a2, priority));
                else
                    root->conflicts.emplace_back(new Conflict(a2, a1, priority));
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
	{
		printPaths();
	}

	return true;
}

bool PBS2::generateChild(int child_id, PBSNode* parent, int low, int high)
{
    assert(child_id == 0 or child_id == 1);
    parent->children[child_id] = new PBSNode(*parent);
    auto node = parent->children[child_id];
    node->constraint.set(low, high);
    priority_graph[high][low] = false;
    priority_graph[low][high] = true;
    if (screen > 2)
        printPriorityGraph();
    topologicalSort(ordered_agents);
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
    vector<bool> lookup_table(num_of_agents, false);  // True if the agent is in to_replan
    to_replan.emplace(topological_orders[low], low);
    lookup_table[low] = true;

    // find conflicts where one agent is higher than high and the other agent is lower than low
    set<int> higher_agents;
    auto p = ordered_agents.rbegin();
    std::advance(p, topological_orders[high]);
    assert(*p == high);
    getHigherPriorityAgents(p, higher_agents);
    higher_agents.insert(high);

    set<int> lower_agents;
    auto p2 = ordered_agents.begin();
    std::advance(p2, num_of_agents - 1 - topological_orders[low]);
    assert(*p2 == low);
    getLowerPriorityAgents(p2, lower_agents);

    for (const auto & conflict : node->conflicts)
    {
        int a1 = conflict->a1;
        int a2 = conflict->a2;
        if (a1 == low or a2 == low)
            continue;
        if (topological_orders[a1] > topological_orders[a2])
        {
            std::swap(a1, a2);  // a1 always has a smaller priority than a2
        }
        if (lower_agents.find(a1) != lower_agents.end() and 
            higher_agents.find(a2) != higher_agents.end() and
            !lookup_table[a1])
        {
            to_replan.emplace(topological_orders[a1], a1);
            lookup_table[a1] = true;
        }
    }

    while(!to_replan.empty())  // Only replan agents with lower priorities AND has known conflicts
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

        #ifndef DEBUG
        if (screen > 3)
        {
            cout << "Before deleting conflicts" << endl;
            printConflicts(*node, 5);
        }
        #endif

        // Delete old conflicts
        for (auto c = node->conflicts.begin(); c != node->conflicts.end();)
        {
            if ((*c)->a1 == a or (*c)->a2 == a)
                c = node->conflicts.erase(c);
            else
                ++c;
        }

        #ifndef DEBUG
        if (screen > 3)
        {
            cout << "After deleting conflicts" << endl;
            printConflicts(*node, 5);
            cout << "----------------------------------" << endl;
        }
        #endif

        // Update conflicts and to_replan
        set<int> lower_agents;
        auto p2 = ordered_agents.begin();
        std::advance(p2, num_of_agents - 1 - rank);
        assert(*p2 == a);
        getLowerPriorityAgents(p2, lower_agents);
        if (screen > 2 and !lower_agents.empty())
        {
            cout << "Lower agents: ";
            for (auto i : lower_agents)
                cout << i << ",";
            cout << endl;
        }

        // Find new conflicts
        #ifndef DEBUG
        if (screen > 2)
            cout << "Find new conflicts" << endl;
        #endif
        for (auto a2 = 0; a2 < num_of_agents; a2++)
        {
            if (a2 == a or lookup_table[a2] or higher_agents.count(a2) > 0) // already in to_replan or has higher priority
                continue;
            auto t = clock();
            int priority = hasConflicts(a, a2);
            if (priority > 0)
            {
                #ifndef DEBUG
                if (screen > 3)
                {
                    printConflicts(*node, 5);
                    cout << "----------------------------------" << endl;
                }
                #endif

                if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                {
                    if (screen > 1)
                        cout << "\t" << a2 << " needs to be replanned due to collisions with " << a << endl;
                    to_replan.emplace(topological_orders[a2], a2);
                    lookup_table[a2] = true;
                }
                else if (priority == 1)
                {
                    node->conflicts.emplace_front(new Conflict(a, a2, priority));
                }
                else if (priority == 2)
                {
                    if (paths[a]->size() < paths[a2]->size())
                        node->conflicts.emplace_back(new Conflict(a, a2, priority));
                    else
                        node->conflicts.emplace_back(new Conflict(a2, a, priority));
                }
            }
            runtime_detect_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
        }
    }

    // Compute the implicit constraints
    vector<int> num_higher_ags(num_of_agents, -1);
    vector<int> num_lower_ags(num_of_agents, -1);
    list<int>::reverse_iterator ag_rit;
    list<int>::iterator ag_it;
    set<int> tmp_agents;

    clock_t t = clock();
    for (auto& conf : node->conflicts)
    {
        if (conf->priority == 2) continue;

        if (num_higher_ags[conf->a1] == -1)
        {
            ag_rit = ordered_agents.rbegin();
            std::advance(ag_rit, topological_orders[conf->a1]);
            assert(*ag_rit == conf->a1);
            getHigherPriorityAgents(ag_rit, tmp_agents);
            num_higher_ags[conf->a1] = (int) tmp_agents.size();
            tmp_agents.clear();
        }

        if (num_lower_ags[conf->a1] == -1)
        {
            ag_it = ordered_agents.begin();
            std::advance(ag_it, num_of_agents - 1 - topological_orders[conf->a1]);
            assert(*ag_it == conf->a1);
            getLowerPriorityAgents(ag_it, tmp_agents);
            num_lower_ags[conf->a1] = (int) tmp_agents.size();
            tmp_agents.clear();
        }

        if (num_higher_ags[conf->a2] == -1)
        {
            ag_rit = ordered_agents.rbegin();
            std::advance(ag_rit, topological_orders[conf->a2]);
            assert(*ag_rit == conf->a2);
            getHigherPriorityAgents(ag_rit, tmp_agents);
            num_higher_ags[conf->a2] = (int) tmp_agents.size();
            tmp_agents.clear();
        }

        if (num_lower_ags[conf->a2] == -1)
        {
            ag_it = ordered_agents.begin();
            std::advance(ag_it, num_of_agents - 1 - topological_orders[conf->a2]);
            assert(*ag_it == conf->a2);
            getLowerPriorityAgents(ag_it, tmp_agents);
            num_lower_ags[conf->a2] = (int) tmp_agents.size();
            tmp_agents.clear();
        }

        int num_ic_a1_a2 = (num_higher_ags[conf->a1]+1) * (num_lower_ags[conf->a2]+1);
        int num_ic_a2_a1 = (num_higher_ags[conf->a2]+1) * (num_lower_ags[conf->a1]+1);
        conf->max_num_ic = max(num_ic_a1_a2, num_ic_a2_a1);
        if (num_ic_a1_a2 > num_ic_a2_a1)
            std::swap(conf->a1, conf->a2);
    }
    runtime_implicit_constraints += (double)(clock() - t) / CLOCKS_PER_SEC;

    num_HL_generated++;
    node->time_generated = num_HL_generated;
    if (screen > 1)
        cout << "Generate " << *node << endl;
    return true;
}

int PBS2::hasConflicts(int a1, int a2) const
{
	int min_path_length = (int) (paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size());
    if (paths[a1]->size() != paths[a2]->size())
	{
		int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
		int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
		int loc1 = paths[a1_]->back().location;
		for (int timestep = min_path_length; timestep < (int)paths[a2_]->size(); timestep++)
		{
			int loc2 = paths[a2_]->at(timestep).location;
			if (loc1 == loc2)
			{
				return 2; // target conflict
			}
		}
	}

    for (int timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = paths[a1]->at(timestep).location;
		int loc2 = paths[a2]->at(timestep).location;
		if (loc1 == loc2 or (timestep < min_path_length - 1 and loc1 == paths[a2]->at(timestep + 1).location
                             and loc2 == paths[a1]->at(timestep + 1).location)) // vertex or edge conflict
		{
            return 1;
		}
	}
    return 0; // conflict-free
}

shared_ptr<Conflict> PBS2::chooseConflict(const PBSNode &node) const
{
    if (screen == 3)
    {
        cout << "------------------------------" << endl;
        printConflicts(node);
    }

    if (node.conflicts.empty())
        return nullptr;

    shared_ptr<Conflict> out = node.conflicts.back();
    if (out->priority == 2)
        return out;
    for (const auto& conf : node.conflicts)
        if (out->max_num_ic < conf->max_num_ic)
            out = conf;
    if (screen == 3 && out->priority == 1)
    {
        cout << "Choose: " << *out << endl;
        cout << "------------------------------" << endl;
    }

    return out;
}
