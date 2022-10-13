//#pragma warning(disable: 4996)
#include <algorithm>
#include <random>
#include <chrono>
#include "PBS2.h"
#include "SIPP.h"
#include "SpaceTimeAStar.h"

PBS2::PBS2(const Instance& instance, bool sipp, int screen,
    bool use_tr, bool use_ic, bool use_rr): PBS(instance, sipp, screen), 
    use_tr(use_tr), use_ic(use_ic), use_rr(use_rr) {}

bool PBS2::solve(clock_t time_limit)
{
    this->time_limit = time_limit;

    if (screen > 0) // 1 or 2
    {
        string name = getSolverName();
        name.resize(35, ' ');
        cout << name << ": ";
    }
    // set timer
    start = steady_clock::now();

    while (solution_cost == -2)  // Not yet find a solution
    {
        generateRoot();

        while (!open_list.empty())
        {
            auto curr = selectNode();

            if (terminate(curr)) break;

            steady_clock::time_point t1;
            if (!curr->is_expanded)  // This is not a back-tracking
            {
                curr->conflict = chooseConflict(*curr);
                curr->is_expanded = true;
                if (screen > 1)
                    cout << "	Expand " << *curr << "	on " << *(curr->conflict) << endl;

                assert(!hasHigherPriority(curr->conflict->a1, curr->conflict->a2) and 
                    !hasHigherPriority(curr->conflict->a2, curr->conflict->a1) );

                t1 = steady_clock::now();
                generateChild(0, curr, curr->conflict->a1, curr->conflict->a2);
                runtime_generate_child += getDuration(t1, steady_clock::now());

                if (curr->children[0] != nullptr)
                    pushNode(curr->children[0]);
            }
            else  // We only generate another child node if back-tracking happens
            {
                if (screen > 1)
                    cout << "	Expand " << *curr << "	on " << *(curr->conflict) << endl;

                open_list.pop();
                t1 = steady_clock::now();    
                generateChild(1, curr, curr->conflict->a2, curr->conflict->a1);
                runtime_generate_child += getDuration(t1, steady_clock::now());

                if (curr->children[1] != nullptr)
                {
                    pushNode(curr->children[1]);
                }
                else
                {
                    num_backtrack ++;
                    if (use_rr)
                    {
                        clear();
                        stack<PBSNode*>().swap(open_list);  // clear the open_list
                        num_restart ++;
                        break;  // leave the while loop of open_list.empty
                    }
                }
                curr->clear();
            }
        }  // end of while loop
    }
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

bool PBS2::generateRoot()
{
	PBSNode* root = new PBSNode();
	root->cost = 0;
	paths = vector<Path*>(num_of_agents, nullptr);
    std::random_shuffle(init_agents.begin(), init_agents.end());

    set<int> higher_agents;
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
    steady_clock::time_point t = steady_clock::now();
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
                    {
                        root->conflicts.emplace_back(new Conflict(a1, a2, priority));
                    }
                    else  // a2 is at its goal location
                    {
                        root->conflicts.emplace_back(new Conflict(a2, a1, priority));
                    }
                }
            }
            else if (PBS::hasConflicts(a1, a2))  // not using target reasoning
            {
                root->conflicts.emplace_back(new Conflict(a1, a2));
            }
        }
    }
    runtime_detect_conflicts += getDuration(t, steady_clock::now());
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
            steady_clock::time_point t = steady_clock::now();

            if (use_tr)
            {
                int priority = hasConflicts(a, a2);
                if (priority > 0)
                {
                    if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                    {
                        if (screen > 1)
                            cout << "\tshould replan " << a2 << " for colliding with " << a << endl;
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
            }
            else if (PBS::hasConflicts(a, a2))
            {
                node->conflicts.emplace_back(new Conflict(a, a2));
                if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                {
                    if (screen > 1)
                        cout << "\tshould replan " << a2 << " for colliding with " << a << endl;
                    to_replan.emplace(topological_orders[a2], a2);
                    lookup_table[a2] = true;
                }
            }
            runtime_detect_conflicts += getDuration(t, steady_clock::now());
        }
    }

    if (use_ic)  // Compute the implicit constraints
    {
        computeImplicitConstraints(node, topological_orders);
    }

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
    if (node.conflicts.empty())
        return nullptr;

    shared_ptr<Conflict> out = node.conflicts.back();

    if (use_tr and out->priority == 2)
        return out;

    if (use_ic)
    {
        for (const auto& conf : node.conflicts)
            if (out->max_num_ic < conf->max_num_ic)
                out = conf;
    }

    return out;
}

void PBS2::computeImplicitConstraints(PBSNode* node, const vector<int>& topological_orders)
{
    vector<int> num_higher_ags(num_of_agents, -1);
    vector<int> num_lower_ags(num_of_agents, -1);
    list<int>::reverse_iterator ag_rit;
    list<int>::iterator ag_it;
    set<int> tmp_agents;

    steady_clock::time_point t = steady_clock::now();
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
    runtime_implicit_constraints += getDuration(t, steady_clock::now());
}