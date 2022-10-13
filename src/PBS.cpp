#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include "PBS.h"
#include "SIPP.h"
#include "SpaceTimeAStar.h"


PBS::PBS(const Instance& instance, bool sipp, int screen) :
        screen(screen),
        num_of_agents(instance.getDefaultNumberOfAgents())
{
    clock_t t = clock();

    init_agents = vector<int>(num_of_agents);
    iota(init_agents.begin(), init_agents.end(), 0);

    search_engines.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
    {
        if (sipp)
            search_engines[i] = new SIPP(instance, i);
        else
            search_engines[i] = new SpaceTimeAStar(instance, i);
    }
    runtime_preprocessing = (double)(clock() - t) / CLOCKS_PER_SEC;

    if (screen >= 2) // print start and goals
    {
        instance.printAgents();
    }
}


bool PBS::solve(double _time_limit)
{
    this->time_limit = _time_limit;

    if (screen > 0) // 1 or 2
    {
        string name = getSolverName();
        name.resize(35, ' ');
        cout << name << ": ";
    }
    start = clock();  // set timer

    generateRoot();

    while (!open_list.empty())
    {
        auto curr = selectNode();

        if (terminate(curr)) break;

        curr->conflict = chooseConflict(*curr);

        if (screen > 1)
            cout << "	Expand " << *curr << "	on " << *(curr->conflict) << endl;

        assert(!hasHigherPriority(curr->conflict->a1, curr->conflict->a2) and
               !hasHigherPriority(curr->conflict->a2, curr->conflict->a1) );
        auto t1 = clock();
        vector<Path*> copy(paths);
        generateChild(0, curr, curr->conflict->a1, curr->conflict->a2);
        paths = copy;
        generateChild(1, curr, curr->conflict->a2, curr->conflict->a1);
        runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
        pushNodes(curr->children[0], curr->children[1]);
        curr->clear();
    }  // end of while loop
    return solution_found;
}

bool PBS::generateChild(int child_id, PBSNode* parent, int low, int high)
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
    auto i = num_of_agents - 1;
    for (const auto & a : ordered_agents)
    {
        topological_orders[a] = i;
        i--;
    }
    checkCycle(topological_orders);

    std::priority_queue<pair<int, int>> to_replan; // <position in ordered_agents, agent id>
    vector<bool> lookup_table(num_of_agents, false);
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
            std::swap(a1, a2);  // a1 has smaller priority than a2
        }
        if (!lookup_table[a1] and lower_agents.find(a1) != lower_agents.end() and higher_agents.find(a2) != higher_agents.end())
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
            if (a2 == a or lookup_table[a2] or higher_agents.count(a2) > 0) // already in to_replan or has higher priority
                continue;
            auto t = clock();
            if (hasConflicts(a, a2))
            {
                node->conflicts.emplace_back(new Conflict(a, a2));
                if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                {
                    if (screen > 1)
                        cout << "\t" << a2 << " needs to be replanned due to collisions with " << a << endl;
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

bool PBS::findPathForSingleAgent(PBSNode& node, const set<int>& higher_agents, int a, Path& new_path)
{
    clock_t t = clock();
    new_path = search_engines[a]->findOptimalPath(higher_agents, paths, a);  //TODO: add runtime check to the low level
    num_LL_search_calls ++;
    num_LL_expanded += search_engines[a]->num_expanded;
    num_LL_generated += search_engines[a]->num_generated;
    runtime_build_CT += search_engines[a]->runtime_build_CT;
    runtime_build_CAT += search_engines[a]->runtime_build_CAT;
    runtime_path_finding += (double)(clock() - t) / CLOCKS_PER_SEC;
    if (new_path.empty())
        return false;
    if (isSamePath(*paths[a], new_path))
    {
        cout << "The path is the same" << endl;
        printPath(new_path);
    }
    assert(paths[a] != nullptr and !isSamePath(*paths[a], new_path));
    node.cost += (int)new_path.size() - (int)paths[a]->size();
    if (node.makespan >= paths[a]->size())
    {
        node.makespan = max(node.makespan, new_path.size() - 1);
    }
    else
    {
        node.makespan = 0;
        for (int i = 0; i < num_of_agents; i++)
        {
            if (i == a and new_path.size() - 1 > node.makespan)
                node.makespan = new_path.size() - 1;
            else
                node.makespan = max(node.makespan, paths[i]->size() - 1);
        }
    }
    node.paths.emplace_back(a, new_path);
    paths[a] = &node.paths.back().second;
    assert(!hasConflicts(a, higher_agents));
    return true;
}

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
void PBS::update(PBSNode* node)
{
    paths.assign(num_of_agents, nullptr);
    priority_graph.assign(num_of_agents, vector<bool>(num_of_agents, false));
    for (auto curr = node; curr != nullptr; curr = curr->parent)
	{
		for (auto & path : curr->paths)
		{
			if (paths[path.first] == nullptr)
			{
				paths[path.first] = &(path.second);
			}
		}
        if (curr->parent != nullptr) // non-root node
            priority_graph[curr->constraint.low][curr->constraint.high] = true;
	}
    assert(getSumOfCosts() == node->cost);
}

int PBS::hasConflicts(int a1, int a2) const
{
	int min_path_length = (int) (paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size());
	for (int timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = paths[a1]->at(timestep).location;
		int loc2 = paths[a2]->at(timestep).location;
		if (loc1 == loc2 or (timestep < min_path_length - 1 and loc1 == paths[a2]->at(timestep + 1).location
                             and loc2 == paths[a1]->at(timestep + 1).location)) // vertex or edge conflict
		{
            return true;
		}
	}
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
				return true; // target conflict
			}
		}
	}
    return false; // conflict-free
}

bool PBS::hasConflicts(int a1, const set<int>& agents) const
{
    for (auto a2 : agents)
    {
        if (hasConflicts(a1, a2))
            return true;
    }
    return false;
}

shared_ptr<Conflict> PBS::chooseConflict(const PBSNode &node) const
{
	if (screen == 3)
		printConflicts(node);
	if (node.conflicts.empty())
		return nullptr;
    return node.conflicts.back();
}

int PBS::getSumOfCosts() const
{
   int cost = 0;
   for (const auto & path : paths)
       cost += (int)path->size() - 1;
   return cost;
}
void PBS::pushNodes(PBSNode* n1, PBSNode* n2)
{
    if (n1 != nullptr and n2 != nullptr)
    {
        if (n1->cost < n2->cost)
        {
            pushNode(n2);
            pushNode(n1);
        }
        else
        {
            pushNode(n1);
            pushNode(n2);
        }
    }
    else if (n1 != nullptr)
    {
        pushNode(n1);
    }
    else if (n2 != nullptr)
    {
        pushNode(n2);
    }
    else
    {
        num_backtrack++;
    }
}

PBSNode* PBS::selectNode()
{
	PBSNode* curr = open_list.top();
    open_list.pop();
    update(curr);
    num_HL_expanded++;
    curr->time_expanded = num_HL_expanded;
	if (screen > 1)
		cout << endl << "Pop " << *curr << endl;
	return curr;
}

void PBS::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent " << i << " (" << search_engines[i]->my_heuristic[search_engines[i]->start_location] << " -->" <<
			paths[i]->size() - 1 << "): ";
		for (const auto & t : *paths[i])
			cout << t.location << "->";
		cout << endl;
	}
}

void PBS::printPath(const Path& _path_) const
{
    for (const auto & t : _path_)
        cout << t.location << "->";
    cout << endl;
}

void PBS::printPriorityGraph() const
{
    cout << "Priority graph:";
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = 0; a2 < num_of_agents; a2++)
        {
            if (priority_graph[a1][a2])
                cout << a1 << "<" << a2 << ",";
        }
    }
    cout << endl;
}

void PBS::printResults() const
{
	if (solution_cost >= 0) // solved
		cout << "Succeed,";
	else if (solution_cost == -1) // time_out
		cout << "Timeout,";
	else if (solution_cost == -2) // no solution
		cout << "No solutions,";
	else if (solution_cost == -3) // nodes out
		cout << "Nodesout,";

	cout << solution_cost << "," << runtime << "," <<
         num_HL_expanded << "," << num_LL_expanded << "," << // HL_num_generated << "," << LL_num_generated << "," <<
		dummy_start->cost << "," << endl;
}

void PBS::saveResults(const string &fileName, const string &instanceName) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist)
	{
		ofstream addHeads(fileName);
		addHeads << "runtime,#high-level expanded,#high-level generated," <<
            "#low-level expanded,#low-level generated,#backtrack," << 
            "#low-level search calls,#restarts," <<
			"solution cost,root g value," <<
			"runtime of detecting conflicts,runtime of building constraint tables," <<
			"runtime of building CATs,runtime of path finding,runtime of generating child nodes," <<
			"preprocessing runtime,runtime of implicit constraints,runtime of MVC," <<
            "solver name,instance name" << endl;
		addHeads.close();
	}
	ofstream stats(fileName, std::ios::app);
	stats << runtime << "," << num_HL_expanded << "," << num_HL_generated << "," <<
        num_LL_expanded << "," << num_LL_generated << "," << num_backtrack << "," << 
        num_LL_search_calls << "," << num_restart << "," <<
        solution_cost << "," << dummy_start->cost << "," <<
		runtime_detect_conflicts << "," << runtime_build_CT << "," << runtime_build_CAT << "," <<
		runtime_path_finding << "," << runtime_generate_child << "," <<
		runtime_preprocessing << "," << runtime_implicit_constraints << "," << 
        runtime_run_mvc << "," <<
        getSolverName() << "," << instanceName << endl;
	stats.close();
}

void PBS::saveCT(const string &fileName) const // write the CT to a file
{
	// Write the tree graph in dot language to a file
	{
		std::ofstream output;
		output.open(fileName + ".tree", std::ios::out);
		output << "digraph G {" << endl;
		output << "size = \"5,5\";" << endl;
		output << "center = true;" << endl;
		set<PBSNode*> path_to_goal;
		auto curr = goal_node;
		while (curr != nullptr)
		{
			path_to_goal.insert(curr);
			curr = curr->parent;
		}
		for (const auto& node : allNodes_table)
		{
			output << node->time_generated << " [label=\"g=" << node->cost;
			if (node->time_expanded > 0) // the node has been expanded
			{
				output << "\n #" << node->time_expanded;
			}
			output << "\"]" << endl;


			if (node == dummy_start)
				continue;
			if (path_to_goal.find(node) == path_to_goal.end())
			{
				output << node->parent->time_generated << " -> " << node->time_generated << endl;
			}
			else
			{
				output << node->parent->time_generated << " -> " << node->time_generated << " [color=red]" << endl;
			}
		}
		output << "}" << endl;
		output.close();
	}

	// Write the stats of the tree to a CSV file
	{
		std::ofstream output;
		output.open(fileName + "-tree.csv", std::ios::out);
		// header
		output << "time generated,g value,h value,h^ value,d value,depth,time expanded,chosen from,h computed,"
			<< "f of best in cleanup,f^ of best in cleanup,d of best in cleanup,"
			<< "f of best in open,f^ of best in open,d of best in open,"
			<< "f of best in focal,f^ of best in focal,d of best in focal,"
			<< "praent,goal node" << endl;
		for (auto& node : allNodes_table)
		{
			output << node->time_generated << ","
                   << node->cost << ","
				<< node->depth << ","
				<< node->time_expanded << ",";
			if (node->parent == nullptr)
				output << "0,";
			else
				output << node->parent->time_generated << ",";
			if (node == goal_node)
				output << "1" << endl;
			else
				output << "0" << endl;
		}
		output.close();
	}

}

void PBS::savePaths(const string &fileName) const
{
    std::ofstream output;
    output.open(fileName, std::ios::out);
    for (int i = 0; i < num_of_agents; i++)
    {
        output << "Agent " << i << ": ";
        for (const auto & t : *paths[i])
            output << "(" << search_engines[0]->instance.getRowCoordinate(t.location)
                   << "," << search_engines[0]->instance.getColCoordinate(t.location) << ")->";
        output << endl;
    }
    output.close();
}

void PBS::printConflicts(const PBSNode &curr, int num)
{
    int counter = 0;
	for (const auto& conflict : curr.conflicts)
	{
		cout << *conflict << endl;
        counter ++;
        if (counter > num)
            break;
	}
}


string PBS::getSolverName() const
{
	return "PBS with " + search_engines[0]->getName();
}


bool PBS::terminate(PBSNode* curr)
{
	runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
	if (curr->conflicts.empty())  // no conflicts, we find a solution
	{
		solution_found = true;
		goal_node = curr;
		solution_cost = goal_node->cost;
		if (!validateSolution())
		{
			cout << "Solution invalid!!!" << endl;
			printPaths();
			exit(-1);
		}
		if (screen > 0) // 1 or 2
			printResults();
		return true;
	}
	if (runtime > time_limit || num_HL_expanded > node_limit)
	{   // time/node out
		solution_cost = -1;
		solution_found = false;
        if (screen > 0) // 1 or 2
            printResults();
		return true;
	}
	return false;
}


bool PBS::generateRoot()
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
            cout << "No path exists for agent " << i << endl;
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
            if(hasConflicts(a1, a2))
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
	if (screen >= 2)  // print start and goals
		printPaths();

	return true;
}

inline void PBS::releaseNodes()
{
	for (auto& node : allNodes_table)
		delete node;
	allNodes_table.clear();
}

PBS::~PBS()
{
	releaseNodes();
}

void PBS::clearSearchEngines()
{
	for (auto s : search_engines)
		delete s;
	search_engines.clear();
}


bool PBS::validateSolution() const
{
	// check whether the paths are feasible
	size_t soc = 0;
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		soc += paths[a1]->size() - 1;
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
			size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
			for (size_t timestep = 0; timestep < min_path_length; timestep++)
			{
				int loc1 = paths[a1]->at(timestep).location;
				int loc2 = paths[a2]->at(timestep).location;
				if (loc1 == loc2)
				{
					cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
					return false;
				}
				else if (timestep < min_path_length - 1
					&& loc1 == paths[a2]->at(timestep + 1).location
					&& loc2 == paths[a1]->at(timestep + 1).location)
				{
					cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
						loc1 << "-->" << loc2 << ") at timestep " << timestep << endl;
					return false;
				}
			}
			if (paths[a1]->size() != paths[a2]->size())
			{
				int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
				int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
				int loc1 = paths[a1_]->back().location;
				for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
				{
					int loc2 = paths[a2_]->at(timestep).location;
					if (loc1 == loc2)
					{
						cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
						return false; // It's at least a semi conflict			
					}
				}
			}
		}
	}
	if ((int)soc != solution_cost)
	{
		cout << "The solution cost is wrong!" << endl;
		return false;
	}
	return true;
}

inline int PBS::getAgentLocation(int agent_id, size_t timestep) const
{
	size_t t = max(min(timestep, paths[agent_id]->size() - 1), (size_t)0);
	return paths[agent_id]->at(t).location;
}

// used for rapid random  restart
void PBS::clear()
{
	releaseNodes();
	paths.clear();
	dummy_start = nullptr;
	goal_node = nullptr;
	solution_found = false;
	solution_cost = -2;
}

void PBS::topologicalSort(list<int>& stack)
{
    stack.clear();
    vector<bool> visited(num_of_agents, false);

    // Call the recursive helper function to store Topological
    // Sort starting from all vertices one by one
    for (int i = 0; i < num_of_agents; i++)
    {
        if (!visited[i])
            topologicalSortUtil(i, visited, stack);
    }
}

void PBS::topologicalSortUtil(int v, vector<bool> & visited, list<int> & stack)
{
    // Mark the current node as visited.
    visited[v] = true;

    // Recur for all the vertices adjacent to this vertex
    assert(!priority_graph.empty());
    for (int i = 0; i < num_of_agents; i++)
    {
        if (priority_graph[v][i] and !visited[i])
            topologicalSortUtil(i, visited, stack);
    }
    // Push current vertex to stack which stores result
    stack.push_back(v);
}

bool PBS::checkCycle(const vector<int>& topological_orders)
{
    // Stores the positions of agents in topological_orders
    for (int a1 = 0; a1 < num_of_agents; a1++)
        for (int a2 = 0; a2 < num_of_agents; a2++)
            if (priority_graph[a1][a2] and
                topological_orders[a1] > topological_orders[a2])
                return true;  // Cycle exists

    return false;  // Return false if cycle does not exist
}

void PBS::getHigherPriorityAgents(const list<int>::reverse_iterator & p1, set<int>& higher_agents)
{
    for (auto p2 = std::next(p1); p2 != ordered_agents.rend(); ++p2)
    {
        if (priority_graph[*p1][*p2])
        {
            auto ret = higher_agents.insert(*p2);
            if (ret.second) // insert successfully
            {
                getHigherPriorityAgents(p2, higher_agents);
            }
        }
    }
}
void PBS::getLowerPriorityAgents(const list<int>::iterator & p1, set<int>& lower_agents)
{
    for (auto p2 = std::next(p1); p2 != ordered_agents.end(); ++p2)
    {
        if (priority_graph[*p2][*p1])
        {
            auto ret = lower_agents.insert(*p2);
            if (ret.second) // insert successfully
            {
                getLowerPriorityAgents(p2, lower_agents);
            }
        }
    }
}

bool PBS::hasHigherPriority(int low, int high) const // return true if agent low is lower than agent high
{
    std::queue<int> Q;
    vector<bool> visited(num_of_agents, false);
    visited[low] = false;
    Q.push(low);
    while(!Q.empty())
    {
        auto n = Q.front();
        Q.pop();
        if (n == high)
            return true;
        for (int i = 0; i < num_of_agents; i++)
        {
            if (priority_graph[n][i] and !visited[i])
                Q.push(i);
        }
    }
    return false;
}
