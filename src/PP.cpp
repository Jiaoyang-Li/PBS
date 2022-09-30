#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include "PP.h"
#include "SIPP.h"
#include "SpaceTimeAStar.h"

PP::PP(const Instance& instance, bool sipp, int screen) :
    screen(screen), num_of_agents(instance.getDefaultNumberOfAgents()),
    num_of_cols(instance.num_of_cols), map_size(instance.map_size)
{
    clock_t t = clock();

    paths = vector<Path*>(num_of_agents, nullptr);
    ordered_agents = vector<int>(num_of_agents);
    iota(ordered_agents.begin(), ordered_agents.end(), 0);

    search_engines.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
    {
        if (sipp)
            search_engines[i] = new SIPP(instance, i);
        else
            search_engines[i] = new SpaceTimeAStar(instance, i);
    }
    runtime_preprocessing = (double) (clock() - t) / CLOCKS_PER_SEC;

    if (screen > 1)  // print start and goals
        instance.printAgents();
}

bool PP::solve(double _time_limit)
{
    this->time_limit = _time_limit;

    if (screen > 0)
    {
        string name = getSolverName();
        name.resize(35, ' ');
        cout << name << ": ";
    }

    start = clock();  // set timer
    
    while (runtime < time_limit)
    {
        std::random_shuffle(ordered_agents.begin(), ordered_agents.end());
        ConstraintTable constraint_table(num_of_cols, map_size);
        auto p = ordered_agents.begin();
        while (p != ordered_agents.end())
        {
            int id = *p;
            clock_t t = clock();
            Path new_path = search_engines[id]->findOptimalPath(constraint_table);
            num_LL_expanded += search_engines[id]->num_expanded;
            num_LL_generated += search_engines[id]->num_generated;
            runtime_build_CT += search_engines[id]->runtime_build_CT;
            runtime_build_CAT += search_engines[id]->runtime_build_CAT;
            runtime_path_finding += (double)(clock() - t) / CLOCKS_PER_SEC;
            runtime = (double)(clock()-start) / CLOCKS_PER_SEC;
            if (runtime >= time_limit)
            {
                solution_cost = -1;
                break;
            }
            else if (new_path.empty())
            {
                if (screen > 1)
                    cout << "No path exists for agent " << id << endl;
                num_of_restart ++;
                break;
            }
            paths[id] = new Path(new_path);
            constraint_table.insert2CT(new_path);

            if (screen > 1)
                printAgentPath(id);
            
            ++p;
        }
        if (p == ordered_agents.end())
        {
            runtime = (double)(clock()-start) / CLOCKS_PER_SEC;
            solution_found = true;
            solution_cost = 0;
            for (const auto& p : paths)
                solution_cost += (int)p->size() - 1;
            if (!validateSolution())
            {
                cout << "Solution invalid!!!" << endl;
                printPaths();
                exit(1);
            }
            if (screen > 0) // 1 or 2
			    printResults();
            return solution_found;
        }
        else if (solution_cost == -1)
        {
            runtime = (double)(clock()-start) / CLOCKS_PER_SEC;
            cout << "Timeout" << endl;
            return solution_found;
        }
        runtime = (double)(clock()-start) / CLOCKS_PER_SEC;
    }
    return solution_found;
}

void PP::clearSearchEngines()
{
	for (auto s : search_engines)
		delete s;
	search_engines.clear();
}

void PP::saveResults(const string &fileName, const string &instanceName) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist)
	{
		ofstream addHeads(fileName);
		addHeads << "runtime,#high-level expanded,#high-level generated," <<
            "#low-level expanded,#low-level generated,#restart" <<
			"solution cost,root g value," <<
			"runtime of detecting conflicts,runtime of building constraint tables,runtime of building CATs," <<
			"runtime of path finding,runtime of generating child nodes," <<
			"preprocessing runtime,solver name,instance name" << endl;
		addHeads.close();
	}
	ofstream stats(fileName, std::ios::app);
	stats << runtime << "," <<
        num_HL_expanded << "," << num_HL_generated << "," <<
        num_LL_expanded << "," << num_LL_generated << "," << num_of_restart << "," <<
        solution_cost << "," << 0 << "," <<
		runtime_detect_conflicts << "," << runtime_build_CT << "," << runtime_build_CAT << "," <<
		runtime_path_finding << "," << 0 << "," <<
		runtime_preprocessing << "," << getSolverName() << "," << instanceName << endl;
	stats.close();
}

void PP::savePaths(const string &fileName) const
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

// used for rapid random  restart
void PP::clear()
{
	paths.clear();
	solution_found = false;
	solution_cost = -2;
}

string PP::getSolverName() const
{
	return "PP with " + search_engines[0]->getName();
}

int PP::getSumOfCosts() const
{
   int cost = 0;
   for (const auto & path : paths)
       cost += (int)path->size() - 1;
   return cost;
}

void PP::printResults() const
{
	if (solution_cost >= 0) // solved
		cout << "Succeed,";
	else if (solution_cost == -1) // time_out
		cout << "Timeout,";
	else if (solution_cost == -2) // no solution
		cout << "No solutions,";
	else if (solution_cost == -3) // nodes out
		cout << "Nodesout,";

	cout << solution_cost << "," << runtime << "," << num_LL_expanded << endl;
    /*if (solution_cost >= 0) // solved
    {
        cout << "fhat = [";
        auto curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->getFHatVal() << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "hhat = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->cost_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "d = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->distance_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "soc = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->getFHatVal() - curr->cost_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
    }*/
}

bool PP::validateSolution() const
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

inline int PP::getAgentLocation(int agent_id, size_t timestep) const
{
	size_t t = max(min(timestep, paths[agent_id]->size() - 1), (size_t)0);
	return paths[agent_id]->at(t).location;
}

void PP::printPaths() const
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

void PP::printAgentPath(int i) const
{
    cout << "Agent " << i << " (" << search_engines[i]->my_heuristic[search_engines[i]->start_location] << " -->" <<
        paths[i]->size() - 1 << "): ";
    for (const auto & t : *paths[i])
        cout << t.location << "->";
    cout << endl;
}