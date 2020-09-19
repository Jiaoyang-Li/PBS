#include "GICBSSearch.h"
//#define ROOT
//#define DEBUG
//#define STAT


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void GICBSSearch::updatePaths(GICBSNode* curr) {
	for(int i = 0; i < num_of_agents; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false);  // initialized for false
												 /* used for backtracking -- only update paths[i] if it wasn't updated before (that is, by a younger node)
												 * because younger nodes take into account ancesstors' nodes constraints. */
	while (curr->parent != NULL)
	{
		for (list<pair<int, vector<PathEntry>>>::iterator it = curr->new_paths.begin(); it != curr->new_paths.end(); it++)
		{
			if (!updated[it->first])
			{
				paths[it->first] = &(it->second);
				updated[get<0>(*it)] = true;
			}
		}
		curr = curr->parent;
	}
}

void GICBSSearch::findConflicts(GICBSNode& curr)
{
	//vector<bool> hasConflicts(num_of_agents, false);
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
			size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
			for (size_t timestep = 0; timestep < min_path_length; timestep++)
			{
				int loc1 = paths[a1]->at(timestep).location;
				int loc2 = paths[a2]->at(timestep).location;
				if (loc1 == loc2)
				{
					curr.conflict = std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1, a2, loc1, -1, timestep));
					return;
					//hasConflicts[a1] = true;
					//hasConflicts[a2] = true;
				}
				else if (timestep < min_path_length - 1
					&& loc1 == paths[a2]->at(timestep + 1).location
					&& loc2 == paths[a1]->at(timestep + 1).location)
				{
					curr.conflict = std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1, a2, loc1, loc2, timestep + 1));
					//hasConflicts[a1] = true;
					//hasConflicts[a2] = true;
					return;
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
						curr.conflict = std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1_, a2_, loc1, -1, timestep)); // It's at least a semi conflict			
						//curr.unknownConf.front()->cost1 = timestep + 1;
						//hasConflicts[a1] = true;
						//hasConflicts[a2] = true;
						return;
					}
				}
			}
		}
	}
	curr.conflict = NULL;
	return;
}

int GICBSSearch::computeCollidingTeams()
{
	int result = 0;
	//vector<bool> hasConflicts(num_of_agents, false);
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
			bool isColliding = false;
			size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
			for (size_t timestep = 0; timestep < min_path_length; timestep++)
			{
				int loc1 = paths[a1]->at(timestep).location;
				int loc2 = paths[a2]->at(timestep).location;
				if (loc1 == loc2)
				{
					result++;
					isColliding = true;
					break;
				}
				else if (timestep < min_path_length - 1
					&& loc1 == paths[a2]->at(timestep + 1).location
					&& loc2 == paths[a1]->at(timestep + 1).location)
				{
					result++;
					isColliding = true;
					break;
				}
			}
			if (!isColliding && paths[a1]->size() != paths[a2]->size())
			{
				int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
				int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
				int loc1 = paths[a1_]->back().location;
				for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
				{
					int loc2 = paths[a2_]->at(timestep).location;
					if (loc1 == loc2)
					{
						result++;
						break;
					}
				}
			}
		}
	}
	return result;
}


/*
return agent_id's location for the given timestep
Note -- if timestep is longer than its plan length,
then the location remains the same as its last cell)
*/
inline int GICBSSearch::getAgentLocation(int agent_id, size_t timestep) {
	// if last timestep > plan length, agent remains in its last location
	if (timestep >= paths[agent_id]->size())
		return paths[agent_id]->at(paths[agent_id]->size() - 1).location;
	// otherwise, return its location for that timestep
	return paths[agent_id]->at(timestep).location;
}


bool GICBSSearch::findPathForSingleAgent(GICBSNode*  node, int ag, double lowerbound)
{
	// extract all constraints on agent ag
	//GICBSNode* curr = node;
	bool foundSol = true;
	vector<vector<bool>> consistent(num_of_agents, vector<bool>(num_of_agents, true));
	vector<vector<PathEntry>> new_paths(num_of_agents, vector<PathEntry>());

	vector<bool> visited(num_of_agents, true);
	for (int i = 0; i < num_of_agents; i++) {
		if (node->trans_priorities[ag][i]) {
			visited[i] = false;
		}
	}
	stack<pair<bool, int> > dfs;
	list<int> topSort;
	dfs.push(make_pair(false, ag));
	while (!dfs.empty()) {
		pair<bool, int> parent = dfs.top();
		dfs.pop();
		if (parent.first) {
			topSort.push_front(parent.second);
			continue;
		}
		visited[parent.second] = true;
		dfs.push(make_pair(true, parent.second));
		for (int i = 0; i < num_of_agents; i++) {
			if (node->priorities[parent.second][i] && !visited[i]) {
				dfs.push(make_pair(false, i));
			}
		}
	}
	for (auto iter = topSort.begin(); iter != topSort.end(); iter++) {
		int curr_agent = *iter;
		bool isColliding = false;
		for (int a2 = 0; a2 < num_of_agents; a2++) {
			if (!consistent[a2][curr_agent]) {
				size_t min_path_length = paths[curr_agent]->size() < paths[a2]->size() ? paths[curr_agent]->size() : paths[a2]->size();
				for (size_t timestep = 0; timestep < min_path_length; timestep++)
				{
					int loc1 = paths[curr_agent]->at(timestep).location;
					int loc2 = paths[a2]->at(timestep).location;
					if (loc1 == loc2)
					{
						isColliding = true;
						break;
					}
					else if (timestep < min_path_length - 1
						&& loc1 == paths[a2]->at(timestep + 1).location
						&& loc2 == paths[curr_agent]->at(timestep + 1).location)
					{
						isColliding = true;
						break;
					}
				}
				if (!isColliding && paths[curr_agent]->size() != paths[a2]->size())
				{
					int a1_ = paths[curr_agent]->size() < paths[a2]->size() ? ag : a2 ;
					int a2_ = paths[curr_agent]->size() < paths[a2]->size() ? a2 : ag;
					int loc1 = paths[a1_]->back().location;
					for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
					{
						int loc2 = paths[a2_]->at(timestep).location;
						if (loc1 == loc2)
						{
							break;
						}
					}
				}
			}
			if (isColliding) {
				break;
			}
		}
		if (!isColliding && curr_agent != ag) {
			continue;
		}
		size_t max_plan_len = node->makespan + 1; //getPathsMaxLength();
												  //bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
												  //bool* res_table_low_prio = new bool[map_size * max_plan_len]();  // initialized to false
												  //updateReservationTable(res_table, res_table_low_prio, curr_agent, *node);
												  // find a path w.r.t cons_vec (and prioretize by res_table).
												  //pair<int, vector<PathEntry>> newPath;
												  //vector<PathEntry> newPath;
												  //newPath.first = curr_agent;
												  //foundSol = search_engines[curr_agent]->findPath(newPath.second, focal_w, node->trans_priorities, paths, max_plan_len, lowerbound);
    num_single_pathfinding += 1;

    auto t1 = std::clock();
    //findConflicts(*node);
		foundSol = search_engines[curr_agent]->findPath(new_paths[curr_agent], focal_w, node->trans_priorities, paths, max_plan_len, lowerbound);
    // cout << "ll solver: " << std::clock() - t1 << endl;
		LL_num_expanded += search_engines[curr_agent]->num_expanded;
		LL_num_generated += search_engines[curr_agent]->num_generated;
		//delete (cons_vec);
		//delete[] res_table;
		if (foundSol) {
			//node->new_paths.push_back(newPath);
			//new_paths[curr_agent] = newPath;
			node->g_val = node->g_val - paths[curr_agent]->size() + new_paths[curr_agent].size();
			//paths[curr_agent] = &node->new_paths.back().second;
			paths[curr_agent] = &new_paths[curr_agent]; // might be used by the next findPath() call
														//node->paths[ag] = search_engines[ag]->getPath();
			node->makespan = max(node->makespan, new_paths[curr_agent].size() - 1);
			for (int i = 0; i < num_of_agents; i++) {
				if (node->trans_priorities[curr_agent][i]) {
					consistent[curr_agent][i] = false;
				}
				if (node->trans_priorities[i][curr_agent] && !consistent[i][curr_agent]) {
					consistent[i][curr_agent] = true;
				}
			}
		}
		else {
			return false;
		}
	}
	if (foundSol) {
		for (int i = 0; i < num_of_agents; i++) {
			if (!new_paths[i].empty()) {
				node->new_paths.push_back(make_pair(i, new_paths[i]));
				paths[i] = &node->new_paths.back().second; // make sure paths[i] gets the correct pointer
			}
		}
	}
  return true;
}

bool GICBSSearch::generateChild(GICBSNode*  node, GICBSNode* curr)
{
	node->parent = curr;
	node->g_val = curr->g_val;
	node->makespan = curr->makespan;
	//node->f_val = curr->f_val - curr->h_val;
	node->depth = curr->depth + 1;
	//node->paths = curr->paths;
	//node->paths.resize(num_of_agents);
	//node->paths.assign(curr->paths.begin(), curr->paths.end());
	//node->single.resize(num_of_agents, NULL);
	std::clock_t t1;

	t1 = std::clock();

	if (get<3>(node->constraint)) //positve constraint
	{
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == node->agent_id)
				continue;
			else if (get<1>(node->constraint) < 0 && // vertex constraint
				getAgentLocation(ag, get<2>(node->constraint)) == get<0>(node->constraint))
			{
				if (!findPathForSingleAgent(node, ag))
					return false;
				//else
				//	node->paths[ag] = &(get<1>(node->paths_updated.back()));
			}
			else if (get<1>(node->constraint) >= 0 && //edge constraint
				getAgentLocation(ag, get<2>(node->constraint) - 1) == get<1>(node->constraint) &&
				getAgentLocation(ag, get<2>(node->constraint)) == get<0>(node->constraint))
			{
				if (!findPathForSingleAgent(node, ag))
					return false;
				else
					paths[ag] = &(node->new_paths.back().second);
			}
		}
	}
	else // negative constraint
	{
		double lowerbound;
		if (get<2>(*curr->conflict) < 0) // rectangle conflict
			lowerbound = (int)paths[node->agent_id]->size() - 1;
		else if(get<4>(*curr->conflict) >= (int)paths[node->agent_id]->size()) //conflict happens after agent reaches its goal
			lowerbound = get<4>(*curr->conflict) + 1;
		else if(!paths[node->agent_id]->at(get<4>(*curr->conflict)).single) // not cardinal
			lowerbound = (int)paths[node->agent_id]->size() - 1;
		else if(get<2>(*curr->conflict) >= 0 && get<3>(*curr->conflict) < 0) // Cardinal vertex
			lowerbound = (int)paths[node->agent_id]->size();
		else if (paths[node->agent_id]->at(get<4>(*curr->conflict) - 1).single) // Cardinal edge
			lowerbound = (int)paths[node->agent_id]->size();
		else // Not cardinal edge
			lowerbound = (int)paths[node->agent_id]->size() - 1;
		
		if (!findPathForSingleAgent(node, node->agent_id))
			return false;
		//else
		//	paths[node->agent_id] = &(get<1>(node->paths_updated.back()));
	}
	
	runtime_lowlevel += std::clock() - t1;

	node->f_val = node->g_val;

	t1 = std::clock();
	//findConflicts(*node);
	runtime_conflictdetection += std::clock() - t1;
	node->num_of_collisions = computeCollidingTeams();

	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	allNodes_table.push_back(node);

	// Copy single vector from parent
	/*node->single.resize(num_of_agents);
	for (int i = 0; i < curr->single.size(); i++)
	{
		if (!curr->single[i])
			continue;
		else
		{
			bool updated = false;
			for (list<int>::iterator it = node->agents_updated.begin(); it != node->agents_updated.end(); it++)
			{
				if (*it == i)
				{
					updated = true;
					break;
				}
			}
			if (!updated)
			{
				node->single[i] = curr->single[i];
			}
		}
	}*/

	return true;
}


void GICBSSearch::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		std::cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			paths[i]->size() - 1 << "): ";
		for (int t = 0; t < paths[i]->size(); t++)
			std::cout << "(" << paths[i]->at(t).location / num_col << "," << paths[i]->at(t).location % num_col << ")->";
		std::cout << std::endl;
	}
}


void GICBSSearch::printConstraints(const GICBSNode* n) const
{
	const GICBSNode* curr = n;
	while (curr != dummy_start)
	{
		std::cout << "<" << curr->agent_id
						<< ", " << get<0>(curr->constraint)
						<< ", " << get<1>(curr->constraint)
						<< ", " << get<2>(curr->constraint);
		if(get<3>(curr->constraint))
			std::cout << ", positive>" << std::endl;
		else
			std::cout << ", negative>" << std::endl;
		curr = curr->parent;
	}
}

// computes g_val based on current paths
inline int GICBSSearch::compute_g_val() {
	int retVal = 0;
	for (int i = 0; i < num_of_agents; i++)
		retVal += paths[i]->size() - 1;
	return retVal;
}

bool GICBSSearch::runGICBSSearch() 
{
	node_stat.clear();
  cout << "       GICBS: ";
	if (solution_cost == -2) {
		runtime = pre_runtime;
		return false;
	}
	// set timer
	std::clock_t start;
	start = std::clock();
	std::clock_t t1;
	runtime_computeh = 0;
	runtime_lowlevel = 0;
	runtime_listoperation = 0;
	runtime_conflictdetection = 0;
	runtime_updatepaths = 0;
	runtime_updatecons = 0;
	// start is already in the open_list
	//upper_bound = DBL_MAX;
	while (!open_list.empty() && !solution_found) 
	{
		// break after 5 min
		runtime = (std::clock() - start) + pre_runtime; // / (double) CLOCKS_PER_SEC;
		if (runtime > TIME_LIMIT || HL_num_expanded > 1000000)
		{  // timeout after 1 minutes
			cout << "TIMEOUT  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime / CLOCKS_PER_SEC << " ; " << endl;
			
			std::cout << "	Runtime Sumarry: lowlevel = " << runtime_lowlevel << " ; listoperation = " << runtime_listoperation << 
						" ; conflictdetection = " << runtime_conflictdetection << " ; computeh = " << runtime_computeh<<
						" ; updatepaths = " << runtime_updatepaths << " ; collectcons = " << runtime_updatecons << std::endl;

			double count = 0, value = 0, maxDepth = 0;
			GICBSNode* curr = NULL;
			bool open_empty = false;
			if ((open_list.empty()))
				open_empty = true;
			while (!open_list.empty())
			{
				curr = open_list.top();
				open_list.pop();
				if (curr->depth > maxDepth)
					maxDepth = curr->depth;
				if (curr->f_val > value + 0.001)
				{
					cout << "				#(f=" << value << ") = " << count << endl;
					count = 1;
					value = curr->f_val;
				}
				else
					count++;
			}
			if (!open_empty) {
				std::cout << "Depth of last node: " << curr->depth << " ; MaxDepth = " << maxDepth << std::endl;
			}
			else {
				std::cout << "Open List Empty!!!" << endl;
			}
			solution_found = false;
			break;
		}
		t1 = std::clock();
		GICBSNode* curr = open_list.top();

		open_list.pop();
		runtime_listoperation += std::clock() - t1;
		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		t1 = std::clock();
		updatePaths(curr);
		runtime_updatepaths += std::clock() - t1;
#ifdef DEBUG
		//printPaths();
#endif
		t1 = std::clock();
		findConflicts(*curr);
		runtime_conflictdetection += std::clock() - t1;


		if (curr->conflict == NULL) //Fail to find a conflict => no conflicts
		{  // found a solution (and finish the while look)
			runtime = (std::clock() - start) + pre_runtime; // / (double) CLOCKS_PER_SEC;
			solution_found = true;
			solution_cost = curr->g_val;
			cout << solution_cost << " ; " << solution_cost - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime / CLOCKS_PER_SEC << " ; ";
			cout << endl;
//#ifdef DEBUG
//			int conflictNum = computeNumOfCollidingAgents();
//			if(conflictNum > 0)
//				std::cout << "ERROR!" << std::endl;
//			std::cout << std::endl << "****** Solution: " << std::endl;
//			printPaths(*curr);
//#endif		
			break;
		}


		 //Expand the node
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;
		
#ifdef DEBUG
		std::cout << std::endl << "****** Expanded #" << curr->time_generated << " with f= " << curr->g_val <<
			"+" << curr->h_val << " (";
		for (int i = 0; i < num_of_agents; i++)
			std::cout << paths[i]->size() - 1 << ", ";
		std::cout << ")" << std::endl;
		std::cout << "Choose conflict <";
		std::cout << "A1=" << get<0>(*curr->conflict) << ",A2=" << get<1>(*curr->conflict)
			<< ",loc1=(" << get<2>(*curr->conflict) / num_col << "," << get<2>(*curr->conflict) % num_col
			<< "),loc2=(" << get<3>(*curr->conflict) / num_col << "," << get<3>(*curr->conflict) % num_col
			<< "),t=" << get<4>(*curr->conflict) << ">" << std::endl;
#endif

		GICBSNode* n1 = new GICBSNode();
		GICBSNode* n2 = new GICBSNode();
		
		n1->agent_id = get<0>(*curr->conflict);
		n2->agent_id = get<1>(*curr->conflict);
		if (get<3>(*curr->conflict) < 0) // vertex conflict
		{
			n1->constraint = make_tuple(get<2>(*curr->conflict), -1, get<4>(*curr->conflict), false);
			n2->constraint = make_tuple(get<2>(*curr->conflict), -1, get<4>(*curr->conflict), false);
		}
		else // edge conflict
		{
			n1->constraint = make_tuple(get<2>(*curr->conflict), get<3>(*curr->conflict), get<4>(*curr->conflict), false);
			n2->constraint = make_tuple(get<3>(*curr->conflict), get<2>(*curr->conflict), get<4>(*curr->conflict), false);
		}

		bool Sol1 = false, Sol2 = false;
		vector<vector<PathEntry>*> copy(paths);
		//int lowerbound1 = max(get<4>(curr->conflict) + 1, (int)paths[n1->agent_id]->size() - 1);
		//int lowerbound2 = max(get<4>(curr->conflict) + 1, (int)paths[n2->agent_id]->size() - 1);
		//lowerbound2 = ; // The cost of path should be at least the confliting time + 1
		//if (!curr->cardinalConf.empty() || !curr->rectCardinalConf.empty()) // Resolve a cardinal conflict
		//{
		//	
		//}
		//else if (!curr->semiConf.empty() || !curr->rectSemiConf.empty()) // Resolve a semi
		//{
		//	if (Sol1 && Sol2 && abs(n1->g_val + n2->g_val - 2 * curr->g_val) < 0.001)
		//	{
		//		std::cout << "***********ERROR**************" << std::endl;
		//		system("pause");
		//	}
		//}

		bool gen_n1 = true, gen_n2 = true;

		if (curr->trans_priorities[n1->agent_id][n2->agent_id]) { // a1->a2 do not generate n1
			gen_n1 = false;
		}
		if (curr->trans_priorities[n2->agent_id][n1->agent_id]) { // a2->a1 do not generate n2
			gen_n2 = false;
		}

		if (gen_n1) {
			n1->priorities = vector<vector<bool>>(curr->priorities);
			n1->trans_priorities = vector<vector<bool>>(curr->trans_priorities);
			n1->priorities[n2->agent_id][n1->agent_id] = true; // a2->a1
			n1->trans_priorities[n2->agent_id][n1->agent_id] = true;
			for (int i = 0; i < num_of_agents; i++) { // transitivity
				if (n1->trans_priorities[i][n2->agent_id] && !n1->trans_priorities[i][n1->agent_id]) {
					for (int j = 0; j < num_of_agents; j++) {
						if (n1->trans_priorities[n1->agent_id][j]) {
							n1->trans_priorities[i][j] = true;
						}
					}
				}
			}

			Sol1 = generateChild(n1, curr);
			if (!gen_n2) {
				n1->depth--;
			}
		}
		paths = copy;
		//updatePaths(curr);
		if (gen_n2) {
			n2->priorities = vector<vector<bool>>(curr->priorities);
			n2->trans_priorities = vector<vector<bool>>(curr->trans_priorities);
			n2->priorities[n1->agent_id][n2->agent_id] = true; // a1->a2
			n2->trans_priorities[n1->agent_id][n2->agent_id] = true;
			for (int i = 0; i < num_of_agents; i++) { // transitivity
				if (n2->trans_priorities[i][n1->agent_id] && !n2->trans_priorities[i][n2->agent_id]) {
					for (int j = 0; j < num_of_agents; j++) {
						if (n2->trans_priorities[n2->agent_id][j]) {
							n2->trans_priorities[i][j] = true;
						}
					}
				}
			}

			Sol2 = generateChild(n2, curr);
			if (!gen_n2) {
				n2->depth--;
			}
		}




    /*if (!Sol1 ){
      std::cout << "Not feasible child for " << n1->agent_id << ", " << n2->agent_id << endl;
    }
    if (!Sol2 ){
      std::cout << "Not feasible child for " << n2->agent_id << ", " << n1->agent_id << endl;
    }

    if (!Sol1 && !Sol2){
      std::cout << "Not able to resolve between" << n1->agent_id << ", " << n2->agent_id << endl;
    }*/


#ifdef DEBUG
		if(Sol1)
		{
			std::cout	<< "Generate #" << n1->time_generated 
							<< " with cost " << n1->g_val 
							<< " and " << n1->num_of_collisions << " conflicts " <<  std::endl;
		}
		else
		{
			std::cout << "No feasible solution for left child! " << std::endl;
		}
		if (Sol2)
		{
			std::cout	<< "Generate #" << n2->time_generated 
							<< " with cost " << n2->g_val 
							<< " and " << n2->num_of_collisions << " conflicts " << std::endl;
		}
		else
		{
			std::cout << "No feasible solution for right child! " << std::endl;
		}

		if (!curr->cardinalConf.empty() || !curr->rectCardinalConf.empty()) // Resolve a cardinal conflict
		{
			if (Sol1 && abs(n1->g_val - curr->g_val) < 0.001)
			{
				std::cout << "***********ERROR**************" << std::endl;
				system("pause");
			}
			if (Sol2 && abs(n2->g_val - curr->g_val) < 0.001)
			{
				std::cout << "***********ERROR**************" << std::endl;
				system("pause");
			}
		}
		else if (!curr->semiConf.empty() || !curr->rectSemiConf.empty()) // Resolve a semi
		{
			if (Sol1 && Sol2 && abs(n1->g_val + n2->g_val - 2 * curr->g_val) < 0.001)
			{
				std::cout << "***********ERROR**************" << std::endl;
				system("pause");
			}
		}
#endif
		if(!Sol1)
		{
			delete (n1);
			n1 = NULL;
		}
		if(!Sol2)
		{
			delete (n2);
			n2 = NULL;
		}
		//if(curr != dummy_start) // We save dummy_start for statistics analysis later

		curr->clear();
		t1 = std::clock();
		if (open_list.size() == 0) {
			solution_found = false;
			break;
		}
		#ifdef DEBUG
        cout << " ; (after) " << focal_list_threshold << endl << endl;
		#endif
		runtime_listoperation += std::clock() - t1;

	}  // end of while loop


	//    printPaths();	
	if (open_list.empty() && solution_cost < 0 && !(runtime > TIME_LIMIT))
	{
		solution_cost = -2;
		cout << "No solutions  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
			HL_num_expanded << " ; " << HL_num_generated << " ; " <<
			LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime / CLOCKS_PER_SEC << " ; " <<
			"|Open|=" << open_list.size() << endl;
		solution_found = false;
	}
	return solution_found;
}


GICBSSearch::GICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, const EgraphReader& egr,
        constraint_strategy c, bool fixed_prior): focal_w(f_w), fixed_prior(fixed_prior)
{
    clock_t start_t = std::clock();

	cons_strategy = c;
	//focal_w = f_w;
	HL_num_expanded = 0;
	HL_num_generated = 0;
	LL_num_expanded = 0;
	LL_num_generated = 0;
	this->num_col = ml.cols;
	this->al = al;
	num_of_agents = al.num_of_agents;
	map_size = ml.rows*ml.cols;
	solution_found = false;
	solution_cost = -1;
	//ll_min_f_vals = vector <double>(num_of_agents);
	//paths_costs = vector <double>(num_of_agents);
	//ll_min_f_vals_found_initially = vector <double>(num_of_agents);
	//paths_costs_found_initially = vector <double>(num_of_agents);
	search_engines = vector < SingleAgentICBS* >(num_of_agents);
	for (int i = 0; i < num_of_agents; i++) {
		int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
		int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
		ComputeHeuristic ch(init_loc, goal_loc, ml.get_map(), ml.rows, ml.cols, ml.moves_offset, ml.actions_offset, 1.0, &egr);
		search_engines[i] = new SingleAgentICBS(i, init_loc, goal_loc, ml.get_map(), ml.rows*ml.cols,
			ml.moves_offset, ml.cols);
		ch.getHVals(search_engines[i]->my_heuristic);
	}

	// initialize allNodes_table (hash table)
	//empty_node = new GICBSNode();
	
	//empty_node->time_generated = -2; empty_node->agent_id = -2;
	//deleted_node = new GICBSNode();
	//deleted_node->time_generated = -3; deleted_node->agent_id = -3;
	//allNodes_table.set_empty_key(empty_node);
	//allNodes_table.set_deleted_key(deleted_node);


	dummy_start = new GICBSNode();
	dummy_start->agent_id = -1;

	// initialize paths_found_initially
    paths.resize(num_of_agents, NULL);
    paths_found_initially.resize(num_of_agents);

	if(fixed_prior) {
	    int iteration = 0;
        while(true) {
            runtime = (std::clock() - start_t);
            if (runtime > TIME_LIMIT) {
                cout << "NO SOLUTION EXISTS AFTER " << iteration << " ITERATIONS";
                solution_cost = -2;
                break;
            }
            iteration++;
            bool found = true;
            dummy_start->makespan = 0;
            paths.clear();
            paths_found_initially.clear();
            paths.resize(num_of_agents, NULL);
            paths_found_initially.resize(num_of_agents);
            dummy_start->priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false));
            dummy_start->trans_priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false));
            vector<int> ordering(num_of_agents);
            for (int i = 0; i < num_of_agents; i++) {
                ordering[i] = i;
            }
            std::random_shuffle(ordering.begin(), ordering.end());
            for (int i = 0; i < num_of_agents; i++) {
                int a = ordering[i];
                if (!search_engines[a]->findPath(paths_found_initially[a], f_w, dummy_start->trans_priorities, paths,
                                                 dummy_start->makespan + 1, 0)) {
                    iteration++;
                    found = false;
                    break;
                }
                for (int j = i + 1; j < num_of_agents; j++) {
                    int b = ordering[j];
                    dummy_start->trans_priorities[a][b] = true;
                }
                paths[a] = &paths_found_initially[a];
                dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[a].size() - 1);
                LL_num_expanded += search_engines[a]->num_expanded;
                LL_num_generated += search_engines[a]->num_generated;
            }
            if (found)
            {
                cout << iteration << " iterations" << endl;
                break;
            }
        }
	}
	else
    {
        dummy_start->priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false));
        dummy_start->trans_priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false));
        for (int i = 0; i < num_of_agents; i++) {
            //    cout << "Computing initial path for agent " << i << endl; fflush(stdout);
            //bool* res_table = new bool[map_size * (dummy_start->makespan + 1)]();  // initialized to false
            //bool* res_table_low_prio = new bool[map_size * (dummy_start->makespan + 1)]();  // initialized to false
            //updateReservationTable(res_table, res_table_low_prio, i, *dummy_start);
            //cout << "*** CALCULATING INIT PATH FOR AGENT " << i << ". Reservation Table[MAP_SIZE x MAX_PLAN_LEN]: " << endl;
            //printResTable(res_table, max_plan_len);
            if (!search_engines[i]->findPath(paths_found_initially[i], f_w, dummy_start->trans_priorities, paths,
                                             dummy_start->makespan + 1, 0)) {
                cout << "NO SOLUTION EXISTS";
                solution_cost = -2;
                break;
            }
            //dummy_start->paths[i] = search_engines[i]->getPath();
            paths[i] = &paths_found_initially[i];
            dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
            //search_engines[i]->path.reset();
            //ll_min_f_vals_found_initially[i] = search_engines[i]->min_f_val;
            //paths_costs_found_initially[i] = search_engines[i]->path_cost;
            LL_num_expanded += search_engines[i]->num_expanded;
            LL_num_generated += search_engines[i]->num_generated;
            //delete[] res_table;
            //    cout << endl;
        }
    }




	
	//ll_min_f_vals = ll_min_f_vals_found_initially;
	//paths_costs = paths_costs_found_initially;

	// generate dummy start and update data structures
  if (solution_cost >0){
    updatePaths(dummy_start);
    findConflicts(*dummy_start);
    assert(dummy_start->conflict==nullptr);
  }


	if (solution_cost != -2) {

		dummy_start->g_val = 0;
		for (int i = 0; i < num_of_agents; i++)
			dummy_start->g_val += paths[i]->size() - 1;
		dummy_start->h_val = 0;
		dummy_start->f_val = dummy_start->g_val;
		//dummy_start->ll_min_f_val = 0;
		dummy_start->depth = 0;

		dummy_start->open_handle = open_list.push(dummy_start);
		//dummy_start->focal_handle = focal_list.push(dummy_start);
		//dummy_start->single.resize(num_of_agents);
		//dummy_start->constraints.resize(num_of_agents);
		HL_num_generated++;
		dummy_start->time_generated = HL_num_generated;
		allNodes_table.push_back(dummy_start);
		findConflicts(*dummy_start);
		//initial_g_val = dummy_start->g_val;
		min_f_val = dummy_start->f_val;
		focal_list_threshold = min_f_val * focal_w;

		//  cout << "Paths in START (high-level) node:" << endl;
		//  printPaths();
		// cout << "SUM-MIN-F-VALS: " << dummy_start->sum_min_f_vals << endl;
	}

	pre_runtime = std::clock() - start_t;
}

inline void GICBSSearch::releaseClosedListNodes() 
{
	for (list<GICBSNode*>::iterator it = allNodes_table.begin(); it != allNodes_table.end(); it++)
		delete *it;
}

inline void GICBSSearch::releaseOpenListNodes()
{
	while(!open_list.empty())
	{
		GICBSNode* curr = open_list.top();
		open_list.pop();
		delete curr;
	}
}

GICBSSearch::~GICBSSearch()
{
	for (size_t i = 0; i < search_engines.size(); i++)
		delete (search_engines[i]);
	//for (size_t i = 0; i < paths_found_initially.size(); i++)
	//	delete (paths_found_initially[i]);
	//  for (size_t i=0; i<paths.size(); i++)
	//    delete (paths[i]);
	releaseClosedListNodes();
	// releaseOpenListNodes();
	//delete (empty_node);
	//delete (deleted_node);
}
