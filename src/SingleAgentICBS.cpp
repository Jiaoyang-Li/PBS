#include "SingleAgentICBS.h"

inline int offset2action(int offset, const int* move_offsets){
  for (int i = 0; i < 5; i ++){
    if (move_offsets[i] == offset){
      return i;
    }
  }
}

inline int action2offset(int action, const int* move_offsets){
  return move_offsets[action];
}


void SingleAgentICBS::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
	//path = std::shared_ptr<vector<PathEntry>>(new vector<PathEntry>(goal->timestep + 1));
	path.resize(goal->g_val + 1);
	const LLNode* curr = goal;
	// cout << "   UPDATING Path for one agent to: ";
	num_of_conf = goal->num_internal_conf;
	for(int t = goal->g_val; t >= 0; t--)
	{
		path[t].location = curr->loc;
		curr = curr->parent;
	}
}


// iterate over the constraints ( cons[t] is a list of all constraints for timestep t) and return the latest
// timestep which has a constraint involving the goal location
int SingleAgentICBS::extractLastGoalTimestep(int goal_location, const vector < vector< bool > >& priorities, const vector<vector<PathEntry>*>& current_paths) {
	int retVal = -1;
	for (int ag = 0; ag < current_paths.size(); ag++) {
		if (ag != agent_id && current_paths[ag] != NULL && priorities[ag][agent_id]) {
			int last_time = current_paths[ag]->size() - 1;
			if (retVal < last_time) {
				retVal = last_time;
			}
		}
	}
	return retVal;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// input: curr_id (location at time next_timestep-1) ; next_id (location at time next_timestep); next_timestep
//        cons[timestep] is a list of <loc1,loc2> of (vertex/edge) constraints for that timestep.

inline bool SingleAgentICBS::isConstrained(int curr_id, int next_id, int next_timestep, const vector < vector< int > >* cons_table){
  if (cons_table->size() == 0){
    return false;
  }
  if (next_timestep >= cons_table->size()){
    return cons_table->at(cons_table->size() - 1)[next_id] > 0;
  }else{
    
    if (cons_table->at(next_timestep)[next_id] > 0){
      // node conflict
      return true;
    }
    if (((cons_table->at(next_timestep - 1)[next_id]) & (1 << offset2action(curr_id - next_id, moves_offset))) > 0){
      // edge conflict
      return true;
    }
  }
  return false;

}


inline bool SingleAgentICBS::isConstrained(int curr_id, int next_id, int next_timestep, const vector < vector< bool > >& priorities, const vector<vector<PathEntry>*>& current_paths)  const {
  for (int ag = 0; ag < current_paths.size(); ag++) {
		if (ag != agent_id && current_paths[ag] != NULL && priorities[ag][agent_id]) { // check only high prior agents
			if (next_timestep >= current_paths[ag]->size()) {
				if (current_paths[ag]->back().location == next_id) { //vertex constraint
					return true;
				}
			}
			else {
				if (current_paths[ag]->at(next_timestep).location == next_id //vertex constraint
					|| (current_paths[ag]->at(next_timestep).location == curr_id && current_paths[ag]->at(next_timestep - 1).location == next_id)) { // edge constraint
					return true;
				}
			}
		}
	}
	return false;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector < vector< int > >* SingleAgentICBS::collectConstraints(const vector < vector< bool > >& priorities, const vector<vector<PathEntry>*>& current_paths)
{
  int max_len = 0;
  for (int ag = 0; ag < current_paths.size(); ag++) {
    if (ag != agent_id && current_paths[ag] != NULL && priorities[ag][agent_id]) { // check only high prior agents
      max_len = std::max(max_len, (int)current_paths[ag]->size());
    }
  }

  vector <vector<int>>* cons_table = new vector<vector<int>>(max_len, vector<int>(map_size, 0));

  for (int ag = 0; ag < current_paths.size(); ag++) {
    if (ag != agent_id && current_paths[ag] != NULL && priorities[ag][agent_id]) { // check only high prior agents
      for (int i = 0; i < max_len; i++){
        if (i + 1 < current_paths[ag]->size()){
          int action = offset2action(current_paths[ag]->at(i + 1).location - current_paths[ag]->at(i).location, moves_offset);
          (*cons_table)[i][current_paths[ag]->at(i).location] = (*cons_table)[i][current_paths[ag]->at(i).location] | (1 << action);
        }else{
          int l = current_paths[ag]->size() - 1;
          (*cons_table)[i][current_paths[ag]->at(l).location]= (*cons_table)[i][current_paths[ag]->at(l).location] | (1 <<MapLoader::valid_moves_t::WAIT_MOVE);
        }
      }
    }
  }
  return cons_table;
}

const int N_CONF_TABLE_OFFSET = 1000;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector < vector< int > >* SingleAgentICBS::countConflict(const vector < vector< bool > >& priorities, const vector<vector<PathEntry>*>& current_paths){
  int max_len = 0;
  for (int ag = 0; ag < current_paths.size(); ag++) {
    // if (ag != agent_id && current_paths[ag] != NULL && !priorities[ag][agent_id] && !colliding_agents[ag]){
    if (ag != agent_id && current_paths[ag] != NULL && !priorities[ag][agent_id]){
      max_len = std::max(max_len, (int)current_paths[ag]->size());
    }
  }
  vector <vector<int>>* cons_table = new vector<vector<int>>(max_len, vector<int>(map_size, 0));

  for (int ag = 0; ag < current_paths.size(); ag++) {
    if (ag != agent_id && current_paths[ag] != NULL && !priorities[ag][agent_id]){
      // if (ag != agent_id && current_paths[ag] != NULL && !priorities[ag][agent_id] && !colliding_agents[ag]){
      for (int i = 0; i < max_len; i++){
        int l = i;
        if (i >=  current_paths[ag]->size()){
          l = current_paths[ag]->size() - 1;
        }

        if (priorities[agent_id][ag]){
          // Low Prior
          (*cons_table)[i][current_paths[ag]->at(l).location] += 1;
        }else{
          (*cons_table)[i][current_paths[ag]->at(l).location] += N_CONF_TABLE_OFFSET;
        }
      }
    }
  }
  return cons_table;
}

pair<int, int> SingleAgentICBS::numOfConflictsForStep(int curr_id, int next_id, int next_timestep, const vector < vector< int > >* cons_table) {
  if (cons_table->size() == 0){
    return {0, 0};
  }
  int t = next_timestep;
  if (next_timestep >= cons_table->size()){
    t = cons_table->size() - 1;
  }
  int cnt = cons_table->at(t)[next_id];

  return {cnt / N_CONF_TABLE_OFFSET, cnt % N_CONF_TABLE_OFFSET};
}

pair<int, int> SingleAgentICBS::numOfConflictsForStep(int curr_id, int next_id, int next_timestep, const vector < vector< bool > >& priorities, const vector<vector<PathEntry>*>& current_paths, vector< bool >& colliding_agents) {
	int retVal = 0;
	int retVal_lowPrior = 0;
	for (int ag = 0; ag < current_paths.size(); ag++) {
		if (ag != agent_id && current_paths[ag] != NULL && !priorities[ag][agent_id] && !colliding_agents[ag]) { // higher prior ag is hard constraints (not soft)
			if (next_timestep >= current_paths[ag]->size()) {
				// check vertex constraints (being at an agent's goal when he stays there because he is done planning)
				if (current_paths[ag]->back().location == next_id) {
					colliding_agents[ag] = true;
					if (priorities[agent_id][ag]) { // low prior ag
						retVal_lowPrior++;
					}
					else {
						retVal++;
					}
					continue;
				}
				// Note -- there cannot be edge conflicts when other agents are done moving
			}
			else {
				// check vertex constraints (being in next_id at next_timestep is disallowed)
				if (current_paths[ag]->at(next_timestep).location == next_id //vertex constraint
					|| (current_paths[ag]->at(next_timestep).location == curr_id && current_paths[ag]->at(next_timestep - 1).location == next_id)) { // edge constraint
					colliding_agents[ag] = true;
					if (priorities[agent_id][ag]) { // low prior ag
						retVal_lowPrior++;
					}
					else {
						retVal++;
					}
					continue;
				}
			}
		}
	}
	//  cout << "#CONF=" << retVal << " ; For: curr_id=" << curr_id << " , next_id=" << next_id << " , next_timestep=" << next_timestep
	//       << " , max_plan_len=" << max_plan_len << endl;
	return pair<int, int>(retVal, retVal_lowPrior);
}

// $$$ -- is there a more efficient way to do that?
void SingleAgentICBS::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight) {
	
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists
bool SingleAgentICBS::findPath(vector<PathEntry> &path, double f_weight, const vector < vector< bool > >& priorities, const vector<vector<PathEntry>*>& current_paths, size_t max_plan_len, double lowerbound) {
	// clear data structures if they had been used before
	// (note -- nodes are deleted before findPath returns)

  // vector<bool> next_colliding_agents = vector<bool>(curr->colliding_agents);
  std::unique_ptr<vector < vector< int > >> cons_table = std::unique_ptr<vector < vector< int > >>(collectConstraints(priorities, current_paths));
  std::unique_ptr<vector < vector< int > >> conf_cnt_table = std::unique_ptr<vector < vector< int > >>(countConflict(priorities, current_paths));




	num_expanded = 0;
	num_generated = 0;

	hashtable_t::iterator it;  // will be used for find()

	 // generate start and add it to the OPEN list
	LLNode* start = new LLNode(start_location, 0, my_heuristic[start_location], NULL, 0, 0, false);
	start->colliding_agents = vector<bool>(current_paths.size(), false);
	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
	allNodes_table[start] = start;
	min_f_val = start->getFVal();
	lower_bound = max(lowerbound, f_weight * min_f_val);

	int lastGoalConsTime = extractLastGoalTimestep(goal_location, priorities, current_paths);

#ifdef  _DEBUG
	if (agent_id == 0 || agent_id == 7) {
		cout << endl;
		for (int ag = 0; ag < current_paths.size(); ag++) {
			if (priorities[ag][agent_id]) {
				cout << ag << "->" << agent_id << endl;
				if (ag == 0 || ag == 7) {
					cout << "!!!";
				}
			}
		}
	}
#endif //  _DEBUG

	while (!focal_list.empty()) {
		//    cout << "|F|=" << focal_list.size() << " ; |O|=" << open_list.size() << endl;
		LLNode* curr = focal_list.top(); focal_list.pop();
		//    cout << "Current FOCAL bound is " << lower_bound << endl;
		//    cout << "POPPED FOCAL's HEAD: (" << curr << ") " << (*curr) << endl;
		open_list.erase(curr->open_handle);
		//    cout << "DELETED" << endl; fflush(stdout);
		curr->in_openlist = false;
		num_expanded++;

		// check if the popped node is a goal
		if (curr->loc == goal_location) {
			bool is_goal = true;
			for (int t = curr->g_val; t < lastGoalConsTime; t++) {
				if (isConstrained(goal_location, goal_location, t + 1, cons_table.get())) {
					is_goal = false;
					break;
				}
			}
			if (is_goal) {
				updatePath(curr, path);
				releaseClosedListNodes(&allNodes_table);
				open_list.clear();
				focal_list.clear();
				allNodes_table.clear();
				return true;
			}
		}

		int next_id;
		for (int i = 0; i < 5; i++)
		{
			if (curr->timestep == lastGoalConsTime && i == 4) { // no reason to wait
				continue;
			}

			next_id = curr->loc + moves_offset[i];

			int next_timestep;
			if (curr->timestep == lastGoalConsTime) {
				next_timestep = curr->timestep;
			}
			else {
				next_timestep = curr->timestep + 1;
			}
			int next_g_val = curr->g_val + 1;

			if (0 <= next_id && next_id < map_size && abs(next_id % moves_offset[MapLoader::valid_moves_t::SOUTH] - curr->loc % moves_offset[MapLoader::valid_moves_t::SOUTH]) < 2)
			{
				//bool free = true;
				//int num_row = map_size / num_col;
				//cout << "NUMBER of rows and cols: " <<num_row << " " << num_col << endl;;
				//int row = next_id / num_col;
				//int col = next_id % num_col;

        // bool is_cons_0 = isConstrained(curr->loc, next_id, next_g_val, priorities, current_paths);
        bool is_cons_1 = isConstrained(curr->loc, next_id, next_g_val, cons_table.get());

				int next_h_val = my_heuristic[next_id];
				if (!my_map[next_id] && !is_cons_1) {
					// compute cost to next_id via curr node

          

					// pair<int, int> next_internal_conflicts_pair = numOfConflictsForStep(curr->loc, next_id, next_g_val, priorities, current_paths, next_colliding_agents);
					pair<int, int> next_internal_conflicts_pair = numOfConflictsForStep(curr->loc, next_id, next_g_val, conf_cnt_table.get());
					// generate (maybe temporary) node
					int next_internal_conflicts = curr->num_internal_conf + next_internal_conflicts_pair.first;
					int next_internal_conflicts_lp = curr->num_internal_conf_lp + next_internal_conflicts_pair.second;
					LLNode* next = new LLNode(next_id, next_g_val, next_h_val,	curr, next_timestep, next_internal_conflicts, next_internal_conflicts_lp, false);

					// next->colliding_agents = next_colliding_agents;
					// cout << "   NEXT(" << next << ")=" << *next << endl;
					// try to retrieve it from the hash table
					it = allNodes_table.find(next);

					if (it == allNodes_table.end()) {
						//          cout << "   ADDING it as new." << endl;
						next->open_handle = open_list.push(next);
						next->in_openlist = true;
						num_generated++;
						if (next->getFVal() <= lower_bound)
							next->focal_handle = focal_list.push(next);
						allNodes_table[next] = next;
					}
					else {  // update existing node's if needed (only in the open_list)
						delete(next);  // not needed anymore -- we already generated it before
						LLNode* existing_next = (*it).second;
						//          cout << "Actually next exists. It's address is " << existing_next << endl;
						if (existing_next->in_openlist == true) {  // if its in the open list
							if (existing_next->getFVal() > next_g_val + next_h_val ||
								(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts) ||
								(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf == next_internal_conflicts && existing_next->num_internal_conf_lp > next_internal_conflicts_lp)) {
								// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
								//              cout << "   UPDATE its f-val in OPEN (decreased or less #conflicts)" << endl;
								//              cout << "   Node state before update: " << *existing_next;
								bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
								bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
								bool update_open = false;
								if ((next_g_val + next_h_val) <= lower_bound) {  // if the new f-val qualify to be in FOCAL
									if (existing_next->getFVal() > lower_bound)
										add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
									else
										update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
								}
								if (existing_next->getFVal() > next_g_val + next_h_val)
									update_open = true;
								// update existing node
								existing_next->g_val = next_g_val;
								existing_next->h_val = next_h_val;
								existing_next->parent = curr;
								existing_next->num_internal_conf = next_internal_conflicts;
								existing_next->num_internal_conf_lp = next_internal_conflicts_lp;
								//              cout << "   Node state after update: " << *existing_next;
								if (update_open) {
									open_list.increase(existing_next->open_handle);  // increase because f-val improved
																					 //                cout << "     Increased in OPEN" << endl;
								}
								if (add_to_focal) {
									existing_next->focal_handle = focal_list.push(existing_next);
									//                cout << "     Inserted to FOCAL" << endl;
								}
								if (update_in_focal) {
									focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
																					 //                cout << "     Updated in FOCAL" << endl;
								}
							}
							//            cout << "   Do NOT update in OPEN (f-val for this node increased or stayed the same and has more conflicts)" << endl;
						}
					}  // end update an existing node
				}  // end if case for grid not blocked
			}
		}  // end for loop that generates successors
		   // update FOCAL if min f-val increased
		if (open_list.size() == 0)  // in case OPEN is empty, no path found...
			break;
		LLNode* open_head = open_list.top();
		if (open_head->getFVal() > min_f_val) {
			double new_min_f_val = open_head->getFVal();
			double new_lower_bound = max(lowerbound, f_weight * new_min_f_val);
			/*
			cout << "LL FOCAL UPDATE! Old-f-min=" << min_f_val << " ; Old-LB=" << lower_bound << endl;
			cout << "OPEN: ";
			for (Node* n : open_list)
			cout << n << " , ";
			cout << endl;
			cout << "FOCAL: ";
			for (Node* n : focal_list)
			cout << n << " , ";
			cout << endl;
			*/
			//  cout << "Update Focal: (old_LB=" << lower_bound << " ; new_LB=" << new_lower_bound << endl;;
			for (LLNode* n : open_list) {
				//    cout << "   Considering " << n << " , " << *n << endl;
				if (n->getFVal() > lower_bound &&
					n->getFVal() <= new_lower_bound) {
					//      cout << "      Added (n->f-val=" << n->getFVal() << ")" << endl;
					n->focal_handle = focal_list.push(n);
				}
			}
			//updateFocalList(lower_bound, new_lower_bound, f_weight);
			min_f_val = new_min_f_val;
			lower_bound = new_lower_bound;
			/*
			cout << "   New-f-min=" << min_f_val << " ; New-LB=" << lower_bound << endl;
			cout << "FOCAL: ";
			for (Node* n : focal_list)
			cout << n << " , ";
			cout << endl;
			*/
		}
		if (focal_list.size() == 0)
			std::cout << "ERROR!" << std::endl;
	}  // end while loop
	   // no path found
	//path.clear();
	releaseClosedListNodes(&allNodes_table);
	open_list.clear();
	focal_list.clear();
	allNodes_table.clear();
	return false;
}

inline void SingleAgentICBS::releaseClosedListNodes(hashtable_t* allNodes_table) {
	hashtable_t::iterator it;
	for (it = allNodes_table->begin(); it != allNodes_table->end(); it++) {
		delete ((*it).second);  // Node* s = (*it).first; delete (s);
	}
}

SingleAgentICBS::SingleAgentICBS(int id, int start_location, int goal_location,
	const bool* my_map, int map_size, const int* moves_offset, int num_col) 
{
	this->agent_id = id;
	this->my_map = my_map;
	this->moves_offset = moves_offset;
	//this->actions_offset = actions_offset;
	this->start_location = start_location;
	this->goal_location = goal_location;
	//this->start_orientation = start_orientation;
	this->map_size = map_size;
	//this->e_weight = e_weight;
	this->num_expanded = 0;
	this->num_generated = 0;
	//this->path_cost = 0;
	this->lower_bound = 0;
	this->min_f_val = 0;
	//this->num_non_hwy_edges = 0;
	this->num_col = num_col;

	// initialize allNodes_table (hash table)
	empty_node = new LLNode();
	empty_node->loc = -1;
	deleted_node = new LLNode();
	deleted_node->loc = -2;
	//allNodes_table.set_empty_key(empty_node);
	//allNodes_table.set_deleted_key(deleted_node);

}


SingleAgentICBS::~SingleAgentICBS()
{
	delete[] my_map;
	delete (empty_node);
	delete (deleted_node);
}
