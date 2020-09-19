#pragma once
#include "SingleAgentICBS.h"
//#include "ecbs_node.h"
//enum conflict_type { UNKNOWN, CARDINAL, SEMI, GEQSEMI, NON,  TYPE_COUNT };

//struct Conflict
//{
//	int timestep;
//	int a1;
//	int a2;
//	int loc1;
//	int loc2; 
//	int cost1; // delta cost of child 1
//	int cost2; // delta cost of child 2
//	conflict_type type;
//	Conflict(int time, int agent1, int agent2, int location1, int location2, conflict_type contype = UNKNOWN)
//	{
//		timestep = time;
//		a1 = agent1;
//		a2 = agent2;
//		loc1 = location1;
//		loc2 = location2;
//		cost1 = 0;
//		cost2 = 0;
//		type = contype;
//	}
//	Conflict(const Conflict& cpy)
//	{
//		timestep = cpy.timestep;
//		a1 = cpy.a1;
//		a2 = cpy.a2;
//		loc1 = cpy.loc1;
//		loc2 = cpy.loc2;
//		cost1 = cpy.cost1;
//		cost2 = cpy.cost2;
//		type = cpy.type;
//	}
//	~Conflict(){}
//	void Swap()
//	{
//		int temp = a1; a1 = a2; a2 = temp;
//		if(loc2 >=0)
//		{
//			temp = loc1; loc1 = loc2; loc2 = temp;
//		}
//		temp = cost1; cost1 = cost2; cost2 = temp;
//	}
//};

class GICBSNode
{
public:
	// the following is used to comapre nodes in the OPEN list
	struct compare_node {
		bool operator()(const GICBSNode* n1, const GICBSNode* n2) const {
			return n1->depth <= n2->depth;
			if (n1->depth == n2->depth)
			{
				if (n1->f_val == n2->f_val)
				{
					if (n1->num_of_collisions == n2->num_of_collisions) {
						if (n1->g_val == n2->g_val) {
							return n1->time_generated > n2->time_generated; // break ties towards earilier generated nodes (FIFO manner)
						}
						return n1->g_val <= n2->g_val;  // break ties towards larger g_val
					}
					return n1->num_of_collisions >= n2->num_of_collisions;
				}
				return n1->f_val >= n2->f_val;
			}
			return n1->depth <= n2->depth;
		}
	};  // used by OPEN to compare nodes by sum_min_f_vals (top of the heap has min sum_min_f_vals)

		// the following is used to comapre nodes in the FOCAL list
	/*
	struct secondary_compare_node {
		bool operator()(const GICBSNode* n1, const GICBSNode* n2) const {
			if (n1->num_of_collisions == n2->num_of_collisions)
			{
				if (n1->depth == n2->depth)
				{
					if (n1->g_val == n2->g_val)
						return n1->time_generated > n2->time_generated; // break ties towards earilier generated nodes (FIFO manner)
					return n1->g_val <= n2->g_val;  // break ties towards larger g_val
				}
				return n1->depth <= n2->depth; // deeper nodes
			}
			return n1->f_val >= n2->f_val;
		}
	}; */ // used by FOCAL to compare nodes by num_of_collisions (top of the heap has min h-val)

	typedef boost::heap::pairing_heap< GICBSNode*, compare<GICBSNode::compare_node> >::handle_type open_handle_t;
	// typedef boost::heap::fibonacci_heap< GICBSNode*, compare<GICBSNode::compare_node> >::handle_type open_handle_t;
	//typedef boost::heap::fibonacci_heap< GICBSNode*, compare<GICBSNode::secondary_compare_node> >::handle_type focal_handle_t;
	
	open_handle_t open_handle;
	//focal_handle_t focal_handle;

	// The following is used by googledensehash for generating the hash value of a nodes
	// this is needed because otherwise we'll have to define the specilized template inside std namespace
	struct GICBSNodeHasher {
		std::size_t operator()(const GICBSNode* n) const {
			size_t agent_id_hash = boost::hash<int>()(n->agent_id);
			size_t time_generated_hash = boost::hash<int>()(n->time_generated);
			return (agent_id_hash ^ (time_generated_hash << 1));
		}
	};

	GICBSNode* parent;
	//vector<MDD*> mdds;
	//vector<std::shared_ptr<vector<bool>>> single;
	//vector<std::shared_ptr<vector<PathEntry>>> paths;
	std::shared_ptr<tuple<int, int, int, int, int>> conflict;
	int agent_id;
	//std::shared_ptr<tuple<int, int, int, bool>> constraint; // <int loc1, int loc2, int timestep, bool positive_constraint> NOTE loc2 = -1 for vertex constraint; loc2 = loation2 for Edge Constraint
	tuple<int, int, int, bool > constraint; // <int loc1, int loc2, int timestep, bool positive_constraint> NOTE loc2 = -1 for vertex constraint; loc2 = loation2 for Edge Constraint
	//vector<list<std::shared_ptr<tuple<int, int, int, bool>>>> constraints;  

	list<pair<int, vector<PathEntry>>> new_paths;

	//list<std::shared_ptr<tuple<int, int, int, bool>>> constraints_positive;
	int g_val;
	int h_val;
	int f_val;
	size_t depth;
	size_t makespan;
	int num_of_collisions;

	uint64_t time_expanded;
	uint64_t time_generated;

	//vector<std::shared_ptr<vector<bool>>> inferiorities;
	vector<vector<bool>> priorities;
	vector<vector<bool>> trans_priorities;

	//bool buildMDD(const vector < list< pair<int, int> > >& constraints, int numOfLevels);
	//bool updateMDD(const tuple<int, int, int> &constraint);
	//bool findPathByMDD(bool* res_table);

	//GICBSNode();
	//GICBSNode(int agent_id, int numAgents, double g_val, double num_of_collisions, int time_expanded, double sum_min_f_vals);
	//GICBSNode(int agent_id, GICBSNode* parent, double g_val, double num_of_collisions, int time_expanded, double sum_min_f_vals);
	void clear();
	//~GICBSNode();
};

