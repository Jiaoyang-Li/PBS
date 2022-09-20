#pragma once
#include "PBSNode.h"
#include "SingleAgentSolver.h"

class PBS
{
public:
	/////////////////////////////////////////////////////////////////////////////////////
	// stats
	double runtime = 0;
	double runtime_generate_child = 0; // runtimr of generating child nodes
	double runtime_build_CT = 0; // runtimr of building constraint table
	double runtime_build_CAT = 0; // runtime of building conflict avoidance table
	double runtime_path_finding = 0; // runtime of finding paths for single agents
	double runtime_detect_conflicts = 0;
	double runtime_preprocessing = 0; // runtime of building heuristic table for the low level

	uint64_t num_HL_expanded = 0;
	uint64_t num_HL_generated = 0;
	uint64_t num_LL_expanded = 0;
	uint64_t num_LL_generated = 0;
	uint64_t num_backtrack = 0;

	PBSNode* dummy_start = nullptr;
    PBSNode* goal_node = nullptr;

	bool solution_found = false;
	int solution_cost = -2;

	/////////////////////////////////////////////////////////////////////////////////////////
	// set params
	void setConflictSelectionRule(conflict_selection c) { conflict_seletion_rule = c;}
	void setNodeLimit(int n) { node_limit = n; }

	////////////////////////////////////////////////////////////////////////////////////////////
	// Runs the algorithm until the problem is solved or time is exhausted 
	virtual bool solve(double _time_limit);

	PBS(const Instance& instance, bool sipp, int screen);
	void clearSearchEngines();
	~PBS();

	// Save results
	void saveResults(const string &fileName, const string &instanceName) const;
	void saveCT(const string &fileName) const; // write the CT to a file
    void savePaths(const string &fileName) const; // write the paths to a file
	void clear(); // used for rapid random  restart

protected:
	conflict_selection conflict_seletion_rule;

    stack<PBSNode*> open_list;
	list<PBSNode*> allNodes_table;


    list<int> ordered_agents;  // ordered from high priority agents to low priority agents
    vector<vector<bool>> priority_graph; // [i][j] = true indicates that i is lower than j

    virtual string getSolverName() const;

	int screen;
	
	double time_limit;
	int node_limit = MAX_NODES;

	clock_t start;

	int num_of_agents;


	vector<Path*> paths;
	vector <SingleAgentSolver*> search_engines;  // used to find (single) agents' paths and mdd

    bool generateChild(int child_id, PBSNode* parent, int low, int high);

	int hasConflicts(int a1, int a2) const;
    bool hasConflicts(int a1, const set<int>& agents) const;
	shared_ptr<Conflict> chooseConflict(const PBSNode &node) const;
    int getSumOfCosts() const;
	inline void releaseNodes();

	// print and save
	void printResults() const;
	static void printConflicts(const PBSNode &curr, int num=INT_MAX);
    void printPriorityGraph() const;

	bool validateSolution() const;
	inline int getAgentLocation(int agent_id, size_t timestep) const;

	bool terminate(PBSNode* curr); // check the stop condition and return true if it meets

    void getHigherPriorityAgents(const list<int>::reverse_iterator & p1, set<int>& higher_agents);
    void getLowerPriorityAgents(const list<int>::iterator & p1, set<int>& lower_agents);
    bool hasHigherPriority(int low, int high) const; // return true if agent low is lower than agent high

	// node operators
	inline void pushNode(PBSNode* node)
	{
		// update handles
    	open_list.push(node);
		allNodes_table.push_back(node);
	}
    void pushNodes(PBSNode* n1, PBSNode* n2);
	PBSNode* selectNode();

	// high level search
	bool generateRoot();
    bool findPathForSingleAgent(PBSNode& node, const set<int>& higher_agents, int a, Path& new_path);
	void classifyConflicts(PBSNode &parent);
	void update(PBSNode* node);
	void printPaths() const;
	void printPath(const Path& _path_) const;

    void topologicalSort(list<int>& stack);
    void topologicalSortUtil(int v, vector<bool> & visited, list<int> & stack);
};
