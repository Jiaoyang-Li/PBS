#include "GICBSNode.h"



//GICBSNode::GICBSNode()
//{
//	agent_id = -1;
//	g_val = 0;
//	h_val = 0;
//	f_val = 0;
//	conflict = NULL; //make_tuple(0,0,0,0,-1);
//	num_of_collisions = 0;
//	time_expanded = -1;
//}

//GICBSNode::GICBSNode(int agent_id, int numAgents, double g_val, double num_of_collisions, int time_expanded, double sum_min_f_vals)
//{
//	this->agent_id = agent_id;
//	this->g_val = g_val;
//	this->h_val = 0;
//	conflict = NULL; //make_tuple(0,0,0,0,-1);
//	this->num_of_collisions = num_of_collisions;
//	this->time_expanded = time_expanded;
//	this->sum_min_f_vals = sum_min_f_vals;
//	//this->cardinal.resize(numAgents);
//	this->parent = NULL;
//	//this->constraintCost.resize(numAgents, 0);
//}

void GICBSNode::clear()
{
	//Clear expanded node's conflicts lists. in order to save some memory
	/*if (cons_strategy == constraint_strategy::R_CBSH)
	{
	for (int i = 0; i < curr->mdds.size(); i++)
	{
	if (curr->mdds[i] != NULL)
	{
	curr->mdds[i]->numPointers--;
	if (curr->mdds[i]->numPointers == 0)
	delete curr->mdds[i];
	}
	}
	}*/

	//single.clear();

	//conflict.reset();
	
	//paths.clear();

	//agents_updated.clear();
}
/*GICBSNode::GICBSNode(int agent_id, GICBSNode* parent, double g_val, double num_of_collisions, int time_expanded, double sum_min_f_vals)
	:parent(parent)
{
	this->agent_id = agent_id;
	this->g_val = g_val;
	this->h_val = 0;
	conflict = make_tuple(0,0,0,0,-1);
	this->num_of_collisions = num_of_collisions;
	this->time_expanded = time_expanded;
	this->sum_min_f_vals = sum_min_f_vals;
	cardinal.resize(parent->cardinal.size());
	for (int i = 0; i < cardinal.size(); i++)
	{
		if(cardinal[i].empty())
			continue;
		cardinal[i].resize(parent->cardinal[i].size());
		cardinal[i].assign(parent->cardinal[i].begin(), parent->cardinal[i].end());
	}
	mdds.resize(parent->mdds.size());
	for (int i = 0; i < mdds.size(); i++)
	{
		mdds[i] = parent->mdds[i];
		if (mdds[i] != NULL)
			mdds[i]->numPointers++;
	}
	//this->constraintCost.resize(parent->constraintCost.size());
	//constraintCost.assign(parent->constraintCost.begin(), parent->constraintCost.end());
}*/



/*GICBSNode::~GICBSNode()
{
	if(mdds.empty())
		return;
	for(int i = 0; i < mdds.size(); i++)
		if(mdds[i] != NULL)
			delete mdds[i];
}*/
