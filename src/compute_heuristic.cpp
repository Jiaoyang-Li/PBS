#include <boost/heap/fibonacci_heap.hpp>
#include "compute_heuristic.h"
#include <cstring>
#include <climits>
#include "node.h"


using std::cout;
using std::endl;
using boost::heap::fibonacci_heap;


ComputeHeuristic::ComputeHeuristic(int start_location, int goal_location, const bool* my_map, int map_rows, int map_cols,
                                   const int* moves_offset, const int* actions_offset,
                                   double e_weight, const EgraphReader* egr) :
    my_map(my_map), map_rows(map_rows), map_cols(map_cols), moves_offset(moves_offset), actions_offset(actions_offset),
	start_location(start_location), goal_location(goal_location), egr(egr), e_weight(e_weight){}

double* ComputeHeuristic::getEstimatedGVals()
{
	return getShortestPathVals(start_location);
}

double* ComputeHeuristic::getHVals() 
{
	return getShortestPathVals(goal_location);
}

void ComputeHeuristic::getHVals(vector<int>& res)
{
	size_t root_location = goal_location;
	res.resize(map_rows * map_cols);
	for (int i = 0; i < map_rows * map_cols; i++)
		res[i] = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::fibonacci_heap< Node*, boost::heap::compare<Node::compare_node> > heap;
	boost::heap::fibonacci_heap< Node*, boost::heap::compare<Node::compare_node> >::handle_type open_handle;
	// generate hash_map (key is a node pointer, data is a node handler,
	//                    NodeHasher is the hash function to be used,
	//                    eqnode is used to break ties when hash values are equal)
    boost::unordered_map<Node*, fibonacci_heap<Node*, boost::heap::compare<Node::compare_node> >::handle_type, Node::NodeHasher, Node::eqnode> nodes;
    boost::unordered_map<Node*, fibonacci_heap<Node*, boost::heap::compare<Node::compare_node> >::handle_type, Node::NodeHasher, Node::eqnode>::iterator it; // will be used for find()

	Node* root = new Node(root_location, MapLoader::WAIT_ACTION, 0, 0, NULL, 0, false);
	root->open_handle = heap.push(root);  // add root to heap
	nodes[root] = root->open_handle;       // add root to hash_table (nodes)
	while (!heap.empty()) {
		Node* curr = heap.top();
		heap.pop();
		// cout << endl << "CURRENT node: " << curr << endl;
		for (int direction = 0; direction < 5; direction++)
		{
			int next_loc = curr->loc + moves_offset[direction];
			if (0 <= next_loc && next_loc < map_rows * map_cols && !my_map[next_loc] &&
				abs(next_loc % moves_offset[MapLoader::valid_moves_t::SOUTH] - curr->loc % moves_offset[MapLoader::valid_moves_t::SOUTH]) < 2)
			{  // if that grid is not blocked
				int next_g_val = curr->g_val + 1;
				Node* next = new Node(next_loc, MapLoader::WAIT_ACTION, next_g_val, 0, NULL, 0, false);
				it = nodes.find(next);
				if (it == nodes.end()) {  // add the newly generated node to heap and hash table
					next->open_handle = heap.push(next);
					nodes[next] = next->open_handle;
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					Node* existing_next = (*it).first;
					open_handle = (*it).second;
					if (existing_next->g_val > next_g_val) {
						existing_next->g_val = next_g_val;
						heap.update(open_handle);
					}
				}
			}
		}
	}
	// iterate over all nodes and populate the num_of_collisionss
	for (it = nodes.begin(); it != nodes.end(); it++) {
		Node* s = (*it).first;
		res[s->loc] = s->g_val;
		delete (s);
	}
	nodes.clear();
	heap.clear();
}

double* ComputeHeuristic::getShortestPathVals(int root_location)
{
	double* res = new double[map_rows * map_cols];
	for (int i = 0; i < map_rows * map_cols; i++)
		res[i] = DBL_MAX;
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::fibonacci_heap< Node*, boost::heap::compare<Node::compare_node> > heap;
	boost::heap::fibonacci_heap< Node*, boost::heap::compare<Node::compare_node> >::handle_type open_handle;
	// generate hash_map (key is a node pointer, data is a node handler,
	//                    NodeHasher is the hash function to be used,
	//                    eqnode is used to break ties when hash values are equal)
    boost::unordered_map<Node*, fibonacci_heap<Node*, boost::heap::compare<Node::compare_node> >::handle_type, Node::NodeHasher, Node::eqnode> nodes;
    boost::unordered_map<Node*, fibonacci_heap<Node*, boost::heap::compare<Node::compare_node> >::handle_type, Node::NodeHasher, Node::eqnode>::iterator it; // will be used for find()

	Node* root = new Node(root_location, MapLoader::WAIT_ACTION, 0, 0, NULL, 0, false);
	root->open_handle = heap.push(root);  // add root to heap
	nodes[root] = root->open_handle;       // add root to hash_table (nodes)
	while (!heap.empty()) {
		Node* curr = heap.top();
		heap.pop();
		// cout << endl << "CURRENT node: " << curr << endl;
		for (int direction = 0; direction < 5; direction++) 
		{
			int next_loc = curr->loc + moves_offset[direction];
			if (0 <= next_loc && next_loc < map_rows * map_cols && !my_map[next_loc] &&
				abs(next_loc % moves_offset[MapLoader::valid_moves_t::SOUTH] - curr->loc % moves_offset[MapLoader::valid_moves_t::SOUTH]) < 2)
			{  // if that grid is not blocked
									// compute cost to next_loc via curr node
				double cost = 1;
				if (!(*egr).isEdge(next_loc, curr->loc))
					cost = cost * e_weight;
				double next_g_val = curr->g_val + cost;
				Node* next = new Node(next_loc, MapLoader::WAIT_ACTION, next_g_val, 0, NULL, 0, false);
				it = nodes.find(next);
				if (it == nodes.end()) {  // add the newly generated node to heap and hash table
					next->open_handle = heap.push(next);
					nodes[next] = next->open_handle;
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					Node* existing_next = (*it).first;
					open_handle = (*it).second;
					if (existing_next->g_val > next_g_val) {
						existing_next->g_val = next_g_val;
						heap.update(open_handle);
					}
				}
			}
		}
	}
	// iterate over all nodes and populate the num_of_collisionss
	for (it = nodes.begin(); it != nodes.end(); it++) {
		Node* s = (*it).first;
		res[s->loc] = s->g_val;
		delete (s);
	}
	nodes.clear();
	heap.clear();
	return res;
}

ComputeHeuristic::~ComputeHeuristic() {
  delete[] my_map;
}
