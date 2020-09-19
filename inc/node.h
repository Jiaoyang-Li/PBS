// Represents 2D-Nodes
#ifndef NODE_H
#define NODE_H

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/functional/hash.hpp>
#include <string>
#include <vector>
#include <functional>  // for std::hash (c++11 and above)
#include <time.h>
#include "map_loader.h"

using boost::heap::fibonacci_heap;
using boost::heap::compare;


using namespace std;

struct pathEntry
{
  int location;
  MapLoader::valid_actions_t action;
  MapLoader::orientation_t orientation;
};

class Node {
 public:

  int loc;
  MapLoader::valid_actions_t action;
  double g_val;
  double h_val = 0;
  Node* parent;
  int timestep = 0;
  int num_internal_conf = 0;  // used in ECBS
  bool in_openlist = false;
  int num_non_hwy_edges = 0;  // used in ECBS_HWY
  MapLoader::orientation_t orientation = MapLoader::FACE_EAST;

  ///////////////////////////////////////////////////////////////////////////////
  // NOTE -- Normally, compare_node (lhs,rhs) suppose to return true if lhs<rhs.
  //         However, Heaps in STL and Boost are implemented as max-Heap.
  //         Hence, to achieve min-Head, we return true if lhs>rhs
  ///////////////////////////////////////////////////////////////////////////////

  // the following is used to comapre nodes in the OPEN list
  struct compare_node {
    // returns true if n1 > n2 (note -- this gives us *min*-heap).
    bool operator()(const Node* n1, const Node* n2) const {
      if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
        return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
      return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
    }
  };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

  // the following is used to comapre nodes in the FOCAL list
  struct secondary_compare_node {
    // returns true if n1 > n2
    bool operator()(const Node* n1, const Node* n2) const {
      if (n1->num_internal_conf == n2->num_internal_conf) {
		//if (n1->g_val == n2->g_val)
		//	return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;  // break ties towards smaller f_vals (prefer shorter solutions)
		//return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
       if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
			//if (n1->g_val == n2->g_val) // return randomly
			//{
			//	 srand(time(0));
			//	 if(rand() % 2 == 0)
			//		return true;
			//	else
			//		return false;
			//}
			return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
		 return n1->g_val+n1->h_val >= n2->g_val+n2->h_val;  // break ties towards smaller f_vals (prefer shorter solutions)
      }
      return n1->num_internal_conf >= n2->num_internal_conf;  // n1 > n2 if it has more conflicts
    }
  };  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)

  // the following is used to comapre nodes in the FOCAL list
  struct secondary_hwy_compare_node {
    // returns true if n1 > n2
    bool operator()(const Node* n1, const Node* n2) const {
      if (n1->num_internal_conf == n2->num_internal_conf) {
	if (n1->num_non_hwy_edges == n2->num_non_hwy_edges) {
	  if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
	    return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
	  return n1->g_val+n1->h_val >= n2->g_val+n2->h_val;  // break ties towards smaller f_vals (prefer shorter solutions)
	}
	return n1->num_non_hwy_edges >= n2->num_non_hwy_edges;  // break ties towards less non-highway edges
      }
      return n1->num_internal_conf >= n2->num_internal_conf;  // prefer less conflicts
    }
  };  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)

  // define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
  typedef boost::heap::fibonacci_heap< Node* , compare<Node::compare_node> >::handle_type open_handle_t;
  typedef boost::heap::fibonacci_heap< Node* , compare<Node::secondary_compare_node> >::handle_type focal_handle_t;  // ECBS
  //typedef boost::heap::fibonacci_heap< Node* , compare<secondary_hwy_compare_node> >::handle_type focal_handle_t;  // ECBS_HWY

  open_handle_t open_handle;
  focal_handle_t focal_handle;


  Node();
  Node(const Node& other);
  Node(int loc, MapLoader::valid_actions_t action,
       double g_val, double num_of_collisions,
       Node* parent, int timestep,
       int num_internal_conf = 0, bool in_openlist = false,
       int num_non_hwy_edges = 0, MapLoader::orientation_t orientation = MapLoader::FACE_EAST);
  inline double getFVal() const {return g_val + h_val;}
  ~Node();

  // The following is used by googledensehash for checking whether two nodes are equal
  // we say that two nodes, s1 and s2, are equal if
  // both are non-NULL and agree on the id and timestep
  struct eqnode {
    bool operator()(const Node* s1, const Node* s2) const {
      return (s1 == s2) || (s1 && s2 &&
                            s1->loc == s2->loc &&
                            // s1->orientation == s2->orientation &&
                            s1->timestep == s2->timestep);
    }
  };

  // The following is used by googledensehash for generating the hash value of a nodes
  // /* TODO:  */his is needed because otherwise we'll have to define the specilized template inside std namespace
  struct NodeHasher {
    std::size_t operator()(const Node* n) const {
      // cout << "COMPUTE HASH: " << *n << " ; Hash=" << hash<int>()(n->id) << endl;
      // cout << "   Pointer Address: " << n << endl;
      size_t loc_hash = boost::hash<int>()(n->loc);
      // size_t orientation_hash = std::hash<int>()(n->orientation);
      size_t timestep_hash = boost::hash<int>()(n->timestep);
	  return (loc_hash  ^ (timestep_hash << 1));
      // return ( loc_hash ^ orientation_hash ^ (timestep_hash << 1) );
    }
  };
};

std::ostream& operator<<(std::ostream& os, const Node& n);

#endif
