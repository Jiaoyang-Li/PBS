#include "node.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

using namespace boost;
using namespace std;

Node::Node() : loc(0), action(MapLoader::WAIT_ACTION), g_val(0), h_val(0), parent(NULL), timestep(0), num_internal_conf(0), in_openlist(false), num_non_hwy_edges(0), orientation(MapLoader::FACE_EAST) {
}

Node::Node(int loc, MapLoader::valid_actions_t action, double g_val, double num_of_collisions, Node* parent, int timestep, int num_internal_conf, bool in_openlist, int num_non_hwy_edges, MapLoader::orientation_t orientation):
    loc(loc),  action(action), g_val(g_val), h_val(num_of_collisions), parent(parent), timestep(timestep),
    num_internal_conf(num_internal_conf), in_openlist(in_openlist), num_non_hwy_edges(num_non_hwy_edges),
    orientation(orientation) {
}

Node::Node(const Node& other) {
  loc = other.loc;
  action = other.action;
  g_val = other.g_val;
  h_val = other.h_val;
  parent = other.parent;
  timestep = other.timestep;
  in_openlist = other.in_openlist;
  open_handle = other.open_handle;
  focal_handle = other.focal_handle;
  num_internal_conf = other.num_internal_conf;
  num_non_hwy_edges = other.num_non_hwy_edges;
  orientation = other.orientation;
}

Node::~Node() {
}

std::ostream& operator<<(std::ostream& os, const Node& n) {
  if ( n.parent != NULL )
    os << "LOC=" << n.loc << " ; ORT=" << n.orientation << " ; TIMESTEP=" << n.timestep << " ; GVAL=" << n.g_val << " ; HVAL=" << std::setprecision(4) << n.h_val
       << " ; #CONF="<< n.num_internal_conf << " ; PARENT=" << (n.parent)->loc
       << " ; IN_OPEN?" << std::boolalpha << n.in_openlist;
  else
    os << "LOC=" << n.loc << " ; TIMESTEP=" << n.timestep << " ; GVAL=" << n.g_val << " ; HVAL=" << std::setprecision(4) << n.h_val
       << " ; #CONF="<< n.num_internal_conf << " ; ROOT (NO PARENT)";
  return os;
}
/*std::ostream& operator<<(std::ostream& os, const Node* n) {
  os << "LOC=" << n->loc << " ; TIMESTEP=" << n->timestep << " ; GVAL=" << n->g_val << " ; PARENT=" << (n->parent)->loc;
  return os;
  }*/
