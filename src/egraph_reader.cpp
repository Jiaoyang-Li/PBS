#include "egraph_reader.h"
#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include "map_loader.h"


// generate the empty egraph ------------------------------------
EgraphReader::EgraphReader() {
  e_graph = new my_digraph();
}  // -----------------------------------------------------------

// generate the egraph from edges map ---------------------------
EgraphReader::EgraphReader(int* edgesMap, const MapLoader* ml) {
  e_graph = new my_digraph();
  for (int i = 0; i < ml->rows; i++)
    for (int j = 0; j < ml->cols; j++)
      switch ( edgesMap[ml->linearize_coordinate(i, j)] ) {
        case 0:
          break;
        case 1:  // East
          if ( !ml->is_blocked(i, j) && !ml->is_blocked(i, j+1) )
            addEdge(ml->linearize_coordinate(i, j),
                    ml->linearize_coordinate(i, j+1));
          break;
        case 2:  // North
          if ( !ml->is_blocked(i, j) && !ml->is_blocked(i-1, j) )
            addEdge(ml->linearize_coordinate(i, j),
                    ml->linearize_coordinate(i-1, j));
          break;
        case 3:  // West
          if ( !ml->is_blocked(i, j) && !ml->is_blocked(i, j-1) )
            addEdge(ml->linearize_coordinate(i, j),
                    ml->linearize_coordinate(i, j-1));
          break;
        case 4:  // South
          if ( !ml->is_blocked(i, j) && !ml->is_blocked(i+1, j) )
            addEdge(ml->linearize_coordinate(i, j),
                    ml->linearize_coordinate(i+1, j));
          break;
      }
}  // -----------------------------------------------------------

// generate an egraph from file
EgraphReader::EgraphReader(string fname) {
  string line;
  ifstream myfile(fname.c_str());
  // int num_of_vertices = 0;
  if (myfile.is_open()) {
    getline(myfile, line);
    boost::tokenizer<> tok(line);
    boost::tokenizer<>::iterator beg = tok.begin();
    beg++; beg++;  // skip the prefix of "p edge"
    // num_of_vertices = atoi ( (*beg).c_str() );
    e_graph = new my_digraph();  // num_of_vertices);
    beg++;
    int num_of_edges = atoi((*beg).c_str());
    for (int i = 0; i < num_of_edges; i++) {
      getline(myfile, line);
      boost::tokenizer<> edge_tok(line);
      boost::tokenizer<>::iterator e_beg = edge_tok.begin();
      e_beg++;
      int from_v = atoi((*e_beg).c_str());
      it = nodes.find(from_v);
      if ( it == nodes.end() ) {  // add this new node
        u1 = add_vertex(*e_graph);
        (*e_graph)[u1].node_id = from_v;
        nodes[from_v] = u1;
      } else {
        u1 = (*it).second;  // bring it up
      }
      e_beg++;
      int to_v = atoi((*e_beg).c_str());
      it = nodes.find(to_v);
      if ( it == nodes.end() ) {  // add this new node
        u2 = add_vertex(*e_graph);
        (*e_graph)[u2].node_id = to_v;
        nodes[to_v] = u2;
      } else {
        u2 =  (*it).second;  // bring it up
      }
      //      cout << from_v << " -> " << to_v << endl;
      add_edge(u1, u2, *e_graph);
    }
    myfile.close();
  }
}


void EgraphReader::printToDOT(string fname) {
  /* Only works when the edge list is not hash (will work for example with vecS)
  ofstream dot_file;
  dot_file.open ( fname );
  write_graphviz( dot_file , *e_graph, 
		  boost::make_label_writer(boost::get(&VertexDesc::node_id, *e_graph)) ); //default_writer for edges
  dot_file.close();
  */
}

/* Old
bool EgraphReader::isEdge(int n1, int n2) const {
  eg_Vertex c_u1, c_u2;  // declaring vertices u1,u2
  dense_hash_map< int, eg_Vertex >::const_iterator const_it;
  const_it = nodes.find(n1);
  if ( const_it == nodes.end() )  // n1 is not in Egraph
    return false;
  else
    c_u1 = const_it->second;  // (used to be =nodes[n1]). But nodes[n1] is not const. However, const_it->second is...
  const_it = nodes.find(n2);
  if ( const_it == nodes.end() )  // n2 is not in Egraph
    return false;
  else
    c_u2 = const_it->second;  //  (used to be =nodes[n1]). nodes[n2] is not const. However, const_it->second is...
  // edge(u,v,g) returns pair<edge_desc,bool>
  return boost::edge(c_u1, c_u2, *e_graph).second;
}
*/
bool EgraphReader::isEdge(int n1, int n2) const {
  eg_Vertex c_u1, c_u2;  // declaring vertices u1,u2
  boost::unordered_map< int, eg_Vertex >::const_iterator const_it;
  const_it = nodes.find(n1);
  if ( const_it == nodes.end() )  // n1 is not in Egraph
    return false;
  else
    c_u1 = const_it->second;  // (used to be =nodes[n1]). But nodes[n1] is not const. However, const_it->second is...
  const_it = nodes.find(n2);
  if ( const_it == nodes.end() )  // n2 is not in Egraph
    return false;
  else
    c_u2 = const_it->second;  //  (used to be =nodes[n1]). nodes[n2] is not const. However, const_it->second is...
  // edge(u,v,g) returns pair<edge_desc,bool>
  return boost::edge(c_u1, c_u2, *e_graph).second;
}


vector< pair<int, int> >* EgraphReader::getAllEdges() {
  vector< pair<int, int> >* retVal = new vector< pair<int, int> >();
  /* iterating over vertices
  boost::graph_traits<my_digraph>::vertex_iterator vi, v_end;
  for (boost::tie(vi, v_end) = boost::vertices(*e_graph); vi != v_end; ++vi) {
    std::cout << (*e_graph)[*vi].node_id << endl; //(*e_graph)[*vi] access the struct...
  }
  */
  // iterate over all edges
  boost::graph_traits<my_digraph>::edge_iterator ei, e_end;
  for (boost::tie(ei, e_end) = boost::edges(*e_graph); ei != e_end; ++ei) {
    u1 = boost::source(*ei, *e_graph);
    u2 = boost::target(*ei, *e_graph);
    //    std::cout << (*e_graph)[u1].node_id << " --> " << (*e_graph)[u2].node_id << endl; //(*e_graph)[*vi] access the struct...
    retVal->push_back ( make_pair( (*e_graph)[u1].node_id , (*e_graph)[u2].node_id ) );
  }
  return retVal;
}

bool EgraphReader::containVertex(int n_id) const {
  boost::unordered_map< int, eg_Vertex >::const_iterator const_it;
  if ( const_it == nodes.end() )  // n_id is not in Egraph
    return false;
  return true;
}

void EgraphReader::removeVertex(int n_id) {
  // Note -- very weird "bug" in Boost::bgl
  // if using the *const* method above (containVertex) then clear_vertex(u1, *e_graph) throws segmentation fault!
  it = nodes.find(n_id);
  if ( it != nodes.end() ) {  // n_id is in EG
    // remove from egraph: 1) all edges to/from n_id (clear_vertex) and 2) delete from graph (remove_vertex)
    u1 = nodes[n_id];
    clear_vertex(u1, *e_graph);
    remove_vertex(u1, *e_graph);
    // remove from nodes
    it = nodes.find(n_id);
    nodes.erase(it);
  }
}

// add an edge from n1 to n2 (and add n1 or n2 to the graph if either is not present in it)
void EgraphReader::addEdge(int n1_id, int n2_id) {
  it = nodes.find(n1_id);
  if ( it == nodes.end() ) {  // add this new node
    u1 = add_vertex(*e_graph);
    (*e_graph)[u1].node_id = n1_id;
    nodes[n1_id] = u1;
  } else {
    u1 = (*it).second;  // bring it up
  }
  it = nodes.find(n2_id);
  if ( it == nodes.end() ) {  // add this new node
    u2 = add_vertex(*e_graph);
    (*e_graph)[u2].node_id = n2_id;
    nodes[n2_id] = u2;
  } else {
    u2 = (*it).second;  // bring it up
  }
  add_edge(u1, u2, *e_graph);
}

void EgraphReader::addVertices(const vector<int>* v_list) {
  for (size_t i = 0; i < v_list->size() - 1; i++)
    addEdge(v_list->at(i) , v_list->at(i + 1));
}


void EgraphReader::saveToFile(string fname) {
  ofstream myfile;
  myfile.open(fname);
  myfile << "p edges " << num_vertices(*e_graph) << " " << num_edges(*e_graph) << endl;  // write header line
  vector < pair<int, int> >* edges = getAllEdges();
  for (vector< pair<int, int> >::const_iterator it = edges->begin(); it != edges->end(); ++it)
    myfile << "e " << it->first << " " << it->second << endl;
  delete(edges);
  myfile.close();
}

void EgraphReader::createCrissCrossHWY(MapLoader* ml) {
  e_graph = new my_digraph();
  // Add vertical edges
  // (note row=0 and row=ml->rows-1 are padding obstacles)
  for (int row = 1; row < ml->rows-2; row++) {
    for (int col = 1; col < ml->cols-1; col++) {
      if ( !ml->is_blocked(row, col) && !ml->is_blocked(row+1, col) ) {
        if ( col % 2 == 0 ) {
          //          cout << "[" << row << "," << col << "] ^^^ [" << row+1 << "," << col << endl;
          addEdge(ml->linearize_coordinate(row, col), ml->linearize_coordinate(row+1, col));
        } else {
          //          cout << "[" << row << "," << col << "] ___ [" << row+1 << "," << col << endl;
          addEdge(ml->linearize_coordinate(row+1, col), ml->linearize_coordinate(row, col));
        }
      }
    }
  }
  // Add horizontal edges
  for (int col = 1; col < ml->cols-2; col++) {
    for (int row = 1; row < ml->rows-1; row++) {
      if ( !ml->is_blocked(row, col) && !ml->is_blocked(row, col+1) ) {
        if ( row % 2 == 0 ) {
          //          cout << "[" << row << "," << col << "] --> [" << row << "," << col+1 << endl;
          addEdge(ml->linearize_coordinate(row, col), ml->linearize_coordinate(row, col+1));
        } else {
          //          cout << "[" << row << "," << col << "] <-- [" << row << "," << col+1 << endl;
          addEdge(ml->linearize_coordinate(row, col+1), ml->linearize_coordinate(row, col));
        }
      }
    }
  }
}


EgraphReader::~EgraphReader() {
  //  delete (e_graph);
}
