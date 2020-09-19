// Load's a 2D map.
// First line: ROWS COLS
// Second line and onward, "1" represent blocked cell (otherwise, open)

#ifndef MAPLOADER_H
#define MAPLOADER_H

#include <string>
#include <vector>

#define TIME_LIMIT 60 * CLOCKS_PER_SEC

enum constraint_strategy { ECBS, N_ECBS, ICBS, N_ICBS, CBSH, N_CBSH, CBSH_CR,CBSH_R,CBSH_RM, STRATEGY_COUNT };

class MapLoader {
 public:
  bool* my_map;
  int rows;
  int cols;

  int start_loc;
  int goal_loc;

  enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, INVALID, MOVE_COUNT };  // MOVE_COUNT is the enum's size
  int* moves_offset;

  enum valid_actions_t { WAIT_ACTION, MOVE, ROTATE_L, ROTATE_R, ACTIONS_COUNT};
  int* actions_offset;

  enum orientation_t { FACE_NORTH, FACE_EAST, FACE_SOUTH, FACE_WEST, ORIENTATION_COUNT};

  
  MapLoader(std::string fname); // load map from file
  MapLoader(int rows, int cols); // initialize new [rows x cols] empty map
  inline bool is_blocked (int row, int col) const { return my_map[row * this->cols + col]; }
  inline bool is_blocked (int loc) const { return my_map[loc]; }
  inline size_t map_size() const { return rows * cols; }
  void printMap ();
  void printMap (char* mapChar);
  void printHeuristic (const double* mapH, const int agent_id);
  char* mapToChar();
  bool* get_map () const; // return a deep-copy of my_map
  inline int linearize_coordinate(int row, int col) const { return ( this->cols * row + col); }
  inline int row_coordinate(int id) const { return id / this->cols; }
  inline int col_coordinate(int id) const { return id % this->cols; }
  void printPath (std::vector<int> path);
  void saveToFile(std::string fname);
  //  valid_actions_t get_action (int id1, int id2) const;
  ~MapLoader();
};

#endif
