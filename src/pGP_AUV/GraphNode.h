#ifndef GRAPHNODE_H
#define GRAPHNODE_H

#include <Eigen/StdVector>

class GraphNode
{
public:
  GraphNode(Eigen::Vector2d location, double value);

  GraphNode(Eigen::Vector2d location, double value, GraphNode *left_neighbour, GraphNode *right_neighbour, GraphNode *front_neighbour, GraphNode *back_neighbour);

  // Neighbours expected to be in the order: left, right, front and back.
  GraphNode(Eigen::Vector2d location, double value, std::vector<GraphNode*> neighbours);

  // All neighbours are returned in order: left, right, front and back.
  std::vector<GraphNode*> get_neighbours() const;

  // Returns neighbours excluding back in order: left, right and front.
  std::vector<GraphNode*> get_neighbours_ahead() const;

  const GraphNode* get_left_neighbour() const;

  void set_left_neighbour(GraphNode* left_neighbour);

  const GraphNode* get_right_neighbour() const;

  void set_right_neighbour(GraphNode* right_neighbour);

  const GraphNode* get_front_neighbour() const;

  void set_front_neighbour(GraphNode* front_neighbour);

  const GraphNode* get_back_neighbour() const;

  void set_back_neighbour(GraphNode* back_neighbour);

  double get_value() const;

  void set_value(double value);

  void set_location(Eigen::Vector2d location);
	
  Eigen::Vector2d get_location() const;

  // Represent indexes of the neighbours in the neighbours vector.
  static const int LEFT_NEIGHBOUR=0, RIGHT_NEIGHBOUR=1, FRONT_NEIGHBOUR=2, BACK_NEIGHBOUR=3;

private:
  // Holds neighbours of the node in the order left, right, front and back.
  std::vector<GraphNode*> m_neighbours;

  // Entropy
  double m_value; 

  // Latitude and longitude
  Eigen::Vector2d m_location;

  static const int MAX_NEIGHBOURS = 4;

  static const int MAX_NEIGHBOURS_AHEAD = 3;
};

#endif
