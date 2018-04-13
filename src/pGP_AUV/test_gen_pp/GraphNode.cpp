#include "GraphNode.h"

GraphNode::GraphNode(Eigen::Vector2d location, double value) :
  m_location(location),
  m_value(value)
{
  m_neighbours.reserve(MAX_NEIGHBOURS);
  for( int i = 0; i < MAX_NEIGHBOURS; i++ ){
    m_neighbours.push_back(NULL);
  }
}

GraphNode::GraphNode(Eigen::Vector2d location, double value, GraphNode *left_neighbour, GraphNode *right_neighbour, GraphNode *front_neighbour, GraphNode *back_neighbour):GraphNode(location,value)
{
  set_left_neighbour(left_neighbour);
  set_right_neighbour(right_neighbour);
  set_front_neighbour(front_neighbour);
  set_back_neighbour(back_neighbour);
}

// The first four elements in vector are assigned as neighbours in the order: left, right, front and back.
// If less than four elements are present then NULL is assigned to remaining neighbours.
GraphNode::GraphNode(Eigen::Vector2d location, double value, std::vector<GraphNode*> neighbours) :
  GraphNode(location,value)
{
  for( int i = 0; i < neighbours.size() && i<MAX_NEIGHBOURS ; i++ ){
    m_neighbours[i] = neighbours[i];
  }
}

// All neighbours returned in order: left, right, front and back.
std::vector<GraphNode*> GraphNode::get_neighbours() const
{
  return m_neighbours;
}

// Neighbours excluding back returned in order: left, right and front.
std::vector<GraphNode*> GraphNode::get_neighbours_ahead() const
{
  return std::vector<GraphNode*>(m_neighbours.begin(), m_neighbours.begin() + MAX_NEIGHBOURS_AHEAD);
}

const GraphNode * GraphNode::get_left_neighbour() const
{
  return m_neighbours[LEFT_NEIGHBOUR];
}

void GraphNode::set_left_neighbour(GraphNode * left_neighbour)
{
  m_neighbours[LEFT_NEIGHBOUR] = left_neighbour;
}

const GraphNode * GraphNode::get_right_neighbour() const
{
  return m_neighbours[RIGHT_NEIGHBOUR];
}

void GraphNode::set_right_neighbour(GraphNode * right_neighbour)
{
  m_neighbours[RIGHT_NEIGHBOUR] = right_neighbour;
}

const GraphNode * GraphNode::get_front_neighbour() const
{
  return m_neighbours[FRONT_NEIGHBOUR];
}

void GraphNode::set_front_neighbour(GraphNode * front_neighbour)
{
  m_neighbours[FRONT_NEIGHBOUR] = front_neighbour;
}

const GraphNode * GraphNode::get_back_neighbour() const
{
  return m_neighbours[BACK_NEIGHBOUR];
}

void GraphNode::set_back_neighbour(GraphNode * back_neighbour)
{
  m_neighbours[BACK_NEIGHBOUR] = back_neighbour;
}

double GraphNode::get_value() const
{
  return m_value;
}

void GraphNode::set_value(double value)
{
  m_value = value;
}

void GraphNode::set_location(Eigen::Vector2d location)
{
  m_location = location;
}

Eigen::Vector2d GraphNode::get_location() const
{
  return m_location;
}
