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

GraphNode::GraphNode(Eigen::Vector2d location, double value, GraphNode *west_neighbour, GraphNode *east_neighbour, GraphNode *north_neighbour, GraphNode *south_neighbour,
    GraphNode *northwest_neighbour, GraphNode *northeast_neighbour, GraphNode *southwest_neighbour, GraphNode *southeast_neighbour):GraphNode(location,value)
{
  set_west_neighbour(west_neighbour);
  set_east_neighbour(east_neighbour);
  set_north_neighbour(north_neighbour);
  set_south_neighbour(south_neighbour);
  set_northwest_neighbour(northwest_neighbour);
  set_northeast_neighbour(northeast_neighbour);
  set_southwest_neighbour(southwest_neighbour);
  set_southeast_neighbour(southeast_neighbour);
 // m_children.reserve(4);
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

const GraphNode * GraphNode::get_west_neighbour() const
{
  return m_neighbours[WEST_NEIGHBOUR];
}

void GraphNode::set_west_neighbour(GraphNode * west_neighbour)
{
  m_neighbours[WEST_NEIGHBOUR] = west_neighbour;
}

const GraphNode * GraphNode::get_east_neighbour() const
{
  return m_neighbours[EAST_NEIGHBOUR];
}

void GraphNode::set_east_neighbour(GraphNode * east_neighbour)
{
  m_neighbours[EAST_NEIGHBOUR] = east_neighbour;
}

const GraphNode * GraphNode::get_north_neighbour() const
{
  return m_neighbours[NORTH_NEIGHBOUR];
}

void GraphNode::set_north_neighbour(GraphNode * north_neighbour)
{
  m_neighbours[NORTH_NEIGHBOUR] = north_neighbour;
}

const GraphNode * GraphNode::get_south_neighbour() const
{
  return m_neighbours[SOUTH_NEIGHBOUR];
}

void GraphNode::set_south_neighbour(GraphNode * south_neighbour)
{
  m_neighbours[SOUTH_NEIGHBOUR] = south_neighbour;
}

const GraphNode * GraphNode::get_northwest_neighbour() const
{
  return m_neighbours[NORTHWEST_NEIGHBOUR];
}

void GraphNode::set_northwest_neighbour(GraphNode * northwest_neighbour)
{
  m_neighbours[NORTHWEST_NEIGHBOUR] = northwest_neighbour;
}

const GraphNode * GraphNode::get_northeast_neighbour() const
{
  return m_neighbours[NORTHEAST_NEIGHBOUR];
}

void GraphNode::set_northeast_neighbour(GraphNode * northeast_neighbour)
{
  m_neighbours[NORTHEAST_NEIGHBOUR] = northeast_neighbour;
}

const GraphNode * GraphNode::get_southwest_neighbour() const
{
  return m_neighbours[SOUTHWEST_NEIGHBOUR];
}

void GraphNode::set_southwest_neighbour(GraphNode * southwest_neighbour)
{
  m_neighbours[SOUTHWEST_NEIGHBOUR] = southwest_neighbour;
}

const GraphNode * GraphNode::get_southeast_neighbour() const
{
  return m_neighbours[SOUTHEAST_NEIGHBOUR];
}

void GraphNode::set_southeast_neighbour(GraphNode * southeast_neighbour)
{
  m_neighbours[SOUTHEAST_NEIGHBOUR] = southeast_neighbour;
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

void GraphNode::set_children(GraphNode* child)
{
    m_children.push_back(child);
}

std::vector<GraphNode*> GraphNode::get_children() const
{
    return m_children;
}
