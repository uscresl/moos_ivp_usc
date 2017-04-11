#include "GraphNode.h"

const GraphNode & GraphNode::get_left_neighbour() const
{
  return m_left_neighbour;
}

void GraphNode::set_left_neighbour(const GraphNode & m_left_neighbour)
{
  GraphNode::m_left_neighbour = m_left_neighbour;
}

const GraphNode & GraphNode::get_right_neighbour() const
{
  return m_right_neighbour;
}

void GraphNode::set_right_neighbour(const GraphNode & m_right_neighbour)
{
  GraphNode::m_right_neighbour = m_right_neighbour;
}

const GraphNode & GraphNode::get_front_neighbour() const
{
  return m_front_neighbour;
}

void GraphNode::set_front_neighbour(const GraphNode & m_front_neighbour)
{
  GraphNode::m_front_neighbour = m_front_neighbour;
}

const GraphNode & GraphNode::get_back_neighbour() const
{
  return m_back_neighbour;
}

void GraphNode::set_back_neighbour(const GraphNode & m_back_neighbour)
{
  GraphNode::m_back_neighbour = m_back_neighbour;
}

double GraphNode::get_value() const
{
  return m_value;
}

void GraphNode::set_value(double m_value)
{
  GraphNode::m_value = m_value;
}

double GraphNode::get_longitude() const
{
  return m_longitude;
}

void GraphNode::set_longitude(double m_longitude)
{
  GraphNode::m_longitude = m_longitude;
}

double GraphNode::get_latitude() const
{
  return m_latitude;
}

void GraphNode::set_latitude(double m_latitude)
{
  GraphNode::m_latitude = m_latitude;
}
