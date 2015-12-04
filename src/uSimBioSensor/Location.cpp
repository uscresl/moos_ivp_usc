#include "Location.h"

#include <iostream>

// default constructor
Location::Location()
{
}

// constructor
Location::Location(double lon, double lat, double depth)
  :
    m_lon(lon),
    m_lat(lat),
    m_depth(depth)
{
}

// copy constructor
Location::Location(Location const &loc)
{
  m_lon = loc.lon();
  m_lat = loc.lat();
  m_depth = loc.depth();
}

// getters and setters in header file

// print
void Location::print() const
{
  std::cout << "Location (lon, lat, depth): " << m_lon << ", " << m_lat << ", "
            << m_depth << std::endl;
}

// operator overloading

bool Location::operator< ( Location const & other ) const
{
  if ( this->lon() > other.lon() )
    return false;
  if ( this->lat() > other.lat() )
    return false;
  if ( this->depth() > other.depth() )
    return false;
  return true;
}

bool Location::operator ==( Location const & other ) const
{
  if ( this->lon() == other.lon() && this->lat() == other.lat() && this->depth() == other.depth() )
    return true;
  return false;
}
