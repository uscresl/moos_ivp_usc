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
// TODO fix: incorrect

//By default std::maps (and std::sets) use operator< to determine sorting.

//You need to define operator< on your class.

//To objects are deemed equal if !(a < b) && !(b < a).

//If for some reason you'd like to use a different comparator, the third template argument of the map can be changed, to std::greater, for example.


bool Location::operator < ( Location const & other ) const
{
  if ( this->lon() < other.lon() && this->lat() == other.lat() && this->depth() <= other.depth() )
    return true;
  else if ( this->lon() == other.lon() && this->lat() < other.lat() && this->depth() <= other.depth() )
    return true;
  else if ( this->lon() < other.lon() && this->lat() < other.lat() && this->depth() <= other.depth() )
    return true;
  else if ( this->lon() == other.lon() && this->lat() == other.lat() && this->depth() < other.depth() )
    return true;
  else
    return false;
}

bool Location::operator ==( Location const & other ) const
{
  if ( this->lon() == other.lon() && this->lat() == other.lat() && this->depth() == other.depth() )
    return true;
  return false;
}
