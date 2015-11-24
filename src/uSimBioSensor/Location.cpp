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
