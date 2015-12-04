#include "DataPoint.h"

#include <iostream>

// default constructor, not used
DataPoint::DataPoint()
{
}

// constructor, use member initializer syntax (no point to init to default)
DataPoint::DataPoint(double lon, double lat,
                     double depth, double value)
  :
    m_location(lon, lat, depth),
    m_value(value)
{
}

// copy constructor
DataPoint::DataPoint(DataPoint const &dp)
  :
    m_location(dp.m_location),
    m_value(dp.m_value)
{
}

// getters and setters in header file
void DataPoint::print() const
{
  m_location.print();
  std::cout << "sensor value: " << m_value << std::endl;
}
