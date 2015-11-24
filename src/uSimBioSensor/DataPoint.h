#ifndef DATAPOINT_H
#define DATAPOINT_H

#include "Location.h"

class DataPoint
{
  public:
    // default constructor
    DataPoint();
    // constructor
    DataPoint(double lon, double lat, double depth, double value);
    // copy constructor
    DataPoint(DataPoint const &dp);

    // getters & setters
    Location data_location() const { return m_location; };
    double data_value() const { return m_value; };
    double set_data_value(double new_val){ m_value = new_val; };

    // print
    void print() const;

  private:
    // location
    Location m_location;
    // sensor value
    double m_value;

};

#endif // DATAPOINT_H
