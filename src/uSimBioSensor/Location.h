#ifndef LOCATION_H
#define LOCATION_H

class Location
{
  public:
    Location();
    Location(double lon, double lat, double depth);
    Location(Location const &loc);

    // getters and setters
    double lon() const { return m_lon; };
    double lat() const { return m_lat; };
    double depth() const { return m_depth; };

    void print() const;

  private:
    double m_lon;
    double m_lat;
    double m_depth;
};

#endif // LOCATION_H
