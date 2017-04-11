#ifndef IVP_EXTEND_GRAPHNODE_H
#define IVP_EXTEND_GRAPHNODE_H


class /**/GraphNode
{
public:
  const GraphNode & get_left_neighbour() const;

  void set_left_neighbour(const GraphNode & m_left_neighbour);

  const GraphNode & get_right_neighbour() const;

  void set_right_neighbour(const GraphNode & m_right_neighbour);

  const GraphNode & get_front_neighbour() const;

  void set_front_neighbour(const GraphNode & m_front_neighbour);

  const GraphNode & get_back_neighbour() const;

  void set_back_neighbour(const GraphNode & m_back_neighbour);

  double get_value() const;

  void set_value(double m_value);

  double get_longitude() const;

  void set_longitude(double m_longitude);

  double get_latitude() const;

  void set_latitude(double m_latitude);

private:
  GraphNode m_left_neighbour, m_right_neighbour, m_front_neighbour, m_back_neighbour;
  double m_value;
  double m_longitude, m_latitude;
};


#endif //IVP_EXTEND_GRAPHNODE_H
