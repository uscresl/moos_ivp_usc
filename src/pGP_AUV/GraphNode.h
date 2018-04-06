#ifndef GRAPHNODE_H
#define GRAPHNODE_H

#include <Eigen/StdVector>

class GraphNode
{
  public:
    // constructors //////////////////////////////////////////////////////////////
    GraphNode(Eigen::Vector2d location, double value);

    GraphNode(Eigen::Vector2d location, double value, GraphNode *west_neighbour, GraphNode *east_neighbour, GraphNode *north_neighbour, GraphNode *south_neighbour,
        GraphNode *northwest_neighbour, GraphNode *northeast_neighbour, GraphNode *southwest_neighbour, GraphNode *southeast_neighbour);

    // Neighbours expected to be in the order: west, east, north, south, northwest, northeast, southwest, southeast.
    GraphNode(Eigen::Vector2d location, double value, std::vector<GraphNode*> neighbours);


    // All neighbours are returned in order: west, east, north, south, northwest, northeast, southwest, southeast.
    std::vector<GraphNode*> get_neighbours() const;

    // Returns neighbours excluding south in order: west, east and north.
    std::vector<GraphNode*> get_neighbours_ahead() const;

    const GraphNode* get_west_neighbour() const;
    void set_west_neighbour(GraphNode* west_neighbour);

    const GraphNode* get_east_neighbour() const;
    void set_east_neighbour(GraphNode* east_neighbour);

    const GraphNode* get_north_neighbour() const;
    void set_north_neighbour(GraphNode* north_neighbour);

    const GraphNode* get_south_neighbour() const;
    void set_south_neighbour(GraphNode* south_neighbour);

    const GraphNode* get_northwest_neighbour() const;
    void set_northwest_neighbour(GraphNode* northwest_neighbour);

    const GraphNode* get_northeast_neighbour() const;
    void set_northeast_neighbour(GraphNode* northeast_neighbour);

    const GraphNode* get_southwest_neighbour() const;
    void set_southwest_neighbour(GraphNode* southwest_neighbour);

    const GraphNode* get_southeast_neighbour() const;
    void set_southeast_neighbour(GraphNode* southeast_neighbour);

    double get_value() const;
    void set_value(double value);

    void set_location(Eigen::Vector2d location);
    Eigen::Vector2d get_location() const;

    // Getters/setters for children
    void set_children(GraphNode* child);
    std::vector<GraphNode*> get_children() const;

    // For FTC-algorithm: Holds children nodes (if empty, then current node is a child)
    std::vector<GraphNode*> m_children;

    // Represent indices of the neighbours in the neighbours vector.
    static const int WEST_NEIGHBOUR=0, EAST_NEIGHBOUR=1, NORTH_NEIGHBOUR=2, SOUTH_NEIGHBOUR=3,
        NORTHWEST_NEIGHBOUR=4, NORTHEAST_NEIGHBOUR=5, SOUTHWEST_NEIGHBOUR=6, SOUTHEAST_NEIGHBOUR=7;

  private:
    // Holds neighbours of the node in the order west, east, north and south, northwest, northeast, southwest, southeast.
    std::vector<GraphNode*> m_neighbours;

    // Entropy
    // For FTC-algorithm, use average of its children nodes
    double m_value;

    // Latitude and longitude
    Eigen::Vector2d m_location;

    static const int MAX_NEIGHBOURS = 8;
    static const int MAX_NEIGHBOURS_AHEAD = 3;
};

#endif
