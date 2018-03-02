//
// Created by Prachi Nawathe on 3/1/18.
//

#include "gen_pp.h"

double distance_between(const GraphNode* a, const GraphNode* b);

void run_genetic_pp(std::vector<GraphNode> grid_pts)
{
  generate_initial_paths(grid_pts);

  for(int i = 0; i < NUM_GENERATIONS; i++)
  {
    //calculate all entropies
    for(int j = 0; j < NUM_PATHS; j++)
    {
      calc_path_entropy(current_population[j]);
    }
    //create probability distribution
    //crossover from two random parents

  }
}

void generate_initial_paths(std::vector<GraphNode> grid_pts)
{
  std::mt19937 generator();

  if(!grid_pts.size()) return;

  for(int i = 0; i < NUM_PATHS; i++)
  {
    initial_seed.push_back(generate_path(grid_pts, generator()));
    current_population.push_back(initial_seed[i]);
  }
}

// ---------------------------------------------------------
// Procedure: generate_path()
//            go through a grid and select random points to
//            create a path of length PATH_LENGTH

std::vector<GraphNode> generate_path(std::vector<GraphNode> grid_pts, std::mt19937& gen)
{
  std::uniform_int_distribution<int> dist(0, grid_pts.size()-1);

  std::unordered_map<std::pair<int, GraphNode> > added_nodes;
  std::vector<GraphNode> path;
  int i = 0;
  while(i < PATH_LENGTH)
  {
    int grid_index = dist(gen);
    if(added_nodes.find(grid_index) == added_nodes.end())
    {
      //add element to path
      added_nodes.insert(std::pair<int, GraphNode>(grid_index, grid_pts[grid_index]));
      path.push_back(grid_pts[grid_index]);
      i++;
    }
  }

  return path;
}


double calc_path_entropy(std::vector<GraphNode> path)
{
  double entropy = 0.0;
  for(int i = 0; i < path.size(); i++)
  {
    GraphNode *node = path[i];
    entropy += node->m_value;
  }
  for(int i = 0; i < path.size()-1; i++)
  {
    entropy += .01 /*some multiplier*/ * distance_between(path[i], path[i+1]);
  }
  return entropy;
}


double distance_between(const GraphNode* a, const GraphNode* b)
{
  return sqrt((a->m_location->m_lon - b->m_location->m_lon)^2 +
      (a->m_location->m_lat - b->m_location->m_lat)^2);
}

