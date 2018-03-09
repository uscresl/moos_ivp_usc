//
// Created by Prachi Nawathe on 3/1/18.
//

#include "gen_pp.h"

double distance_between(const GraphNode* a, const GraphNode* b);
void select_parent(std::vector<double> &probability_dist,
                   std::uniform_real_distribution<double> dist,
                   std::mt19937 &generator, GraphNode *parent);

void run_genetic_pp(std::vector<GraphNode *> * grid_pts)
{
  std::mt19937 generator();
  generate_initial_paths(grid_pts, generator);

  for(int i = 0; i < NUM_GENERATIONS; i++)
  {
    // sort current population by fitness
    std::sort(current_population.begin(), current_population.end(),
              [](std::vector<GraphNode *> a, std::vector<GraphNode *> b) {
      return calc_path_entropy(a) > calc_path_entropy(b);
    });

    // create probability distribution
    std::vector<double> probability_dist(NUM_PATHS);
    create_prob_dist(probability_dist);

    // select parents
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    GraphNode *parentA, *parentB;
    parentA = nullptr;
    parentB = nullptr;
    select_parents(probability_dist, dist, parentA, parentB);

    crossover(parentA, parentB, generator());


    //crossover from two random parents

  }
}

void generate_initial_paths(std::vector<GraphNode* > grid_pts, std::mt19937 &generator)
{


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

std::vector<GraphNode *> generate_path(std::vector<GraphNode* > grid_pts, std::mt19937& gen)
{
  std::uniform_int_distribution<int> dist(0, grid_pts.size()-1);

  std::unordered_map<std::pair<int, GraphNode> > added_nodes;
  std::vector<GraphNode *> path;
  int i = 0;
  while(i < PATH_LENGTH)
  {
    int grid_index = dist(gen);
    if(added_nodes.find(grid_index) == added_nodes.end())
    {
      //add element to path
      added_nodes.insert(std::pair<int, GraphNode*>(grid_index, grid_pts[grid_index]));
      path.push_back(grid_pts[grid_index]);
      i++;
    }
  }

  return path;
}

// ---------------------------------------------------------
// Procedure: calc_path_entropy()
//            go through a path and sum all normalized
//            entropies, sum all normalized edge differences
double calc_path_entropy(const std::vector<GraphNode* > path)
{
  double entropy = 0.0, distance = 0.0;
  for(int i = 0; i < path.size(); i++)
  {
    GraphNode *node = path[i];
    entropy += node->m_value;
  }
  for(int i = 0; i < path.size()-1; i++)
  {
    entropy += 1000 /*some multiplier*/ * distance_between(path[i], path[i+1]);
  }
  return entropy;
}

// ---------------------------------------------------------
// Procedure: create_prob_dist()
//            for a given population, create a probability
//            distribution depending on the fitness of each
//            path
std::vector<double> create_prob_dist(std::vector<double> &probability_dist)
{
  //create probability distribution
  std::generate(probability_dist.begin(), probability_dist.end(),
                [&current_population, n = 0]() mutable
                {
                  return calc_path_entropy(current_population[n++]);
                });
  double entropy_normalizer = std::accumulate(probability_dist.begin(),
                                              probability_dist.end(), 0.0);
  std::transform(probability_dist.begin(), probability_dist.end(),
                 [&entropy_normalizer](double *prob)
                 {
                   return (prob / entropy_normalizer);
                 });
  return probability_dist;
}

// ---------------------------------------------------------
// Procedure: select_parents()
//            using a uniform distribution between 0-1,
//            generates a random probability and sets the
//            corresponding parent to that GraphNode //not the best function descriptor
void select_parents(std::vector<double> &probability_dist,
                    std::uniform_real_distribution<double> dist,
                    std::mt19937 &generator,
                    GraphNode *parentA, GraphNode *parentB)
{
  select_parent(probability_dist, dist, generator, parentA);
  select_parent(probability_dist, dist, generator, parentB);
}

void select_parent(std::vector<double> &probability_dist,
                   std::uniform_real_distribution<double> dist,
                   std::mt19937 &generator, GraphNode *parent) //is this the right way to pass it
{
  double parent;
  parent = dist(generator);
  double total_prob = 0.0;
  int i = 0;
  while(total_prob < parent)
  {
    total_prob += probability_dist[i++];
  }
  parent = probability_dist[i];
}


void crossover(std::vector<GraphNode *> &a, std::vector<GraphNode*> &b, std::mt19937 &generator)
{
  std::uniform_int_distribution<int> dist(0, PATH_LENGTH);
  int crossover_pt = dist(generator);
  std::vector<GraphNode *> child;
  std::copy_n(a.begin(), crossover_pt, std::back_inserter(child));
  //find point in b closest to last node of child
  // complete path from that point onward
  // if path length is < actual path
  // if path length is > actual path
  // if path length is == actual path hallelujah
}


double distance_between(const GraphNode* a, const GraphNode* b)
{
  return sqrt((a->m_location->m_lon - b->m_location->m_lon)^2 +
      (a->m_location->m_lat - b->m_location->m_lat)^2);
}

// f = entropy + alpha * (1 - L where L is distance, scaled from 0 to 1)
// or L is summation of all distances, then normalized from 0 to 1
// f = sum( entropy + alpha *(1-L))


double normalize_graph_node(GraphNode* a, GraphNode* b)
{
  return 1;
}

double normalize_entropy(GraphNode *a)
{
  return 1;
}

