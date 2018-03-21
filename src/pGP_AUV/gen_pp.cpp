//
// Created by Prachi Nawathe on 3/1/18.
//



// add goal, paths should go to goal
// crossover implementation: get random index, crossover at that pt on both paths

// Improvement is designed for feasible solutions. Randomly chose one node, do a
// local search in the neighboring grids of the node, move to a best grid. This
// operator is used for fine tuning of a feasible solution.


// Mutation is that randomly choose a node and replace it with a node that is not
// included in the path. Mutation is served as a key role to diversify the solution
// population. Therefore, it is not necessary that a solution is better after it
// is mutated. mutation probability: .2
// other operations: .9

#include "gen_pp.h"

double distance_between(const GraphNode* a, const GraphNode* b);
void select_parent(std::vector<double> &probability_dist,
                   std::uniform_real_distribution<double> dist,
                   std::mt19937 &generator, std::vector<GraphNode *> parent);

void run_genetic_pp(std::vector<GraphNode *> grid_pts)
{
  std::random_device rd;
  std::mt19937 generator(rd());
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


    // select x pairs of parents & crossover
    std::vector<std::vector<GraphNode *> > new_population;
    for(int j = 0; j < NUM_PATHS; j++)
    {
      std::vector<GraphNode *> child;
      std::uniform_real_distribution<double> crossover_mutate_dist(0.0, 1.0);
      double crossover_is = crossover_mutate_dist(generator());
      if(crossover_is < CROSSOVER_PROBABILITY)
      {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        std::vector<GraphNode *> parentA, parentB;
        select_parents(probability_dist, dist, generator, parentA, parentB);


        crossover(parentA, parentB, generator, child);
      }
      double mutation_is = crossover_mutate_dist(generator());
      if(mutation_is < MUTATION_PROBABILITY)
      {
        //mutate child
        //select node to replace

        //select graph node to replace it with
        //later: add neighborhood qualifier
      }

      //store child paths
      new_population.push_back(child);
    }
    all_populations.push_back(new_population);
    current_population = new_population;
  }
}

void generate_initial_paths(std::vector<GraphNode* > grid_pts, std::mt19937 &generator)
{
  if(!grid_pts.size()) return;

  for(int i = 0; i < NUM_PATHS; i++)
  {
    initial_seed.push_back(generate_path(grid_pts, generator));
    current_population.push_back(initial_seed[i]);
  }
  all_populations.push_back(current_population);
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
    entropy += node->get_value();
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
  static int n = 0;
  std::generate(probability_dist.begin(), probability_dist.end(),
                [&current_population]()
                {
                  return calc_path_entropy(current_population[n++]);
                });
  double entropy_normalizer = std::accumulate(probability_dist.begin(),
                                              probability_dist.end(), 0.0);
  std::transform(probability_dist.begin(), probability_dist.end(), probability_dist.begin(),
                 [&entropy_normalizer](double prob)
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
                    std::vector<GraphNode *> parentA, std::vector<GraphNode *> parentB)
{
  select_parent(probability_dist, dist, generator, parentA);
  select_parent(probability_dist, dist, generator, parentB);
}

void select_parent(std::vector<double> &probability_dist,
                   std::uniform_real_distribution<double> dist,
                   std::mt19937 &generator, std::vector<GraphNode *> parent) //is this the right way to pass it
{
  double prob;
  prob = dist(generator);
  double total_prob = 0.0;
  int i = 0;
  while(total_prob < prob)
  {
    total_prob += probability_dist[i++];
  }
  parent = current_population[i];
}


void crossover(std::vector<GraphNode *> &a, std::vector<GraphNode*> &b, std::mt19937 &generator,
               std::vector<GraphNode *> &child)
{
  std::uniform_int_distribution<int> dist(0, PATH_LENGTH);
  std::uniform_int_distribution<int> parent(0, 1);

  int crossover_pt = dist(generator);
  int parentBFirst = parent(generator);
  std::vector<GraphNode * > parent1, parent2;

  parent1 = (parentBFirst ? b : a);
  parent2 = (parentBFirst ? a : b);

  // copy up to crossover point
  std::copy_n(parent1.begin(), crossover_pt, std::back_inserter(child));
  // copy from second parent
  std::copy(parent2.begin() + crossover_pt, parent2.end(), std::back_inserter(child));
}


double distance_between(const GraphNode* a, const GraphNode* b)
{
  return sqrt(((a->get_location())(0) - (b->get_location())(0))^2 +
      (((a->get_location())(1) - (b->get_location())(1))^2));
}

// f = entropy + alpha * (1 - L where L is distance, scaled from 0 to 1)
// or L is summation of all distances, then normalized from 0 to 1
// f = sum( entropy + alpha *(1-L))


double normalize_graph_node(GraphNode* a, GraphNode* b)
{

  //take m_min_lon, m_max_lon etc from GP_AUV.h
  //weigh converting this to meters
  //using manhattan distance?
  //
  return 1;
}

double normalize_entropy(GraphNode *a)
{
  //minimum value: -1.3 (maybe -1.5)
  //maximum value: 5.3 (maybe 5.5 or 5.6)
  // (-2)-(6)
  // size of area doesn't matter
  // if data is in different range, then it could cause problems

  return 1;
}

