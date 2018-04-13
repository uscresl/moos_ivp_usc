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

#define INITPOP "INITIALIZE POPULATION \n\n"
#define CROSSOVER "CROSSOVER PAIRS \n\n"
#define SELECTPARENTS "SELECT PARENTS \n\n"
#define MUTATE "MUTATE PATHS \n\n"
#define SORTBYFITNESS "SORT PATHS BY FITNESS \n\n"
#define PROBABILITYDIST "CREATE PROBABILITY DISTRIBUTION \n\n"

double distance_between(const GraphNode* a, const GraphNode* b);
void select_parent(std::vector<double> &probability_dist,
                   std::uniform_real_distribution<double> dist,
                   std::mt19937 &generator, std::vector<GraphNode *> &parent);

double normalize_entropy(GraphNode *a);
double normalize_graph_node_distance(GraphNode* a, GraphNode* b);
bool in_current_path(std::vector<GraphNode *> path, GraphNode * mutation);

void genetic_pp_init(double max_lon, double min_lon, double max_lat, double min_lat,
                    double min_ent, double max_ent)
{
  //set normalizing variables
  m_lon_max = max_lon;
  m_lon_min = min_lon;
  m_lat_max = max_lat;
  m_lat_min = min_lat;
  min_entropy = min_ent;
  max_entropy = max_ent;

  entropy_normalizing_factor = 1.0 / (max_entropy - min_entropy);

  end_pt = NULL;

  //TODO track end of given path, not current location: we want to build path from where we WILL BE in the end
  //is this something relevant?
}

void run_genetic_pp(std::vector<GraphNode *> grid_pts)
{
  std::random_device rd;
  std::mt19937 generator(rd());
  set_end_pt(grid_pts);
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
      double crossover_is = crossover_mutate_dist(generator);
      if(crossover_is < CROSSOVER_PROBABILITY)
      {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        std::vector<GraphNode *> parentA;
        std::vector<GraphNode *> parentB;
        select_parents(probability_dist, dist, generator, parentA, parentB);

        crossover(parentA, parentB, generator, child);
      }
      double mutation_is = crossover_mutate_dist(generator);
      if(mutation_is < MUTATION_PROBABILITY)
      {
        //mutate child
        //select graph node to replace it with
        std::uniform_int_distribution<int> mutate(0, grid_pts.size() - 1);
        //select node to replace
        std::uniform_int_distribution<int> mutation_pt(1, PATH_LENGTH - 2);
        int mutation_index = mutate(generator);
        while(in_current_path(child, grid_pts[mutation_index]))
        {}
        child[mutation_pt(generator)] = grid_pts[mutation_index];

        //TODO later: add neighborhood qualifier
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
  print_current_population();
}

// ---------------------------------------------------------
// Procedure: generate_path()
//            go through a grid and select random points to
//            create a path of length PATH_LENGTH

std::vector<GraphNode *> generate_path(std::vector<GraphNode* > grid_pts, std::mt19937& gen)
{
  int max_grid_pt = grid_pts.size() - 1;
  std::uniform_int_distribution<int> dist(0, max_grid_pt);

  //set beginning pt to currect location
  //set end pt to last location
//  GraphNode * end_pt =

  std::unordered_map<int, GraphNode * > added_nodes;
  std::vector<GraphNode *> path;
  int i = 0;
  while(i < PATH_LENGTH)
  {
    int grid_index = dist(gen);
    if(added_nodes.find(grid_index) == added_nodes.end())
    {
      //add element to path
      added_nodes.insert( std::pair< int, GraphNode* >(grid_index, grid_pts[grid_index]) );
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
    entropy += normalize_entropy(node);
  }
  for(int i = 0; i < path.size()-1; i++)
  {
    entropy += 1000 /*some multiplier*/ * normalize_graph_node_distance(path[i], path[i+1]);
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
                [&]()
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
                    std::vector<GraphNode *> &parentA, std::vector<GraphNode *> &parentB)
{
  select_parent(probability_dist, dist, generator, parentA);
  select_parent(probability_dist, dist, generator, parentB);
}

void select_parent(std::vector<double> &probability_dist,
                   std::uniform_real_distribution<double> dist,
                   std::mt19937 &generator, std::vector<GraphNode *> &parent) //is this the right way to pass it
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


void crossover(const std::vector<GraphNode *> &a, const std::vector<GraphNode*> &b, std::mt19937& generator,
               std::vector<GraphNode *> &child)
{
  std::uniform_int_distribution<int> dist(1, PATH_LENGTH-1); //don't want to change beginning and end node
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
  return sqrt(pow((a->get_location())(0) - (b->get_location())(0), 2) +
      (pow((a->get_location())(1) - (b->get_location())(1), 2)));
}

// f = entropy + alpha * (1 - L where L is distance, scaled from 0 to 1)
// or L is summation of all distances, then normalized from 0 to 1
// f = sum( entropy + alpha *(1-L))


double normalize_graph_node_distance(GraphNode* a, GraphNode* b)
{

  //take m_min_lon, m_max_lon etc from GP_AUV.h
  //weigh converting this to meters
  //using manhattan distance?
  //
  double lon_delta = m_lon_max - m_lon_min;
  double lat_delta = m_lat_max - m_lat_min;
  double max_dist_to_travel = sqrt(pow(lon_delta, 2) + pow(lat_delta, 2));
  double normalizing_factor = 1.0 / max_dist_to_travel;

  double a_b_lon_delta, a_b_lat_delta;
  a_b_lon_delta = abs(a->get_location()(1) - b->get_location()(1));
  a_b_lat_delta = abs(a->get_location()(0) - b->get_location()(0));
  double dist_to_travel = sqrt(pow(a_b_lon_delta, 2) + pow(a_b_lat_delta, 2));

  return 1.0 - dist_to_travel * normalizing_factor; //maximize the lower distance
}

double normalize_entropy(GraphNode *a)
{
  //minimum value: -1.3 (maybe -1.5)
  //maximum value: 5.3 (maybe 5.5 or 5.6)
  // (-2)-(6)
  // size of area doesn't matter
  // if data is in different range, then it could cause problems

  return 1.0 - (a->get_value() * entropy_normalizing_factor); //maximize lower entropy
}

void set_end_pt(std::vector<GraphNode *> grid_pts)
{
  double max_entropy = (double)INT16_MIN;

  for(int i = 0; i < grid_pts.size(); i++)
  {
    if(grid_pts[i]->get_value() > max_entropy)
    {
      max_entropy = grid_pts[i]->get_value();
      end_pt = grid_pts[i];
    }
  }

  if(end_pt == NULL)
  {
    //error, grid is empty;
  }
}

bool in_current_path(std::vector<GraphNode *> path, GraphNode * mutation)
{
  for(int i = 0; i < path.size(); i++)
  {
    if(path[i] == mutation)
      return true;
  }
  return false;
}

void print_current_population()
{
  std::for_each(current_population.begin(), current_population.end(), [](std::vector<GraphNode *>&path) {
    std::for_each(path.begin(), path.end(), [](GraphNode* &node)
    {
      std::cout << "(" << node->get_location()(0) << "," << node->get_location()(1) << ") ";
    });
    std::cout << std::endl;
  });
}

