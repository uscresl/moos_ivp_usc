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
#include "GraphNode.h"
#include <vector>

std::ofstream filestream;

#define INITPOP "INITIALIZE POPULATION \n\n"
#define CROSSOVER "CROSSOVER PAIRS \n\n"
#define SELECTPARENTS "SELECT PARENTS \n\n"
#define MUTATE "MUTATE PATHS \n\n"
#define SORTBYFITNESS "SORT PATHS BY FITNESS \n\n"
#define PROBABILITYDIST "CREATE PROBABILITY DISTRIBUTION \n\n"
#define PWD "/Users/prachinawathe/Documents/resl/moos_ivp_usc/src/pGP_AUV/test_gen_pp/data/"

double distance_between(const GraphNode* a, const GraphNode* b);

void GenPP::genetic_pp_init(double max_lon, double min_lon, double max_lat, double min_lat,
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
  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );

  char buf[24];
  strftime (buf,24,"%Y-%m-%d-%H-%M-%S.txt",curtime);

  std::string time = PWD;
  time += "data";
  time.append(buf);
  filestream.open(time);


  //TODO track end of given path, not current location: we want to build path from where we WILL BE in the end
  //is this something relevant?
}

void GenPP::run_genetic_pp(std::vector<GraphNode *> grid_pts)
{
  std::random_device rd;
  std::mt19937 generator(rd());
  end_pt = grid_pts[grid_pts.size() - 1];
  generate_initial_paths(grid_pts, generator);

  for(int i = 0; i < NUM_GENERATIONS; i++)
  {
    // sort current population by fitness
//    std::sort(current_population.begin(), current_population.end(),
//              [&](std::vector<GraphNode *> a, std::vector<GraphNode *> b) {
//      return calc_path_entropy(a) > calc_path_entropy(b);
//    });

    // create probability distribution
    std::vector<double> probability_dist = create_prob_dist();


    // select x pairs of parents & crossover
    std::vector<std::vector<GraphNode *> > new_population;
    for(int j = 0; j < NUM_PATHS; j++)
    {
      std::vector<GraphNode *> child;
      std::uniform_real_distribution<double> crossover_mutate_dist(0.0, 1.0);
      double crossover_is = crossover_mutate_dist(generator);
      std::uniform_real_distribution<double> dist(0.0, 1.0);
      if(crossover_is < CROSSOVER_PROBABILITY)
      {
        std::vector<GraphNode *> parentA;
        std::vector<GraphNode *> parentB;
        select_parents(probability_dist, dist, generator, parentA, parentB);

        crossover(parentA, parentB, generator, child);
      }
      else
      {
        select_parent(probability_dist, dist, generator, child);
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
        {
          mutation_index = mutate(generator);
        }
        child[mutation_pt(generator)] = grid_pts[mutation_index];

        //TODO later: add neighborhood qualifier
      }

      //store child paths
      new_population.push_back(child);
    }
    all_populations.push_back(new_population);
    current_population = new_population;
    print_current_population("new population");
  }


  filestream.close();
}

void GenPP::generate_initial_paths(std::vector<GraphNode* > grid_pts, std::mt19937 &generator)
{
  if(!grid_pts.size()) return;

  for(int i = 0; i < NUM_PATHS; i++)
  {
    initial_seed.push_back(generate_path(grid_pts, generator));
    current_population.push_back(initial_seed[i]);
  }
  all_populations.push_back(current_population);
  print_current_population(INITPOP);
}

// ---------------------------------------------------------
// Procedure: generate_path()
//            go through a grid and select random points to
//            create a path of length PATH_LENGTH

std::vector<GraphNode *> GenPP::generate_path(std::vector<GraphNode* > grid_pts, std::mt19937& gen)
{
  int max_grid_pt = grid_pts.size() - 1;
  std::uniform_int_distribution<int> dist(0, max_grid_pt);

  //set beginning pt to currect location
  //set end pt to last location
//  GraphNode * end_pt =

  std::unordered_map<int, GraphNode * > added_nodes;
  std::vector<GraphNode *> path;
  int i = 1;
  path.push_back(grid_pts[0]);
  while(i < PATH_LENGTH - 1)
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
  path.push_back(end_pt);

  return path;
}

// ---------------------------------------------------------
// Procedure: calc_path_entropy()
//            go through a path and sum all normalized
//            entropies, sum all normalized edge differences
double GenPP::calc_path_entropy(const std::vector<GraphNode* >& path)
{
  double entropy = 0.0, distance = 0.0;
  for(int i = 0; i < path.size(); i++)
  {
    GraphNode *node = path[i];
    entropy += normalize_entropy(node);
  }
  for(int i = 0; i < path.size()-1; i++)
  {
    entropy +=  /*some multiplier*/ normalize_graph_node_distance(path[i], path[i+1]);
  }
  return entropy;
}

// ---------------------------------------------------------
// Procedure: create_prob_dist()
//            for a given population, create a probability
//            distribution depending on the fitness of each
//            path
std::vector<double> GenPP::create_prob_dist()
{
  //create probability distribution
  std::vector<double> probability_dist(NUM_PATHS);
  static int n = 0;
  for(int n = 0; n < current_population.size(); n++)
  {
    double entropy = 0.0, distance = 0.0;
    for(int i = 0; i < current_population[n].size(); i++)
    {
      GraphNode *node = current_population[n][i];
      entropy += normalize_entropy(node);
    }
    for(int i = 0; i < current_population[n].size()-1; i++)
    {
      entropy +=  /*some multiplier*/ normalize_graph_node_distance(current_population[n][i], current_population[n][i+1]);
    }
    n++;
    probability_dist[n] = entropy;
  }
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
void GenPP::select_parents(std::vector<double> &probability_dist,
                    std::uniform_real_distribution<double> dist,
                    std::mt19937 &generator,
                    std::vector<GraphNode *> &parentA, std::vector<GraphNode *> &parentB)
{
  select_parent(probability_dist, dist, generator, parentA);
  select_parent(probability_dist, dist, generator, parentB);
}

void GenPP::select_parent(std::vector<double> &probability_dist,
                   std::uniform_real_distribution<double> dist,
                   std::mt19937 &generator, std::vector<GraphNode *> &parent) //is this the right way to pass it
{
  double prob;
  prob = dist(generator);
  double total_prob = 0.0;
  int i = -1;
  while(total_prob < prob)
  {
    i++;
    total_prob += probability_dist[i];
  }
  parent = current_population[i];
}


void GenPP::crossover(const std::vector<GraphNode *> &a, const std::vector<GraphNode*> &b, std::mt19937& generator,
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


double GenPP::normalize_graph_node_distance(GraphNode* a, GraphNode* b)
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

double GenPP::normalize_entropy(GraphNode *a)
{
  //minimum value: -1.3 (maybe -1.5)
  //maximum value: 5.3 (maybe 5.5 or 5.6)
  // (-2)-(6)
  // size of area doesn't matter
  // if data is in different range, then it could cause problems

  return ((a->get_value()-min_entropy) * entropy_normalizing_factor); //maximize lower entropy
}

void GenPP::set_end_pt(std::vector<GraphNode *> grid_pts)
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

bool GenPP::in_current_path(std::vector<GraphNode *> path, GraphNode * mutation)
{
  for(int i = 0; i < path.size(); i++)
  {
    if(path[i] == mutation)
      return true;
  }
  return false;
}

void GenPP::print_current_population(std::string printstring)
{
  std::cout << printstring << std::endl;
  filestream << std::endl;
  std::for_each(current_population.begin(), current_population.end(), [](std::vector<GraphNode *>&path) {
    std::for_each(path.begin(), path.end(), [&](GraphNode* &node)
    {
      std::cout << "(" << node->get_location()(0) << "," << node->get_location()(1) << ") ";
      filestream << "(" << node->get_location()(0) << "," << node->get_location()(1) << ") ";
    });
    std::cout << std::endl;
    filestream << std::endl;
  });
}

