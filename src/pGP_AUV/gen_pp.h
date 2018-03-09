//
// Created by Prachi Nawathe on 3/1/18.
//

#ifndef IVP_EXTEND_GEN_PP_H
#define IVP_EXTEND_GEN_PP_H

#define NUM_PATHS 10
#define PATH_LENGTH 20
#define NUM_GENERATIONS 5


//do we need to worry about deep copies
std::vector<std::vector<GraphNode *> > initial_seed;
std::vector<std::vector<GraphNode *> > current_population;
std::unordered_map< std::vector<GraphNode *>, double> current_pop_fitness;

void run_genetic_pp(std::vector< GraphNode*> * grid_pts);
void generate_initial_paths(std::vector< GraphNode*> grid_pts);
std::vector<GraphNode> generate_path(std::vector< GraphNode*> grid_pts, std::mt19937& gen);
double calc_path_entropy(std::vector< GraphNode*> path);
void crossover();
void mutate(); //not sure if necessary
void select_parents(std::vector<double> &probability_dist,
                    std::uniform_real_distribution<double> dist,
                    std::mt19937 &generator,p
                    GraphNode *parentA, GraphNode *parentB);
void create_prob_dist(std::vector<double> &probability_dist);


#endif //IVP_EXTEND_GEN_PP_H
