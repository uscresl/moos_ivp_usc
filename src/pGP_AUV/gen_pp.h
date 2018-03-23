//
// Created by Prachi Nawathe on 3/1/18.
//

#ifndef IVP_EXTEND_GEN_PP_H
#define IVP_EXTEND_GEN_PP_H

#include <iostream>
#include <vector>
#include <random>
#include <unordered_map>
#include <algorithm>
#include "GraphNode.h"


#define NUM_PATHS 10
#define PATH_LENGTH 20
#define NUM_GENERATIONS 5
#define CROSSOVER_PROBABILITY .9
#define MUTATION_PROBABILITY .2


double m_lon_max, m_lon_min, m_lat_max, m_lat_min;
double min_entropy, max_entropy;
double entropy_normalizing_factor;
GraphNode *end_pt;

//struct char_seq_hash
//{
//  std::size_t operator() ( const std::vector<GraphNode *> & vec ) const
//  { return std::hash<std::vector<GraphNode*> >(vec.begin(), vec.end()); }
//};

void genetic_pp_init(double max_lon, double min_lon, double max_lat, double min_lat);
void run_genetic_pp(std::vector< GraphNode* > * grid_pts);
void generate_initial_paths(std::vector< GraphNode* > grid_pts, std::mt19937 &generator);
std::vector<GraphNode *> generate_path(std::vector< GraphNode* > grid_pts, std::mt19937& gen);
double calc_path_entropy(std::vector< GraphNode* > path);
void crossover(const std::vector<GraphNode *> &a, const std::vector<GraphNode *> &b, std::mt19937 &generator,
               std::vector<GraphNode *> &child);
void mutate();
void select_parents(std::vector<double> &probability_dist,
                    std::uniform_real_distribution<double> dist, std::mt19937 &generator,
                    std::vector<GraphNode *> &parentA, std::vector<GraphNode *> &parentB);
std::vector<double> create_prob_dist(std::vector<double> &probability_dist);

void set_end_pt(std::vector<GraphNode *> grid_pts);

std::vector<std::vector<GraphNode *> > initial_seed;
std::vector<std::vector<GraphNode *> > current_population;
std::vector< std::vector< std::vector<GraphNode *> > > all_populations;
//std::unordered_map< std::vector<GraphNode *>, double> current_pop_fitness;


#endif //IVP_EXTEND_GEN_PP_H
