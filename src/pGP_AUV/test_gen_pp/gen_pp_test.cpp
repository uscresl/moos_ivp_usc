//
// Created by Prachi Nawathe on 4/5/18.
//

#include "gen_pp.h"
#include <iostream>
#include <vector>

int main(int argc, char *argv[])
{
  GenPP genPP;
  genPP.genetic_pp_init(20, 0, 20, 0, .5, 1);

  std::vector<int> test_vec;
  test_vec.push_back(0);
  test_vec.push_back(1);

  //generate grid:
  std::vector<GraphNode *> grid_pts;
  for(int i = 0; i < 20; i++)
  {
    for(int j = 0; j < 20; j++)
    {
      Eigen::Vector2d vector2d(i,j);
      GraphNode* node;
      if(j >= 10 && j < 15 && i >= 10 && j < 15) {
        node = new GraphNode(vector2d, 1);
      }
      else
      {
        node = new GraphNode(vector2d, .5);
      }
      grid_pts.push_back(node);
    }
  }

  //grid generated

  genPP.run_genetic_pp(grid_pts);

}
