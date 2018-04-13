//
// Created by Prachi Nawathe on 4/5/18.
//

#include "gen_pp.h"

int main(int argc, char *argv[])
{
  genetic_pp_init(20, 0, 20, 0, -3, 6.5);

  //generate grid:
  std::vector<GraphNode *> grid_pts;
  for(int i = 0; i < 20; i++)
  {
    for(int j = 0; j < 20; j++)
    {
      Eigen::Vector2d vector2d(i,j);
      GraphNode* node = new GraphNode(vector2d, 1);
      grid_pts.push_back(node);
    }
  }

  //grid generated

  run_genetic_pp(grid_pts);

}
