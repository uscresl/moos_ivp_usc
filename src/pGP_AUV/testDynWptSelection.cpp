#include <iostream>
#include <vector>
#include <numeric>

volatile int grid[5][5];
volatile int direction[5][5];

std::vector<int> maxPath(int val, std::vector<int> nodes, std::vector<int>& path, int steps);
std::vector<int> max(std::vector<int> a, std::vector<int> b, std::vector<int> c, std::vector<int> d);

int main() {

    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++)
        {
            grid[i][j] = i*5 + j;
        }
    }
    std::vector<int> nodes;
    std::vector<int> path;
    nodes = maxPath(grid[0][0], nodes, path, 0);

    for(auto i : path)
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    return 0;
}

std::vector<int> maxPath(int val, std::vector<int> nodes, std::vector<int>& path, int steps)
{
//    std::cout << "current val: " << val << " steps: " << steps << std::endl;
    int repeat = std::count(nodes.begin(), nodes.end(), val);
    if(repeat > 0)
    {
        return std::vector<int>();
    }
    if(val < 0 || val > 24)
    {
        return std::vector<int>();
    }
    if(steps == 6)
    {
        if(std::accumulate(nodes.begin(), nodes.end(), 0) > std::accumulate(path.begin(), path.end(), 0))
        {
            path = nodes;
        }
        return nodes;
    }
    else
    {
        int i = val / 5, j = val % 5;
        int left = (i-1) >= 0 ? grid[i-1][j] : -1;
        int right = (i+1) <= 4 ? grid[i+1][j] : -1;
        int up = (j-1) >= 0 ? grid[i][j-1] : -1;
        int down = (j+1) <= 4 ? grid[i][j+1] : -1;

//        std::cout << "value: " << val << " " << left << " " << right << " " << up << " " << down << std::endl;


        nodes.push_back(val);
        std::for_each(nodes.begin(), nodes.end(), [](int &a) {
            std::cout << " " << a;
        });
        std::cout << std::endl;

        return max(maxPath(left, nodes, path, steps+1), maxPath(right, nodes, path, steps+1),
                                                maxPath(up, nodes, path, steps+1),
                                                maxPath(down, nodes, path, steps+1));
    }
}

std::vector<int> max(std::vector<int> a, std::vector<int> b, std::vector<int> c, std::vector<int> d)
{
    //std::for_each, vec.push_back(accumulate), take max and find index in vec.

    //getmax of sum of nodes
    int aSum, bSum, cSum, dSum;

    int max = std::max(std::accumulate(a.begin(), a.end(), aSum),
    std::max(std::accumulate(b.begin(), b.end(), bSum),
    std::max(std::accumulate(c.begin(), c.end(), cSum), std::accumulate(d.begin(), d.end(), dSum))));

    return max == aSum ? a : (max == bSum ? b : (max == cSum ? c : d));
}
