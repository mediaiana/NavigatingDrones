#include <vector>
using namespace std;

struct treePath
{
    vector<pair<int,int>> path;
    pair<int,int> location;
    double cost;
    double hcost;
};

vector<pair<int,int>> astar(vector<vector<int>>& grid, const pair<int,int> startPoint,const pair<int,int> goal, double heuristic);

double heuristic(const pair<int,int>& a, const pair<int,int>& b);


vector<pair<int, int>> getValidNeighbors(const pair<int, int>& loc, const vector<vector<char>>& grid);
