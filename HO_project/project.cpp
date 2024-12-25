#include "mex.h"
#include "project.h"
#include <cmath>
#include <unordered_set>
#include <unordered_map>
#include <queue>
using namespace std;

// Comparison method used for prioritry queue (min heap) on the overall cost for A*
struct CompareNode_Astar
{
    bool operator()(const treePath *n1, const treePath *n2) const
    {
        return (n1->cost + n1->hcost) > (n2->cost + n2->hcost);
    }
};

vector<pair<int, int>> getValidNeighbors(const pair<int, int>& loc, const vector<vector<char>>& grid) {
    vector<pair<int, int>> neighbors = {
        {loc.first - 1, loc.second},
        {loc.first, loc.second - 1},
        {loc.first + 1, loc.second},
        {loc.first, loc.second + 1}
    };

    neighbors.erase(
        remove_if(neighbors.begin(), neighbors.end(),
            [&grid](const pair<int, int>& p) {
                return p.first >= grid.size() || 
                       p.second >= grid[0].size() ||
                       p.first < 0 || 
                       p.second < 0 ||
                       grid[p.first][p.second] == '1'; // obstacles
            }), 
        neighbors.end()
    );

    return neighbors;
}


double heuristic(const pair<int,int>& a, const pair<int,int>& b)
{
    //eucilean distance
    return sqrt(pow((b.second - a.second),2) + pow((b.first - a.first),2));
}

struct hashFunction 
{ 
  size_t operator()(const pair<int,int> &x) const
  { 
    return x.first ^ x.second; 
  } 
}; 

vector<pair<int, int>> astar(vector<vector<char>>& grid, const pair<int, int> startPoint, const pair<int, int> goal)
{
    // Initialize the answer structure
    treePath* ans = new treePath;
    ans->path = {};
    ans->location = {0, 0};
    ans->cost = 0;
    ans->hcost = 0;

    // Expanded nodes counter (optional, for debugging)
    unsigned int expandedNodes = 0;

    // Set to track visited nodes
    unordered_set<pair<int, int>, hashFunction> visited;

    // Check for valid grid
    if (!grid.empty())
    {
        // Initialize the root node
        treePath* root = new treePath;
        root->path.push_back(startPoint);
        root->cost = 0;
        root->location = startPoint;
        root->hcost = heuristic(startPoint, goal);

        // Priority queue for the fringe
        priority_queue<treePath*, vector<treePath*>, CompareNode_Astar> fringe;
        fringe.push(root);

        while (!fringe.empty())
        {
            treePath* stateToExpand = fringe.top();
            fringe.pop();

            if (visited.count(stateToExpand->location) == 0)
            {
                visited.insert(stateToExpand->location);
                expandedNodes++;
            }

            // Check if goal is reached
            if (stateToExpand->location == goal)
            {
                ans->path = stateToExpand->path;
                ans->cost = stateToExpand->cost;
                ans->hcost = stateToExpand->hcost;
                delete stateToExpand;

                // Clean up remaining nodes in the fringe
                while (!fringe.empty())
                {
                    treePath* toDelete = fringe.top();
                    fringe.pop();
                    delete toDelete;
                }
                return ans->path; // Return the path to the goal
            }
            else
            {
                // Get valid neighbors
                vector<pair<int, int>> neighbors = getValidNeighbors(stateToExpand->location, grid);

                for (int i = 0; i < neighbors.size(); i++)
                {
                    if (visited.count(neighbors[i]) == 0)
                    {
                        // Add neighbors to the fringe
                        treePath* states = new treePath;
                        states->path = stateToExpand->path;
                        states->path.push_back(neighbors[i]);
                        states->cost = stateToExpand->cost + 1;
                        states->location = neighbors[i];
                        states->hcost = heuristic(states->location, goal);
                        fringe.push(states);
                    }
                }
                delete stateToExpand; // Deallocate the expanded node
            }
        }
    }

    // If the fringe is empty and goal not reached, return an empty path
    return {}; // No path found
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 3) {
        mexErrMsgIdAndTxt("AStar:InvalidInput", "Three inputs required: grid, start, goal.");
    }

    // Convert grid to C++ vector
    size_t rows = mxGetM(prhs[0]);
    size_t cols = mxGetN(prhs[0]);
    double* gridData = mxGetPr(prhs[0]);
    vector<vector<char>> grid(rows, vector<char>(cols));
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            grid[i][j] = (gridData[i + j * rows] == 1) ? '1' : '0';
        }
    }

    // Convert start and goal to pairs
    double* startData = mxGetPr(prhs[1]);
    double* goalData = mxGetPr(prhs[2]);
    pair<int, int> startPoint = {static_cast<int>(startData[0]), static_cast<int>(startData[1])};
    pair<int, int> goalPoint = {static_cast<int>(goalData[0]), static_cast<int>(goalData[1])};

    // Run A* algorithm
    vector<pair<int, int>> path = astar(grid, startPoint, goalPoint);

    // Convert path to MATLAB output
    if (path.empty()) {
        // Return an empty matrix if no path is found
        plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
    } else {
        // Return the found path
        plhs[0] = mxCreateDoubleMatrix(path.size(), 2, mxREAL);
        double* pathData = mxGetPr(plhs[0]);
        for (size_t i = 0; i < path.size(); ++i) {
            pathData[i] = path[i].first;
            pathData[i + path.size()] = path[i].second;
        }
    }
}
