#include "mex.h"
#include "project.h"
#include <cmath>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <sstream>


using namespace std;


struct CompareNode_Astar
{
    bool operator()(const treePath *n1, const treePath *n2) const
    {
        return (n1->cost + n1->hcost) > (n2->cost + n2->hcost);
    }
};

#include <vector>
#include <utility>
#include <algorithm>

using namespace std;

vector<pair<int, int>> getValidNeighbors(const pair<int, int>& loc, const vector<vector<char>>& grid) {
    if (grid.empty() || grid[0].empty()) {
        return {}; // Return an empty list if the grid is invalid.
    }

    vector<pair<int, int>> neighbors = {
        {loc.first - 1, loc.second}, // Up
        {loc.first, loc.second - 1}, // Left
        {loc.first + 1, loc.second}, // Down
        {loc.first, loc.second + 1}  // Right
    };

    neighbors.erase(
        remove_if(neighbors.begin(), neighbors.end(),
            [&grid](const pair<int, int>& p) {
                // Validate indices and check if the cell is traversable
                return p.first < 0 || p.first >= grid.size() ||
                       p.second < 0 || p.second >= grid[0].size() ||
                       grid[p.first][p.second] == '1' ||  // Assuming '1' is an obstacle
                       grid[p.first][p.second] == ' ';  // Assuming space is not valid
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

vector<vector<pair<int, int>>> astar( vector<pair<int, int>> startPoint, const pair<int, int> goal)
{

        // Attempt to open the grid file
    vector<vector<char>> grid;
    ifstream file("gridMap.txt");

    if (file.is_open()) {
        string line;
        while (getline(file, line)) {
            vector<char> row;
            istringstream iss(line);
            char cell;
            while (iss >> cell) { 
                row.push_back(cell);
            }
            grid.push_back(row);
        }
        file.close();
    } else {
        mexErrMsgIdAndTxt("AStar:FileError", "Failed to open the file 'gridMap.txt'. Ensure it's in the correct location.");
    }
    
    vector<vector<pair<int,int>>> locationCheck;
    vector<vector<pair<int,int>>> paths;
    paths.resize(startPoint.size());
    locationCheck.resize(startPoint.size());
    
    for(int i = 0; i < startPoint.size(); i++)
    {
        treePath* ans = new treePath;
        ans->path = {};
        ans->location = {0, 0};
        ans->cost = 0;
        ans->hcost = 0;

        unsigned int expandedNodes = 0;

   
        unordered_set<pair<int, int>, hashFunction> visited;


        if (!grid.empty())
        {

            treePath* root = new treePath;
            root->path.push_back(startPoint[i]);
            root->cost = 0;
            root->location = startPoint[i];
            root->hcost = heuristic(startPoint[i], goal);

    
            priority_queue<treePath*, vector<treePath*>, CompareNode_Astar> fringe;
            fringe.push(root);

            while (!fringe.empty())
            {
                treePath* stateToExpand = fringe.top();

                if(!locationCheck[i].empty()){    
                    for(int j = 0; j < locationCheck[i].size(); j++)
                    {
                        if(locationCheck[i][j] == stateToExpand->location)
                        {
                            stateToExpand->path.push_back(stateToExpand->location);
                            continue;
                        }
                    }
                }
                fringe.pop();

                locationCheck[i].push_back(stateToExpand->location);
                if (expandedNodes > 1e6) {
                    cerr << "Too many nodes expanded. Terminating to avoid excessive memory usage.\n";
                    break;
                }
                if (visited.count(stateToExpand->location) == 0)
                {
                    visited.insert(stateToExpand->location);
                    expandedNodes++;
                    

                    if (stateToExpand->location == goal)
                    {
                        ans->path = stateToExpand->path;
                        ans->cost = stateToExpand->cost;
                        ans->hcost = stateToExpand->hcost;
                        cout << "Agent " << i+1 << " result path size: " << ans->path.size() << endl;
                        delete stateToExpand;

                        
                        while (!fringe.empty())
                        {
                            treePath* toDelete = fringe.top();
                            fringe.pop();
                            delete toDelete;
                        }
                        printf("Expanded nodes: %d\n", expandedNodes);

                        paths[i].resize(ans->path.size());
                        for (int k = 0; k < ans->path.size(); k++)
                        {
                            paths[i][k]= ans->path[k];
                        }
                    }
                    else
                    {

                        vector<pair<int, int>> neighbors = getValidNeighbors(stateToExpand->location, grid);

                        for (int j = 0; j < neighbors.size(); j++)
                        {
                            if (visited.count(neighbors[j]) == 0)
                            {

                                treePath* states = new treePath;
                                states->path = stateToExpand->path;
                                states->path.push_back(neighbors[j]);
                                states->cost = stateToExpand->cost + 1;
                                states->location = neighbors[j];
                                states->hcost = heuristic(states->location, goal);
                                fringe.push(states);
                            }
                        }
                        delete stateToExpand;
                    }
                }
            }
        }
        
    }
    return paths;
}

bool isPointsValid(vector<vector<char>>& grid, vector<pair<int, int>> start, pair<int, int> goal)
{
    for (int i = 0; i < start.size(); i++)
    {
        if (start[i].first < 0 || start[i].first >= grid.size() || start[i].second < 0 || start[i].second >= grid[0].size())
        {
            return false;
        }
        if (goal.first < 0 || goal.first >= grid.size() || goal.second < 0 || goal.second >= grid[0].size())
        {
            return false;
        }
        if (grid[start[i].first][start[i].second] == '1' || grid[goal.first][goal.second] == '1')
        {
            return false;
        } 
    }
    return true;
}
/*
int main()
{
    vector<vector<char>> grid;

    ifstream file("gridMap.txt");

    if (file.is_open()) {
        string line;
        while (getline(file, line)) {
            vector<char> row;
            istringstream iss(line);
            char cell;
            while (iss >> cell) { 
                row.push_back(cell);
            }
            grid.push_back(row);
        }
        file.close();
    } else {
        cerr << "Failed to open the file.\n";
    }


    vector<pair<int, int> > start = {{0, 99}, {2, 2}, {96, 3}};
    pair<int, int> goal = {97, 95};

    if (!isPointsValid(grid, start, goal))
    {
        cout << "Invalid start or goal points" << endl;
        return 0;
    }

    vector<vector<pair<int, int>>> paths = astar(grid, start, goal);
    for (int i = 0; i < paths.size(); i++)
    {
        if(paths[i].empty())
        {
            cout << "No path found for agent:"<< i+1 << endl;
        }
    }
    
    cout << "path 1 size:"<< paths[0].size() << endl;
    cout << "path 2 size:"<< paths[1].size() << endl;
    cout << "path 3 size:"<< paths[2].size() << endl;
    for (int i = 0; i < grid.size(); i++)
    {
        for (int j = 0; j < grid[i].size(); j++)
        {
            cout << grid[i][j] << " ";
        }
        cout << endl;
    }

}
*/

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) { // Only start points and goal are required as inputs
        mexErrMsgIdAndTxt("AStar:InvalidInput", "Two inputs required: start points, goal.");
    }
    

    // Convert start points to a vector of pairs
    size_t numStartPoints = mxGetM(prhs[0]);
    size_t startPointCols = mxGetN(prhs[0]);
    if (startPointCols != 2) {
        mexErrMsgIdAndTxt("AStar:InvalidInput", "Start points must be an Nx2 array.");
    }

    double* startData = mxGetPr(prhs[0]);
    vector<pair<int, int>> startPoints;
    for (size_t i = 0; i < numStartPoints; ++i) {
        int x = static_cast<int>(startData[i]);              // Row index
        int y = static_cast<int>(startData[i + numStartPoints]); // Column index
        startPoints.emplace_back(x, y);
    }

    // Convert goal point to a pair
    double* goalData = mxGetPr(prhs[1]);
    pair<int, int> goalPoint = {static_cast<int>(goalData[0]), static_cast<int>(goalData[1])};

    // Run A* algorithm
    vector<vector<pair<int, int>>> paths = astar(startPoints, goalPoint);

    // Convert paths to MATLAB output
    plhs[0] = mxCreateCellMatrix(numStartPoints, 1);
    for (size_t i = 0; i < numStartPoints; ++i) {
        const vector<pair<int, int>>& path = paths[i];
        size_t pathSize = path.size();

        if (path.empty()) {
            // Add an empty array for this drone
            mxSetCell(plhs[0], i, mxCreateDoubleMatrix(0, 0, mxREAL));
        } else {
            // Create a MATLAB matrix for the path
            mxArray* pathArray = mxCreateDoubleMatrix(pathSize, 2, mxREAL);
            double* pathData = mxGetPr(pathArray);
            for (size_t j = 0; j < pathSize; ++j) {
                pathData[j] = path[j].first + 1;        // Row index (1-based for MATLAB)
                pathData[j + pathSize] = path[j].second + 1; // Column index
            }
            mxSetCell(plhs[0], i, pathArray);
        }
    }
}
