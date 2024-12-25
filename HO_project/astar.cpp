#include <iostream>
#include <list>
#include <utility>
#include <queue>
#include <stack>
#include <iterator>
#include <unordered_set>
#include <unordered_map>
using namespace std;

// Data structure for a node of the graph
struct GraphNode
{
    string name;                                 // name of the node (in our case, a Romanian city)
    list<pair<unsigned int, GraphNode *> > links; // list of arcs, roads to other cities, identified as pair "cost - connected node"
};

// Data structure for a node of the search tree, which will contain a whole path, not just a city
struct TreeNode
{
    GraphNode *node;    // current node
    string path;        // path with all the cities reached so far
    unsigned int cost;  // current overall cost of this path g(n)
    unsigned int hCost; // current overall cost of this path h(n)
    unsigned int depth; // depth of the node in the tree (only for iterative deepening)
};

// Comparison method used for prioritry queue (min heap) on the overall cost for USC
struct CompareNode
{
    bool operator() (const TreeNode* n1, const TreeNode* n2) const
    {
        return n1->cost > n2->cost;
    }
};

// Comparison method used for prioritry queue (min heap) on the overall cost for A*
struct CompareNode_Astar
{
    bool operator()(const TreeNode *n1, const TreeNode *n2) const
    {
        return (n1->cost + n1->hCost) > (n2->cost + n2->hCost);
    }
};

// Comparison method used for prioritry queue (min heap) on the overall cost for Greedy
struct CompareNode_Greedy
{
    bool operator()(const TreeNode *n1, const TreeNode *n2) const
    {
        return n1->hCost > n2->hCost;
    }
};

// Solve the problem with BFS allowing cycles
TreeNode* solveBFS_withCycles(GraphNode* graph, string goal)
{
    cout << "Breadth First Search (With Cycles)" << endl;

    TreeNode* ans = new TreeNode;
    ans->node = nullptr;
    ans->cost = 0;
    ans->path = "NO SOLUTION";

    unsigned int expandedNodes = 0;

    if(graph != nullptr)
    {
        // Root of the State Tree is the starting state of the State Graph
        TreeNode* root = new TreeNode;
        root->node = graph;
        root->path = graph->name;
        root->cost = 0;

        queue<TreeNode*> fringe;
        fringe.push(root);
        
        while(!fringe.empty())
        {
            TreeNode* stateToExpand = fringe.front();
            fringe.pop();

            expandedNodes++;    // Increase counter for node who have been expanded

            // Goal reached
            if(stateToExpand->node->name == goal)
            {
                ans->node = stateToExpand->node;
                ans->cost = stateToExpand->cost;
                ans->path = stateToExpand->path;
                delete stateToExpand;
                // delete all nodes in fringe
                while(!fringe.empty())
                {
                    TreeNode* toDelete = fringe.front();
                    fringe.pop();
                    delete toDelete;
                }
                break;  // if the node is reached, break the loop and exit the function
            }
            else
            {
                // Expand nodes by adding all its links to the fringe
                list<pair<unsigned int, GraphNode *> >::iterator it;
                for(it = stateToExpand->node->links.begin(); it != stateToExpand->node->links.end(); ++it)
                {
                    TreeNode* state = new TreeNode;                                 // create new tree node to be added in the fringe
                    state->node = it->second;                                       // save the node from the iterator
                    state->path = stateToExpand->path + ", " + it->second->name;    // update the path with new name
                    state->cost = it->first + stateToExpand->cost;                  // update overall cost
                    fringe.push(state);                                             // add new tree node in the fringe
                }
                delete stateToExpand;   // deallocate the node that was just expanded
            }
        }
    }

    cout << "Expanded nodes: " << expandedNodes << endl;

    return ans;
}

// Solve the problem with BFS without cycles
TreeNode* solveBFS(GraphNode* graph, string goal)
{
    cout << "Breadth First Search" << endl;

    TreeNode* ans = new TreeNode;
    ans->node = nullptr;
    ans->cost = 0;
    ans->path = "NO SOLUTION";

    unsigned int expandedNodes = 0;
    unordered_set<GraphNode*> visited;  // hash set to check which nodes have been visited

    if(graph != nullptr)
    {
        // Root of the State Tree is the starting state of the State Graph
        TreeNode* root = new TreeNode;
        root->node = graph;
        root->path = graph->name;
        root->cost = 0;

        queue<TreeNode*> fringe;
        fringe.push(root);
        
        while(!fringe.empty())
        {
            TreeNode* stateToExpand = fringe.front();
            fringe.pop();

            if (visited.count(stateToExpand->node) == 0)
            {
                visited.insert(stateToExpand->node);

                expandedNodes++;    // Increase counter for node who have been expanded

                // Goal reached
                if(stateToExpand->node->name == goal)
                {
                    ans->node = stateToExpand->node;
                    ans->cost = stateToExpand->cost;
                    ans->path = stateToExpand->path;
                    delete stateToExpand;
                    // delete all nodes in fringe
                    while(!fringe.empty())
                    {
                        TreeNode* toDelete = fringe.front();
                        fringe.pop();
                        delete toDelete;
                    }
                    break;  // if the node is reached, break the loop and exit the function
                }
                else
                {
                    // Expand nodes by adding all its links to the fringe
                    list<pair<unsigned int, GraphNode *> >::iterator it;
                    for(it = stateToExpand->node->links.begin(); it != stateToExpand->node->links.end(); ++it)
                    {
                        TreeNode* state = new TreeNode;                                 // create new tree node to be added in the fringe
                        state->node = it->second;                                       // save the node from the iterator
                        state->path = stateToExpand->path + ", " + it->second->name;    // update the path with new name
                        state->cost = it->first + stateToExpand->cost;                  // update overall cost
                        fringe.push(state);                                             // add new tree node in the fringe
                    }
                    delete stateToExpand;   // deallocate the node that was just expanded
                }
            }
        }
    }

    cout << "Expanded nodes: " << expandedNodes << endl;

    return ans;
}

// Solve the problem with DFS without cycles
TreeNode* solveDFS(GraphNode* graph, string goal)
{
    cout << "Depth First Search" << endl;

    TreeNode* ans = new TreeNode;
    ans->node = nullptr;
    ans->cost = 0;
    ans->path = "NO SOLUTION";

    unsigned int expandedNodes = 0;
    unordered_set<GraphNode*> visited;  // hash set to check which nodes have been visited

    if(graph != nullptr)
    {
        // Root of the State Tree is the starting state of the State Graph
        TreeNode* root = new TreeNode;
        root->node = graph;
        root->path = graph->name;
        root->cost = 0;

        stack<TreeNode*> fringe;
        fringe.push(root);
        
        while(!fringe.empty())
        {
            TreeNode* stateToExpand = fringe.top();
            fringe.pop();

            if (visited.count(stateToExpand->node) == 0)
            {
                visited.insert(stateToExpand->node);

                expandedNodes++;    // Increase counter for node who have been expanded

                // Goal reached
                if(stateToExpand->node->name == goal)
                {
                    ans->node = stateToExpand->node;
                    ans->cost = stateToExpand->cost;
                    ans->path = stateToExpand->path;
                    delete stateToExpand;
                    // delete all nodes in fringe
                    while(!fringe.empty())
                    {
                        TreeNode* toDelete = fringe.top();
                        fringe.pop();
                        delete toDelete;
                    }
                    break;  // if the node is reached, break the loop and exit the function
                }
                else
                {
                    // Expand nodes by adding all its links to the fringe
                    list<pair<unsigned int, GraphNode *> >::iterator it;
                    for(it = stateToExpand->node->links.begin(); it != stateToExpand->node->links.end(); ++it)
                    {
                        TreeNode* state = new TreeNode;                                 // create new tree node to be added in the fringe
                        state->node = it->second;                                       // save the node from the iterator
                        state->path = stateToExpand->path + ", " + it->second->name;    // update the path with new name
                        state->cost = it->first + stateToExpand->cost;                  // update overall cost
                        fringe.push(state);                                             // add new tree node in the fringe
                    }
                    delete stateToExpand;   // deallocate the node that was just expanded
                }
            }
        }
    }

    cout << "Expanded nodes: " << expandedNodes << endl;

    return ans;
}

// Solve the problem with Iterative DS without cycles
TreeNode* solveIDS(GraphNode *graph, string goal, unsigned int maxDepth)
{
    cout << "Iterative Deepening Search" << endl;

    TreeNode* ans = new TreeNode;
    ans->node = nullptr;
    ans->cost = 0;
    ans->depth = 0;
    ans->path = "NO SOLUTION";

    unsigned int expandedNodes = 0;

    if (graph != nullptr)
    {
        for (int d = 0; d < maxDepth; d++)
        {
            unordered_set<GraphNode*> visited;

            // Root of the State Tree is the starting state of the State Graph
            TreeNode* root = new TreeNode;
            root->node = graph;
            root->path = graph->name;
            root->cost = 0;
            root->depth = 0;

            stack<TreeNode *> fringe;
            fringe.push(root);

            while (!fringe.empty())
            {
                TreeNode *stateToExpand = fringe.top();
                fringe.pop();

                if (stateToExpand->depth < d)
                {
                    if (visited.count(stateToExpand->node) == 0)
                    {
                        visited.insert(stateToExpand->node);

                        // Increase counter for node who have been expanded
                        expandedNodes++;

                        // Goal reached
                        if (stateToExpand->node->name == goal)
                        {
                            ans->node = stateToExpand->node;
                            ans->cost = stateToExpand->cost;
                            ans->path = stateToExpand->path;
                            ans->depth = stateToExpand->depth;

                            // delete all nodes in fringe
                            delete stateToExpand;
                            while (!fringe.empty())
                            {
                                TreeNode *toDelete = fringe.top();
                                fringe.pop();
                                delete toDelete;
                            }
                            cout << "Node expanded: " << expandedNodes << endl;
                            return ans;
                        }
                        // Expand nodes by adding all its links to the fringe
                        list<pair<unsigned int, GraphNode *> >::iterator it;
                        for (it = stateToExpand->node->links.begin(); it != stateToExpand->node->links.end(); ++it)
                        {
                            TreeNode *state = new TreeNode;                                         // create new tree node to be added in the fringe
                            state->node = it->second;                                               // save the node from the iterator
                            state->path = stateToExpand->path + ", " + state->node->name;           // update the path with new name
                            state->cost = stateToExpand->cost + it->first;                          // update overall cost
                            state->depth = stateToExpand->depth + 1;                                // update depth of the node
                            fringe.push(state);                                                     // add new tree node in the fringe
                        }
                    }
                }
                delete stateToExpand;
            }
        }
    }
    cout << "Node expanded: " << expandedNodes << endl;
    return ans;
}

TreeNode* solveBidirectionalSearch(GraphNode *graphForwards, GraphNode *graphBackwards)
{
    // Did Fabio forget to implement this? 
    // Or did Fabio puprosely left this empty such that you could show it to him and perhaps get a bonus point? 
    // Only one way to find out.
    return nullptr;
}

// Solve the problem with UCS without cycles
TreeNode* solveUCS(GraphNode* graph, string goal)
{
    cout << "Uniform Cost Search" << endl;

    TreeNode* ans = new TreeNode;
    ans->node = nullptr;
    ans->cost = 0;
    ans->path = "NO SOLUTION";

    unsigned int expandedNodes = 0;
    unordered_set<GraphNode*> visited;  // hash set to check which nodes have been visited

    if(graph != nullptr)
    {
        // Root of the State Tree is the starting state of the State Graph
        TreeNode* root = new TreeNode;
        root->node = graph;
        root->path = graph->name;
        root->cost = 0;

        priority_queue<TreeNode*, vector<TreeNode*>, CompareNode> fringe;
        fringe.push(root);
        
        while(!fringe.empty())
        {
            TreeNode* stateToExpand = fringe.top();
            fringe.pop();

            if (visited.count(stateToExpand->node) == 0)
            {
                visited.insert(stateToExpand->node);

                expandedNodes++;    // Increase counter for node who have been expanded

                // Goal reached
                if(stateToExpand->node->name == goal)
                {
                    ans->node = stateToExpand->node;
                    ans->cost = stateToExpand->cost;
                    ans->path = stateToExpand->path;
                    delete stateToExpand;
                    // delete all nodes in fringe
                    while(!fringe.empty())
                    {
                        TreeNode* toDelete = fringe.top();
                        fringe.pop();
                        delete toDelete;
                    }
                    break;  // if the node is reached, break the loop and exit the function
                }
                else
                {
                    // Expand nodes by adding all its links to the fringe
                    list<pair<unsigned int, GraphNode *> >::iterator it;
                    for(it = stateToExpand->node->links.begin(); it != stateToExpand->node->links.end(); ++it)
                    {
                        TreeNode* state = new TreeNode;                                 // create new tree node to be added in the fringe
                        state->node = it->second;                                       // save the node from the iterator
                        state->path = stateToExpand->path + ", " + it->second->name;    // update the path with new name
                        state->cost = it->first + stateToExpand->cost;                  // update overall cost
                        fringe.push(state);                                             // add new tree node in the fringe
                    }
                    delete stateToExpand;   // deallocate the node that was just expanded
                }
            }
        }
    }

    cout << "Expanded nodes: " << expandedNodes << endl;

    return ans;
}

// Solve the problem with Greedy Search without cycles
TreeNode* solveGreedy(GraphNode* graph, string goal, unordered_map<string, unsigned int> &heuristic)
{
    cout << "Greedy Search" << endl;

    TreeNode* ans = new TreeNode;
    ans->node = nullptr;
    ans->cost = 0;
    ans->hCost = 0;
    ans->path = "NO SOLUTION";

    unsigned int expandedNodes = 0;
    unordered_set<GraphNode*> visited;  // hash set to check which nodes have been visited

    if(graph != nullptr)
    {
        // Root of the State Tree is the starting state of the State Graph
        TreeNode* root = new TreeNode;
        root->node = graph;
        root->path = graph->name;
        root->cost = 0;
        root->hCost = heuristic.at(root->node->name);

        priority_queue<TreeNode*, vector<TreeNode*>, CompareNode_Greedy> fringe;
        fringe.push(root);
        
        while(!fringe.empty())
        {
            TreeNode* stateToExpand = fringe.top();
            fringe.pop();

            if (visited.count(stateToExpand->node) == 0)
            {
                visited.insert(stateToExpand->node);

                expandedNodes++;    // Increase counter for node who have been expanded

                // Goal reached
                if(stateToExpand->node->name == goal)
                {
                    ans->node = stateToExpand->node;
                    ans->cost = stateToExpand->cost;
                    ans->path = stateToExpand->path;
                    delete stateToExpand;
                    // delete all nodes in fringe
                    while(!fringe.empty())
                    {
                        TreeNode* toDelete = fringe.top();
                        fringe.pop();
                        delete toDelete;
                    }
                    break;  // if the node is reached, break the loop and exit the function
                }
                else
                {
                    // Expand nodes by adding all its links to the fringe
                    list<pair<unsigned int, GraphNode *> >::iterator it;
                    for(it = stateToExpand->node->links.begin(); it != stateToExpand->node->links.end(); ++it)
                    {
                        TreeNode* state = new TreeNode;                                 // create new tree node to be added in the fringe
                        state->node = it->second;                                       // save the node from the iterator
                        state->path = stateToExpand->path + ", " + it->second->name;    // update the path with new name
                        state->cost = it->first + stateToExpand->cost;                  // update overall cost
                        state->hCost = heuristic.at(state->node->name);                 // update overall cost
                        fringe.push(state);                                             // add new tree node in the fringe
                    }
                    delete stateToExpand;   // deallocate the node that was just expanded
                }
            }
        }
    }

    cout << "Expanded nodes: " << expandedNodes << endl;

    return ans;
}

// Solve the problem with Greedy Search without cycles
TreeNode* solveAstar(GraphNode* graph, string goal, unordered_map<string, unsigned int> &heuristic)
{
    cout << "A* Search" << endl;

    TreeNode* ans = new TreeNode;
    ans->node = nullptr;
    ans->cost = 0;
    ans->hCost = 0;
    ans->path = "NO SOLUTION";

    unsigned int expandedNodes = 0;
    unordered_set<GraphNode*> visited;  // hash set to check which nodes have been visited

    if(graph != nullptr)
    {
        // Root of the State Tree is the starting state of the State Graph
        TreeNode* root = new TreeNode;
        root->node = graph;
        root->path = graph->name;
        root->cost = 0;
        root->hCost = heuristic.at(root->node->name);

        priority_queue<TreeNode*, vector<TreeNode*>, CompareNode_Astar> fringe;
        fringe.push(root);
        
        while(!fringe.empty())
        {
            TreeNode* stateToExpand = fringe.top();
            fringe.pop();

            if (visited.count(stateToExpand->node) == 0)
            {
                visited.insert(stateToExpand->node);

                expandedNodes++;    // Increase counter for node who have been expanded

                // Goal reached
                if(stateToExpand->node->name == goal)
                {
                    ans->node = stateToExpand->node;
                    ans->cost = stateToExpand->cost;
                    ans->path = stateToExpand->path;
                    delete stateToExpand;
                    // delete all nodes in fringe
                    while(!fringe.empty())
                    {
                        TreeNode* toDelete = fringe.top();
                        fringe.pop();
                        delete toDelete;
                    }
                    break;  // if the node is reached, break the loop and exit the function
                }
                else
                {
                    // Expand nodes by adding all its links to the fringe
                    list<pair<unsigned int, GraphNode *> >::iterator it;
                    for(it = stateToExpand->node->links.begin(); it != stateToExpand->node->links.end(); ++it)
                    {
                        TreeNode* state = new TreeNode;                                 // create new tree node to be added in the fringe
                        state->node = it->second;                                       // save the node from the iterator
                        state->path = stateToExpand->path + ", " + it->second->name;    // update the path with new name
                        state->cost = it->first + stateToExpand->cost;                  // update overall cost
                        state->hCost = heuristic.at(state->node->name);                 // update overall cost
                        fringe.push(state);                                             // add new tree node in the fringe
                    }
                    delete stateToExpand;   // deallocate the node that was just expanded
                }
            }
        }
    }

    cout << "Expanded nodes: " << expandedNodes << endl;

    return ans;
}



int main()
{
    GraphNode states[20];

    // names
    states[0].name = "Arad";
    states[1].name = "Timisoara";
    states[2].name = "Zerind";
    states[3].name = "Oradea";
    states[4].name = "Lugoj";
    states[5].name = "Drobeta";
    states[6].name = "Mehadia";
    states[7].name = "Sibiu";
    states[8].name = "Rimnicu Vilcea";
    states[9].name = "Craiova";
    states[10].name = "Fagaras";
    states[11].name = "Pitesti";
    states[12].name = "Giurgiu";
    states[13].name = "Bucharest";
    states[14].name = "Neamt";
    states[15].name = "Urziceni";
    states[16].name = "Iasi";
    states[17].name = "Vaslui";
    states[18].name = "Hirsova";
    states[19].name = "Eforie";

    // roads
    states[0].links.push_back(pair<unsigned int, GraphNode *>(75, &states[2]));  // Arad - Zerind
    states[0].links.push_back(pair<unsigned int, GraphNode *>(140, &states[7])); // Arad - Sibiu
    states[0].links.push_back(pair<unsigned int, GraphNode *>(118, &states[1])); // Arad - Timisoara

    states[1].links.push_back(pair<unsigned int, GraphNode *>(118, &states[0])); // Timisoara - Arad
    states[1].links.push_back(pair<unsigned int, GraphNode *>(111, &states[4])); // Timisoara - Lugoj

    states[2].links.push_back(pair<unsigned int, GraphNode *>(75, &states[0])); // Zerind - Arad
    states[2].links.push_back(pair<unsigned int, GraphNode *>(71, &states[3])); // Zerind - Oradea

    states[3].links.push_back(pair<unsigned int, GraphNode *>(71, &states[2]));  // Oradea - Zerind
    states[3].links.push_back(pair<unsigned int, GraphNode *>(151, &states[7])); // Oradea - Sibiu

    states[4].links.push_back(pair<unsigned int, GraphNode *>(111, &states[1])); // Lugoj - Timisoara
    states[4].links.push_back(pair<unsigned int, GraphNode *>(70, &states[6]));  // Lugoj - Mehadia

    states[5].links.push_back(pair<unsigned int, GraphNode *>(75, &states[6]));  // Drobeta - Mehadia
    states[5].links.push_back(pair<unsigned int, GraphNode *>(120, &states[9])); // Drobeta - Craiova

    states[6].links.push_back(pair<unsigned int, GraphNode *>(70, &states[4])); // Mehadia - Lugoj
    states[6].links.push_back(pair<unsigned int, GraphNode *>(75, &states[5])); // Mehadia - Drobeta

    states[7].links.push_back(pair<unsigned int, GraphNode *>(151, &states[7])); // Sibu - Oradea
    states[7].links.push_back(pair<unsigned int, GraphNode *>(140, &states[0])); // Sibu - Arad
    states[7].links.push_back(pair<unsigned int, GraphNode *>(80, &states[8]));  // Sibu - Rimnicu Vilcea
    states[7].links.push_back(pair<unsigned int, GraphNode *>(99, &states[10])); // Sibu - Fagaras

    states[8].links.push_back(pair<unsigned int, GraphNode *>(80, &states[7]));  // Rimnicu Vilcea - Sibu
    states[8].links.push_back(pair<unsigned int, GraphNode *>(146, &states[9])); // Rimnicu Vilcea - Craiova
    states[8].links.push_back(pair<unsigned int, GraphNode *>(97, &states[11])); // Rimnicu Vilcea - Pitesti

    states[9].links.push_back(pair<unsigned int, GraphNode *>(120, &states[5]));  // Craiova - Drobeta
    states[9].links.push_back(pair<unsigned int, GraphNode *>(146, &states[8]));  // Craiova - Rimnicu Vilcea
    states[9].links.push_back(pair<unsigned int, GraphNode *>(138, &states[11])); // Craiova - Pitesti

    states[10].links.push_back(pair<unsigned int, GraphNode *>(99, &states[10]));  // Fagaras - Sibu
    states[10].links.push_back(pair<unsigned int, GraphNode *>(211, &states[13])); // Fagaras - Bucharest

    states[11].links.push_back(pair<unsigned int, GraphNode *>(97, &states[8]));   // Pitesti - Rimnicu Vilcea
    states[11].links.push_back(pair<unsigned int, GraphNode *>(138, &states[9]));  // Pitesti - Craiova
    states[11].links.push_back(pair<unsigned int, GraphNode *>(101, &states[13])); // Pitesti - Bucharest

    states[12].links.push_back(pair<unsigned int, GraphNode *>(90, &states[13])); // Giurgiu - Bucharest

    states[13].links.push_back(pair<unsigned int, GraphNode *>(211, &states[10])); // Bucharest - Fagaras
    states[13].links.push_back(pair<unsigned int, GraphNode *>(101, &states[11])); // Bucharest - Pitesti
    states[13].links.push_back(pair<unsigned int, GraphNode *>(90, &states[12]));  // Bucharest - Giurgiu
    states[13].links.push_back(pair<unsigned int, GraphNode *>(85, &states[15]));  // Bucharest - Urziceni

    states[14].links.push_back(pair<unsigned int, GraphNode *>(87, &states[16])); // Neamt - Iasi

    states[15].links.push_back(pair<unsigned int, GraphNode *>(85, &states[13]));  // Urziceni - Bucharest
    states[15].links.push_back(pair<unsigned int, GraphNode *>(142, &states[17])); // Urziceni - Vaslui
    states[15].links.push_back(pair<unsigned int, GraphNode *>(98, &states[18]));  // Urziceni - Hirsova

    states[16].links.push_back(pair<unsigned int, GraphNode *>(87, &states[14])); // Iasi - Neamt
    states[16].links.push_back(pair<unsigned int, GraphNode *>(92, &states[17])); // Iasi - Valsui

    states[17].links.push_back(pair<unsigned int, GraphNode *>(92, &states[16]));  // Vaslui - Iasi
    states[17].links.push_back(pair<unsigned int, GraphNode *>(142, &states[15])); // Vaslui - Urziceni

    states[18].links.push_back(pair<unsigned int, GraphNode *>(98, &states[15])); // Hirsova - Urziceni
    states[18].links.push_back(pair<unsigned int, GraphNode *>(86, &states[19])); // Hirsova - Eforie

    states[19].links.push_back(pair<unsigned int, GraphNode *>(86, &states[18])); // Eforie - Hirsova

    // Heuristic city --> estimated distance to goal (Bucharest)
    unordered_map<string, unsigned int> heuristic_toBucharest;
    heuristic_toBucharest.insert(pair<string, unsigned int>("Arad", 366));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Timisoara", 329));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Zerind", 374));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Oradea", 380));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Lugoj", 244));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Drobeta", 242));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Mehadia", 241));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Sibiu", 253));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Rimnicu Vilcea", 193));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Craiova", 160));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Fagaras", 178));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Pitesti", 98));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Giurgiu", 77));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Bucharest", 0));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Neamt", 234));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Urziceni", 80));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Iasi", 226));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Vaslui", 199));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Hirsova", 151));
    heuristic_toBucharest.insert(pair<string, unsigned int>("Eforie", 161));


    /* --------- RUN ALGORITHMS --------- */

    GraphNode* start = &states[0];
    string goal = "Bucharest";

    TreeNode* sol = solveBFS(start,goal);
    cout << sol->path << endl;
    cout << "Cost: " << sol->cost << endl;
    delete sol;
    cout << endl;

    sol = solveDFS(start,goal);
    cout << sol->path << endl;
    cout << "Cost: " << sol->cost << endl;
    delete sol;
    cout << endl;

    sol = solveIDS(start,goal,10);
    cout << sol->path << endl;
    cout << "Cost: " << sol->cost << endl;
    delete sol;
    cout << endl;

    sol = solveUCS(start,goal);
    cout << sol->path << endl;
    cout << "Cost: " << sol->cost << endl;
    delete sol;
    cout << endl;

    sol = solveGreedy(start,goal,heuristic_toBucharest);
    cout << sol->path << endl;
    cout << "Cost: " << sol->cost << endl;
    delete sol;
    cout << endl;

    sol = solveAstar(start,goal,heuristic_toBucharest);
    cout << sol->path << endl;
    cout << "Cost: " << sol->cost << endl;
    delete sol;
    cout << endl;

}