# Heuristic Optimization Project: Multi-Agent Path Planning

## Project Overview
This project implements a multi-agent path planning system using heuristic optimization techniques. The system consists of four agents and one randomly placed target inside a map containing obstacles. The objective is to efficiently locate and reach the target using a combination of the **Rapidly-Exploring Random Tree (RRT) algorithm** and the **Multi-Agent A* algorithm**.

## Methodology
1. **Target Search with RRT**:
   - The target is randomly placed on the map.
   - Agents explore the environment using the RRT algorithm to locate the target efficiently.
   - Once an agent finds the target, it communicates the exact location to the other agents.

2. **Path Optimization with Multi-Agent A***:
   - After receiving the target's location, the remaining agents switch to the Multi-Agent A* algorithm.
   - The algorithm computes the shortest path for each agent while avoiding obstacles.
   - Collision prevention mechanisms are implemented to ensure agents do not collide with each other while moving towards the target.

## Technologies Used
- **MATLAB** for RRT implementation and visualization of agent movements
- **C++** for Multi-Agent A* algorithm implementation
- **MEX** to integrate C++ A* algorithm with MATLAB

## Installation & Usage
### Prerequisites
Ensure you have MATLAB installed along with a compatible C++ compiler for MEX.

### Running the Project
To execute the simulation, run the MATLAB script:
```matlab
run astar.m
```

## Features
- Randomized target placement
- Multi-agent coordination
- RRT-based exploration
- Multi-Agent A*-based optimal pathfinding
- Obstacle avoidance
- Collision prevention between agents
- MATLAB visualization

## Future Improvements
- Implementing additional heuristics for improved efficiency
- Adding real-time visualization with animations
- Enhancing agent communication for decentralized decision-making

## Contributors
- Samir Ertürk
- Ayana Mederbek kyzy

## License
This project is licensed under the MIT License.
