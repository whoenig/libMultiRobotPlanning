# libMultiRobotPlanning {#mainpage}

This library contains implementations of search algorithms in C++(11).
The library is header only and uses templates.

## Supported Algorithms

### Single-Robot

Algorithm   | Optimality         |
------------|--------------------|
[A*](@ref libMultiRobotPlanning::AStar)          | optimal            |

### Multi-Robot

Algorithm   | Optimality         |
------------|--------------------|
[Conflict-Based-Search (CBS)](@ref libMultiRobotPlanning::CBS)          | optimal            |
