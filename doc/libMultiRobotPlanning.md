# libMultiRobotPlanning {#mainpage}

This library contains implementations of search algorithms in C++(14).
The library is header only and uses templates.

## Supported Algorithms

### Single-Robot

Algorithm   | Optimality         |
------------|--------------------|
[A*](@ref libMultiRobotPlanning::AStar)          | optimal            |
[A*_epsilon](@ref libMultiRobotPlanning::AStarEpsilon)          | w-bounded suboptimal            |
[SIPP](@ref libMultiRobotPlanning::SIPP)          | optimal            |

### Multi-Robot

Algorithm   | Optimality         |
------------|--------------------|
[Conflict-Based Search (CBS)](@ref libMultiRobotPlanning::CBS)          | optimal (sum-of-cost)           |
[Enhanced Conflict-Based Search (ECBS)](@ref libMultiRobotPlanning::ECBS)          | w-bounded suboptimal (sum-of-cost)           |
[Conflict-Based Search with Optimal Task Assignment (CBS-TA)](@ref libMultiRobotPlanning::CBSTA)          | optimal (sum-of-cost)           |
[Enhanced Conflict-Based Search with Optimal Task Assignment (ECBS-TA)](@ref libMultiRobotPlanning::ECBSTA)          | w-bounded suboptimal (sum-of-cost)           |
[Prioritized Planning with SIPP]        | No           |

### Assignment

Algorithm   | Optimality         |
------------|--------------------|
[Best Assignment (Network flow based)](@ref libMultiRobotPlanning::Assignment)          | optimal (sum-of-cost)           |
[Next Best Assignment](@ref libMultiRobotPlanning::NextBestAssignment)          | series of optimal solutions (sum-of-cost)           |
