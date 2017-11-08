# libMultiRobotPlanning

libMultiRobotPlanning is a library with search algorithms primarily for task and path planning for multi robot/agent systems.
It is written in C++(14), highly templated for good performance, and comes with useful examples.

The following algorithms are currently supported:

* Single-Robot Algorithms
  * A*
  * A* epsilon (also known as focal search)

* Multi-Robot Algorithms
  * Conflict-Based-Search (CBS)
  * Enhanced Conflict-Based-Search (ECBS)

## Building

Tested on Ubuntu 16.04.

```
mkdir build
cd build
cmake ..
make
```

### Targets

* `make`: Build examples, only
* `make docs`: build doxygen documentation
* `make clang-format`: Re-format all source files
* `make clang-tidy`: Run linter & static code analyzer
* `make run-test`: Run unit-tests
