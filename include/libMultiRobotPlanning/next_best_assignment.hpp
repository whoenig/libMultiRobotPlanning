#pragma once

#include <queue>
#include <set>

#include <libMultiRobotPlanning/assignment.hpp>

namespace libMultiRobotPlanning {

/*!
  \example next_best_assignment.cpp example that takes cost mappings from a file
*/

/*! \brief Find series of assignment, ordered by sum-of-cost

This class can find a series of assignment, ordered by sum-of-cost (lowest first)
for given agents and tasks. The costs must be integers, the agents and
tasks can be of any user-specified type.

This method is a an iterative variant of k-best-assignment.
Details of the algorithm can be found in the following paper:\n
W. Hönig, S. Kiesel, A. Tinka, J. W. Durham, and N. Ayanian.\n
"Conflict-Based Search with Optimal Task Assignment",\n
In Proc. of the 17th International Conference on Autonomous Agents and Multiagent Systems (AAMAS)\n
Stockholm, Sweden, July 2018.

\tparam Agent Type of the agent. Needs to be copy'able and comparable
\tparam Task Type of task. Needs to be copy'able and comparable
*/
template <typename Agent, typename Task>
class NextBestAssignment {
 private:
  // Each low-level task is either a user-provided task, or a special "noTask"
  // for this particular agent
  struct AugmentedTask {
    // TODO: ideally this would be union...
    Task task;
    Agent agent;

    enum Type {
      UserProvidedTask,  // task valid
      NoTask,            // agent valid
    } type;

    bool operator<(const AugmentedTask& n) const {
      if (type == n.type) {
        if (type == UserProvidedTask) {
          return task < n.task;
        } else {
          return agent < n.agent;
        }
      } else if (type == UserProvidedTask) {
        return true;
      } else {
        return false;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const AugmentedTask& n) {
      if (n.type == UserProvidedTask) {
        os << n.task;
      } else {
        os << n.agent;
      }
      return os;
    }
  };

 public:
  NextBestAssignment() : m_cost(), m_open(), m_numMatching(0) {}

  void setCost(const Agent& agent, const Task& task, long cost) {
    // std::cout << "setCost: " << agent << "->" << task << ": " << cost <<
    // std::endl;
    AugmentedTask augmentedTask;
    augmentedTask.task = task;
    augmentedTask.type = AugmentedTask::UserProvidedTask;
    m_cost[std::make_pair<>(agent, augmentedTask)] = cost;
    if (m_agentsSet.find(agent) == m_agentsSet.end()) {
      m_agentsSet.insert(agent);
      m_agentsVec.push_back(agent);
    }
    // m_tasksSet.insert(task);
  }

  // find first (optimal) solution with minimal cost
  void solve() {
    // add (fake) costs for "NoTask" solution
    for (const auto& agent : m_agentsSet) {
      AugmentedTask augmentedTask;
      augmentedTask.agent = agent;
      augmentedTask.type = AugmentedTask::NoTask;
      // TODO: figure out what value to use...
      m_cost[std::make_pair<>(agent, augmentedTask)] = 1e9;
    }

    const std::set<std::pair<Agent, AugmentedTask> > I, O;
    Node n;
    n.cost = constrainedMatching(I, O, n.solution);
    m_open.emplace(n);
    m_numMatching = numMatching(n.solution);
  }

  // find next solution
  long nextSolution(std::map<Agent, Task>& solution) {
    solution.clear();
    if (m_open.empty()) {
      return std::numeric_limits<long>::max();
    }

    const Node next = m_open.top();
    // std::cout << "next: " << next << std::endl;
    m_open.pop();
    for (const auto& entry : next.solution) {
      if (entry.second.type == AugmentedTask::UserProvidedTask) {
        solution.insert(std::make_pair(entry.first, entry.second.task));
      }
    }
    long result = next.cost;

    std::set<Agent> fixedAgents;
    for (const auto c : next.I) {
      fixedAgents.insert(c.first);
    }

    // prepare for next query
    for (size_t i = 0; i < m_agentsVec.size(); ++i) {
      if (fixedAgents.find(m_agentsVec[i]) == fixedAgents.end()) {
        Node n;
        n.I = next.I;
        n.O = next.O;
        // fix assignment for agents 0...i
        for (size_t j = 0; j < i; ++j) {
          const Agent& agent = m_agentsVec[j];
          n.I.insert(std::make_pair<>(agent, next.solution.at(agent)));
          // const auto iter = solution.find(agent);
          // if (iter != solution.end()) {
          //   n.I.insert(std::make_pair<>(agent, iter->second));
          // } else {
          //   // this agent should keep having no solution =>
          //   // enforce that no task is allowed
          //   for (const auto& task : m_tasksSet) {
          //     n.O.insert(std::make_pair<>(agent, task));
          //   }
          // }
        }
        n.O.insert(
            std::make_pair<>(m_agentsVec[i], next.solution.at(m_agentsVec[i])));
        // const auto iter = solution.find(m_agentsVec[i]);
        // if (iter != solution.end()) {
        //   n.O.insert(std::make_pair<>(m_agentsVec[i], iter->second));
        // } else {
        //   // this agent should have a solution next
        //   std::cout << "should have sol: " << m_agentsVec[i] << std::endl;
        // }
        // std::cout << " consider adding: " << n << std::endl;
        n.cost = constrainedMatching(n.I, n.O, n.solution);
        if (n.solution.size() > 0) {
          m_open.push(n);
          // std::cout << "add: " << n << std::endl;
        }
      }
    }

    return result;
  }

 protected:
  // I enforces that the respective pair is part of the solution
  // O enforces that the respective pair is not part of the solution
  long constrainedMatching(const std::set<std::pair<Agent, AugmentedTask> >& I,
                           const std::set<std::pair<Agent, AugmentedTask> >& O,
                           std::map<Agent, AugmentedTask>& solution) {
    // prepare assignment problem

    Assignment<Agent, AugmentedTask> assignment;

    std::set<AugmentedTask> assignedTasks;
    for (const auto& c : I) {
      assignedTasks.insert(c.second);
      assignment.setCost(c.first, c.second, 0);
    }

    for (const auto& c : m_cost) {
      if (assignedTasks.find(c.first.second) == assignedTasks.end() &&
          O.find(c.first) == O.end()) {
        assignment.setCost(c.first.first, c.first.second, c.second);
      }
    }

    assignment.solve(solution);
    size_t matching = numMatching(solution);
    if (solution.size() < m_agentsSet.size() || matching < m_numMatching) {
      solution.clear();
      return std::numeric_limits<long>::max();
    }
    return cost(solution);
  }

  long cost(const std::map<Agent, AugmentedTask>& solution) {
    long result = 0;
    for (const auto& entry : solution) {
      if (entry.second.type == AugmentedTask::UserProvidedTask) {
        result += m_cost.at(entry);
      }
    }
    return result;
  }

  size_t numMatching(const std::map<Agent, AugmentedTask>& solution) {
    size_t result = 0;
    for (const auto& entry : solution) {
      if (entry.second.type == AugmentedTask::UserProvidedTask) {
        ++result;
      }
    }
    return result;
  }

 private:
  struct Node {
    Node() : I(), O(), solution(), cost(0) {}

    std::set<std::pair<Agent, AugmentedTask> > I;  // enforced assignment
    std::set<std::pair<Agent, AugmentedTask> > O;  // invalid assignments
    std::map<Agent, AugmentedTask> solution;
    long cost;

    bool operator<(const Node& n) const {
      // Our heap is a maximum heap, so we invert the comperator function here
      return cost > n.cost;
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& n) {
      os << "Node with cost: " << n.cost << std::endl;
      os << "  I: ";
      for (const auto& c : n.I) {
        os << c.first << "->" << c.second << ",";
      }
      os << std::endl;
      os << "  O: ";
      for (const auto& c : n.O) {
        os << c.first << "->" << c.second << ",";
      }
      os << std::endl;
      os << "  solution: ";
      for (const auto& c : n.solution) {
        os << "    " << c.first << "->" << c.second << std::endl;
      }
      os << std::endl;
      return os;
    }
  };

 private:
  std::map<std::pair<Agent, AugmentedTask>, long> m_cost;
  std::vector<Agent> m_agentsVec;
  std::set<Agent> m_agentsSet;
  // std::set<Task> m_tasksSet;
  // size_t m_numAgents;
  // size_t m_numTasks;
  // std::vector<long> m_costMatrix;
  std::priority_queue<Node> m_open;
  size_t m_numMatching;
};

}  // namespace libMultiRobotPlanning
