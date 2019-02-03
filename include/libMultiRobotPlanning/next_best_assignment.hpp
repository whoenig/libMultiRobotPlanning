#pragma once

#include <queue>
#include <set>

#include <libMultiRobotPlanning/assignment.hpp>

namespace libMultiRobotPlanning {

/*!
  \example next_best_assignment.cpp example that takes cost mappings from a file
*/

/*! \brief Find series of assignment, ordered by sum-of-cost

This class can find a series of assignment, ordered by sum-of-cost (lowest
first)
for given agents and tasks. The costs must be integers, the agents and
tasks can be of any user-specified type.

This method is a an iterative variant of k-best-assignment.
Details of the algorithm can be found in the following paper:\n
W. Hönig, S. Kiesel, A. Tinka, J. W. Durham, and N. Ayanian.\n
"Conflict-Based Search with Optimal Task Assignment",\n
In Proc. of the 17th International Conference on Autonomous Agents and
Multiagent Systems (AAMAS)\n
Stockholm, Sweden, July 2018.

\tparam Agent Type of the agent. Needs to be copy'able and comparable
\tparam Task Type of task. Needs to be copy'able and comparable
*/
template <typename Agent, typename Task,
          typename Assignment = Assignment<Agent, Task> >
class NextBestAssignment {
 public:
  NextBestAssignment(const Assignment& assignment = Assignment())
      : m_assignment(assignment), m_cost(), m_open(), m_numMatching(0) {}

  void setCost(const Agent& agent, const Task& task, long cost) {
    // std::cout << "setCost: " << agent << "->" << task << ": " << cost <<
    // std::endl;
    m_cost[std::make_pair<>(agent, task)] = cost;
    if (m_agentsSet.find(agent) == m_agentsSet.end()) {
      m_agentsSet.insert(agent);
      m_agentsVec.push_back(agent);
    }
    // m_tasksSet.insert(task);
  }

  // find first (optimal) solution with minimal cost
  void solve() {
    const std::set<std::pair<Agent, Task> > I, O;
    const std::set<Agent> Iagents, Oagents;
    Node n;
    n.cost = constrainedMatching(I, O, Iagents, Oagents, n.solution);
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
      solution.insert(std::make_pair(entry.first, entry.second));
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
        n.Iagents = next.Iagents;
        n.Oagents = next.Oagents;
        // fix assignment for agents 0...i
        for (size_t j = 0; j < i; ++j) {
          const Agent& agent = m_agentsVec[j];
          // n.I.insert(std::make_pair<>(agent, next.solution.at(agent)));
          const auto iter = solution.find(agent);
          if (iter != solution.end()) {
            n.I.insert(std::make_pair<>(agent, iter->second));
          } else {
            // this agent should keep having no solution =>
            // enforce that no task is allowed
            n.Oagents.insert(agent);
            // for (const auto& task : m_tasksSet) {
            //   n.O.insert(std::make_pair<>(agent, task));
            // }
          }
        }
        // n.O.insert(
        //     std::make_pair<>(m_agentsVec[i],
        //     next.solution.at(m_agentsVec[i])));
        const auto iter = solution.find(m_agentsVec[i]);
        if (iter != solution.end()) {
          n.O.insert(std::make_pair<>(m_agentsVec[i], iter->second));
        } else {
          // this agent should have a solution next
          // std::cout << "should have sol: " << m_agentsVec[i] << std::endl;
          n.Iagents.insert(m_agentsVec[i]);
        }
        // std::cout << " consider adding: " << n << std::endl;
        n.cost =
            constrainedMatching(n.I, n.O, n.Iagents, n.Oagents, n.solution);
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
  // Iagents enforces that these agents must have a task assignment
  // Oagents enforces that these agents should not have any task assignment
  long constrainedMatching(const std::set<std::pair<Agent, Task> >& I,
                           const std::set<std::pair<Agent, Task> >& O,
                           const std::set<Agent>& Iagents,
                           const std::set<Agent>& Oagents,
                           std::map<Agent, Task>& solution) {
    // prepare assignment problem

    m_assignment.clear();

    for (const auto& c : I) {
      if (Oagents.find(c.first) == Oagents.end()) {
        m_assignment.setCost(c.first, c.second, 0);
      }
    }

    for (const auto& c : m_cost) {
      if (O.find(c.first) == O.end() && I.find(c.first) == I.end() &&
          Oagents.find(c.first.first) == Oagents.end()) {
        long costOffset = 1e9;  // TODO: what is a good value here?
        // all agents that should have any solution will get a lower cost offset
        // enforcing this agents inclusion in the result
        if (Iagents.find(c.first.first) != Iagents.end()) {
          costOffset = 0;
        }
        m_assignment.setCost(c.first.first, c.first.second,
                             c.second + costOffset);
      }
    }

    m_assignment.solve(solution);
    size_t matching = numMatching(solution);

    // std::cout << "constrainedMatching: internal Solution: " << std::endl;
    // for (const auto& c : solution) {
    //   std::cout << "    " << c.first << "->" << c.second << std::endl;
    // }

    // check if all agents in Iagents have an assignment as requested
    bool solutionValid = true;
    for (const auto& agent : Iagents) {
      if (solution.find(agent) == solution.end()) {
        solutionValid = false;
        break;
      }
    }
    // check that I constraints have been fulfilled
    for (const auto& c : I) {
      const auto& iter = solution.find(c.first);
      if (iter == solution.end() || !(iter->second == c.second)) {
        solutionValid = false;
        break;
      }
    }

    if (!solutionValid || matching < m_numMatching) {
      solution.clear();
      return std::numeric_limits<long>::max();
    }
    return cost(solution);
  }

  long cost(const std::map<Agent, Task>& solution) {
    long result = 0;
    for (const auto& entry : solution) {
      result += m_cost.at(entry);
    }
    return result;
  }

  size_t numMatching(const std::map<Agent, Task>& solution) {
    return solution.size();
  }

 private:
  struct Node {
    Node() : I(), O(), Iagents(), Oagents(), solution(), cost(0) {}

    std::set<std::pair<Agent, Task> > I;  // enforced assignment
    std::set<std::pair<Agent, Task> > O;  // invalid assignments
    std::set<Agent> Iagents;              // agents that must have an assignment
    std::set<Agent> Oagents;  // agents that should not have an assignment
    std::map<Agent, Task> solution;
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
      os << "  Iagents: ";
      for (const auto& c : n.Iagents) {
        os << c << ",";
      }
      os << std::endl;
      os << "  Oagents: ";
      for (const auto& c : n.Oagents) {
        os << c << ",";
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
  Assignment m_assignment;
  std::map<std::pair<Agent, Task>, long> m_cost;
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
