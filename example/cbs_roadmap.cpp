#include <fstream>
#include <iostream>
#include <unordered_map>

// #include <boost/bimap.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/cbs.hpp>
#include "timer.hpp"

using libMultiRobotPlanning::CBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;


// roadmap data structures
typedef boost::adjacency_list_traits<boost::vecS, boost::vecS,
                                     boost::directedS>
    roadmapTraits_t;
typedef roadmapTraits_t::vertex_descriptor vertex_t;
typedef roadmapTraits_t::edge_descriptor edge_t;

namespace std {
template <>
struct hash<edge_t> {
  size_t operator()(const edge_t& e) const {
    size_t seed = 0;
    boost::hash_combine(seed, e.m_source);
    boost::hash_combine(seed, e.m_target);
    return seed;
  }
};
}  // namespace std

struct Vertex {
  std::string name;
};

struct Edge {
  std::unordered_set<edge_t> conflictingEdges;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                              Vertex, Edge>
    roadmap_t;

struct State {
  State(int time, vertex_t vertex) : time(time), vertex(vertex) {}

  bool operator==(const State& s) const {
    return time == s.time && vertex == s.vertex;
  }

  bool equalExceptTime(const State& s) const { return vertex == s.vertex; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ": (" << s.vertex << ")";
    // return os << "(" << s.x << "," << s.y << ")";
  }

  int time;
  vertex_t vertex;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.vertex);
    return seed;
  }
};
}  // namespace std

///
typedef edge_t Action; // index to the specified edge

struct Conflict {
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;

  vertex_t vertex; // for Type == Vertex
  edge_t edge1;
  edge_t edge2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << c.time << ": Vertex(" << c.vertex << ")";
      case Edge:
        return os << c.time << ": Edge(" << c.edge1 << "," << c.edge2 << ")";
    }
    return os;
  }
};

struct VertexConstraint {
  VertexConstraint(int time, vertex_t vertex) : time(time), vertex(vertex) {}
  int time;
  vertex_t vertex;

  bool operator<(const VertexConstraint& other) const {
    return std::tie(time, vertex) < std::tie(other.time, other.vertex);
  }

  bool operator==(const VertexConstraint& other) const {
    return std::tie(time, vertex) == std::tie(other.time, other.vertex);
  }

  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
    return os << "VC(" << c.time << "," << c.vertex << ")";
  }
};

namespace std {
template <>
struct hash<VertexConstraint> {
  size_t operator()(const VertexConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.vertex);
    return seed;
  }
};
}  // namespace std

struct EdgeConstraint {
  EdgeConstraint(int time, edge_t edge)
      : time(time), edge(edge) {}
  int time;
  edge_t edge;

  bool operator<(const EdgeConstraint& other) const {
    return std::tie(time, edge) <
           std::tie(other.time, other.edge);
  }

  bool operator==(const EdgeConstraint& other) const {
    return std::tie(time, edge) ==
           std::tie(other.time, other.edge);
  }

  friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
    return os << "EC(" << c.time << "," << c.edge << ")";
  }
};

namespace std {
template <>
struct hash<EdgeConstraint> {
  size_t operator()(const EdgeConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.edge);
    return seed;
  }
};
}  // namespace std

struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints& other) const {
    for (const auto& vc : vertexConstraints) {
      if (other.vertexConstraints.count(vc) > 0) {
        return true;
      }
    }
    for (const auto& ec : edgeConstraints) {
      if (other.edgeConstraints.count(ec) > 0) {
        return true;
      }
    }
    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto& ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
};

///
class Environment {
 public:
  Environment(const roadmap_t& roadmap,
              const std::vector<vertex_t>& goals, bool disappearAtGoal = false)
      : m_roadmap(roadmap),
        m_goals(std::move(goals)),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0),
        m_disappearAtGoal(disappearAtGoal)
  {
    auto V = boost::num_vertices(m_roadmap);
    // Upper bound on the makespan, see
    // Jingjin Yu, Daniela Rus:
    // Pebble Motion on Graphs with Rotations: Efficient Feasibility Tests and Planning Algorithms. WAFR 2014: 729-746
    // NOTE: that the constant factor is not known
    m_timeLimit = pow(V, 3);
  }

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.vertex == m_goals[m_agentIdx]) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  int admissibleHeuristic(const State& s) {
    // TODO: we should actually compute this using A*
    return 0;
  }

  bool isSolution(const State& s) {
    return s.vertex == m_goals[m_agentIdx] &&
           s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {
    // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   std::endl;
    // }
    neighbors.clear();

    if (s.time > m_timeLimit) {
      return;
    }

    auto es = boost::out_edges(s.vertex, m_roadmap);
    for (auto eit = es.first; eit != es.second; ++eit) {
      vertex_t v = boost::target(*eit, m_roadmap);
      State n(s.time + 1, v);
      if (stateValid(n) && transitionValid(s.time, *eit)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, *eit, 1));
      }
    }

    // // Wait action
    // {
    //   State n(s.time + 1, s.vertex);
    //   if (stateValid(n)) {
    //     neighbors.emplace_back(
    //         Neighbor<State, Action, int>(n, edge_t(), 1));
    //   }
    // }
  }

  bool getFirstConflict(
      const std::vector<PlanResult<State, Action, int> >& solution,
      Conflict& result) {
    size_t max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<size_t>(max_t, sol.states.size() - 1);
    }

    for (size_t t = 0; t <= max_t; ++t) {
      // check vertex/vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.vertex = state1.vertex;
            return true;
          }
        }
      }
      // edge/edge
      for (size_t i = 0; i < solution.size(); ++i) {
        for (size_t j = i + 1; j < solution.size(); ++j) {
          if (t < solution[i].actions.size() &&
              t < solution[j].actions.size()) {
            auto e1 = solution[i].actions[t].first;
            auto e2 = solution[j].actions[t].first;
            if (e1.m_eproperty) {
              const auto& ce = m_roadmap[e1].conflictingEdges;
              if (ce.find(e2) != ce.end()) {
                result.time = t;
                result.agent1 = i;
                result.agent2 = j;
                result.type = Conflict::Edge;
                result.edge1 = e1;
                result.edge2 = e2;
                return true;
              }
            }
          }
        }
      }
    }

    return false;
  }

  void createConstraintsFromConflict(
      const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.vertex));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.edge1));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.edge2));
      constraints[conflict.agent2] = c2;
    }
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, int> >& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    if (m_disappearAtGoal) {
      // This is a trick to avoid changing the rest of the code significantly
      // After an agent disappeared, put it at a unique but invalid position
      // This will cause all calls to equalExceptTime(.) to return false.
      return State(-1, -1-agentIdx);
    }
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State& s) {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;
    return con.find(VertexConstraint(s.time, s.vertex)) == con.end();
  }

  bool transitionValid(int time, edge_t edge) {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(time, edge)) == con.end();
  }
private:
  const roadmap_t& m_roadmap;
  std::vector<vertex_t> m_goals;
  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  bool m_disappearAtGoal;
  int m_timeLimit;
};

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  bool disappearAtGoal;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)")(
      "disappear-at-goal", po::bool_switch(&disappearAtGoal), "make agents to disappear at goal rather than staying there");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  // some sanity checks
  if (config["roadmap"]["conflicts"]) {
    if (config["roadmap"]["undirected"].as<bool>()) {
      throw std::runtime_error("If conflicts are specified, the roadmap cannot be undirected!");
    }
    if (config["roadmap"]["allow_wait_actions"].as<bool>()) {
      throw std::runtime_error("If conflicts are specified, the wait actions must be already encoded in the edge set!");
    }
    if (config["roadmap"]["conflicts"].size() != config["roadmap"]["edges"].size()) {
      throw std::runtime_error("If conflicts are specified, the cardinality of conflicts and edges must match!");
    }
  }

  // read roadmap
  roadmap_t roadmap;
  std::unordered_map<std::string, vertex_t> vertexMap;

  std::vector<edge_t> edgeVec;
  for (const auto& edge : config["roadmap"]["edges"]) {
    // find or insert vertex1
    auto v1str = edge[0].as<std::string>();
    vertex_t v1;
    const auto iter1 = vertexMap.find(v1str);
    if (iter1 != vertexMap.end()) {
      v1 = iter1->second;
    } else {
      v1 = boost::add_vertex(roadmap);
      vertexMap.insert(std::make_pair(v1str, v1));
      roadmap[v1].name = v1str;
    }
    // find or insert vertex2
    auto v2str = edge[1].as<std::string>();
    vertex_t v2;
    const auto iter2 = vertexMap.find(v2str);
    if (iter2 != vertexMap.end()) {
      v2 = iter2->second;
    } else {
      v2 = boost::add_vertex(roadmap);
      vertexMap.insert(std::make_pair(v2str, v2));
      roadmap[v2].name = v2str;
    }
    auto e1 = boost::add_edge(v1, v2, roadmap);
    edgeVec.push_back(e1.first);
    if (config["roadmap"]["undirected"].as<bool>()) {
      auto e2 = boost::add_edge(v2, v1, roadmap);
      edgeVec.push_back(e2.first);
      roadmap[e1.first].conflictingEdges.insert(e2.first);
      roadmap[e2.first].conflictingEdges.insert(e1.first);
    }
  }

  if (config["roadmap"]["allow_wait_actions"].as<bool>()) {
    for (const auto& v : vertexMap) {
      auto e = boost::add_edge(v.second, v.second, roadmap);
      edgeVec.push_back(e.first);
    }
  }

  if (config["roadmap"]["conflicts"]) {
    size_t i = 0;
    for (const auto& conflicts : config["roadmap"]["conflicts"]) {
      for (const auto& conflict : conflicts) {
        size_t j = conflict.as<size_t>();
        roadmap[edgeVec[i]].conflictingEdges.insert(edgeVec[j]);
      }
      ++i;
    }
  }

  // read agents
  std::vector<vertex_t> goalVertices;
  std::vector<State> startStates;

  for (const auto& node : config["agents"]) {
    const auto start = node["start"].as<std::string>();
    const auto goal = node["goal"].as<std::string>();
    startStates.emplace_back(State(0, vertexMap[start]));
    goalVertices.push_back(vertexMap[goal]);
  }

  Environment mapf(roadmap, goalVertices, disappearAtGoal);
  CBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf);
  std::vector<PlanResult<State, Action, int> > solution;

  Timer timer;
  bool success = cbs.search(startStates, solution);
  timer.stop();

  if (success) {
    std::cout << "Planning successful! " << std::endl;
    int cost = 0;
    int makespan = 0;
    for (const auto& s : solution) {
      cost += s.cost;
      makespan = std::max<int>(makespan, s.cost);
    }

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  success: " << true << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  runtime: " << timer.elapsedSeconds() << std::endl;
    out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
    out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {
      // std::cout << "Solution for: " << a << std::endl;
      // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
      //   std::cout << solution[a].states[i].second << ": " <<
      //   solution[a].states[i].first << "->" << solution[a].actions[i].first
      //   << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
      // }
      // std::cout << solution[a].states.back().second << ": " <<
      // solution[a].states.back().first << std::endl;

      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "    - v: " << roadmap[state.first.vertex].name << std::endl
            << "      t: " << state.second << std::endl;
      }
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  success: " << false << std::endl;
  }

  return 0;
}
