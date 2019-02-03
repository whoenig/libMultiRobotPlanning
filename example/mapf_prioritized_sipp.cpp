#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/sipp.hpp>

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::SIPP;

struct State {
  State(int x, int y) : x(x), y(y) {}

  bool operator==(const State& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  bool operator!=(const State& other) const {
    return std::tie(x, y) != std::tie(other.x, other.y);
  }

  bool operator<(const State& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.x << "," << s.y << ")";
  }

  int x;
  int y;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

enum class Action {
  Up,
  Down,
  Left,
  Right,
  Wait,
};

std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Up:
      os << "Up";
      break;
    case Action::Down:
      os << "Down";
      break;
    case Action::Left:
      os << "Left";
      break;
    case Action::Right:
      os << "Right";
      break;
    case Action::Wait:
      os << "Wait";
      break;
  }
  return os;
}

class Environment {
 public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<State> obstacles,
              State goal)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goal(goal) {}

  float admissibleHeuristic(const State& s) {
    return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
  }

  bool isSolution(const State& s) { return s == m_goal; }

  State getLocation(const State& s) { return s; }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int>>& neighbors) {
    neighbors.clear();

    State up(s.x, s.y + 1);
    if (stateValid(up)) {
      neighbors.emplace_back(Neighbor<State, Action, int>(up, Action::Up, 1));
    }
    State down(s.x, s.y - 1);
    if (stateValid(down)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(down, Action::Down, 1));
    }
    State left(s.x - 1, s.y);
    if (stateValid(left)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(left, Action::Left, 1));
    }
    State right(s.x + 1, s.y);
    if (stateValid(right)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(right, Action::Right, 1));
    }
  }

  void onExpandNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {
    // std::cout << "expand: " << s << "g: " << gScore << std::endl;
  }

  void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/) {
    // std::cout << "  discover: " << s << std::endl;
  }

  bool isCommandValid(
      const State& /*s1*/, const State& /*s2*/, const Action& /*a*/,
      int earliestStartTime,      // can start motion at this time
      int /*latestStartTime*/,    // must have left s by this time
      int earliestArrivalTime,    // can only arrive at (s+cmd)
      int /*latestArrivalTime*/,  // needs to arrive by this time at (s+cmd)
      int& t) {
    t = std::max<int>(earliestArrivalTime, earliestStartTime + 1);

    // TODO(whoenig): need to check for swaps here...

    // return t - 1 <= latestStartTime;
    return true;
  }

 private:
  bool stateValid(const State& s) {
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(s) == m_obstacles.end();
  }

 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<State> m_obstacles;
  State m_goal;
};

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)")
      // ("url",
      // po::value<std::string>(&url)->default_value("http://0.0.0.0:8080"),
      // "server URL")
      ;

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

  // Configure SIPP based on config file
  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_set<State> obstacles;
  std::vector<State> goals;
  std::vector<State> startStates;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(State(node[0].as<int>(), node[1].as<int>()));
  }

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(State(start[0].as<int>(), start[1].as<int>()));
    goals.emplace_back(State(goal[0].as<int>(), goal[1].as<int>()));
  }
  typedef SIPP<State, State, Action, int, Environment> sipp_t;

  std::ofstream out(outputFile);
  out << "schedule:" << std::endl;

  // Plan (sequentially)
  std::map<State, std::vector<sipp_t::interval>> allCollisionIntervals;
  long cost = 0;
  for (size_t i = 0; i < goals.size(); ++i) {
    std::cout << "Planning for agent " << i << std::endl;
    out << "  agent" << i << ":" << std::endl;

    Environment env(dimx, dimy, obstacles, goals[i]);
    sipp_t sipp(env);

    for (const auto& collisionIntervals : allCollisionIntervals) {
      sipp.setCollisionIntervals(collisionIntervals.first,
                                 collisionIntervals.second);
    }

    // Plan
    PlanResult<State, Action, int> solution;
    bool success = sipp.search(startStates[i], Action::Wait, solution);

    if (success) {
      std::cout << "Planning successful! Total cost: " << solution.cost
                << std::endl;

      // update collision intervals
      auto lastState = solution.states[0];
      for (size_t i = 1; i < solution.states.size(); ++i) {
        if (solution.states[i].first != lastState.first) {
          allCollisionIntervals[lastState.first].push_back(sipp_t::interval(
              lastState.second, solution.states[i].second - 1));
          lastState = solution.states[i];
        }
      }
      allCollisionIntervals[solution.states.back().first].push_back(
          sipp_t::interval(solution.states.back().second,
                           std::numeric_limits<int>::max()));
      // update statistics
      cost += solution.cost;

      // print solution
      for (size_t i = 0; i < solution.actions.size(); ++i) {
        std::cout << solution.states[i].second << ": "
                  << solution.states[i].first << "->"
                  << solution.actions[i].first
                  << "(cost: " << solution.actions[i].second << ")"
                  << std::endl;
      }
      std::cout << solution.states.back().second << ": "
                << solution.states.back().first << std::endl;

      for (size_t i = 0; i < solution.states.size(); ++i) {
        out << "    - x: " << solution.states[i].first.x << std::endl
            << "      y: " << solution.states[i].first.y << std::endl
            << "      t: " << solution.states[i].second << std::endl;
      }
    } else {
      std::cout << "Planning NOT successful!" << std::endl;
      out << "    []" << std::endl;
    }
  }

  out << "statistics:" << std::endl;
  out << "  cost: " << cost << std::endl;
  // out << "  makespan: " << makespan << std::endl;
  // out << "  runtime: " << timer.elapsedSeconds() << std::endl;

  // for (const auto& node : config["environment"]["collisionIntervals"]) {
  //   State state(node["location"][0].as<int>(),
  //   node["location"][1].as<int>());

  //   std::vector<sipp_t::interval> collisionIntervals;

  //   for (const auto& interval : node["intervals"]) {
  //     collisionIntervals.emplace_back(
  //         sipp_t::interval(interval[0].as<int>(), interval[1].as<int>()));
  //   }
  //   sipp.setCollisionIntervals(state, collisionIntervals);
  // }

  return 0;
}
