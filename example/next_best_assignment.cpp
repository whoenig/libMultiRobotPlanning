#include <fstream>
#include <iostream>
#include <regex>

#include <boost/program_options.hpp>

#include <libMultiRobotPlanning/next_best_assignment.hpp>

using libMultiRobotPlanning::NextBestAssignment;

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input cost (txt)")("output,o",
                          po::value<std::string>(&outputFile)->required(),
                          "output file (YAML)");

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

  NextBestAssignment<std::string, std::string> assignment;

  std::ifstream input(inputFile);
  std::regex re("(\\w+)\\s*->\\s*(\\w+)\\s*:\\s*(\\d+)");
  for (std::string line; getline(input, line);) {
    std::smatch match;
    if (std::regex_search(line, match, re) && match.size() == 4) {
      std::string agent = match.str(1);
      std::string task = match.str(2);
      int cost = std::stoi(match.str(3));
      assignment.setCost(agent, task, cost);
    } else {
      std::cerr << "Couldn't match line \"" << line << "\"!" << match.size()
                << std::endl;
    }
  }

  // const size_t numAgents = 4;
  // const size_t numTasks = 4;

  // const int64_t cost[numAgents][numTasks] = {{90, 76, 75, 80},
  //                                            {35, 85, 55, 65},
  //                                            {125, 95, 90, 105},
  //                                            {45, 110, 95, 115}};

  // for (size_t i = 0; i < numAgents; ++i) {
  //   for (size_t j = 0; j < numTasks; ++j) {
  //     a.setCost("a" + std::to_string(i), "t" + std::to_string(j),
  //     cost[i][j]);
  //   }
  // }

  std::map<std::string, std::string> solution;
  assignment.solve();

  std::ofstream out(outputFile);
  out << "solutions:" << std::endl;

  for (size_t i = 0;; ++i) {
    int64_t c = assignment.nextSolution(solution);
    if (solution.empty()) {
      if (i == 0) {
        out << "  []" << std::endl;
      }
      break;
    }
    out << "  - cost: " << c << std::endl;
    out << "    assignment:" << std::endl;
    for (const auto& s : solution) {
      out << "      " << s.first << ": " << s.second << std::endl;
    }
  }

  return 0;
}
