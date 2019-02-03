#include <fstream>
#include <iostream>
#include <unordered_set>

#include <boost/functional/hash.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include "timer.hpp"

struct Location {
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

#include "shortest_path_heuristic.hpp"

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
                           "output file (CSV)");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_set<Location> obstacles;
#if 0
  typedef boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::undirectedS > searchGraphTraits_t;
  typedef searchGraphTraits_t::vertex_descriptor vertex_t;
  typedef searchGraphTraits_t::edge_descriptor edge_t;

  struct Vertex
  {
  };

  struct Edge
  {
    float weight;
  };

  typedef boost::adjacency_list<
          boost::vecS, boost::vecS, boost::undirectedS,
          Vertex, Edge>
          searchGraph_t;
  typedef boost::exterior_vertex_property<searchGraph_t, float> distanceProperty_t;
  typedef distanceProperty_t::matrix_type distanceMatrix_t;
  typedef distanceProperty_t::matrix_map_type distanceMatrixMap_t;

  searchGraph_t searchGraph;
  std::map<Location, vertex_t> mapLocToVertex;
#endif
  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
  }
#if 0
  // add vertices
  for (int x = 0; x < dimx; ++x) {
    for (int y = 0; y < dimy; ++y) {
      Location l(x, y);
      auto v = boost::add_vertex(searchGraph);
      mapLocToVertex[l] = v;
    }
  }

  // add edges
  for (int x = 0; x < dimx; ++x) {
    for (int y = 0; y < dimy; ++y) {
      Location l(x, y);
      if (obstacles.find(l) == obstacles.end()) {
        Location right(x+1, y);
        if (x < dimx - 1 && obstacles.find(right) == obstacles.end()) {
          auto e = boost::add_edge(mapLocToVertex[l], mapLocToVertex[right], searchGraph);
          searchGraph[e.first].weight = 1;
        }
        Location below(x, y+1);
        if (y < dimy - 1 && obstacles.find(below) == obstacles.end()) {
          auto e = boost::add_edge(mapLocToVertex[l], mapLocToVertex[below], searchGraph);
          searchGraph[e.first].weight = 1;
        }
      }
    }
  }

  distanceMatrix_t m_shortestDistance(boost::num_vertices(searchGraph));
  distanceMatrixMap_t distanceMap(m_shortestDistance, searchGraph);
  boost::floyd_warshall_all_pairs_shortest_paths(searchGraph, distanceMap, boost::weight_map(boost::get(&Edge::weight, searchGraph)));

  std::ofstream fileStream(outputFile.c_str());
  fileStream << dimx << "," << dimy << std::endl;
  for (size_t row = 0; row < dimx * dimy; ++row) {
    for (size_t column = 0; column < m_shortestDistance[row].size(); ++column) {
      fileStream << m_shortestDistance[row][column] << ",";
    }
    fileStream << std::endl;
  }
#endif

  ShortestPathHeuristic h(dimx, dimy, obstacles);
  std::cout << h.getValue(Location(0, 0), Location(3, 0)) << std::endl;

  return 0;
}
