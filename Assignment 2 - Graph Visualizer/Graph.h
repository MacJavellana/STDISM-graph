#pragma once
#include <string>
#include <vector>
#include <set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <limits>
#include <functional>

struct Edge {
    std::string source;
    std::string target;
    int weight;
    Edge(const std::string& s, const std::string& t, int w)
        : source(s), target(t), weight(w) {
    }
};

struct Path {
    std::vector<std::string> nodes;
    int totalWeight;
    Path() : totalWeight(0) {}
};

class Graph {
private:
    std::set<std::string> nodes;
    std::map<std::string, std::map<std::string, int>> edges;
    const int NUM_THREADS = 16;
    bool isPrime(int n) const;
    Path parallelFindPathHelper(const std::string& start, const std::string& end,
        std::function<Path(const std::string&, const std::string&)> findFunction);

public:
    void addNode(const std::string& node);
    void addEdge(const std::string& source, const std::string& target, int weight);
    bool hasNode(const std::string& node) const;
    bool hasEdge(const std::string& source, const std::string& target) const;
    std::vector<std::string> getNodes() const;
    std::vector<Edge> getEdges() const;

    Path findPath(const std::string& start, const std::string& end) const;
    Path findShortestPath(const std::string& start, const std::string& end) const;
    Path findPrimePath(const std::string& start, const std::string& end) const;
    Path findShortestPrimePath(const std::string& start, const std::string& end) const;

    Path parallelFindPath(const std::string& start, const std::string& end);
    Path parallelFindShortestPath(const std::string& start, const std::string& end);
    Path parallelFindPrimePath(const std::string& start, const std::string& end);
    Path parallelFindShortestPrimePath(const std::string& start, const std::string& end);
};