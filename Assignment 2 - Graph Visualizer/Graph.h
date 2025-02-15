#pragma once
#include <string>
#include <vector>
#include <set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>


class Graph {
private:
    std::set<std::string> nodes;
    std::map<std::string, std::set<std::string>> edges;
    const int NUM_THREADS = 4;
    
public:
    void addNode(const std::string& node);
    void addEdge(const std::string& source, const std::string& target);
    bool hasNode(const std::string& node) const;
    bool hasEdge(const std::string& source, const std::string& target) const;
    std::vector<std::string> getNodes() const;
    std::vector<std::pair<std::string, std::string>> getEdges() const;
    // Parallel methods
    bool parallelHasNode(const std::string& node);
    bool parallelHasEdge(const std::string& source, const std::string& target);
    std::vector<std::string> parallelFindPath(const std::string& start, const std::string& end);
    std::vector<std::string> findPath(const std::string& start, const std::string& end) const;
};
