#include "Graph.h"
#include <algorithm>
#include <cmath>

ThreadPool::ThreadPool(size_t threads) : stop(false) {
    for (size_t i = 0; i < threads; ++i) {
        workers.emplace_back([this] {
            while (true) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(queue_mutex);
                    condition.wait(lock, [this] { return stop || !tasks.empty(); });
                    if (stop && tasks.empty()) return;
                    task = std::move(tasks.front());
                    tasks.pop();
                }
                task();
            }
            });
    }
}

ThreadPool::~ThreadPool() {
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;
    }
    condition.notify_all();
    for (std::thread& worker : workers) {
        worker.join();
    }
}

bool Graph::isPrime(int n) const {
    if (n <= 1) return false;
    if (n == 2) return true;
    if (n % 2 == 0) return false;

    for (int i = 3; i <= sqrt(n); i += 2) {
        if (n % i == 0) return false;
    }
    return true;
}

void Graph::addNode(const std::string& node) {
    nodes.insert(node);
}

void Graph::addEdge(const std::string& source, const std::string& target, int weight) {
    if (!hasNode(source) || !hasNode(target)) return;
    edges[source][target] = weight;
}

bool Graph::hasNode(const std::string& node) const {
    return nodes.find(node) != nodes.end();
}

bool Graph::hasEdge(const std::string& source, const std::string& target) const {
    auto it = edges.find(source);
    if (it == edges.end()) return false;
    return it->second.find(target) != it->second.end();
}

std::vector<std::string> Graph::getNodes() const {
    return std::vector<std::string>(nodes.begin(), nodes.end());
}

std::vector<Edge> Graph::getEdges() const {
    std::vector<Edge> result;
    for (const auto& source : edges) {
        for (const auto& target : source.second) {
            result.emplace_back(source.first, target.first, target.second);
        }
    }
    return result;
}

void Graph::findAllPathsDFS(const std::string& current, const std::string& end,
    std::unordered_set<std::string>& visited, Path& currentPath,
    std::vector<Path>& result, bool primeOnly) const {
    if (current == end) {
        if (!primeOnly || isPrime(currentPath.totalWeight)) {
            result.push_back(currentPath);
        }
        return;
    }

    visited.insert(current);
    auto it = edges.find(current);
    if (it != edges.end()) {
        for (const auto& [next, weight] : it->second) {
            if (visited.find(next) == visited.end()) {
                currentPath.nodes.push_back(next);
                currentPath.totalWeight += weight;
                findAllPathsDFS(next, end, visited, currentPath, result, primeOnly);
                currentPath.nodes.pop_back();
                currentPath.totalWeight -= weight;
            }
        }
    }
    visited.erase(current);
}

std::vector<Path> Graph::findAllPaths(const std::string& start, const std::string& end, bool primeOnly) const {
    std::vector<Path> result;
    if (!hasNode(start) || !hasNode(end)) return result;

    Path currentPath;
    currentPath.nodes.push_back(start);
    std::unordered_set<std::string> visited;
    findAllPathsDFS(start, end, visited, currentPath, result, primeOnly);
    return result;
}

Path Graph::getShortestPath(const std::vector<Path>& paths) const {
    if (paths.empty()) return Path();
    return *std::min_element(paths.begin(), paths.end(),
        [](const Path& a, const Path& b) { return a.totalWeight < b.totalWeight; });
}

Path Graph::findPath(const std::string& start, const std::string& end) const {
    auto paths = findAllPaths(start, end);
    return paths.empty() ? Path() : paths[0];
}

Path Graph::findShortestPath(const std::string& start, const std::string& end) const {
    auto paths = findAllPaths(start, end);
    return getShortestPath(paths);
}

Path Graph::findPrimePath(const std::string& start, const std::string& end) const {
    auto paths = findAllPaths(start, end, true);
    return paths.empty() ? Path() : paths[0];
}

Path Graph::findShortestPrimePath(const std::string& start, const std::string& end) const {
    auto paths = findAllPaths(start, end, true);
    return getShortestPath(paths);
}

Path Graph::parallelPathHelper(const std::string& start, const std::string& end,
    std::function<Path(const std::string&, const std::string&)> findFunction) {
    const int NUM_TASKS = std::thread::hardware_concurrency();
    std::vector<std::future<Path>> futures;
    std::mutex mtx;
    Path bestPath;
    std::atomic<bool> found{ false };

    for (int i = 0; i < NUM_TASKS; i++) {
        futures.push_back(
            pool.enqueue([&, i]() {
                if (!found) {
                    Path path = findFunction(start, end);
                    if (!path.nodes.empty()) {
                        std::lock_guard<std::mutex> lock(mtx);
                        if (!found) {
                            bestPath = path;
                            found = true;
                        }
                    }
                }
                return Path();
                })
        );
    }

    for (auto& future : futures) {
        future.wait();
    }

    return bestPath;
}

Path Graph::parallelFindPath(const std::string& start, const std::string& end) {
    return parallelPathHelper(start, end,
        [this](const std::string& s, const std::string& e) { return findPath(s, e); });
}

Path Graph::parallelFindShortestPath(const std::string& start, const std::string& end) {
    return parallelPathHelper(start, end,
        [this](const std::string& s, const std::string& e) { return findShortestPath(s, e); });
}

Path Graph::parallelFindPrimePath(const std::string& start, const std::string& end) {
    return parallelPathHelper(start, end,
        [this](const std::string& s, const std::string& e) { return findPrimePath(s, e); });
}

Path Graph::parallelFindShortestPrimePath(const std::string& start, const std::string& end) {
    return parallelPathHelper(start, end,
        [this](const std::string& s, const std::string& e) { return findShortestPrimePath(s, e); });
}