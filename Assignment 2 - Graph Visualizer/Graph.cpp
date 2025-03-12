#include "Graph.h"
#include <algorithm>
#include <queue>
#include <limits>
#include <thread>
#include <future>
#include <stack>

bool Graph::isPrime(int n) const {
    if (n <= 1) return false;
    for (int i = 2; i * i <= n; i++) {
        if (n % i == 0) return false;
    }
    return true;
}

void Graph::addNode(const std::string& node) {
    nodes.insert(node);
    if (edges.find(node) == edges.end()) {
        edges[node] = std::map<std::string, int>();
    }
}

void Graph::addEdge(const std::string& source, const std::string& target, int weight) {
    if (hasNode(source) && hasNode(target)) {
        edges[source][target] = weight;
    }
}

bool Graph::hasNode(const std::string& node) const {
    return nodes.find(node) != nodes.end();
}

bool Graph::hasEdge(const std::string& source, const std::string& target) const {
    auto it = edges.find(source);
    if (it != edges.end()) {
        return it->second.find(target) != it->second.end();
    }
    return false;
}

std::vector<std::string> Graph::getNodes() const {
    return std::vector<std::string>(nodes.begin(), nodes.end());
}

std::vector<Edge> Graph::getEdges() const {
    std::vector<Edge> edgeList;
    for (const auto& sourceEntry : edges) {
        for (const auto& targetEntry : sourceEntry.second) {
            edgeList.emplace_back(sourceEntry.first, targetEntry.first, targetEntry.second);
        }
    }
    return edgeList;
}

Path Graph::findPath(const std::string& start, const std::string& end) const {
    std::map<std::string, std::string> parent;
    std::map<std::string, int> distance;
    std::queue<std::string> q;

    q.push(start);
    parent[start] = "";
    distance[start] = 0;

    Path result;
    bool found = false;

    while (!q.empty() && !found) {
        std::string current = q.front();
        q.pop();

        const auto& neighbors = edges.at(current);
        for (const auto& neighborEntry : neighbors) {
            const std::string& neighbor = neighborEntry.first;
            int weight = neighborEntry.second;

            if (parent.find(neighbor) == parent.end()) {
                parent[neighbor] = current;
                distance[neighbor] = distance[current] + weight;
                q.push(neighbor);

                if (neighbor == end) {
                    found = true;
                    break;
                }
            }
        }
    }

    if (found) {
        std::string current = end;
        while (!current.empty()) {
            result.nodes.insert(result.nodes.begin(), current);
            current = parent[current];
        }
        result.totalWeight = distance[end];
    }

    return result;
}

Path Graph::findShortestPath(const std::string& start, const std::string& end) const {
    std::map<std::string, int> distance;
    std::map<std::string, std::string> parent;

    for (const auto& node : nodes) {
        distance[node] = std::numeric_limits<int>::max();
    }
    distance[start] = 0;

    std::priority_queue<std::pair<int, std::string>,
        std::vector<std::pair<int, std::string>>,
        std::greater<>> pq;
    pq.push(std::make_pair(0, start));

    while (!pq.empty()) {
        int dist = pq.top().first;
        std::string current = pq.top().second;
        pq.pop();

        if (current == end) break;
        if (dist > distance[current]) continue;

        const auto& neighbors = edges.at(current);
        for (const auto& neighborEntry : neighbors) {
            const std::string& neighbor = neighborEntry.first;
            int weight = neighborEntry.second;

            int newDist = distance[current] + weight;
            if (newDist < distance[neighbor]) {
                distance[neighbor] = newDist;
                parent[neighbor] = current;
                pq.push(std::make_pair(newDist, neighbor));
            }
        }
    }

    Path result;
    if (distance[end] != std::numeric_limits<int>::max()) {
        std::string current = end;
        while (!current.empty()) {
            result.nodes.insert(result.nodes.begin(), current);
            if (current == start) break;
            current = parent[current];
        }
        result.totalWeight = distance[end];
    }
    return result;
}

Path Graph::findPrimePath(const std::string& start, const std::string& end) const {
    std::vector<Path> allPaths;
    std::stack<std::pair<std::string, Path>> stack;
    std::set<std::string> visited;

    Path initialPath;
    initialPath.nodes.push_back(start);
    initialPath.totalWeight = 0;
    stack.push({ start, initialPath });
    visited.insert(start);

    while (!stack.empty()) {
        std::string current = stack.top().first;
        Path path = stack.top().second;
        stack.pop();

        if (current == end && isPrime(path.totalWeight)) {
            allPaths.push_back(path);
            continue;
        }

        const auto& neighbors = edges.at(current);
        for (const auto& neighborEntry : neighbors) {
            const std::string& neighbor = neighborEntry.first;
            int weight = neighborEntry.second;

            if (visited.find(neighbor) == visited.end()) {
                Path newPath = path;
                newPath.nodes.push_back(neighbor);
                newPath.totalWeight += weight;
                visited.insert(neighbor);
                stack.push({ neighbor, newPath });
            }
        }
    }

    if (!allPaths.empty()) {
        return allPaths[0];
    }
    return Path();
}

Path Graph::findShortestPrimePath(const std::string& start, const std::string& end) const {
    Path shortestPrime;
    shortestPrime.totalWeight = std::numeric_limits<int>::max();

    std::stack<std::pair<std::string, Path>> stack;
    std::set<std::string> visited;

    Path initialPath;
    initialPath.nodes.push_back(start);
    initialPath.totalWeight = 0;
    stack.push({ start, initialPath });
    visited.insert(start);

    while (!stack.empty()) {
        std::string current = stack.top().first;
        Path path = stack.top().second;
        stack.pop();

        if (current == end && isPrime(path.totalWeight) && path.totalWeight < shortestPrime.totalWeight) {
            shortestPrime = path;
            continue;
        }

        const auto& neighbors = edges.at(current);
        for (const auto& neighborEntry : neighbors) {
            const std::string& neighbor = neighborEntry.first;
            int weight = neighborEntry.second;

            if (visited.find(neighbor) == visited.end() &&
                path.totalWeight + weight < shortestPrime.totalWeight) {
                Path newPath = path;
                newPath.nodes.push_back(neighbor);
                newPath.totalWeight += weight;
                visited.insert(neighbor);
                stack.push({ neighbor, newPath });
            }
        }
    }

    if (shortestPrime.totalWeight == std::numeric_limits<int>::max()) {
        return Path();
    }
    return shortestPrime;
}

Path Graph::parallelFindPathHelper(const std::string& start, const std::string& end,
    std::function<Path(const std::string&, const std::string&)> findFunction) {
    std::vector<std::future<Path>> futures;
    std::mutex resultMutex;
    Path bestPath;
    std::atomic<bool> found{ false };

    const auto& startNeighbors = edges[start];
    for (const auto& neighborEntry : startNeighbors) {
        const std::string& neighbor = neighborEntry.first;
        int weight = neighborEntry.second;

        futures.push_back(std::async(std::launch::async, [&, neighbor, weight]() {
            if (!found) {
                Path path = findFunction(neighbor, end);
                if (!path.nodes.empty()) {
                    std::lock_guard<std::mutex> lock(resultMutex);
                    if (!found) {
                        path.nodes.insert(path.nodes.begin(), start);
                        path.totalWeight += weight;
                        bestPath = path;
                        found = true;
                    }
                }
            }
            return Path();
            }));
    }

    for (auto& future : futures) {
        future.wait();
    }

    return bestPath;
}

Path Graph::parallelFindPath(const std::string& start, const std::string& end) {
    return parallelFindPathHelper(start, end, [this](const std::string& s, const std::string& e) {
        return findPath(s, e);
        });
}

Path Graph::parallelFindShortestPath(const std::string& start, const std::string& end) {
    return parallelFindPathHelper(start, end, [this](const std::string& s, const std::string& e) {
        return findShortestPath(s, e);
        });
}

Path Graph::parallelFindPrimePath(const std::string& start, const std::string& end) {
    if (hasEdge(start, end)) {
        int directWeight = edges.at(start).at(end);
        if (isPrime(directWeight)) {
            Path directPath;
            directPath.nodes = { start, end };
            directPath.totalWeight = directWeight;
            return directPath;
        }
    }

    return parallelFindPathHelper(start, end, [this](const std::string& s, const std::string& e) {
        return findPrimePath(s, e);
        });
}

Path Graph::parallelFindShortestPrimePath(const std::string& start, const std::string& end) {
    if (hasEdge(start, end)) {
        int directWeight = edges.at(start).at(end);
        if (isPrime(directWeight)) {
            Path directPath;
            directPath.nodes = { start, end };
            directPath.totalWeight = directWeight;
            return directPath;
        }
    }

    return parallelFindPathHelper(start, end, [this](const std::string& s, const std::string& e) {
        return findShortestPrimePath(s, e);
        });
}