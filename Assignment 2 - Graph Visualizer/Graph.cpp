#include "Graph.h"
#include <algorithm>

void Graph::addNode(const std::string& node) {
    nodes.insert(node);
    if (edges.find(node) == edges.end()) {
        edges[node] = std::set<std::string>();
    }
}

void Graph::addEdge(const std::string& source, const std::string& target) {
    if (hasNode(source) && hasNode(target)) {
        edges[source].insert(target);
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

std::vector<std::pair<std::string, std::string>> Graph::getEdges() const {
    std::vector<std::pair<std::string, std::string>> edgeList;
    for (const auto& source : edges) {
        for (const auto& target : source.second) {
            edgeList.push_back({ source.first, target });
        }
    }
    return edgeList;
}

std::vector<std::string> Graph::findPath(const std::string& start, const std::string& end) const {
    std::vector<std::string> shortestPath;
    std::map<std::string, std::string> parent;
    std::queue<std::string> currentLevel;

    currentLevel.push(start);
    parent[start] = "";

    while (!currentLevel.empty() && shortestPath.empty()) {
        std::queue<std::string> nextLevel;

        while (!currentLevel.empty()) {
            std::string current = currentLevel.front();
            currentLevel.pop();

            for (const auto& neighbor : edges.at(current)) {
                if (parent.find(neighbor) == parent.end()) {
                    parent[neighbor] = current;
                    nextLevel.push(neighbor);

                    if (neighbor == end) {
                        std::vector<std::string> path;
                        std::string node = end;
                        while (!node.empty()) {
                            path.insert(path.begin(), node);
                            node = parent[node];
                        }
                        shortestPath = path;
                        return shortestPath;
                    }
                }
            }
        }
        currentLevel = nextLevel;
    }
    return shortestPath;
}


bool Graph::parallelHasNode(const std::string& node) {
    std::vector<std::thread> threads;
    std::atomic<bool> found{ false };
    auto nodesList = getNodes();
    int segmentSize = std::max(1, static_cast<int>(nodesList.size()) / NUM_THREADS);

    for (int i = 0; i < NUM_THREADS; i++) {
        threads.push_back(std::thread([&, i]() {
            int start = i * segmentSize;
            int end = (i == NUM_THREADS - 1) ? nodesList.size() : (i + 1) * segmentSize;

            for (int j = start; j < end && !found; j++) {
                if (nodesList[j] == node) {
                    found = true;
                    break;
                }
            }
            }));
    }

    for (auto& thread : threads) {
        thread.join();
    }
    return found;
}

bool Graph::parallelHasEdge(const std::string& source, const std::string& target) {
    std::vector<std::thread> threads;
    std::atomic<bool> found{ false };
    auto edgesList = getEdges();
    int segmentSize = std::max(1, static_cast<int>(edgesList.size()) / NUM_THREADS);

    for (int i = 0; i < NUM_THREADS; i++) {
        threads.push_back(std::thread([&, i]() {
            int start = i * segmentSize;
            int end = (i == NUM_THREADS - 1) ? edgesList.size() : (i + 1) * segmentSize;

            for (int j = start; j < end && !found; j++) {
                if (edgesList[j].first == source && edgesList[j].second == target) {
                    found = true;
                    break;
                }
            }
            }));
    }

    for (auto& thread : threads) {
        thread.join();
    }
    return found;
}

std::vector<std::string> Graph::parallelFindPath(const std::string& start, const std::string& end) {
    std::mutex pathMutex;
    std::vector<std::string> shortestPath;
    std::atomic<int> shortestLength{ INT_MAX };
    std::map<std::string, std::string> parent;
    std::mutex parentMutex;

    // Use level-synchronized BFS
    std::queue<std::string> currentLevel;
    currentLevel.push(start);
    parent[start] = "";

    while (!currentLevel.empty() && shortestPath.empty()) {
        std::queue<std::string> nextLevel;
        std::vector<std::thread> threads;
        std::mutex levelMutex;

        // Process current level in parallel
        for (int i = 0; i < NUM_THREADS; i++) {
            threads.push_back(std::thread([&]() {
                while (true) {
                    std::string current;
                    {
                        std::lock_guard<std::mutex> lock(levelMutex);
                        if (currentLevel.empty()) break;
                        current = currentLevel.front();
                        currentLevel.pop();
                    }

                    for (const auto& neighbor : edges[current]) {
                        std::lock_guard<std::mutex> lock(parentMutex);
                        if (parent.find(neighbor) == parent.end()) {
                            parent[neighbor] = current;
                            nextLevel.push(neighbor);

                            if (neighbor == end) {
                                // Reconstruct path
                                std::vector<std::string> path;
                                std::string node = end;
                                while (!node.empty()) {
                                    path.insert(path.begin(), node);
                                    node = parent[node];
                                }
                                shortestPath = path;
                                return;
                            }
                        }
                    }
                }
                }));
        }

        for (auto& thread : threads) {
            thread.join();
        }

        currentLevel = nextLevel;
    }

    return shortestPath;
}
