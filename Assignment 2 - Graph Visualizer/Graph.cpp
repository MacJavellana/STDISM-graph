#include "Graph.h"
#include <algorithm>
#include <cmath>
#include <random>
#include <chrono>
#include <thread>
#include <iostream>
#include <sstream>
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

bool Graph::isPrime(uint64_t n) const {
    if (n <= 1) return false;
    if (n == 2 || n == 3) return true; // 2 and 3 are prime
    if (n % 2 == 0 || n % 3 == 0) return false; // Eliminate even numbers and multiples of 3

    uint64_t limit = sqrt(n); // Compute sqrt(n) once
    for (int i = 5; i <= limit; i += 6) {
        if (n % i == 0 || n % (i + 2) == 0) return false;
    }
    return true;
}


void Graph::addNode(const std::string& node) {
    nodes.insert(node);
    node_info[node]; // Ensure NodeInfo exists
}

void Graph::addEdge(const std::string& source, const std::string& target, uint64_t weight) {

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

void Graph::addAgent(const std::string& agent_name, const std::string& initial_node) {
    size_t dash_pos = agent_name.find('-');
    if (dash_pos == std::string::npos) {
        return;
    }
    std::string x_str = agent_name.substr(0, dash_pos);
    int required_weight;
    try {
        required_weight = std::stoi(x_str);
    }
    catch (...) {
        return;
    }

    std::lock_guard<std::mutex> agent_lock(agents_mutex);
    agents.emplace_back(Agent{ agent_name, initial_node, x_str, required_weight });

    std::lock_guard<std::mutex> node_lock(node_info[initial_node].mtx);
    node_info[initial_node].agent_name = agent_name;
}

void Graph::simulateAgents() {
    logInitialNodes();
    logInitialEdges();
    logInitialAgents();

    std::vector<std::thread> threads;
    {
        std::lock_guard<std::mutex> lock(agents_mutex);
        for (const auto& agent : agents) {
            threads.emplace_back(&Graph::agentThread, this, agent);
        }
    }

    for (auto& t : threads) {
        if (t.joinable()) {
            t.join();
        }
    }
}

void Graph::log(const std::string& message) const {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << message << std::endl;
}

void Graph::logInitialNodes() const {
    std::ostringstream oss;
    oss << "Nodes:";
    for (const auto& node : nodes) {
        oss << " " << node;
    }
    log(oss.str());
}

void Graph::logInitialEdges() const {
    std::ostringstream oss;
    oss << "Edges:";
    for (const auto& source : edges) {
        for (const auto& target : source.second) {
            oss << " " << source.first << "->" << target.first << " (" << target.second << ")";
        }
    }
    log(oss.str());
}

void Graph::logInitialAgents() const {
    std::lock_guard<std::mutex> lock(agents_mutex);
    std::ostringstream oss;
    oss << "Agents:";
    for (const auto& agent : agents) {
        oss << " Agent " << agent.name << " at node " << agent.current_node << ",";
    }
    std::string result = oss.str();
    if (!agents.empty()) {
        result.pop_back(); // Remove trailing comma
    }
    log(result);
}

std::vector<Edge> Graph::getEdgesFrom(const std::string& source, int weight) const {
    std::vector<Edge> result;
    auto it = edges.find(source);
    if (it != edges.end()) {
        for (const auto& target_entry : it->second) {
            if (target_entry.second == weight) {
                result.emplace_back(source, target_entry.first, target_entry.second);
            }
        }
    }
    return result;
}
void Graph::logReachedDestination(const std::string& agent_name, const std::string& node) {
    std::ostringstream oss;
    oss << "Agent " << agent_name << " reaches node " << node << " and is removed from the graph.";
    log(oss.str());
}

Path Graph::findPathWithRequiredWeight(const std::string& start, const std::string& end, int requiredWeight) const {
    std::vector<Path> allPaths;
    Path currentPath;
    currentPath.nodes.push_back(start);
    std::unordered_set<std::string> visited;

    findPathWithWeightDFS(start, end, visited, currentPath, allPaths, requiredWeight);

    // Return the shortest valid path
    return getShortestPath(allPaths);
}

void Graph::findPathWithWeightDFS(const std::string& current, const std::string& end,
    std::unordered_set<std::string>& visited, Path& currentPath,
    std::vector<Path>& result, int requiredWeight) const {
    if (current == end) {
        result.push_back(currentPath);
        return;
    }

    visited.insert(current);
    auto it = edges.find(current);
    if (it != edges.end()) {
        for (const auto& [next, weight] : it->second) {
            if (visited.find(next) == visited.end() && weight == requiredWeight) {
                currentPath.nodes.push_back(next);
                currentPath.totalWeight += weight;
                findPathWithWeightDFS(next, end, visited, currentPath, result, requiredWeight);
                currentPath.nodes.pop_back();
                currentPath.totalWeight -= weight;
            }
        }
    }
    visited.erase(current);
}

bool Graph::tryLockPath(const Path& path, const std::string& agentName) {
    std::vector<std::unique_lock<std::mutex>> locks;
    std::vector<std::string> lockedNodes;

    // First, sort nodes to prevent deadlocks when locking
    std::vector<std::string> sortedNodes = path.nodes;
    std::sort(sortedNodes.begin(), sortedNodes.end());

    // Try to lock all nodes in the path
    for (const auto& node : sortedNodes) {
        std::unique_lock<std::mutex> lock(node_info[node].mtx);

        // If node is occupied by another agent, we can't lock the path
        if (!node_info[node].agent_name.empty() && node_info[node].agent_name != agentName) {
            // Release all locks we've acquired so far
            return false;
        }

        locks.push_back(std::move(lock));
        lockedNodes.push_back(node);
    }

    // Mark all nodes as reserved for this agent
    for (const auto& node : path.nodes) {
        if (node_info[node].agent_name.empty()) {
            node_info[node].reserved_by = agentName;
        }
    }

    // Release all locks
    for (auto& lock : locks) {
        lock.unlock();
    }

    return true;
}

void Graph::unlockRemainingNodes(const Path& path, const std::string& currentNode) {
    bool foundCurrent = false;

    for (const auto& node : path.nodes) {
        if (node == currentNode) {
            foundCurrent = true;
            continue;
        }

        if (foundCurrent) {
            std::lock_guard<std::mutex> lock(node_info[node].mtx);
            if (node_info[node].reserved_by == node_info[currentNode].agent_name) {
                node_info[node].reserved_by = "";
            }
        }
    }
}

void Graph::agentThread(Agent agent) {
    std::random_device rd;
    std::mt19937 g(rd());

    while (true) {
        // Check if agent has reached destination
        if (agent.current_node == agent.destination_node) {
            logReachedDestination(agent.name, agent.destination_node);
            {
                std::lock_guard<std::mutex> lock(node_info[agent.current_node].mtx);
                node_info[agent.current_node].agent_name = "";
            }
            {
                std::lock_guard<std::mutex> lock(agents_mutex);
                auto it = std::find_if(agents.begin(), agents.end(), [&](const Agent& a) {
                    return a.name == agent.name;
                    });
                if (it != agents.end()) {
                    agents.erase(it);
                }
            }
            return;
        }

        // Find a valid path from current node to destination
        Path path = findPathWithRequiredWeight(agent.current_node, agent.destination_node, agent.required_edge_weight);

        if (path.nodes.empty() || path.nodes.size() < 2) {
            log("Agent " + agent.name + " cannot find a valid path to destination.");
            return;
        }

        // Try to lock all nodes in the path
        bool pathLocked = tryLockPath(path, agent.name);

        if (pathLocked) {
            // Move along the locked path
            for (size_t i = 1; i < path.nodes.size(); i++) {
                std::string from = path.nodes[i - 1];
                std::string to = path.nodes[i];

                // Update agent position
                {
                    std::lock_guard<std::mutex> lock_from(node_info[from].mtx);
                    node_info[from].agent_name = "";
                }
                {
                    std::lock_guard<std::mutex> lock_to(node_info[to].mtx);
                    node_info[to].agent_name = agent.name;
                }

                log("Agent " + agent.name + " at node " + from + " jumps to node " + to + ".");
                agent.current_node = to;

                // If this is the destination, break
                if (to == agent.destination_node) {
                    break;
                }

                // Small delay between moves
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            // Unlock the remaining nodes in the path (if any)
            unlockRemainingNodes(path, agent.current_node);
        }
        else {
            // Path locking failed, wait and try again
            log("Agent " + agent.name + " couldn't secure a path, waiting to try again.");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}
