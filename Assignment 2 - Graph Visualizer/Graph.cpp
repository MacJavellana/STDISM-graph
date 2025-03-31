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
    node_info[node]; // Ensure NodeInfo exists
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
void Graph::agentThread(Agent agent) {
    std::random_device rd;
    std::mt19937 g(rd());
    int failed_attempts = 0;

    while (true) {
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

        auto possible_edges = getEdgesFrom(agent.current_node, agent.required_edge_weight);
        if (possible_edges.empty()) {
            log("Agent " + agent.name + " has no possible edges to move.");
            return;
        }

        std::shuffle(possible_edges.begin(), possible_edges.end(), g);

        bool moved = false;
        for (const auto& edge : possible_edges) {
            const std::string& target_node = edge.target;

            std::string first = agent.current_node < target_node ? agent.current_node : target_node;
            std::string second = agent.current_node < target_node ? target_node : agent.current_node;

            std::unique_lock<std::mutex> lock_first(node_info[first].mtx, std::defer_lock);
            std::unique_lock<std::mutex> lock_second(node_info[second].mtx, std::defer_lock);
            std::lock(lock_first, lock_second);

            if (node_info[target_node].agent_name.empty()) {
                node_info[agent.current_node].agent_name = "";
                node_info[target_node].agent_name = agent.name;
                log("Agent " + agent.name + " at node " + agent.current_node + " jumps to node " + target_node + ".");
                agent.current_node = target_node;
                moved = true;
                failed_attempts = 0;
                break;
            }
            else {
                log("Agent " + agent.name + " at node " + agent.current_node + " waits for node " + target_node + " to be free.");
            }
        }

        if (!moved) {
            failed_attempts++;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (failed_attempts >= 3) {
                log("Deadlock detected involving Agent " + agent.name + ".");
                log("Resolving deadlock for Agent " + agent.name + ".");
                failed_attempts = 0;
            }
        }
    }
}