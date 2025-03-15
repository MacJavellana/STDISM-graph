#pragma once
#include <string>
#include <vector>
#include <set>
#include <map>
#include <unordered_set>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <functional>
#include <future>

class ThreadPool {
private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;

public:
    ThreadPool(size_t threads);
    ~ThreadPool();

    template<class F>
    std::future<typename std::invoke_result<F>::type> enqueue(F&& f) {
        using return_type = typename std::invoke_result<F>::type;
        auto task = std::make_shared<std::packaged_task<return_type()>>(std::forward<F>(f));
        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            tasks.emplace([task]() { (*task)(); });
        }
        condition.notify_one();
        return res;
    }
};

struct Edge {
    std::string source;
    std::string target;
    uint64_t weight;
    Edge(const std::string& s, const std::string& t, uint64_t w)
        : source(s), target(t), weight(w) {
    }
};

struct Path {
    std::vector<std::string> nodes;
    uint64_t totalWeight;
    Path() : totalWeight(0) {}
};

class Graph {
private:
    std::set<std::string> nodes;
    std::map<std::string, std::map<std::string, uint64_t>> edges;
    mutable ThreadPool pool;

    bool isPrime(uint64_t n) const;
    std::vector<Path> findAllPaths(const std::string& start, const std::string& end, bool primeOnly = false) const;
    void findAllPathsDFS(const std::string& current, const std::string& end,
        std::unordered_set<std::string>& visited, Path& currentPath,
        std::vector<Path>& result, bool primeOnly) const;
    Path getShortestPath(const std::vector<Path>& paths) const;
    Path parallelPathHelper(const std::string& start, const std::string& end,
        std::function<Path(const std::string&, const std::string&)> findFunction);

public:
    Graph() : pool(8) {} // just adjust the no. of threads
    Graph(const Graph&) = delete;
    Graph& operator=(const Graph&) = delete;
    Graph(Graph&&) = default;
    Graph& operator=(Graph&&) = default;

    void addNode(const std::string& node);
    void addEdge(const std::string& source, const std::string& target, uint64_t weight);
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
