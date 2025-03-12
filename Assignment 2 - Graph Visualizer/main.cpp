#include "Graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

bool useParallel = true;
Graph parseGraphFile(const std::string& filename) {
    Graph graph;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return graph;
    }

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        char type;
        iss >> type;

        if (type == '*') {
            std::string node;
            iss >> node;
            graph.addNode(node);
        }
        else if (type == '-') {
            std::string source, target;
            int weight;
            if (!(iss >> source >> target >> weight)) continue;
            graph.addEdge(source, target, weight);
        }
    }

    return graph;
}

void printPath(const Path& path, const std::string& pathType) {
    if (path.nodes.empty()) {
        std::cout << "No " << pathType << " exists" << std::endl;
        return;
    }

    std::cout << pathType << ": ";
    for (size_t i = 0; i < path.nodes.size(); i++) {
        std::cout << path.nodes[i];
        if (i < path.nodes.size() - 1) std::cout << " -> ";
    }
    std::cout << " with weight/length=" << path.totalWeight << std::endl;
}

void runTerminal(Graph& graph) {
    std::string command;
    std::cout << "Graph Terminal Started\n";
    std::cout << "Available commands:\n  nodes\n  node x\n  edges\n  edge a b\n";
    std::cout << "  path a b\n  shortest-path a b\n  prime-path a b\n  shortest-prime-path a b\n";
    std::cout << "  toggle (switches between parallel/sequential)\n  exit\n";
    std::cout << "Current mode: " << (useParallel ? "parallel" : "sequential") << "\n";

    while (true) {
        std::cout << "\nEnter command: ";
        std::getline(std::cin, command);
        std::istringstream iss(command);
        std::string cmd;
        iss >> cmd;

        if (cmd == "exit") break;
        else if (cmd == "toggle") {
            useParallel = !useParallel;
            std::cout << "Switched to " << (useParallel ? "parallel" : "sequential") << " mode\n";
        }
        else if (cmd == "nodes") {
            auto nodes = graph.getNodes();
            for (const auto& node : nodes) std::cout << node << "\n";
        }
        else if (cmd == "edges") {
            auto edges = graph.getEdges();
            for (const auto& edge : edges) {
                std::cout << edge.source << " " << edge.target << " " << edge.weight << "\n";
            }
        }
        else if (cmd == "path" || cmd == "shortest-path" || cmd == "prime-path" || cmd == "shortest-prime-path") {
            std::string start, end;
            if (!(iss >> start >> end)) {
                std::cout << "Invalid arguments for " << cmd << "\n";
                continue;
            }
            Path path = (useParallel ?
                (cmd == "path" ? graph.parallelFindPath(start, end) :
                    cmd == "shortest-path" ? graph.parallelFindShortestPath(start, end) :
                    cmd == "prime-path" ? graph.parallelFindPrimePath(start, end) :
                    graph.parallelFindShortestPrimePath(start, end))
                :
                (cmd == "path" ? graph.findPath(start, end) :
                    cmd == "shortest-path" ? graph.findShortestPath(start, end) :
                    cmd == "prime-path" ? graph.findPrimePath(start, end) :
                    graph.findShortestPrimePath(start, end)));
            printPath(path, cmd);
        }
        else {
            std::cout << "Invalid command\n";
        }
    }
}

int main(int argc, char* argv[]) {
    std::string filename;
    if (argc > 1) filename = argv[1];
    else {
        std::cout << "Enter graph file name: ";
        std::getline(std::cin, filename);
    }

    Graph graph = parseGraphFile(filename);
    runTerminal(graph);
    return 0;
}

