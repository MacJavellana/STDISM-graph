#include "Graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

bool useParallel = true;

void parseGraphFile(const std::string& filename, Graph& graph) {
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cout << "Error opening file: " << filename << std::endl;
        return;
    }

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        // Remove leading/trailing whitespace
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        if (line[0] == '*') {
            std::string node = line.substr(1);
            node.erase(0, node.find_first_not_of(" \t"));
            graph.addNode(node);
        }
        else if (line[0] == '-') {
            std::string edgeInfo = line.substr(1);
            edgeInfo.erase(0, edgeInfo.find_first_not_of(" \t"));

            std::istringstream iss(edgeInfo);
            std::string source, target;
            uint64_t weight;
            
            iss >> source >> target >> weight;
            
            if (iss) {
                graph.addEdge(source, target, weight);
            }
        }
    }
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
    std::cout << " with weight/length=" << (uint64_t)path.totalWeight << std::endl;
}

void runTerminal(Graph& graph) {
    std::string command;
    std::cout << "Current mode: " << (useParallel ? "parallel" : "sequential") << "\n\n";

    while (true) {
        std::cout << "Enter command: ";
        std::getline(std::cin, command);
        std::istringstream iss(command);
        std::string cmd;
        iss >> cmd;

        if (cmd == "exit") {
            break;
        }
        else if (cmd == "toggle") {
            useParallel = !useParallel;
            std::cout << "Switched to " << (useParallel ? "parallel" : "sequential") << " mode" << std::endl;
        }
        else if (cmd == "nodes") {
            auto nodes = graph.getNodes();
            for (const auto& node : nodes) {
                std::cout << "* " << node << std::endl;
            }
        }
        else if (cmd == "node") {
            std::string node;
            iss >> node;
            std::cout << (graph.hasNode(node) ? "yes" : "no") << std::endl;
        }
        else if (cmd == "edges") {
            auto edges = graph.getEdges();
            for (const auto& edge : edges) {
                std::cout << "- " << edge.source << " " << edge.target
                    << " " << edge.weight << std::endl;
            }
        }
        else if (cmd == "edge") {
            std::string source, target;
            iss >> source >> target;
            std::cout << (graph.hasEdge(source, target) ? "yes" : "no") << std::endl;
        }
        else if (cmd == "path") {
            std::string start, end;
            iss >> start >> end;
            Path path = useParallel ?
                graph.parallelFindPath(start, end) :
                graph.findPath(start, end);
            printPath(path, "path");
        }
        else if (cmd == "shortest-path") {
            std::string start, end;
            iss >> start >> end;
            Path path = useParallel ?
                graph.parallelFindShortestPath(start, end) :
                graph.findShortestPath(start, end);
            printPath(path, "shortest path");
        }
        else if (cmd == "prime-path") {
            std::string start, end;
            iss >> start >> end;
            Path path = useParallel ?
                graph.parallelFindPrimePath(start, end) :
                graph.findPrimePath(start, end);
            printPath(path, "prime path");
        }
        else if (cmd == "shortest-prime-path") {
            std::string start, end;
            iss >> start >> end;
            Path path = useParallel ?
                graph.parallelFindShortestPrimePath(start, end) :
                graph.findShortestPrimePath(start, end);
            printPath(path, "shortest prime path");
        }
        else {
            std::cout << "Unknown command" << std::endl;
        }
    }
}

int main(int argc, char* argv[]) {
    std::string filename;
    if (argc > 1) {
        filename = argv[1];
    }
    else {
        std::cout << "Enter graph file name: ";
        std::getline(std::cin, filename);
    }

    Graph graph;  // Create graph first
    parseGraphFile(filename, graph);  // Pass by reference
    runTerminal(graph);

    return 0;
}
