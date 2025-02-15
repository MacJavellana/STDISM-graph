#include "Graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include "GraphVisualizer.h"
bool useParallel = true;
Graph parseGraphFile(const std::string& filename) {
    Graph graph;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cout << "Error opening file: " << filename << std::endl;
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
            iss >> source >> target;
            graph.addEdge(source, target);
        }
    }

    return graph;
}

void printNodes(const std::vector<std::string>& nodes) {
    std::cout << "Nodes: ";
    for (const auto& node : nodes) {
        std::cout << node << " ";
    }
    std::cout << std::endl;
}

void printEdges(const std::vector<std::pair<std::string, std::string>>& edges) {
    std::cout << "Edges: ";
    for (const auto& edge : edges) {
        std::cout << "(" << edge.first << "," << edge.second << ") ";
    }
    std::cout << std::endl;
}

void runTerminal(Graph& graph) {
    std::string command;
    std::cout << "Graph Terminal Started\n";
    std::cout << "Available commands: nodes, node x, edges, edge a b, exit\n";
    std::cout << "Current mode: " << (useParallel ? "parallel" : "sequential") << "\n";


    while (true) {
        std::cout << "\nEnter command: ";
        std::getline(std::cin, command);
        std::istringstream iss(command);
        std::string cmd;
        iss >> cmd;

        if (cmd == "exit") {
            break;
        }
        else if (cmd == "toggle") {
            useParallel = !useParallel;
            std::cout << "Switched to " << (useParallel ? "parallel" : "sequential") << " mode\n";
        }
        else if (cmd == "node") {
            std::string node;
            iss >> node;
            bool exists = useParallel ?
                graph.parallelHasNode(node) :
                graph.hasNode(node);
            std::cout << "Node " << node << (exists ? " exists" : " does not exist")
                << " in the graph\n";
        }
        else if (cmd == "edge") {
            std::string source, target;
            iss >> source >> target;
            bool exists = useParallel ?
                graph.parallelHasEdge(source, target) :
                graph.hasEdge(source, target);
            std::cout << "Edge (" << source << "," << target << ")"
                << (exists ? " exists" : " does not exist")
                << " in the graph\n";
        }
        else if (cmd == "path") {
            std::string start, end;
            iss >> start >> end;
            auto path = useParallel ?
                graph.parallelFindPath(start, end) :
                graph.findPath(start, end);
            if (!path.empty()) {
                std::cout << "Path found: ";
                for (size_t i = 0; i < path.size(); i++) {
                    std::cout << path[i];
                    if (i < path.size() - 1) std::cout << " -> ";
                }
                std::cout << std::endl;
            }
            else {
                std::cout << "No path from " << start << " to " << end << std::endl;
            }
        }
        else {
            std::cout << "Invalid command\n";
        }
    }
}

int main() {
    std::string filename;
    std::cout << "Enter graph file name: ";
    std::getline(std::cin, filename);

    Graph graph = parseGraphFile(filename);

    /*
    GraphVisualizer visualizer(graph);
    visualizer.run();
    */
    runTerminal(graph);

    return 0;
}
