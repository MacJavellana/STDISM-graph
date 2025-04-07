#include "Graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <thread>
#include <filesystem>

#pragma comment(lib, "ws2_32.lib")

const int PORT = 8080;
const std::string GRAPH_FOLDER = ".\\graphs\\";
const int BUFFER_SIZE = 4096;

// Function to parse a graph file and load it into a Graph object
void parseGraphFile(const std::string& filename, Graph& graph) {
    std::ifstream file(filename);
    std::string line;
    if (!file.is_open()) {
        std::cout << "Error opening file: " << filename << std::endl;
        return;
    }

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        if (line[0] == '*') {
            std::string nodePart = line.substr(1);
            std::istringstream iss(nodePart);
            std::string nodeName, agentName;
            iss >> nodeName;
            if (iss >> agentName) {
                graph.addAgent(agentName, nodeName);
            }
            graph.addNode(nodeName);
        }
        else if (line[0] == '-') {
            std::string edgeInfo = line.substr(1);
            edgeInfo.erase(0, edgeInfo.find_first_not_of(" \t"));
            std::istringstream iss(edgeInfo);
            std::string source, target;
            int weight;
            if (iss >> source >> target >> weight) {
                graph.addEdge(source, target, weight);
            }
        }
    }
}

// Function to format a path as a string
std::string formatPath(const Path& path, const std::string& pathType) {
    std::ostringstream oss;
    if (path.nodes.empty()) {
        oss << "No " << pathType << " exists";
        return oss.str();
    }

    oss << pathType << ": ";
    for (size_t i = 0; i < path.nodes.size(); i++) {
        oss << path.nodes[i];
        if (i < path.nodes.size() - 1) oss << " -> ";
    }
    oss << " with weight/length=" << (uint64_t)path.totalWeight;
    return oss.str();
}

// Function to handle receiving a file from client
bool receiveFile(SOCKET client_socket, const std::string& filename) {
    char buffer[BUFFER_SIZE];
    std::ofstream file(filename, std::ios::binary);

    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    // First receive the file size
    int64_t fileSize;
    recv(client_socket, (char*)&fileSize, sizeof(fileSize), 0);

    // Send acknowledgment
    const char* ack = "ACK";
    send(client_socket, ack, (int)strlen(ack), 0);

    // Receive the file content
    int64_t bytesReceived = 0;
    int n;

    while (bytesReceived < fileSize) {
        n = recv(client_socket, buffer, min(BUFFER_SIZE, (int)(fileSize - bytesReceived)), 0);
        if (n <= 0) {
            std::cerr << "Error receiving file data" << std::endl;
            file.close();
            return false;
        }
        file.write(buffer, n);
        bytesReceived += n;
    }

    file.close();
    return true;
}

// Function to handle a client connection
void handleClient(SOCKET client_socket) {
    char buffer[BUFFER_SIZE];
    std::string currentGraphFile;
    // Create a pointer to Graph so we can recreate it when needed
    std::unique_ptr<Graph> graph = std::make_unique<Graph>();

    // Create graphs directory if it doesn't exist
    std::filesystem::create_directory(GRAPH_FOLDER);

    while (true) {
        // Clear buffer
        memset(buffer, 0, BUFFER_SIZE);

        // Receive command from client
        int bytesReceived = recv(client_socket, buffer, BUFFER_SIZE, 0);
        if (bytesReceived <= 0) {
            std::cout << "Client disconnected" << std::endl;
            break;
        }

        std::string command(buffer);
        std::cout << "Received command: " << command << std::endl;

        std::istringstream iss(command);
        std::string cmd;
        iss >> cmd;

        if (cmd == "disconnect") {
            std::cout << "Client requested disconnect" << std::endl;
            const char* response = "Disconnected";
            send(client_socket, response, (int)strlen(response), 0);
            break;
        }
        else if (cmd == "upload") {
            std::string filename;
            iss >> filename;

            // Send acknowledgment to start file transfer
            const char* ready = "READY";
            send(client_socket, ready, (int)strlen(ready), 0);

            // Receive the file
            std::string fullPath = GRAPH_FOLDER + filename;
            if (receiveFile(client_socket, fullPath)) {
                currentGraphFile = fullPath;
                // Create a new Graph object instead of trying to assign
                graph = std::make_unique<Graph>();
                parseGraphFile(fullPath, *graph);

                const char* response = "File uploaded and graph loaded successfully";
                send(client_socket, response, (int)strlen(response), 0);
            }
            else {
                const char* response = "Failed to receive file";
                send(client_socket, response, (int)strlen(response), 0);
            }
        }
        else if (cmd == "use") {
            std::string filename;
            iss >> filename;
            std::string fullPath = GRAPH_FOLDER + filename;

            if (std::filesystem::exists(fullPath)) {
                currentGraphFile = fullPath;
                // Create a new Graph object instead of trying to assign
                graph = std::make_unique<Graph>();
                parseGraphFile(fullPath, *graph);

                const char* response = "Graph loaded successfully";
                send(client_socket, response, (int)strlen(response), 0);
            }
            else {
                const char* response = "Graph file not found";
                send(client_socket, response, (int)strlen(response), 0);
            }
        }
        else if (cmd == "prime-path") {
            if (currentGraphFile.empty()) {
                const char* response = "No graph loaded. Please upload or use a graph file first.";
                send(client_socket, response, (int)strlen(response), 0);
                continue;
            }

            std::string start, end;
            iss >> start >> end;

            // Execute the prime path query
            Path path = graph->parallelFindPrimePath(start, end);
            std::string result = formatPath(path, "prime path");

            // Send the result back to the client
            send(client_socket, result.c_str(), (int)result.length(), 0);
        }
        else if (cmd == "shortest-path") {
            if (currentGraphFile.empty()) {
                const char* response = "No graph loaded. Please upload or use a graph file first.";
                send(client_socket, response, (int)strlen(response), 0);
                continue;
            }

            std::string start, end;
            iss >> start >> end;

            // Execute the shortest path query
            Path path = graph->parallelFindShortestPath(start, end);
            std::string result = formatPath(path, "shortest path");

            // Send the result back to the client
            send(client_socket, result.c_str(), (int)result.length(), 0);
        }
        else {
            const char* response = "Unknown command";
            send(client_socket, response, (int)strlen(response), 0);
        }
    }

    closesocket(client_socket);
}

int main() {
    WSADATA wsaData;
    SOCKET server_fd = INVALID_SOCKET;
    SOCKET client_socket = INVALID_SOCKET;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    // Initialize Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed" << std::endl;
        return 1;
    }

    // Create socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == INVALID_SOCKET) {
        std::cerr << "Socket creation error: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return 1;
    }

    // Set up the address structure
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Bind the socket
    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) == SOCKET_ERROR) {
        std::cerr << "Bind failed: " << WSAGetLastError() << std::endl;
        closesocket(server_fd);
        WSACleanup();
        return 1;
    }

    // Listen for connections
    if (listen(server_fd, SOMAXCONN) == SOCKET_ERROR) {
        std::cerr << "Listen failed: " << WSAGetLastError() << std::endl;
        closesocket(server_fd);
        WSACleanup();
        return 1;
    }

    std::cout << "Server started on port " << PORT << std::endl;
    std::cout << "Waiting for connections..." << std::endl;

    // Accept and handle client connections
    while (true) {
        if ((client_socket = accept(server_fd, (struct sockaddr*)&address, &addrlen)) == INVALID_SOCKET) {
            std::cerr << "Accept failed: " << WSAGetLastError() << std::endl;
            continue;
        }

        std::cout << "Client connected" << std::endl;

        // Handle client in a new thread
        std::thread clientThread(handleClient, client_socket);
        clientThread.detach();
    }

    // Clean up
    closesocket(server_fd);
    WSACleanup();

    return 0;
}