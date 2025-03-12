/*
#pragma once
#include <SFML/Graphics.hpp>
#include "Graph.h"
#include <map>
#include <cmath>
#define _USE_MATH_DEFINES
class GraphVisualizer {
private:
    const float NODE_RADIUS = 20.0f;
    const float WINDOW_WIDTH = 800.0f;
    const float WINDOW_HEIGHT = 600.0f;

    Graph& graph;
    sf::RenderWindow window;
    std::map<std::string, sf::Vector2f> nodePositions;

    void calculateNodePositions();
    void drawNodes();
    void drawEdges();
    void drawArrow(const sf::Vector2f& start, const sf::Vector2f& end);
    void drawNodeLabels();

public:
    GraphVisualizer(Graph& g);
    void run();
};

*/