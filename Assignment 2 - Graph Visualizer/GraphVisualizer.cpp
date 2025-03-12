
/*
#include "GraphVisualizer.h"
#include <cmath>



//create window of width 800 and height 600
//then call CalculateNodePositions
GraphVisualizer::GraphVisualizer(Graph& g) : graph(g) {
    sf::VideoMode mode(WINDOW_WIDTH,WINDOW_HEIGHT);
    window.create(mode, "Graph Visualizer");
    window.setFramerateLimit(60);
    calculateNodePositions();
}



// assign each node to a position in a circular manner 
void GraphVisualizer::calculateNodePositions() {
    auto nodes = graph.getNodes();
    int nodeCount = static_cast<int>(nodes.size());
    float angleStep = 2.0f * 3.14159f / nodeCount; //evenly distribute each node 
    float radius = std::min(WINDOW_WIDTH, WINDOW_HEIGHT) * 0.35f;


    //asign each node to their respective location
    for (int i = 0; i < nodeCount; i++) {
        float angle = i * angleStep;
        float x = WINDOW_WIDTH / 2 + radius * std::cos(angle);
        float y = WINDOW_HEIGHT / 2 + radius * std::sin(angle);
        nodePositions[nodes[i]] = sf::Vector2f(x, y);
    }
}

void GraphVisualizer::drawArrow(const sf::Vector2f& start, const sf::Vector2f& end) {

    //find distance (length) between start to end
    sf::Vector2f direction(end.x - start.x, end.y - start.y);
    float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    direction.x /= length;
    direction.y /= length;


    //offset arrow so it doesn't overlap
    sf::Vector2f arrowEnd(end.x - direction.x * NODE_RADIUS,
        end.y - direction.y * NODE_RADIUS);
    sf::Vector2f arrowStart(start.x + direction.x * NODE_RADIUS,
        start.y + direction.y * NODE_RADIUS);


    //draw line from start to end
    sf::RectangleShape line;
    line.setSize(sf::Vector2f(length - 2 * NODE_RADIUS, 2.0f));
    line.setPosition(arrowStart);
    line.setRotation(std::atan2(direction.y, direction.x) * 180.0f / 3.14159f);
    line.setFillColor(sf::Color::Blue);
    window.draw(line);


    //create arrow shape
    sf::ConvexShape arrow;
    arrow.setPointCount(3);

    float arrowSize = 10.0f;
    sf::Vector2f perpendicular(-direction.y, direction.x);



    //creating the points for the arrow head
    arrow.setPoint(0, arrowEnd);
    arrow.setPoint(1, sf::Vector2f(
        arrowEnd.x - direction.x * arrowSize + perpendicular.x * arrowSize * 0.5f,
        arrowEnd.y - direction.y * arrowSize + perpendicular.y * arrowSize * 0.5f));
    arrow.setPoint(2, sf::Vector2f(
        arrowEnd.x - direction.x * arrowSize - perpendicular.x * arrowSize * 0.5f,
        arrowEnd.y - direction.y * arrowSize - perpendicular.y * arrowSize * 0.5f));


    //draw arrowhead
    arrow.setFillColor(sf::Color::Red);
    window.draw(arrow);
}

void GraphVisualizer::drawNodes() {
    for (const auto& nodePair : nodePositions) {
        sf::CircleShape circle(NODE_RADIUS);
        circle.setPosition(nodePair.second.x - NODE_RADIUS,
            nodePair.second.y - NODE_RADIUS);
        circle.setFillColor(sf::Color::White);
        circle.setOutlineThickness(2.0f);
        circle.setOutlineColor(sf::Color::Black);
        window.draw(circle);
    }
}


//draw node
//font of arial
//fill white for node
//outline black for node
void GraphVisualizer::drawNodeLabels() {
    static sf::Font font;
    static bool fontLoaded = false;

    if (!fontLoaded) {
        if (!font.loadFromFile("C:\\Windows\\Fonts\\arial.ttf")) {
            // If Arial isn't available, you can try loading any other font file
            return;
        }
        fontLoaded = true;
    }


    for (const auto& nodePair : nodePositions) {
        sf::Text text(nodePair.first, font, 20);
        text.setFillColor(sf::Color::Black);

        sf::FloatRect textRect = text.getGlobalBounds();
        text.setPosition(
            nodePair.second.x - textRect.width / 2.0f,
            nodePair.second.y - textRect.height / 2.0f);

        window.draw(text);
    }
}


//draw edges which will call drawArrow()
void GraphVisualizer::drawEdges() {
    auto edges = graph.getEdges();
    for (const auto& edge : edges) {
        const auto& startPos = nodePositions[edge.first];
        const auto& endPos = nodePositions[edge.second];
        drawArrow(startPos, endPos);
    }
}

void GraphVisualizer::run() {
    while (window.isOpen()) {
        sf::Event event{};
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::White);
        drawEdges();
        drawNodes();
        drawNodeLabels();
        window.display();
    }
}

*/