// Mikestanley Ochieng Odwour
// FEE3 2553 2024
 
#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <limits>
#include <string>
#include <algorithm>
#include <ctime>

using namespace std;

class Edge;

class Node {
public:
    int id;
    string name;
    string type; 
    vector<Edge*> edges;

    Node(int id, const string& name, const string& type = "") : id(id), name(name), type(type) {}
};

class Edge {
public:
    Node* destination;
    double cost;
    string mode;
    int timeWindowStart; 
    int timeWindowEnd;

    Edge(Node* dest, double cost, const string& mode, int start = 0, int end = 2359)
        : destination(dest), cost(cost), mode(mode), timeWindowStart(start), timeWindowEnd(end) {}
};

class Graph {
private:
    map<int, Node*> nodes;

public:
    void addNode(int id, const string& name, const string& type = "");
    void addEdge(int from, int to, double cost, const string& mode, int start = 0, int end = 2359);
    void printGraph() const;
    vector<Node*> shortestPath(int startId, int endId, int currentTime);
    void printPath(const vector<Node*>& path) const;
    bool hasNode(int id) const;
    void preloadCity();
    void listPOIs() const;
    ~Graph();
};

void Graph::addNode(int id, const string& name, const string& type) {
    if (nodes.find(id) == nodes.end()) {
        nodes[id] = new Node(id, name, type);
        cout << "Node added: " << name << " (ID: " << id << ", Type: " << type << ")\n";
    } else {
        cout << "Node with ID " << id << " already exists.\n";
    }
}

void Graph::addEdge(int from, int to, double cost, const string& mode, int start, int end) {
    if (!hasNode(from) || !hasNode(to)) {
        cout << "Invalid node ID(s).\n";
        return;
    }
    Node* fromNode = nodes[from];
    Node* toNode = nodes[to];
    fromNode->edges.push_back(new Edge(toNode, cost, mode, start, end));
    cout << "Edge added from " << fromNode->name << " to " << toNode->name
         << " (" << mode << ", Cost: " << cost << ", Available: " << start << " to " << end << ")\n";
}

void Graph::printGraph() const {
    cout << "\nCity Graph:\n";
    for (const auto& pair : nodes) {
        const Node* node = pair.second;
        cout << node->name << " (" << node->type << "):\n";
        for (const Edge* edge : node->edges) {
            cout << "  -> " << edge->destination->name
                 << " | Cost: " << edge->cost
                 << " | Mode: " << edge->mode
                 << " | Time: " << edge->timeWindowStart << "-" << edge->timeWindowEnd << endl;
        }
    }
}

vector<Node*> Graph::shortestPath(int startId, int endId, int currentTime) {
    map<int, double> dist;
    map<int, Node*> prev;
    auto cmp = [](pair<double, Node*> a, pair<double, Node*> b) { return a.first > b.first; };
    priority_queue<pair<double, Node*>, vector<pair<double, Node*>>, decltype(cmp)> pq(cmp);

    if (!hasNode(startId) || !hasNode(endId)) {
        cout << "Invalid node ID(s).\n";
        return {};
    }

    for (auto& pair : nodes)
        dist[pair.first] = numeric_limits<double>::infinity();

    dist[startId] = 0;
    pq.push({0, nodes[startId]});

    while (!pq.empty()) {
        Node* current = pq.top().second;
        pq.pop();

        if (current->id == endId) break;

        for (Edge* edge : current->edges) {
            if (currentTime < edge->timeWindowStart || currentTime > edge->timeWindowEnd)
                continue;

            Node* neighbor = edge->destination;
            double alt = dist[current->id] + edge->cost;
            if (alt < dist[neighbor->id]) {
                dist[neighbor->id] = alt;
                prev[neighbor->id] = current;
                pq.push({alt, neighbor});
            }
        }
    }

    vector<Node*> path;
    if (dist[endId] == numeric_limits<double>::infinity()) {
        cout << "No path found.\n";
        return path;
    }

    for (Node* at = nodes[endId]; at != nullptr; at = prev[at->id]) {
        path.push_back(at);
        if (prev.find(at->id) == prev.end()) break;
    }
    reverse(path.begin(), path.end());
    return path;
}

void Graph::printPath(const vector<Node*>& path) const {
    if (path.empty()) return;
    cout << "Shortest Route with Landmarks: ";
    for (size_t i = 0; i < path.size(); ++i) {
        cout << path[i]->name;
        if (!path[i]->type.empty()) cout << " [" << path[i]->type << "]";
        if (i != path.size() - 1) cout << " -> ";
    }
    cout << endl;
}

bool Graph::hasNode(int id) const {
    return nodes.find(id) != nodes.end();
}

void Graph::listPOIs() const {
    cout << "\nPoints of Interest:\n";
    for (const auto& pair : nodes) {
        if (pair.second->type == "POI")
            cout << " - " << pair.second->name << " (ID: " << pair.first << ")\n";
    }
}

void Graph::preloadCity() {
    addNode(1, "Central Park", "Landmark");
    addNode(2, "Main Library", "POI");
    addNode(3, "City Center", "Station");
    addNode(4, "Airport", "Station");
    addNode(5, "Tech University", "POI");

    addEdge(1, 2, 4, "walk", 600, 2200);
    addEdge(2, 3, 3, "bike", 600, 2000);
    addEdge(1, 3, 6, "bus", 700, 2300);
    addEdge(3, 4, 10, "train", 500, 2330);
    addEdge(4, 5, 5, "drive", 0, 2359);
    addEdge(2, 5, 8, "bus", 800, 2100);
}

Graph::~Graph() {
    for (auto& pair : nodes) {
        for (Edge* edge : pair.second->edges)
            delete edge;
        delete pair.second;
    }
}

void menu() {
    cout << "\n==== City Navigation System ====" << endl;
    cout << "1. Add New Location" << endl;
    cout << "2. Add New Route" << endl;
    cout << "3. Display City Map" << endl;
    cout << "4. Find Shortest Path" << endl;
    cout << "5. Show Points of Interest" << endl;
    cout << "6. Exit" << endl;
    cout << "Choose an option: ";
}

void handleAddNode(Graph& city) {
    int id;
    string name, type;
    cout << "Enter location ID: ";
    cin >> id;
    cin.ignore();
    cout << "Enter location name: ";
    getline(cin, name);
    cout << "Enter type (e.g., POI, Landmark, Station): ";
    getline(cin, type);
    city.addNode(id, name, type);
}

void handleAddEdge(Graph& city) {
    int from, to, start, end;
    double cost;
    string mode;
    cout << "Enter source node ID: ";
    cin >> from;
    cout << "Enter destination node ID: ";
    cin >> to;
    cout << "Enter cost: ";
    cin >> cost;
    cout << "Enter transport mode: ";
    cin >> mode;
    cout << "Enter available start time (e.g., 600 for 6AM): ";
    cin >> start;
    cout << "Enter available end time (e.g., 2200 for 10PM): ";
    cin >> end;
    city.addEdge(from, to, cost, mode, start, end);
}

void handleShortestPath(Graph& city) {
    int start, end;
    time_t now = time(0);
    tm* local = localtime(&now);
    int currentTime = local->tm_hour * 100 + local->tm_min;

    cout << "Enter start node ID: ";
    cin >> start;
    cout << "Enter destination node ID: ";
    cin >> end;

    auto path = city.shortestPath(start, end, currentTime);
    city.printPath(path);
}

int main() {
    Graph city;
    city.preloadCity();

    int choice;
    do {
        menu();
        cin >> choice;
        switch (choice) {
            case 1: handleAddNode(city); break;
            case 2: handleAddEdge(city); break;
            case 3: city.printGraph(); break;
            case 4: handleShortestPath(city); break;
            case 5: city.listPOIs(); break;
            case 6: cout << "Exiting system.\n"; break;
            default: cout << "Invalid option. Try again.\n";
        }
    } while (choice != 6);

    return 0;
}
