/**
 * @file Graph.h
 * @brief Contains the declaration of the Graph class and its related classes: Vertex and Edge.
 * It provides functionalities for graph manipulation.
 *
 */

#ifndef PROJ_DA_02_GRAPH
#define PROJ_DA_02_GRAPH

#include <iostream>
#include <queue>
#include <limits>
#include <fstream>
#include <algorithm>
#include <unordered_set>
#include "MutablePriorityQueue.h"

template <class T>
class Edge;

#define INF std::numeric_limits<double>::max()

/************************* Vertex  **************************/

/**
 * @class Vertex
 * @brief Represents a city or location
 *
 * This class represents a vertex in a graph data structure.
 */
template <class T>
class Vertex {
public:
    Vertex(T in);
    Vertex(T in, std::string label);
    Vertex(T in, double longitude, double latitude);
    bool operator<(Vertex<T> & vertex) const; // // required by MutablePriorityQueue

    T getInfo() const;
    std::vector<Edge<T> *> getAdj() const;
    bool isVisited() const;
    bool isProcessing() const;
    unsigned int getIndegree() const;
    double getDist() const;
    Edge<T> *getPath() const;
    std::vector<Edge<T> *> getIncoming() const;
    std::string getLabel() const;
    double getLongitude();
    double getLatitude();

    void setInfo(T info);
    void setVisited(bool visited);
    void setProcesssing(bool processing);
    void setIndegree(unsigned int indegree);
    void setDist(double dist);
    void setPath(Edge<T> *path);
    void setCoordinates(double longitude, double latitude);
    Edge<T> * addEdge(Vertex<T> *dest, double w);
    bool removeEdge(T in);
    void removeOutgoingEdges();
    void setLabel(std::string);
    int getQueueIndex() const;
    Vertex<T>* nearestNeighbor() const;

    friend class MutablePriorityQueue<Vertex>;
protected:
    T info;                // info node
    std::vector<Edge<T> *> adj;  // outgoing edges
    std::string label;  //label for toy graphs, null otherwise

    double longitude = 0;
    double latitude = 0;

    // auxiliary fields
    bool visited = false; // used by DFS, BFS, Prim ...
    bool processing = false; // used by isDAG (in addition to the visited attribute)
    unsigned int indegree; // used by topsort
    double dist = 0;
    Edge<T> *path = nullptr;

    std::vector<Edge<T> *> incoming; // incoming edges

    int queueIndex = 0; 		// required by MutablePriorityQueue and UFDS

    void deleteEdge(Edge<T> *edge);
};

/********************** Edge  ****************************/

/**
 * @class Edge
 * @brief Represents the connection between the the cities or locations
 *
 * This class represents an edge in a graph data structure.
 */
template <class T>
class Edge {
public:
    Edge(Vertex<T> *orig, Vertex<T> *dest, double w);

    Vertex<T> * getDest() const;
    double getWeight() const;
    bool isSelected() const;
    Vertex<T> * getOrig() const;
    Edge<T> *getReverse() const;
    double getFlow() const;

    void setSelected(bool selected);
    void setReverse(Edge<T> *reverse);
    void setFlow(double flow);

    void setDest(Vertex<T>* v);
    void setWeight(double w);
protected:
    Vertex<T> * dest; // destination vertex
    double weight; // edge weight, can also be used for capacity

    // auxiliary fields
    bool selected = false;

    // used for bidirectional edges
    Vertex<T> *orig;
    Edge<T> *reverse = nullptr;

    double flow; // for flow-related problems
};

/********************** Graph  ****************************/

/**
 * @class Graph
 * @brief Represents the TSP network
 *
 * This class represents a graph data structure and provides functionalities for graph manipulation and algorithms.
 */
template <class T>
class Graph {
public:
    ~Graph();
    /*
    * Auxiliary function to find a vertex with a given the content.
    */
    Vertex<T> *findVertex(const T &in) const;
    /*
     *  Adds a vertex with a given content or info (in) to a graph (this).
     *  Returns true if successful, and false if a vertex with that content already exists.
     */
    bool addVertex(const T &in);
    bool addVertexLabel(const T &in, const std::string& label);
    bool addVertexCoordinates(const T &in, const double& longitude, const double& latitude);
    bool removeVertex(const T &in);

    /*
     * Adds an edge to a graph (this), given the contents of the source and
     * destination vertices and the edge weight (w).
     * Returns true if successful, and false if the source or destination vertex does not exist.
     */
    bool addEdge(const T &sourc, const T &dest, double w);
    bool removeEdge(const T &source, const T &dest);
    bool addBidirectionalEdge(const T &sourc, const T &dest, double w);

    int getNumVertex() const;
    std::vector<Vertex<T> *> getVertexSet() const;

    std:: vector<T> dfs() const;
    std:: vector<T> dfs(const T & source) const;
    void dfsVisit(Vertex<T> *v,  std::vector<T> & res) const;
    std::vector<T> bfs(const T & source) const;

    bool isDAG() const;
    bool dfsIsDAG(Vertex<T> *v) const;
    std::vector<T> topsort() const;
    Graph<T> convertToMST() const;
    void setAllNotVisited() const;
    void twoOptSwap(Vertex<T> *u, Vertex<T> *v, Vertex<T> *x, Vertex<T> *y) const;
    void printGraph(std::string filename) const;
protected:
    std::vector<Vertex<T> *> vertexSet;    // vertex set

    double ** distMatrix = nullptr;   // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;   // path matrix for Floyd-Warshall

    /*
     * Finds the index of the vertex with a given content.
     */
    int findVertexIdx(const T &in) const;
};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);


/************************* Vertex  **************************/

template <class T>
Vertex<T>::Vertex(T in): info(in) {}

template <class T>
Vertex<T>::Vertex(T in, std::string label) {
    this->info = in;
    this->label = label;
}
template <class T>
Vertex<T>::Vertex(T in, double longitude, double latitude) {
    this->info = in;
    this->longitude = longitude;
    this->latitude = latitude;
}

/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
template <class T>
Edge<T> * Vertex<T>::addEdge(Vertex<T> *d, double w) {
    auto newEdge = new Edge<T>(this, d, w);
    adj.push_back(newEdge);
    d->incoming.push_back(newEdge);
    return newEdge;
}

/*
 * Auxiliary function to remove an outgoing edge (with a given destination (d))
 * from a vertex (this).
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T>
bool Vertex<T>::removeEdge(T in) {
    bool removedEdge = false;
    auto it = adj.begin();
    while (it != adj.end()) {
        Edge<T> *edge = *it;
        Vertex<T> *dest = edge->getDest();
        if (dest->getInfo() == in) {
            it = adj.erase(it);
            deleteEdge(edge);
            removedEdge = true; // allows for multiple edges to connect the same pair of vertices (multigraph)
        }
        else {
            it++;
        }
    }
    return removedEdge;
}

/*
 * Auxiliary function to remove an outgoing edge of a vertex.
 */
template <class T>
void Vertex<T>::removeOutgoingEdges() {
    auto it = adj.begin();
    while (it != adj.end()) {
        Edge<T> *edge = *it;
        it = adj.erase(it);
        deleteEdge(edge);
    }
}

template <class T>
bool Vertex<T>::operator<(Vertex<T> & vertex) const {
    return this->dist < vertex.dist;
}

template <class T>
T Vertex<T>::getInfo() const {
    return this->info;
}

template <class T>
std::vector<Edge<T>*> Vertex<T>::getAdj() const {
    return this->adj;
}

template <class T>
bool Vertex<T>::isVisited() const {
    return this->visited;
}

template <class T>
bool Vertex<T>::isProcessing() const {
    return this->processing;
}

template <class T>
unsigned int Vertex<T>::getIndegree() const {
    return this->indegree;
}

template <class T>
double Vertex<T>::getDist() const {
    return this->dist;
}

template <class T>
Edge<T> *Vertex<T>::getPath() const {
    return this->path;
}

template <class T>
std::vector<Edge<T> *> Vertex<T>::getIncoming() const {
    return this->incoming;
}

template <class T>
std::string Vertex<T>::getLabel() const {
    return this->label;
}
template <class T>
double Vertex<T>::getLongitude() {
    return this->longitude;
}

template <class T>
double Vertex<T>::getLatitude() {
    return this->latitude;
}

template <class T>
void Vertex<T>::setInfo(T in) {
    this->info = in;
}

template <class T>
void Vertex<T>::setVisited(bool visited) {
    this->visited = visited;
}

template <class T>
void Vertex<T>::setProcesssing(bool processing) {
    this->processing = processing;
}

template <class T>
void Vertex<T>::setIndegree(unsigned int indegree) {
    this->indegree = indegree;
}

template <class T>
void Vertex<T>::setDist(double dist) {
    this->dist = dist;
}

template <class T>
void Vertex<T>::setPath(Edge<T> *path) {
    this->path = path;
}

template <class T>
void Vertex<T>::setLabel(std::string label) {
    this->label = label;
}
template <class T>
void Vertex<T>::setCoordinates(double longitude, double latitude) {
    this->longitude = longitude;
    this->latitude = latitude;
}

template <class T>
int Vertex<T>::getQueueIndex() const {
    return queueIndex;
}

template <class T>
void Vertex<T>::deleteEdge(Edge<T> *edge) {
    Vertex<T> *dest = edge->getDest();
    // Remove the corresponding edge from the incoming list
    auto it = dest->incoming.begin();
    while (it != dest->incoming.end()) {
        if ((*it)->getOrig()->getInfo() == info) {
            it = dest->incoming.erase(it);
        }
        else {
            it++;
        }
    }
    delete edge;
}

/**
 * Finds the nearest neighbor vertex connected to the current vertex.
 * @tparam T
 * @return Pointer to the nearest neighbor vertex, or nullptr if no neighbors exist.
 *
 * - Time Complexity: O(V)
 */
template<class T>
Vertex<T>* Vertex<T>::nearestNeighbor() const {
    double minDist = INF;
    Vertex<T>* nearestNode = nullptr;
    for (auto e : adj) {
        auto w = e->getDest();
        if (!w->isVisited() && e->getWeight() < minDist) {
            minDist = e->getWeight();
            nearestNode = w;
        }
    }

    return nearestNode;
}

/********************** Edge  ****************************/

template <class T>
Edge<T>::Edge(Vertex<T> *orig, Vertex<T> *dest, double w): orig(orig), dest(dest), weight(w) {}

template <class T>
Vertex<T> * Edge<T>::getDest() const {
    return this->dest;
}

template <class T>
double Edge<T>::getWeight() const {
    return this->weight;
}

template <class T>
Vertex<T> * Edge<T>::getOrig() const {
    return this->orig;
}

template <class T>
Edge<T> *Edge<T>::getReverse() const {
    return this->reverse;
}

template <class T>
bool Edge<T>::isSelected() const {
    return this->selected;
}

template <class T>
double Edge<T>::getFlow() const {
    return flow;
}

template <class T>
void Edge<T>::setSelected(bool selected) {
    this->selected = selected;
}

template <class T>
void Edge<T>::setReverse(Edge<T> *reverse) {
    this->reverse = reverse;
}

template <class T>
void Edge<T>::setFlow(double flow) {
    this->flow = flow;
}

template <class T>
void Edge<T>::setDest(Vertex<T>* v) {
    this->dest = v;
}

template <class T>
void Edge<T>::setWeight(double w) {
    this->weight = w;
}

/********************** Graph  ****************************/

template <class T>
int Graph<T>::getNumVertex() const {
    return vertexSet.size();
}

template <class T>
std::vector<Vertex<T> *> Graph<T>::getVertexSet() const {
    return vertexSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
template <class T>
Vertex<T> * Graph<T>::findVertex(const T &in) const {
    for (auto v : vertexSet)
        if (v->getInfo() == in)
            return v;
    return nullptr;
}

/*
 * Finds the index of the vertex with a given content.
 */
template <class T>
int Graph<T>::findVertexIdx(const T &in) const {
    for (unsigned i = 0; i < vertexSet.size(); i++)
        if (vertexSet[i]->getInfo() == in)
            return i;
    return -1;
}
/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
template <class T>
bool Graph<T>::addVertex(const T &in) {
    if (findVertex(in) != nullptr)
        return false;
    vertexSet.push_back(new Vertex<T>(in));
    return true;
}

template <class T>
bool Graph<T>::addVertexLabel(const T &in, const std::string& label) {
    if (findVertex(in) != nullptr)
        return false;
    vertexSet.push_back(new Vertex<T>(in, label));
    return true;
}

template <class T>
bool Graph<T>::addVertexCoordinates(const T &in, const double& longitude, const double& latitude) {
    if (findVertex(in) != nullptr)
        return false;
    vertexSet.push_back(new Vertex<T>(in, longitude, latitude));
    return true;
}

/*
 *  Removes a vertex with a given content (in) from a graph (this), and
 *  all outgoing and incoming edges.
 *  Returns true if successful, and false if such vertex does not exist.
 */
template <class T>
bool Graph<T>::removeVertex(const T &in) {
    for (auto it = vertexSet.begin(); it != vertexSet.end(); it++) {
        if ((*it)->getInfo() == in) {
            auto v = *it;
            v->removeOutgoingEdges();
            for (auto u : vertexSet) {
                u->removeEdge(v->getInfo());
            }
            vertexSet.erase(it);
            delete v;
            return true;
        }
    }
    return false;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
template <class T>
bool Graph<T>::addEdge(const T &sourc, const T &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    v1->addEdge(v2, w);
    return true;
}

/*
 * Removes an edge from a graph (this).
 * The edge is identified by the source (sourc) and destination (dest) contents.
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T>
bool Graph<T>::removeEdge(const T &sourc, const T &dest) {
    Vertex<T> * srcVertex = findVertex(sourc);
    if (srcVertex == nullptr) {
        return false;
    }
    return srcVertex->removeEdge(dest);
}

template <class T>
bool Graph<T>::addBidirectionalEdge(const T &sourc, const T &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    auto e1 = v1->addEdge(v2, w);
    auto e2 = v2->addEdge(v1, w);
    e1->setReverse(e2);
    e2->setReverse(e1);
    return true;
}

/****************** DFS ********************/

/*
 * Performs a depth-first search (dfs) traversal in a graph (this).
 * Returns a vector with the contents of the vertices by dfs order.
 */
template <class T>
std::vector<T> Graph<T>::dfs() const {
    std::vector<T> res;
    for (auto v : vertexSet)
        v->setVisited(false);
    for (auto v : vertexSet)
        if (!v->isVisited())
            dfsVisit(v, res);
    return res;
}

/*
 * Performs a depth-first search (dfs) in a graph (this) from the source node.
 * Returns a vector with the contents of the vertices by dfs order.
 */
template <class T>
std::vector<T> Graph<T>::dfs(const T & source) const {
    std::vector<int> res;
    // Get the source vertex
    auto s = findVertex(source);
    if (s == nullptr) {
        return res;
    }
    // Set that no vertex has been visited yet
    for (auto v : vertexSet) {
        v->setVisited(false);
    }
    // Perform the actual DFS using recursion
    dfsVisit(s, res);

    return res;
}

/*
 * Auxiliary function that visits a vertex (v) and its adjacent, recursively.
 * Updates a parameter with the list of visited node contents.
 */
template <class T>
void Graph<T>::dfsVisit(Vertex<T> *v, std::vector<T> & res) const {
    v->setVisited(true);
    res.push_back(v->getInfo());
    for (auto & e : v->getAdj()) {
        auto w = e->getDest();
        if (!w->isVisited()) {
            dfsVisit(w, res);
        }
    }
}

/****************** BFS ********************/
/*
 * Performs a breadth-first search (bfs) in a graph (this), starting
 * from the vertex with the given source contents (source).
 * Returns a vector with the contents of the vertices by bfs order.
 */
template <class T>
std::vector<T> Graph<T>::bfs(const T & source) const {
    std::vector<int> res;
    // Get the source vertex
    auto s = findVertex(source);
    if (s == nullptr) {
        return res;
    }

    // Set that no vertex has been visited yet
    for (auto v : vertexSet) {
        v->setVisited(false);
    }

    // Perform the actual BFS using a queue
    std::queue<Vertex<T> *> q;
    q.push(s);
    s->setVisited(true);
    while (!q.empty()) {
        auto v = q.front();
        q.pop();
        res.push_back(v->getInfo());
        for (auto & e : v->getAdj()) {
            auto w = e->getDest();
            if ( ! w->isVisited()) {
                q.push(w);
                w->setVisited(true);
            }
        }
    }
    return res;
}

/****************** isDAG  ********************/
/*
 * Performs a depth-first search in a graph (this), to determine if the graph
 * is acyclic (acyclic directed graph or DAG).
 * During the search, a cycle is found if an edge connects to a vertex
 * that is being processed in the stack of recursive calls (see theoretical classes).
 * Returns true if the graph is acyclic, and false otherwise.
 */

template <class T>
bool Graph<T>::isDAG() const {
    for (auto v : vertexSet) {
        v->setVisited(false);
        v->setProcesssing(false);
    }
    for (auto v : vertexSet) {
        if (! v->isVisited()) {
            if ( ! dfsIsDAG(v) ) return false;
        }
    }
    return true;
}

/**
 * Auxiliary function that visits a vertex (v) and its adjacent, recursively.
 * Returns false (not acyclic) if an edge to a vertex in the stack is found.
 */
template <class T>
bool Graph<T>::dfsIsDAG(Vertex<T> *v) const {
    v->setVisited(true);
    v->setProcesssing(true);
    for (auto e : v->getAdj()) {
        auto w = e->getDest();
        if (w->isProcessing()) return false;
        if (! w->isVisited()) {
            if (! dfsIsDAG(w)) return false;
        }
    }
    v->setProcesssing(false);
    return true;
}

/****************** toposort ********************/
//=============================================================================
// Exercise 1: Topological Sorting
//=============================================================================
// TODO
/*
 * Performs a topological sorting of the vertices of a graph (this).
 * Returns a vector with the contents of the vertices by topological order.
 * If the graph has cycles, returns an empty vector.
 * Follows the algorithm described in theoretical classes.
 */

template<class T>
std::vector<T> Graph<T>::topsort() const {
    std::vector<int> res;

    for (auto v : vertexSet) {
        v->setIndegree(0);
    }
    for (auto v : vertexSet) {
        for (auto e : v->getAdj()) {
            unsigned int indegree = e->getDest()->getIndegree();
            e->getDest()->setIndegree(indegree + 1);
        }
    }

    std::queue<Vertex<T> *> q;
    for (auto v : vertexSet) {
        if (v->getIndegree() == 0) {
            q.push(v);
        }
    }

    while( !q.empty() ) {
        Vertex<T> * v = q.front();
        q.pop();
        res.push_back(v->getInfo());
        for(auto e : v->getAdj()) {
            auto w = e->getDest();
            w->setIndegree(w->getIndegree() - 1);
            if(w->getIndegree() == 0) {
                q.push(w);
            }
        }
    }

    if ( res.size() != vertexSet.size() ) {
        //std::cout << "Impossible topological ordering!" << std::endl;
        res.clear();
        return res;
    }

    return res;
}

/**
 * This function uses Prim's algorithm with a priority queue (implemented as a MutablePriorityQueue) to construct the MST.
 * Each vertex is inserted into and extracted from the priority queue once, and each edge is examined once.
 *
 * @tparam T Class type of the Graph.
 * @return MST graph
 *
 * - Time Complexity: O((V+E)logV)
 */
template <class T>
Graph<T> Graph<T>::convertToMST() const {
    Graph<T> mst;
    MutablePriorityQueue<Vertex<T>> pq;

    for (auto v : vertexSet) {
        v->setVisited(false);
        v->setDist(std::numeric_limits<double>::max());
        mst.addVertexCoordinates(v->getInfo(), v->getLongitude(), v->getLatitude());
    }

    Vertex<T> *startVertex = vertexSet[0];
    startVertex->setDist(0);
    pq.insert(startVertex);

    while (!pq.empty()) {
        Vertex<T> *currentVertex = pq.extractMin();
        if (currentVertex->isVisited()) {
            continue;
        }
        currentVertex->setVisited(true);

        if (currentVertex->getPath() != nullptr) {
            mst.addBidirectionalEdge(currentVertex->getPath()->getOrig()->getInfo(), currentVertex->getInfo(), currentVertex->getPath()->getWeight());
        }

        for (auto edge : currentVertex->getAdj()) {
            Vertex<T> *adjVertex = edge->getDest();
            if (!adjVertex->isVisited() && edge->getWeight() < adjVertex->getDist()) {
                adjVertex->setDist(edge->getWeight());
                adjVertex->setPath(edge);
                if (adjVertex->getQueueIndex() == 0) {
                    pq.insert(adjVertex);
                } else {
                    pq.decreaseKey(adjVertex);
                }
            }
        }
    }

    return mst;
}

/**
 * Iterates through the graph nodes and set them all to NOT-VISITED
 * @tparam T Class type of the graph
 *
 * - Time Complexity: O(V)
 */
template <class T>
void Graph<T>::setAllNotVisited() const {
    for (auto v : vertexSet) {
        v->setVisited(false);
    }
}

template<class T>
void Graph<T>::twoOptSwap(Vertex<T> *u, Vertex<T> *v, Vertex<T> *x, Vertex<T> *y) const {
    Edge<T> *ux = nullptr, *vx = nullptr, *xy = nullptr, *yx = nullptr;

    for (Edge<T> *edge : u->getAdj()) {
        if (edge->getDest() == x) {
            ux = edge;
        } else if (edge->getDest() == v) {
            vx = edge;
        }
    }

    for (Edge<T> *edge : x->getAdj()) {
        if (edge->getDest() == y) {
            xy = edge;
        } else if (edge->getDest() == u) {
            yx = edge;
        }
    }

    if (ux && vx && xy && yx) {
        double weight_ux = ux->getWeight();
        double weight_vx = vx->getWeight();
        double weight_xy = xy->getWeight();
        double weight_yx = yx->getWeight();

        ux->setDest(v);
        vx->setDest(u);
        xy->setDest(u);
        yx->setDest(v);

        ux->setWeight(weight_vx);
        vx->setWeight(weight_ux);
        xy->setWeight(weight_yx);
        yx->setWeight(weight_xy);
    }
}


/**
 * Writes the graph information to a text file.
 * Mainly for debug purpose.
 *
 * @tparam T Class type of the Graph.
 * @param filename Path to the text file.
 *
 * - Time Complexity: O(VE)
 */
template<class T>
void Graph<T>::printGraph(std::string filename) const {
    std::ofstream outputFile(filename);
    if (outputFile.is_open()) {
        outputFile << "Vertices: " << vertexSet.size() << std::endl;
        outputFile << "(target vertex, edge weight)\n" << std::endl;
        for (auto v: vertexSet) {
            outputFile << "Vertex [" << v->getInfo() << "] :" << std::endl;
            outputFile << "     Adjacent edges: ";
            for (auto edge : v->getAdj()) {
                outputFile << "(" << edge->getDest()->getInfo() << "," << edge->getWeight() << ") ";
            }
            outputFile << std::endl;
        }
        outputFile.close();
    }
}

inline void deleteMatrix(int **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

inline void deleteMatrix(double **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

template <class T>
Graph<T>::~Graph() {
    deleteMatrix(distMatrix, vertexSet.size());
    deleteMatrix(pathMatrix, vertexSet.size());
}

// make a deep copy of a graph

/**
 * @brief Creates a deep of graph of the original one.
 * @tparam T Class type of the Graph.
 * @param originalGraph Original Graph.
 * @return Copy Graph.
 *
 * - Time Complexity: O(V+E)
 */
template <class T>
Graph<T> deepGraphCopy (Graph<T> originalGraph){
    Graph<T> graphCopy;
    for (const auto& v : originalGraph.getVertexSet()) {
        graphCopy.addVertex(v->getInfo());
    }

    for (const auto& vertex : originalGraph.getVertexSet()) {
        for (const auto& edge : vertex->getAdj()) {
            auto orig = graphCopy.findVertex(edge->getOrig()->getInfo());
            auto dest = graphCopy.findVertex(edge->getDest()->getInfo());

            graphCopy.addEdge(orig->getInfo(), dest->getInfo(), edge->getWeight());
        }
    }

    for (const auto& vertex : originalGraph.getVertexSet()) {
        for (const auto& edge : vertex->getAdj()) {
            if (edge->getReverse() != nullptr) {
                auto orig = graphCopy.findVertex(edge->getOrig()->getInfo());
                auto dest = graphCopy.findVertex(edge->getDest()->getInfo());

                graphCopy.removeEdge(orig->getInfo(), dest->getInfo());
                graphCopy.removeEdge(dest->getInfo(), orig->getInfo());
                graphCopy.addBidirectionalEdge(orig->getInfo(), dest->getInfo(), edge->getWeight());
            }
        }
    }

    return graphCopy;
}

#endif /* PROJ_DA_02_GRAPH */