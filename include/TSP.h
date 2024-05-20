/**
 * @file TSP.h
 * @brief Contains the declaration of the TSP class.
 *
 * This header file contains the declaration of the TSP class which is responsible for solving the Travelling Salesman Problem.
 */

#ifndef FEUP_DA_02_TSP_H
#define FEUP_DA_02_TSP_H

#include "DataParser.h"
#define DOUBLE_INF std::numeric_limits<double>::max()
#define LONG_LONG_INF std::numeric_limits<long long>::max()
#define MAX_ITERATIONS 5
#define ALPHA 1.0
#define BETA 5.0
#define EVAPORATION_RATE 0.5

/**
 * @class TSP
 * @brief Solves the Travelling Salesman Problem.
 */
class TSP {
private:
    /**
     * @brief Graph to perform the TSP on.
     */
    Graph<int> tspGraph;

    std::vector<std::vector<double>> pheromones;

    /**
     * @brief Prints the path of the TSP solution.
     * @param distance The total distance of the path.
     * @param path The path of the TSP solution.
     */
    template <typename T>
    void printPath(T distance, const std::vector<Vertex<int>*> &path);

    /**
     * @brief Auxiliary function for the backtracking algorithm.
     * @param currVertex The current vertex in the path.
     * @param currDistance The current total distance of the path.
     * @param path The current path.
     * @param visited A vector indicating which vertices have been visited.
     * @param minDistance The minimum distance found so far.
     * @param bestPath The best path found so far.
     */
    void backtrackingAlgorithmAux(Vertex<int>* currVertex, double currDistance, std::vector<Vertex<int>*> &path, std::vector<bool> &visited, double &minDistance, std::vector<Vertex<int>*> &bestPath);

    /**
     * @brief Performs a depth-first search traversal on the graph.
     * @param current The current vertex.
     * @param path The current path.
     */
    static void dfsTraversal(Vertex<int>* current, std::vector<Vertex<int>*>& path);

    /**
     * @brief Traverses the minimum spanning tree of the graph.
     * @param graph The graph.
     * @param start The starting vertex.
     * @param path The current path.
     */
    static void traverseMST(const Graph<int>& graph, Vertex<int>* start, std::vector<Vertex<int>*>& path);

    /**
     * @brief Calculates the total distance of a path, considering the graph as not fully connected.
     * @param path The path.
     * @param distance The total distance of the path.
     * @return True if a direct edge exists between all pairs of consecutive vertices in the path, false otherwise.
     */
    bool pathDistanceNotFullyConnected(const std::vector<Vertex<int>*>& path, long long& distance);

    /**
     * @brief Calculates the total distance of a path, considering the graph as fully connected.
     * @param path The path.
     * @return The total distance of the path.
     */
    long long pathDistanceFullyConnected(const std::vector<Vertex<int>*>& path);

    /**
     * @brief Finds a path to the origin vertex in the graph.
     * @param origin The origin vertex.
     * @param tour The current tour.
     * @return True if a path to the origin exists, false otherwise.
     */
    bool findPathToOrigin(Vertex<int>* origin, std::vector<Vertex<int>*>& tour);

    /**
     * @brief Constructs a path using the nearest neighbor heuristic.
     * @param origin The origin vertex.
     * @return The constructed path.
     */
    std::vector<Vertex<int>*> nearestNeighborPath(Vertex<int>* origin);

    /**
     * @brief Constructs a path using the k-nearest neighbors heuristic.
     * @param origin The origin vertex.
     * @param numNeighbors The number of nearest neighbors to consider.
     * @return The constructed path.
     */
    std::vector<Vertex<int>*> kNearestNeighborPath(Vertex<int>* origin, int numNeighbors);

    /**
     * @brief Applies a 3-opt swap to a path.
     * @param path The original path.
     * @param i The first index for the swap.
     * @param j The second index for the swap.
     * @param k The third index for the swap.
     * @return The new path after applying the 3-opt swap.
     */
    static std::vector<Vertex<int>*> applyThreeOptSwap(const std::vector<Vertex<int>*>& path, int i, int j, int k);

    /**
     * @brief Initializes the pheromone levels for the ant colony optimization algorithm.
     * @param initialPheromoneLevel The initial pheromone level.
     */
    void initializePheromones(double initialPheromoneLevel);

    /**
     * @brief Calculates the heuristic value for an edge in the graph.
     * @param s The source vertex.
     * @param t The target vertex.
     * @return The heuristic value.
     */
    static double calculateHeuristic(Vertex<int>* s, Vertex<int>* t);

    /**
     * @brief Constructs a solution for the ant colony optimization algorithm.
     * @param start The starting vertex.
     * @return The constructed solution.
     */
    std::vector<Vertex<int>*> constructSolution(Vertex<int>* start);

    /**
     * @brief Updates the pheromone levels based on the tours of all ants.
     * @param allTours The tours of all ants.
     * @param distances The total distances of all tours.
     */
    void updatePheromones(const std::vector<std::vector<Vertex<int>*>>& allTours, const std::vector<long long>& distances);

public:
    /**
     * @brief Default constructor.
     */
    TSP();

    /**
     * @brief Constructor that initializes the TSP graph.
     * @param graph The graph to be used for TSP.
     */
    explicit TSP(const Graph<int> &graph);

    /**
     * @brief Getter for the TSP graph.
     * @return The TSP graph.
     */
    Graph<int> getTspGraph();

    /**
     * @brief Solves the TSP using the backtracking algorithm.
     *
     * - Time Complexity: O(V!)
     * - Space Complexity: O(V)
     */
    void backtrackingAlgorithm();

    /**
     * @brief Solves the TSP using the Held-Karp algorithm.
     * @param origin The starting vertex for the TSP.
     *
     * - Time Complexity: O(V^2 * 2^V)
     * - Space Complexity: O(V * 2^V)
     */
    void heldKarp(const int& origin);

    /**
     * @brief Solves the TSP using the triangular approximation MST algorithm.
     * @param origin The starting vertex for the TSP.
     * @param fullyConnected Whether considering the graph as fully connected or not.
     *
     * - Time Complexity: O((V+E) * logV)
     * - Space Complexity: O(V)
     */
    void triangularApproximationMSTAlgorithm(int origin, bool fullyConnected);

    /**
     * @brief Solves the TSP using the nearest neighbor algorithm.
     * @param origin The starting vertex for the TSP.
     *
     * - Time Complexity: O(V^2)
     * - Space Complexity: O(V)
     */
    void nearestNeighborAlgorithm(const int& origin);

    /**
     * @brief Solves the TSP using the k-nearest neighbor algorithm.
     * @param origin The starting vertex for the TSP.
     * @param k The number of nearest neighbors to consider.
     *
     * - Time Complexity: O(V^2 * logV + V * k), where k is the number of nearest neighbors.
     * - Space Complexity: O(V)
     */
    void kNearestNeighborAlgorithm(const int& origin, int k);

    /**
     * @brief Solves the TSP using the 2-opt algorithm.
     * @param origin The starting vertex for the TSP.
     *
     * - Time Complexity: O(kV^2), where k is the number of iterations.
     * - Space Complexity: O(V)
     */
    void twoOptAlgorithm(const int& origin);

    /**
     * @brief Solves the TSP using the 3-opt algorithm.
     * @param origin The starting vertex for the TSP.
     *
     * - Time Complexity: O(kV^3), where k is the number of iterations.
     * - Space Complexity: O(V)
     */
    void threeOptAlgorithm(const int& origin);

    /**
     * @brief Solves the TSP using the ant colony optimization algorithm.
     * @param origin The starting vertex for the TSP.
     * @param numAnts The number of ants in the ant colony.
     * @param numIterations The number of iterations for the algorithm.
     * @param fullyConnected Whether considering the graph as fully connected or not.
     *
     * - Time Complexity: O(V^2 * numAnts * numIterations)
     * - Space Complexity: O(V^2)
     */
    void antColonyOptimization(const int& origin, int numAnts, int numIterations, bool fullyConnected);
};


#endif //FEUP_DA_02_TSP_H
