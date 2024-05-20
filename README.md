# TSP - Traveling Salesman Problem

## Description
This project enables comparison between different solutions for the Traveling Salesman Problem (TSP), a renowned challenge in combinatorial optimization. The objective is to determine the shortest route that visits each city exactly once and returns to the starting city. Being [NP-hard](https://en.wikipedia.org/wiki/NP-hardness), the TSP lacks a polynomial-time solution algorithm.

[RESULTS SHEET](https://docs.google.com/spreadsheets/d/1I6d1NvNj34K96kJRol0O1ccQ8HP2-nnEXvQ0IW0alAs/edit?usp=sharing)

## Algorithms

### Backtracking
- Brute force algorithm that explores all possible routes.
- __🚀 Time Complexity:__ O(V!), where V is the number of vertices in the graph. 

### Triangular Approximation
- Creates a MST using Prim's algorithm and traverses it using a DFS.
- __🚀 Time Complexity:__ O((V+E) logV), where V is the number of vertices and E is the number of edges in the graph.

### Nearest Neighbor
- Greedy algorithm that selects the nearest unvisited vertex to the current vertex.
- __🚀 Time Complexity:__ O(V^2), where V is the number of vertices in the graph.

### K-Nearest Neighbor
- Greedy algorithm that selects the k nearest unvisited vertices to the current vertex.
- __🚀 Time Complexity:__ O(V^2 * logV + V * k), where V is the number of vertices in the graph.
<br></br>
💡 Tips: Increasing the value of K provides better chance to find a feasible path. However, the larger is K, the longer computation time is to complete the traversal.

### 2-Opt
- Give a solution path and iteratively swaps two edges to reduce the total distance of the path.
- __🚀 Time Complexity:__ O(V^2), where V is the number of vertices in the graph.

### 3-Opt
- Give a solution path and iteratively swaps three edges to reduce the total distance of the path.
- __🚀 Time Complexity:__ O(V^3), where V is the number of vertices in the graph.
<br></br>
📝 Note: The Opt algorithms that we implemented, uses as initial solution the nearest neighbor algorithm (NNA). Which means that the time complexity of the Opt algorithms is `max(Opt, NNA)`.

### Held-Karp
- Dynamic programming algorithm that uses the concept of state and transition.
- __🚀 Time Complexity:__ O(2^V * V^2), where V is the number of vertices in the graph.

### Ant Colony Optimization (ACO)
- Probabilistic technique that simulates the behavior of ants in finding paths from the colony to food.
- __🚀 Time Complexity:__ O(V^2 * numAnts * numIterations), where V is the number of vertices in the graph, numAnts is the number of ants, and numIterations is the number of iterations.

### TSP in Real World
We tailored the Triangular Approximation, Nearest Neighbor Algorithm (NNA), K-Nearest Neighbor (K-NN), and Ant Colony Optimization (ACO) to tackle real-world TSP scenarios. Users now have the flexibility to __select__ the starting vertex and determine whether to treat the graph as fully connected or not.

With these adjustments, our algorithms provide solutions even when the graphs __may not be fully connected__, by returning `feasible` or `not feasible`, ensuring that a solution is always provided.

## Reflection
The Ant Colony Optimization (ACO) algorithm performs well on larger datasets and is able to find good solutions due to its probabilistic nature and the use of both pheromone trails and heuristic information. However, the time complexity is dependent on the number of ants and iterations, which can be a limiting factor for very large problems or for a very high number of ants or iterations.
With a greater number of ants and iterations, we could potentially get more optimal solutions, but the time to run the algorithm would also increase.

Both the Triangular Approximation and Nearest Neighbor Algorithms demonstrate speed and efficacy, with the NNA edging slightly ahead providing marginally superior results in large dataset. Overall, both algorithms perfom incredibly well

The 2-Opt is very good for small dataset which always provides a better solution than Nearest Neighbor Algorithm (NNA). However, as the dataset grows larger, it becomes impractial due to its exponential increase in execution time.

Even though the 3-opt heuristic takes more time than the 2-opt heuristic, it doesn't consistently yield better results. From this observation, we can conclude that it's not an effective algorithm in this scenario.
