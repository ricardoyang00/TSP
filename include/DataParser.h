/**
 * @file DataParser.h
 * @brief Contains the declaration of the DataParser class.
 *
 */

#ifndef FEUP_DA_02_DATAPARSER_H
#define FEUP_DA_02_DATAPARSER_H

#include "Graph.h"
#include "Utilities.h"

/**
 * @class DataParser
 * @brief Parses input data files and populates data structures.
 *
 * This class is responsible for parsing input CSV files containing data graphs for TSP.
 */
class DataParser {
public:
    /**
     * @brief Default constructor for the DataParser class.
     */
    DataParser();

    /**
     * @brief Parses a CSV file containing a toy graph.
     * @param filePath Path to the CSV file.
     * @param graph Graph to populate.
     */
    static void ParseToyGraphs(const std::string& filePath, Graph<int>& graph);

    /**
     * @brief Parses a CSV file containing a toy tourism graph.
     * @param tourismPath Path to the tourism CSV file.
     * @param graph Graph to populate.
     */
    static void ParseToyGraphTourism(const std::string& tourismPath, Graph<int>& graph);

    /**
     * @brief Parses a CSV file containing a fully connected graph.
     * @param vertexPath Path to the vertex CSV file.
     * @param edgePath Path to the edge CSV file.
     * @param graph Graph to populate.
     * @param nodesNum Number of nodes in the graph.
     */
    static void ParseFullyConnectedGraph(const std::string& vertexPath, const std::string& edgePath, Graph<int>& graph, const int& nodesNum);

    /**
     * @brief Parses a CSV file containing a real world graph.
     * @param vertexPath Path to the vertex CSV file.
     * @param edgePath Path to the edge CSV file.
     * @param graph Graph to populate.
     */
    static void ParseRealWorldGraph(const std::string& vertexPath, const std::string& edgePath, Graph<int>& graph);
};


#endif
