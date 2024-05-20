/**
 * @file Menu.h
 * @brief Contains the declaration of the Menu class.
 *
 * This header file contains the declaration of the Menu class which is responsible for handling the user interface
 * for the TSP.
 */

#ifndef FEUP_DA_02_MENU_H
#define FEUP_DA_02_MENU_H

#include "TSP.h"
#include <chrono>

/**
 * @class Menu
 * @brief Handles the user interface for the TSP.
 *
 * This class is responsible for handling the user interface for the TSP. It allows the user to select a graph to
 * perform the TSP on and select the algorithm to use.
 */
class Menu {
public:
    /**
     * @brief Default constructor for the Menu class.
     */
    Menu();

    /**
     * @brief Runs the user interface for the TSP.
     */
    void run();

    /**
     * @brief Initializes the Menu class allowing user to choose the graph to perform.
     * @return 0 if successful, 1 otherwise.
     */
    int init();

private:
    /**
     * @brief Graph to perform the TSP on.
     */
    Graph<int> graph;

    /**
     * @brief Current graph's label being used.
     */
    std::string currentGraph = "-";

    /**
     * @brief TSP object to perform the TSP on the graph.
     */
    TSP tsp;

    /**
     * @brief Prints the menu for the user with the available options.
     * @param menuIndex Vector containing the menu options.
     */
    static void printMenu(const std::vector<std::string>& menuIndex);

    /**
     * @brief Prints a single text question for the user and waits for an input choice.
     * @param choice User's choice.
     * @param text Text to be displayed.
     * @return 0 if successful, 1 otherwise.
     */
    static int waitInput(int& choice, const std::string& text);

    /**
     * @brief Waits for the user to press a key.
     */
    static void waitPress();

    /**
     * @brief Clears the screen.
     */
    static void clearScreen();

    /**
     * @brief Calculates and prints the execution time of the algorithm.
     * @param start beginning of the execution.
     * @param end end of the execution.
     */
    static void executionTime(std::chrono::time_point<std::chrono::high_resolution_clock> start, std::chrono::time_point<std::chrono::high_resolution_clock> end);

    /**
     * @brief Path to the CSV files containing the extra fully connected graphs nodes.
     */
    std::string extraFullyConnectedGraphsNodes = "data/Extra_Fully_Connected_Graphs/nodes.csv";

    /**
     * @brief Paths to the CSV files containing the extra fully connected graphs edges.
     */
    std::vector<std::string> extraFullyConnectedGraphsEdges = {
            "data/Extra_Fully_Connected_Graphs/edges_25.csv",
            "data/Extra_Fully_Connected_Graphs/edges_50.csv",
            "data/Extra_Fully_Connected_Graphs/edges_75.csv",
            "data/Extra_Fully_Connected_Graphs/edges_100.csv",
            "data/Extra_Fully_Connected_Graphs/edges_200.csv",
            "data/Extra_Fully_Connected_Graphs/edges_300.csv",
            "data/Extra_Fully_Connected_Graphs/edges_400.csv",
            "data/Extra_Fully_Connected_Graphs/edges_500.csv",
            "data/Extra_Fully_Connected_Graphs/edges_600.csv",
            "data/Extra_Fully_Connected_Graphs/edges_700.csv",
            "data/Extra_Fully_Connected_Graphs/edges_800.csv",
            "data/Extra_Fully_Connected_Graphs/edges_900.csv"
    };

    /**
     * @brief Paths to the CSV files containing the real world graphs.
     */
    std::vector<std::string> realWorldGraphs = {
            "data/Real_world_Graphs/graph1/",
            "data/Real_world_Graphs/graph2/",
            "data/Real_world_Graphs/graph3/"
    };

    /**
     * @brief Paths to the CSV files containing the shipping toy graphs.
     */
    std::string toyGraphShipping = "data/Toy_Graphs/shipping.csv";

    /**
     * @brief Paths to the CSV files containing the stadiums toy graphs.

     */
    std::string toyGraphStadiums = "data/Toy_Graphs/stadiums.csv";

    /**
     * @brief Paths to the CSV files containing the tourism toy graphs.
     */
    std::string toyGraphTourism = "data/Toy_Graphs/tourism.csv";
};


#endif
