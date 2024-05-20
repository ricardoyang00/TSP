#include "DataParser.h"

using namespace std;

DataParser::DataParser() {}

void DataParser::ParseToyGraphs(const string& filePath, Graph<int>& graph) {
    ifstream file(filePath);
    if (!file.is_open()) {
        cerr << "Error: Unable to open file " << filePath << endl;
        return;
    }

    string line;
    getline(file, line);

    while(getline(file, line)){
        stringstream ss(line);
        string nonTrimmed;
        int source, target;
        double distance;

        getline(ss, nonTrimmed, ',');
        source = stoi(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        target = stoi(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        distance = stod(TrimString(nonTrimmed));

        graph.addVertex(source);
        graph.addVertex(target);
        graph.addBidirectionalEdge(source, target, distance);
    }
    file.close();
    cout << "Success: " << filePath << " parsed!" << endl;
}

void DataParser::ParseToyGraphTourism(const string& tourismPath, Graph<int>& graph) {
    ifstream file(tourismPath);
    if (!file.is_open()) {
        cerr << "Error: Unable to open file " << tourismPath << endl;
        return;
    }

    string line;
    getline(file, line);

    while(getline(file, line)){
        stringstream ss(line);
        string nonTrimmed, sourceLabel, targetLabel;
        int source, target;
        double distance;

        getline(ss, nonTrimmed, ',');
        source = stoi(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        target = stoi(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        distance = stod(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        sourceLabel = TrimString(nonTrimmed);

        getline(ss, nonTrimmed, ',');
        targetLabel = TrimString(nonTrimmed);

        graph.addVertexLabel(source, sourceLabel);
        graph.addVertexLabel(target, targetLabel);
        graph.addBidirectionalEdge(source, target, distance);
    }
    file.close();
    cout << "Success: " << tourismPath << " parsed!" << endl;
}

void DataParser::ParseFullyConnectedGraph(const string& vertexPath, const string& edgePath, Graph<int>& graph, const int& nodesNum) {
    /*      NODES PARSER    */
    ifstream nodeFile(vertexPath);
    if (!nodeFile.is_open()) {
        cerr << "Error: Unable to open file " << vertexPath << endl;
        return;
    }

    int numberOfNodes = nodesNum;

    string line;
    getline(nodeFile, line);

    while(getline(nodeFile, line) && numberOfNodes > 0){
        numberOfNodes--;
        stringstream ss(line);
        string nonTrimmed;
        int vertexID;
        double longitude, latitude;

        getline(ss, nonTrimmed, ',');
        vertexID = stoi(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        longitude = stod(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        latitude = stod(TrimString(nonTrimmed));

        graph.addVertexCoordinates(vertexID, longitude, latitude);
    }
    nodeFile.close();
    cout << "Success: " << vertexPath << " parsed!" << endl;

    /*      EDGES PARSER    */
    ifstream edgeFile(edgePath);
    if (!edgeFile.is_open()) {
        cerr << "Error: Unable to open file " << edgePath << endl;
        return;
    }

    while(getline(edgeFile, line)){
        stringstream ss(line);
        string nonTrimmed;
        int source, target;
        double distance;

        getline(ss, nonTrimmed, ',');
        source = stoi(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        target = stoi(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        distance = stod(TrimString(nonTrimmed));

        graph.addBidirectionalEdge(source, target, distance);
    }
    nodeFile.close();
    cout << "Success: " << edgePath << " parsed!" << endl;
}

void DataParser::ParseRealWorldGraph(const string& vertexPath, const string& edgePath, Graph<int>& graph) {
    /*      NODES PARSER    */
    ifstream nodeFile(vertexPath);
    if (!nodeFile.is_open()) {
        cerr << "Error: Unable to open file " << vertexPath << endl;
        return;
    }

    string line;
    getline(nodeFile, line);

    while(getline(nodeFile, line)){
        stringstream ss(line);
        string nonTrimmed;
        int vertexID;
        double longitude, latitude;

        getline(ss, nonTrimmed, ',');
        vertexID = stoi(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        longitude = stod(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        latitude = stod(TrimString(nonTrimmed));

        graph.addVertexCoordinates(vertexID, longitude, latitude);
    }
    nodeFile.close();
    cout << "Success: " << vertexPath << " parsed!" << endl;

    /*      EDGES PARSER    */
    ifstream edgeFile(edgePath);
    if (!edgeFile.is_open()) {
        cerr << "Error: Unable to open file " << edgePath << endl;
        return;
    }

    getline(edgeFile, line);

    while(getline(edgeFile, line)){
        stringstream ss(line);
        string nonTrimmed;
        int source, target;
        double distance;

        getline(ss, nonTrimmed, ',');
        source = stoi(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        target = stoi(TrimString(nonTrimmed));

        getline(ss, nonTrimmed, ',');
        distance = stod(TrimString(nonTrimmed));

        graph.addBidirectionalEdge(source, target, distance);
    }
    nodeFile.close();
    cout << "Success: " << edgePath << " parsed!" << endl;
}
