#include <stdlib.h>
#include <vector>
#include <float.h>
#include <list>
#include <set>
#include <iterator>
#include <iostream>

using namespace std;
// This class represents a directed graph using 
// adjacency list representation
class Graph1
{
	int V;    // No. of vertices

	// In a weighted graph, we need to store vertex 
	// and weight pair for every edge
	list< pair<int, double> > *adj;
	vector<int> backtrack;

public:
	Graph1(int V);  // Constructor

	// function to add an edge to graph
	void addEdge(int u, int v, double w);

	// prints shortest path from s
	vector<int> shortestPath(int src, int des);
};