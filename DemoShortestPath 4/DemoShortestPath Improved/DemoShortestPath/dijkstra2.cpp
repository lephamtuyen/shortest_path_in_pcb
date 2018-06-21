// Program to find Dijkstra's shortest path using STL set
#include "dijkstra2.h"
#include <limits.h>
using namespace std;
# define INF 0x3f3f3f3f

// Allocates memory for adjacency list
Graph2::Graph2(int V)
{
    this->V = V;
    adj = new list< pair<int, double> >[V];
}

void Graph2::addEdge(int u, int v, double w)
{
    adj[u].push_back(make_pair(v, w));
    adj[v].push_back(make_pair(u, w));
}

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int Graph2::minDistance(vector<double> dist, vector<bool> sptSet)
{
	// Initialize min value
	double min = INF;
	int min_index;

	for (int v = 0; v < V; v++)
		if (sptSet[v] == false && dist[v] <= min)
			min = dist[v], min_index = v;

	return min_index;
}


// Prints shortest paths from src to all other vertices
vector<int> Graph2::shortestPath(int src, int des)
{
	vector<int> parent(V, -1);

    // Create a vector for distances and initialize all
    // distances as infinite (INF)
    vector<double> dist(V, INF);
	vector<bool> sptSet;sptSet.resize(V);

    // initialize its distance as 0.
    dist[src] = 0;

    /* Looping till all shortest distance are finalized
       then setds will become empty */
	// Find shortest path for all vertices
	for (int count = 0; count < V-1; count++)
	{
		// Pick the minimum distance vertex from the set of vertices not
		// yet processed. u is always equal to src in first iteration.
		int u = minDistance(dist, sptSet);

		// Mark the picked vertex as processed
		sptSet[u] = true;

		// 'i' is used to get all adjacent vertices of a vertex
		list< pair<int, double> >::iterator i;
		for (i = adj[u].begin(); i != adj[u].end(); ++i)
		{
			// Get vertex label and weight of current adjacent
			// of u.
			int v = (*i).first;
			double weight = (*i).second;

			// Update dist[v] only if is not in sptSet, there is an edge from 
			// u to v, and total weight of path from src to  v through u is 
			// smaller than current value of dist[v]
			if ((dist[u]+weight) < dist[v])
			{
				dist[v] = dist[u] + weight;
				parent[v]=u;
			}	
		}
	}

    // Print shortest distances stored in dist[]
    for (int i = 0; i < V; ++i)
	{
		if (i == des)
		{
			cout << "shortest length: " << dist[i] << "\n";
		}
		
	}

	// Get Shortest path
	vector<int>path;
	double leng = 0;
	path.clear();
	std::vector<int>::iterator it;
	it = path.begin();
	path.insert (it , des);
	int idx = parent[des];
	int next = des;
	while (idx != -1)
	{
		list< pair<int, double> >::iterator i;
		for (i = adj[idx].begin(); i != adj[idx].end(); ++i)
		{
			// Get vertex label and weight of current adjacent
			// of u.
			int v = (*i).first;
			double weight = (*i).second;
			if (v == next)
			{
				leng += weight;
			}
		}
		it = path.begin();
		path.insert (it , idx);
		next = idx;
		idx = parent[idx];
	}
	cout << "backtrack length: " << leng << "\n";

	return path;
}
