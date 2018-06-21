// Program to find Dijkstra's shortest path using STL set
#include "dijkstra1.h"
using namespace std;
# define INF 0x3f3f3f3f

// Allocates memory for adjacency list
Graph1::Graph1(int V)
{
    this->V = V;
    adj = new list< pair<int, double> >[V];
}

void Graph1::addEdge(int u, int v, double w)
{
    adj[u].push_back(make_pair(v, w));
    adj[v].push_back(make_pair(u, w));
}

// Prints shortest paths from src to all other vertices
vector<int> Graph1::shortestPath(int src, int des)
{
	vector<int> parent(V, -1);

    // Create a set to store vertices that are being
    // preprocessed
    set< pair<double, int> > setds;

    // Create a vector for distances and initialize all
    // distances as infinite (INF)
    vector<double> dist(V, INF);

    // Insert source itself in Set and initialize its
    // distance as 0.
    setds.insert(make_pair(0, src));
    dist[src] = 0;

    /* Looping till all shortest distance are finalized
       then setds will become empty */
    while (!setds.empty())
    {
        // The first vertex in Set is the minimum distance
        // vertex, extract it from set.
        pair<double, int> tmp = *(setds.begin());
        setds.erase(setds.begin());
 
        // vertex label is stored in second of pair (it
        // has to be done this way to keep the vertices
        // sorted distance (distance must be first item
        // in pair)
        int u = tmp.second;
 
        // 'i' is used to get all adjacent vertices of a vertex
        list< pair<int, double> >::iterator i;
        for (i = adj[u].begin(); i != adj[u].end(); ++i)
        {
            // Get vertex label and weight of current adjacent
            // of u.
            int v = (*i).first;
            double weight = (*i).second;
 
            //  If there is shorter path to v through u.
            if (dist[v] > dist[u] + weight)
            {
                /*  If distance of v is not INF then it must be in
                    our set, so removing it and inserting again
                    with updated less distance.  
                    Note : We extract only those vertices from Set
                    for which distance is finalized. So for them, 
                    we would never reach here.  */
                if (dist[v] != INF)
                    setds.erase(setds.find(make_pair(dist[v], v)));
 
                // Updating distance of v
                dist[v] = dist[u] + weight;
                setds.insert(make_pair(dist[v], v));
				parent[v] = u;
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
