#include <stdlib.h>
#include <vector>
#include <float.h>
#include <set>


// A structure to represent a node in adjacency list
struct AdjListNode
{
	int dest;
	double weight;
	struct AdjListNode* next;
};

// A structure to represent an adjacency list
struct AdjList
{
	struct AdjListNode *head;  // pointer to head node of list

	int backtrack;
};

// A structure to represent a graph. A graph is an array of adjacency lists.
// Size of array will be V (number of vertices in graph)
struct Graph
{
	int V;
	struct AdjList* vertices;
};

// Structure to represent a min heap node
struct MinHeapNode
{
	int  v;
	double dist;
};

// Structure to represent a min heap
struct MinHeap
{
	int size;      // Number of heap nodes present currently
	int capacity;  // Capacity of min heap
	int *pos;     // This is needed for decreaseKey()
	struct MinHeapNode **array;
};

void releaseGraph(Graph* graph);
struct Graph* createGraph(int V);
void addEdge(struct Graph* graph, int src, int dest, double weight);
void dijkstra(struct Graph* graph, int src, int dest, std::vector<int> & shortestPath);
void findShortestChannel(struct Graph* graph, int src, int dest, std::vector<int> & shortestPath);