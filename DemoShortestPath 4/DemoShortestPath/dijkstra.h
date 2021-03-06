#include <stdlib.h>
#include <vector>
#include <float.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <time.h>

#include <gl/glut.h>
#include <gl/GL.h>
#include <gl/GLU.h>

#include "triangle.h"

typedef enum PointType {start_line, start_arc, on_arc};
typedef enum VertexType {boundary_type = 2, other_type};
typedef enum ArcDirection {CW, CCW};
struct DoublePoint 
{
	double x;
	double y;

	DoublePoint(double x_ = 0, double y_ = 0): x(x_), y(y_) {};

	friend inline bool operator== (const DoublePoint& a, const DoublePoint& b)
	{
		return abs(a.x-b.x) < 0.0001 && abs(a.y-b.y) < 0.0001;
	}
	friend inline bool operator!= (const DoublePoint& a, const DoublePoint& b)
	{
		return abs(a.x-b.x) > 0.0001 || abs(a.y-b.y) > 0.0001;
	}
	friend inline DoublePoint operator- (const DoublePoint& a, const DoublePoint& b)
	{
		return DoublePoint(a.x-b.x, a.y-b.y);
	}
};

struct Triangle
{
	std::vector<int> apexes;

	std::vector<int> adjacentTriangles;

	std::vector<int> boundaryEdges;
};

struct Edge
{
	int idxA;
	int idxB;

	Edge(int a_ = 0, int b_ = 0): idxA(a_), idxB(b_) {};

	friend inline bool operator== (const Edge& a, const Edge& b)
	{
		return (a.idxA==b.idxA && a.idxB==b.idxB) || (a.idxB==b.idxA && a.idxA==b.idxB);
	}
	friend inline bool operator!= (const Edge& a, const Edge& b)
	{
		return (a.idxA!=b.idxA && a.idxA!=b.idxB) || (a.idxB!=b.idxA && a.idxB!=b.idxB);
	}
};

struct Vertex
{
	DoublePoint point;
	PointType type;

	// Only for arc
	double radius;
	DoublePoint center;
	ArcDirection arcDirection;
	// Divide arc into more small line with degree 10
	double startDegree;
	double endDegree;

	std::vector<int>adjacentEdges;
};


typedef std::vector< Edge > Edges;
typedef std::vector< Vertex > Vertices;
typedef std::vector< Vertices > Polygon;
typedef std::vector< int > Channel;
typedef std::vector< Channel > Channels;
typedef std::vector< DoublePoint > ShortestPath;
typedef std::vector<Triangle *> Triangles;

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

	bool isBoundaryNode;
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

struct Graph* createGraph(Triangles triangles);
void addEdge(struct Graph* graph, int src, int dest, double weight);
void dijkstra(struct Graph* graph, int src, int dest, std::vector<int> & shortestPath);
void findChannelsToTriangleBoundary(struct Graph* graph, int src, Channels & channels);