#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <time.h>

#include <gl/glut.h>
#include <gl/GL.h>
#include <gl/GLU.h>

#include "triangle.h"
#include "dijkstra2.h"
#include "dijkstra1.h"
#include "dijkstra.h"

typedef enum PointType {start_line, start_arc, on_arc};
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
};


typedef std::vector< Vertex *> Vertices;
typedef std::vector< Vertices > Polygon;
typedef std::vector< int > ShortestChannel;
typedef std::vector< DoublePoint > ShortestPath;
typedef std::vector< Edge *> Edges;
typedef std::vector<Triangle *> Triangles;