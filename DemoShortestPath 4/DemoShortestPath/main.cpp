#include "dijkstra.h"

using namespace std;

Polygon g_poly;
Vertices g_vertices;
Edges g_edges;
Channels g_channels;
Channel g_shortestChannel;
ShortestPath g_shortestPath;
vector<ShortestPath> g_shortestPaths;
Triangles g_triangles;

// Only for OpenGL
double m_scale = 0.05;
double m_translateX = 0;
double m_translateY = 0;
double center_objectX = 0;
double center_objectY = 0;
int window_width = 1680;
int window_height = 1050;

// Left and right point of channel point
vector<int> g_leftVertices;
vector<int> g_rightVertices;

#define PI 3.14159265
#define MIN_DOUBLE -10000000000.0
#define MAX_DOUBLE 10000000000.0

void initializeOpengl(int width, int height)
{	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glViewport(0, 0, width, height); 

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// Calculate The Aspect Ratio Of The Window
	gluPerspective(45.0f,(GLfloat)width/(GLfloat)height, 0.1f, 100.0f);
}

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

DoublePoint PointInsidePolygon(double minX, double minY, double maxX, double maxY, Vertices poly)
{
	unsigned int i, j;
	bool c = false;
	double testx, testy;

	while (c == false)
	{
		testx = fRand(minX, maxX);
		testy = fRand(minY, maxY);
		for (i = 0, j = poly.size()-1; i < poly.size(); j = i++) {
			if ( ((poly[i].point.y>testy) != (poly[j].point.y>testy)) &&
				(testx < (poly[j].point.x-poly[i].point.x) * (testy-poly[i].point.y) / (poly[j].point.y-poly[i].point.y) + poly[i].point.x) )
				c = !c;
		}
	}

	return DoublePoint(testx, testy);
}

bool CheckPointInPolygon(DoublePoint point, Triangle *triangle) 
{
	int i, j;
	bool c = false;

	for (i = 0, j = 2; i < 3; j = i++) 
	{
		if (((g_vertices[triangle->apexes[i]].point.y > point.y) != (g_vertices[triangle->apexes[j]].point.y > point.y)) &&
			(point.x < (g_vertices[triangle->apexes[j]].point.x - g_vertices[triangle->apexes[i]].point.x) * (point.y - g_vertices[triangle->apexes[i]].point.y) / (g_vertices[triangle->apexes[j]].point.y - g_vertices[triangle->apexes[i]].point.y) + g_vertices[triangle->apexes[i]].point.x))
			c = !c;
	}

	return c;
}

int getCommonEdge(Vertex A, Vertex B)
{
	for (int i = 0; i < A.adjacentEdges.size(); i++)
	{
		for (int j = 0; j < B.adjacentEdges.size(); j++)
		{
			if (A.adjacentEdges[i]==B.adjacentEdges[j])
			{
				return A.adjacentEdges[i];
			}
		}
	}
	return -1;
}

void Triangulate(Polygon poly, Vertices & vertices, Edges & edges, Triangles & triangles)
{
	// Declare parameter of triangulate function
	struct triangulateio in, out;
	memset(&in, 0, sizeof(triangulateio));
	memset(&out, 0, sizeof(triangulateio));

	// Only for render OpenGL
	double object_minX = MAX_DOUBLE;
	double object_minY = MAX_DOUBLE;
	double object_maxX = MIN_DOUBLE;
	double object_maxY = MIN_DOUBLE;

	// Mark point,edge are boundary or not
	std::vector<int> vertexType;
	std::vector<int> edgeType;

	// Build input point
	int idxEdge = 0;
	for (unsigned int i = 0; i < poly.size(); i++)
	{
		int startIdxEgde = idxEdge;
		for (unsigned int j = 0; j < poly[i].size(); j++)
		{
			vertices.push_back(poly[i][j]);

			if (j < poly[i].size()-1)
			{
				edges.push_back(Edge(idxEdge, idxEdge+1));
			}
			else
			{
				edges.push_back(Edge(idxEdge, startIdxEgde));
			}
			
			if (i == 0)
			{
				vertexType.push_back(boundary_type);
				edgeType.push_back(boundary_type);
			}
			else
			{
				vertexType.push_back(other_type);
				edgeType.push_back(other_type);
			}

			idxEdge++;


			// Only for OpenGL
			if (vertices[i].point.x < object_minX)
			{
				object_minX = vertices[i].point.x;
			}
			if (vertices[i].point.x > object_maxX)
			{
				object_maxX = vertices[i].point.x;
			}
			if (vertices[i].point.y < object_minY)
			{
				object_minY = vertices[i].point.y;
			}
			if (vertices[i].point.y > object_maxY)
			{
				object_maxY = vertices[i].point.y;
			}
		}
	}

	// Only for OpenGL
	center_objectX = (object_maxX + object_minX)/2;
	center_objectY = (object_maxY + object_minY)/2;
	
	// Make point list, segment list, point maker list, segment maker list
	double *pointlist = new double[vertices.size()*2]();
	int *pointmarkerlist = new int[vertices.size()]();
	for (unsigned int i = 0; i < vertices.size(); i++)
	{
		pointlist[2*i]=vertices[i].point.x;
		pointlist[2*i+1]=vertices[i].point.y;

		pointmarkerlist[i]=vertexType[i];
	}

	int *segmentlist = new int[edges.size()*2]();
	int *segmentmarkerlist = new int[edges.size()]();
	for (unsigned int i = 0; i < edges.size(); i++)
	{
		segmentlist[2*i]=edges[i].idxA;
		segmentlist[2*i+1]=edges[i].idxB;

		segmentmarkerlist[i]=edgeType[i];
	}

	/// Passing input points
	in.numberofpoints = vertices.size();
	in.pointlist = pointlist;

	// Passing input point marker
	in.pointmarkerlist = pointmarkerlist;

	/// Passing input segments
	in.numberofsegments = edges.size();
	in.segmentlist = segmentlist;

	// Passing input segment marker
	in.segmentmarkerlist = segmentmarkerlist;

	//Passing input holes
	double *holes = new double[(poly.size()-1)*2]();
	in.numberofholes = poly.size()-1;
	// Find a point inside each hole 
	for (int i = 1; i <= in.numberofholes; i++)
	{
		double minX = MAX_DOUBLE;
		double minY = MAX_DOUBLE;
		double maxX = MIN_DOUBLE;
		double maxY = MIN_DOUBLE;
		for (unsigned int j = 0; j < poly[i].size(); j++)
		{
			minX = (minX < poly[i][j].point.x) ? minX : poly[i][j].point.x;
			maxX = (maxX > poly[i][j].point.x) ? maxX : poly[i][j].point.x;
			minY = (minY < poly[i][j].point.y) ? minY : poly[i][j].point.y;
			maxY = (maxY > poly[i][j].point.y) ? maxY : poly[i][j].point.y;
		}
		
		DoublePoint pointInHole = PointInsidePolygon(minX, minY, maxX, maxY, poly[i]);
		holes[(i-1)*2]=pointInHole.x;
		holes[(i-1)*2+1]=pointInHole.y;
	}
	in.holelist = holes;

	// Triangulate polygon using library function
	triangulate("pDzen", &in, &out, 0);

	// Vertices list after triangulation
	vertices.resize(0);
	int numberofvertices = out.numberofpoints;
	for (int i = 0; i < numberofvertices; ++i)
	{
		Vertex vertex;
		vertex.point=DoublePoint(out.pointlist[2*i], out.pointlist[2*i+1]);
		vertices.push_back(vertex);
	}

	// Edges list after triangulation
	edges.resize(0);
	int numberofedges = out.numberofedges;
	for (int i = 0; i < numberofedges; ++i)
	{
		Edge edge = Edge(out.edgelist[2*i], out.edgelist[2*i+1]);
		edges.push_back(edge);

		vertices[out.edgelist[2*i]].adjacentEdges.push_back(i);
		vertices[out.edgelist[2*i+1]].adjacentEdges.push_back(i);
	}

	// Triangles list after triangulation
	int k = 0;
	for (int i = 0; i < out.numberoftriangles; i++)
	{
		Triangle * triangle = new Triangle;
		vector<int> boudary_vertices;
		for (int j = 0; j < 3; j++)
		{
			triangle->apexes.push_back(out.trianglelist[3*i+j]);

			if (out.neighborlist[3*i+j] != -1)
			{
				triangle->adjacentTriangles.push_back(out.neighborlist[3*i+j]);
			}

			int pointmarker = out.pointmarkerlist[triangle->apexes.at(j)];
			if (pointmarker == boundary_type)
			{
				boudary_vertices.push_back(triangle->apexes.at(j));
			}
		}
		
		int number_of_boundary_vertices = boudary_vertices.size();
		for (int j = 0; j < number_of_boundary_vertices; j++)
		{
			for (int k = j+1; k < number_of_boundary_vertices; k++)
			{
				int commonEdge = getCommonEdge(vertices[boudary_vertices.at(j)], vertices[boudary_vertices.at(k)]);
				if (commonEdge != -1)
				{
					int edgemarker = out.edgemarkerlist[commonEdge];
					if (edgemarker == boundary_type)
					{
						triangle->boundaryEdges.push_back(commonEdge);
					}
				}
			}
		}

		triangles.push_back(triangle);
	}

	// Free memory
	free(out.pointlist);
	free(out.pointmarkerlist);
	free(out.trianglelist);
	free(out.neighborlist);
	free(out.segmentlist);
	free(out.segmentmarkerlist);
	free(out.edgelist);

	delete[] segmentlist;
	delete[] pointlist;
	delete[] pointmarkerlist;
	delete[] segmentmarkerlist;
}

void mykey(int key, int x, int y)
{
	double next_scale;
	switch (key)
	{
	case GLUT_KEY_PAGE_UP:
		m_scale += m_scale/2;
		break;
	case GLUT_KEY_PAGE_DOWN:
		next_scale = m_scale - m_scale/2;
		if (next_scale > 0)
			m_scale -= next_scale;
		break;
	case GLUT_KEY_UP:
		m_translateY -= 1/m_scale;
		break;
	case GLUT_KEY_DOWN:
		m_translateY += 1/m_scale;
		break;
	case GLUT_KEY_LEFT:
		m_translateX += 1/m_scale;
		break;
	case GLUT_KEY_RIGHT:
		m_translateX -= 1/m_scale;
		break;
	default:
		break;
	}

	glutPostRedisplay();
}


void displayFunc(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glScalef(m_scale, m_scale, 1.0f );
	glTranslatef(m_translateX-center_objectX, m_translateY-center_objectY, 1.0f);

	double x1, y1;
	double x2, y2;
	double x3, y3;

	// Draw shortest channel
	glColor3f(223.0/255.0, 119.0/255.0, 39.0/255.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < g_shortestChannel.size(); i++)
	{
		x1 = g_vertices[g_triangles[g_shortestChannel[i]]->apexes[0]].point.x;
		y1 = g_vertices[g_triangles[g_shortestChannel[i]]->apexes[0]].point.y;
		x2 = g_vertices[g_triangles[g_shortestChannel[i]]->apexes[1]].point.x;
		y2 = g_vertices[g_triangles[g_shortestChannel[i]]->apexes[1]].point.y;
		x3 = g_vertices[g_triangles[g_shortestChannel[i]]->apexes[2]].point.x;
		y3 = g_vertices[g_triangles[g_shortestChannel[i]]->apexes[2]].point.y;

		glVertex3f(x1, y1, -15.0f);
		glVertex3f(x2, y2, -15.0f);
		glVertex3f(x3, y3, -15.0f);
	}
	glEnd();

	/*glColor3f(223.0/255.0, 119.0/255.0, 39.0/255.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < g_channels.size(); i++)
	{
		if (i == 43)
		{
			for (unsigned int j = 0; j < g_channels[i].size(); j++)
			{
				x1 = g_vertices[g_triangles[g_channels[i][j]]->apexes[0]].point.x;
				y1 = g_vertices[g_triangles[g_channels[i][j]]->apexes[0]].point.y;
				x2 = g_vertices[g_triangles[g_channels[i][j]]->apexes[1]].point.x;
				y2 = g_vertices[g_triangles[g_channels[i][j]]->apexes[1]].point.y;
				x3 = g_vertices[g_triangles[g_channels[i][j]]->apexes[2]].point.x;
				y3 = g_vertices[g_triangles[g_channels[i][j]]->apexes[2]].point.y;

				glVertex3f(x1, y1, -15.0f);
				glVertex3f(x2, y2, -15.0f);
				glVertex3f(x3, y3, -15.0f);
			}
		}
	}
	glEnd();*/

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	// Draw triangle
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < g_triangles.size(); i++)
	{
		x1 = g_vertices[g_triangles[i]->apexes[0]].point.x;
		y1 = g_vertices[g_triangles[i]->apexes[0]].point.y;
		x2 = g_vertices[g_triangles[i]->apexes[1]].point.x;
		y2 = g_vertices[g_triangles[i]->apexes[1]].point.y;
		x3 = g_vertices[g_triangles[i]->apexes[2]].point.x;
		y3 = g_vertices[g_triangles[i]->apexes[2]].point.y;

		glVertex3f(x1, y1, -15.0f);
		glVertex3f(x2, y2, -15.0f);
		glVertex3f(x3, y3, -15.0f);
	}
	glEnd();

	// Draw polygon only
	/*glColor3f(1.0, 1.0,1.0);
	glBegin(GL_LINES);
	int idxEdge = 0;
	for (unsigned int i = 0; i < g_poly.size(); i++)
	{
		unsigned int j;
		for (j = 0; j < g_poly[i].size()-1; j++)
		{
			x1 = g_poly[i][j].point.x;
			y1 = g_poly[i][j].point.y;
			x2 = g_poly[i][j+1].point.x;
			y2 = g_poly[i][j+1].point.y;

			glVertex3f(x1, y1, -15.0f);
			glVertex3f(x2, y2, -15.0f);
		}

		x1 = g_poly[i][j].point.x;
		y1 = g_poly[i][j].point.y;
		x2 = g_poly[i][0].point.x;
		y2 = g_poly[i][0].point.y;

		glVertex3f(x1, y1, -15.0f);
		glVertex3f(x2, y2, -15.0f);
	}
	glEnd();*/

	//// Draw Left Vertex
	//glColor3f(1.0, 1.0, 0.0);
	//glBegin(GL_LINES);
	//for (unsigned int i = 0; i < g_leftVertices.size()-1; i++)
	//{
	//	x1 = g_vertices[g_leftVertices[i]].point.x;
	//	y1 = g_vertices[g_leftVertices[i]].point.y;
	//	x2 = g_vertices[g_leftVertices[i+1]].point.x;
	//	y2 = g_vertices[g_leftVertices[i+1]].point.y;

	//	glVertex3f(x1, y1, -15.0f);
	//	glVertex3f(x2, y2, -15.0f);
	//}
	//glEnd();

	//// Draw Right Vertex
	//glColor3f(0.0, 1.0, 0.0);
	//glBegin(GL_LINES);
	//for (unsigned int i = 0; i < g_rightVertices.size()-1; i++)
	//{
	//	x1 = g_vertices[g_rightVertices[i]].point.x;
	//	y1 = g_vertices[g_rightVertices[i]].point.y;
	//	x2 = g_vertices[g_rightVertices[i+1]].point.x;
	//	y2 = g_vertices[g_rightVertices[i+1]].point.y;

	//	glVertex3f(x1, y1, -15.0f);
	//	glVertex3f(x2, y2, -15.0f);
	//}
	//glEnd();

	// Draw shortest path
	
	/*for (unsigned int i = 0; i < g_shortestPaths.size(); i++)
	{
		if (i == 43)
		{
			glColor3f(0.0, 0.5, 1.0);
			glBegin(GL_LINES);
			for (unsigned int j = 0; j < g_shortestPaths[i].size()-1; j++)
			{
				x1 = g_shortestPaths[i][j].x;
				y1 = g_shortestPaths[i][j].y;
				x2 = g_shortestPaths[i][j+1].x;
				y2 = g_shortestPaths[i][j+1].y;

				glVertex3f(x1, y1, -15.0f);
				glVertex3f(x2, y2, -15.0f);
			}
			glEnd();
		}
	}*/

	if (g_shortestPath.size() != 0)
	{
		glColor3f(0.0, 0.5, 1.0);
		glBegin(GL_LINES);
		for (unsigned int i = 0; i < g_shortestPath.size()-1; i++)
		{
			x1 = g_shortestPath[i].x;
			y1 = g_shortestPath[i].y;
			x2 = g_shortestPath[i+1].x;
			y2 = g_shortestPath[i+1].y;

			glVertex3f(x1, y1, -15.0f);
			glVertex3f(x2, y2, -15.0f);
		}
		glEnd();
	}
	

	glutSwapBuffers();
}

//---------------------------------------------------------------------------
void ReadFromFile(char *filename, Polygon &paths)
{
	int size = 1024, pos;
	int c;
	char *buffer = (char *)malloc(size);
	FILE *f = fopen(filename, "r");

	paths.resize(0);
	if(f) {
		do { // read all lines in file
			pos = 0;
			do{ // read one line
				c = fgetc(f);
				if(c != EOF) buffer[pos++] = (char)c;
				if(pos >= size - 1) { // increase buffer length - leave room for 0
					size *=2;
					buffer = (char*)realloc(buffer, size);
				}
			}while(c != EOF && c != '\n');
			buffer[pos] = 0;

			// line is now in buffer
			if (strstr(buffer, "POLYGON"))
			{
				Vertices vertices;
				vertices.resize(0);
				paths.push_back(vertices);
			}
			else if (!strstr(buffer, "VOID"))
			{
				char *result[11];
				char * temp;
				int cur_idx = 0;
				temp = strtok(buffer," )(");

				while (temp != NULL)
				{
					result[cur_idx] = temp;
					cur_idx++;
					temp = strtok(NULL," )(\n\r");
				}

				if (cur_idx > 4)
				{
					if (strcmp(result[0], "DL") == 0)
					{
						float startX;
						float startY;

						sscanf(result[1],"%f",&startX);
						sscanf(result[2],"%f",&startY);

						Vertex vertex;
						vertex.point = DoublePoint(startX,startY);
						vertex.type = start_line;
						paths[paths.size()-1].push_back(vertex);
					}
					else
					{
						float startX;
						float startY;
						float centerX;
						float centerY;
						float radius;
						float startDegree;
						float endDegree;
						ArcDirection arcDirection;

						sscanf(result[1],"%f",&radius);
						sscanf(result[2],"%f",&startDegree);
						sscanf(result[3],"%f",&endDegree);
						sscanf(result[5],"%f",&startX);
						sscanf(result[6],"%f",&startY);
						sscanf(result[9],"%f",&centerX);
						sscanf(result[10],"%f",&centerY);

						if (strcmp(result[4], "CCW") == 0)
						{
							arcDirection = CCW;
						}
						else
						{
							arcDirection = CW;
						}

						if (paths.size()-1 == 0 && arcDirection == CW)
						{
							int sdsdsd=0;
						}

						double arc_angle;
						if (abs(endDegree - startDegree) < 0.0001)
						{
							arc_angle = 360;
						}
						else
						{
							if (endDegree - startDegree < 0)
							{
								if (arcDirection == CCW)
								{
									arc_angle = 360 - abs(endDegree - startDegree);
								}
								else
								{
									arc_angle = endDegree - startDegree;
								}
							}
							else
							{
								if (arcDirection == CCW)
								{
									arc_angle = endDegree - startDegree;
								}
								else
								{
									arc_angle = (endDegree - startDegree) - 360;
								}
							}
						}

						int num_segments = floor(abs(arc_angle));
						double theta = arc_angle / double(num_segments - 1);

						double tangetial_factor = tan(theta * PI / 180.0);

						double radial_factor = cos(theta * PI / 180.0);


						double x = radius * cos(startDegree * PI / 180.0);
						double y = radius * sin(startDegree * PI / 180.0); 

						for(int ii = 0; ii < num_segments; ii++)
						{
							Vertex vertex;
							vertex.type = start_arc;
							vertex.point = DoublePoint(x + centerX, y + centerY);
							vertex.center = DoublePoint(centerX, centerY);
							vertex.radius = radius;
							vertex.arcDirection = arcDirection;
							vertex.startDegree = startDegree;
							vertex.endDegree = endDegree;
							paths[paths.size()-1].push_back(vertex);

							double tx = -y; 
							double ty = x; 

							x += tx * tangetial_factor; 
							y += ty * tangetial_factor; 

							x *= radial_factor; 
							y *= radial_factor; 
						}
					}
				}
			}
		} while(c != EOF); 
		fclose(f);           
	}
	free(buffer);
}

void MakePolygon(Polygon & poly)
{
	/*poly.resize(2);

	Vertex vertex1;
	vertex1.point = DoublePoint(0.0, 0.0);
	Vertex vertex2;
	vertex2.point = DoublePoint(0.0, 5.0);
	Vertex vertex3;
	vertex3.point = DoublePoint(5.0, 5.0);
	Vertex vertex4;
	vertex4.point = DoublePoint(5.0, 0.0);
	Vertex vertex5;
	vertex5.point = DoublePoint(4.0, 1.0);
	Vertex vertex6;
	vertex6.point = DoublePoint(1.0, 3.0);
	Vertex vertex7;
	vertex7.point = DoublePoint(1.0, 4.0);
	Vertex vertex8;
	vertex8.point = DoublePoint(2.0, 4.0);
	Vertex vertex9;
	vertex9.point = DoublePoint(3.0, 2.0);
	Vertex vertex10;
	vertex10.point = DoublePoint(2.0, 3.0);

	poly[0].push_back(vertex1);
	poly[0].push_back(vertex2);
	poly[0].push_back(vertex3);
	poly[0].push_back(vertex4);
	poly[0].push_back(vertex5);

	poly[1].push_back(vertex6);
	poly[1].push_back(vertex7);
	poly[1].push_back(vertex8);
	poly[1].push_back(vertex9);
	poly[1].push_back(vertex10);*/

	ReadFromFile("input1.txt", poly);
}

double getDistance(DoublePoint A, DoublePoint B)
{
	return sqrt(pow(A.x-B.x, 2)+pow(A.y-B.y, 2));
}

// Get distance between center of 2 triangles
double getAverageDistanceBetween2Triangle(Triangle* A, Triangle* B)
{
	Vertex A0 = g_vertices[A->apexes[0]];
	Vertex A1 = g_vertices[A->apexes[1]];
	Vertex A2 = g_vertices[A->apexes[2]];

	Vertex B0 = g_vertices[B->apexes[0]];
	Vertex B1 = g_vertices[B->apexes[1]];
	Vertex B2 = g_vertices[B->apexes[2]];
	
	DoublePoint centerA = DoublePoint((A0.point.x + A1.point.x + A2.point.x) / 3, (A0.point.y + A1.point.y + A2.point.y) / 3);
	DoublePoint centerB = DoublePoint((B0.point.x + B1.point.x + B2.point.x) / 3, (B0.point.y + B1.point.y + B2.point.y) / 3);

	return getDistance(centerA, centerB);
}

int getTriangleOfStartPoint(DoublePoint start, Triangles triangles)
{
	for (unsigned int i = 0; i < triangles.size(); i++)
	{
		if (CheckPointInPolygon(start, triangles[i]))
		{
			return i;
		}
	}

	return -1;
}

double CrossProduct(DoublePoint vector1, DoublePoint vector2)
{
	return vector1.x*vector2.y - vector1.y*vector2.x;
}

void minDistanceFromAPointToSegment(DoublePoint point, DoublePoint startLine, DoublePoint endLine, double & shortestDist, DoublePoint & pointOnSegment)
{
	double lengh_of_segment;
	double U;

	lengh_of_segment = getDistance(startLine, endLine);

	U = ( ( ( point.x - startLine.x ) * ( endLine.x - startLine.x ) ) +
		( ( point.y - startLine.y ) * ( endLine.y - startLine.y ) ) ) /
		( lengh_of_segment * lengh_of_segment );

	// Point outside the segment
	if( U < 0.0f || U > 1.0f )
	{
		double distance1 = getDistance(point, startLine);
		double distance2 = getDistance(point, endLine);
		if (distance1 > distance2)
		{
			shortestDist = distance2;
			pointOnSegment = endLine;
		}
		else
		{
			shortestDist = distance1;
			pointOnSegment = startLine;
		}

		return;
	}

	// point inside segment
	pointOnSegment.x = startLine.x + U * ( endLine.x - startLine.x );
	pointOnSegment.y = startLine.y + U * ( endLine.y - startLine.y );

	shortestDist = getDistance(point, pointOnSegment);
}

bool CheckReadyConnectToEndPoint(DoublePoint startPoint, DoublePoint endPoint, vector<int> leftVertices, vector<int> rightVertices, int currentIdx)
{
	DoublePoint vectorCenter = endPoint - startPoint;
	for (unsigned int i = currentIdx; i < leftVertices.size() - 1; i++)
	{
		DoublePoint vectorLeft = g_vertices[leftVertices[i]].point - startPoint;
		DoublePoint vectorRight = g_vertices[rightVertices[i]].point - startPoint;
		if (CrossProduct(vectorCenter, vectorLeft) < 0 || CrossProduct(vectorCenter, vectorRight) > 0)
		{
			return false;
		}
	}

	return true;
}

void ConnectToBoundary(DoublePoint startPoint, Triangle *triangle, double & minDist, DoublePoint & lastPoint)
{
	// Find intersection point
	minDist = MAX_DOUBLE;
	lastPoint = NULL;
	for (unsigned int i = 0; i < triangle->boundaryEdges.size(); i++)
	{
		DoublePoint pointResult;
		double localShortestDistance;
		minDistanceFromAPointToSegment(startPoint, 
			g_vertices[g_edges[triangle->boundaryEdges[i]].idxA].point, 
			g_vertices[g_edges[triangle->boundaryEdges[i]].idxB].point, 
			localShortestDistance, pointResult);

		if (minDist > localShortestDistance)
		{
			minDist = localShortestDistance;
			lastPoint = pointResult;
		}
	}
}

void Funnel(DoublePoint startPoint, Channel channel, ShortestPath & shortestPath, double & shortestDist)
{
	g_leftVertices.resize(channel.size() + 1);
	g_rightVertices.resize(channel.size() + 1);

	// Initialize portal vertices.
	for (unsigned int i = 0; i < channel.size() - 1; i++)
	{
		Triangle *currentTriangle = g_triangles[channel[i]];
		Triangle *nextTriangle = g_triangles[channel[i+1]];
		
		vector<int> commonVertex;
		int uncommonVertex;
		for (int current = 0; current < 3; current++)
		{
			uncommonVertex = current;
			for (int j = 0; j < 3; j++)
			{
				if (currentTriangle->apexes[current] == nextTriangle->apexes[j])
				{
					commonVertex.push_back(current);
					uncommonVertex = -1;
					break;
				}
			}
			
			if (uncommonVertex != -1)
			{
				for (int common = current+1; common < 3; common++)
				{
					commonVertex.push_back(common);
				}
				break;
			}
		}
		
		if (i == 0)
		{
			DoublePoint vector1 = g_vertices[currentTriangle->apexes[commonVertex[0]]].point - g_vertices[currentTriangle->apexes[uncommonVertex]].point;
			DoublePoint vector2 = g_vertices[currentTriangle->apexes[commonVertex[1]]].point - g_vertices[currentTriangle->apexes[uncommonVertex]].point;
			if (CrossProduct(vector1, vector2) > 0)
			{
				g_leftVertices[i+1] = currentTriangle->apexes[commonVertex[1]];
				g_rightVertices[i+1] = currentTriangle->apexes[commonVertex[0]];
			}
			else
			{
				g_leftVertices[i+1] = currentTriangle->apexes[commonVertex[0]];
				g_rightVertices[i+1] = currentTriangle->apexes[commonVertex[1]];
			}
		}
		else
		{
			if ((currentTriangle->apexes[commonVertex[0]] == g_leftVertices[i]) 
				|| (currentTriangle->apexes[commonVertex[1]] == g_rightVertices[i]))
			{
				g_leftVertices[i+1] = currentTriangle->apexes[commonVertex[0]];
				g_rightVertices[i+1] = currentTriangle->apexes[commonVertex[1]];
			}
			else
			{
				g_leftVertices[i+1] = currentTriangle->apexes[commonVertex[1]];
				g_rightVertices[i+1] = currentTriangle->apexes[commonVertex[0]];
			}
		}
	}

	// Initialize portal vertices first point.
	Triangle *firstTriangle = g_triangles[channel[0]];
	for (int j = 0; j < 3; j++)
	{
		if (firstTriangle->apexes[j] != g_leftVertices[1]
		&& firstTriangle->apexes[j] != g_rightVertices[1])
		{
			g_leftVertices[0] = firstTriangle->apexes[j];
			g_rightVertices[0] = firstTriangle->apexes[j];
		}
	}

	// Initialize portal vertices last point.
	Triangle *lastTriangle = g_triangles[channel[channel.size() - 1]];
	for (int j = 0; j < 3; j++)
	{
		if (lastTriangle->apexes[j] != g_leftVertices[channel.size() - 1]
		&& lastTriangle->apexes[j] != g_rightVertices[channel.size() - 1])
		{
			g_leftVertices[channel.size()] = lastTriangle->apexes[j];
			g_rightVertices[channel.size()] = lastTriangle->apexes[j];
		}
	}


	// Step through channel.
	unsigned int left = 1;
	unsigned int right = 1;
	shortestPath.push_back(startPoint);
	DoublePoint apex = startPoint;
	shortestDist = 0;

	double minDist;
	DoublePoint lastPoint;
	ConnectToBoundary(startPoint, lastTriangle, minDist, lastPoint);
	if (CheckReadyConnectToEndPoint(startPoint, lastPoint, g_leftVertices, g_rightVertices, 1))
	{
		shortestDist += minDist;
		shortestPath.push_back(lastPoint);
	}
	else
	{
		for (unsigned int i = 2; i <= channel.size(); i++)
		{
			// If new left vertex is different, process.
			if (g_leftVertices[i] != g_leftVertices[left] && i > left)
			{
				DoublePoint newVector = g_vertices[g_leftVertices[i]].point - apex;
				DoublePoint leftVector = g_vertices[g_leftVertices[left]].point - apex;
				DoublePoint rightVector = g_vertices[g_rightVertices[right]].point - apex;

				// If new side does not widen funnel, update.
				if (CrossProduct(newVector, leftVector) > 0)
				{
					// If new side crosses other side, update apex.
					if (CrossProduct(newVector, rightVector) > 0)
					{
						// Find next vertex.
						int next = right;
						for (unsigned int j = next+1; j < g_rightVertices.size(); j++)
						{
							if (g_rightVertices[j] != g_rightVertices[next])
							{
								next = j;
								break;
							}
						}

						shortestDist += getDistance(apex, g_vertices[g_rightVertices[right]].point);
						apex = g_vertices[g_rightVertices[right]].point;
						shortestPath.push_back(apex);
						right = next;
						left = next;
						i = next;

						double minDist;
						DoublePoint lastPoint;
						ConnectToBoundary(apex, lastTriangle, minDist, lastPoint);
						if (CheckReadyConnectToEndPoint(apex, lastPoint, g_leftVertices, g_rightVertices, next))
						{
							break;
						}
					}
					else
					{
						left = i;
					}
				}
			}

			// If new right vertex is different, process.
			if (g_rightVertices[i] != g_rightVertices[right] && i > right)
			{
				DoublePoint newVector = g_vertices[g_rightVertices[i]].point - apex;
				DoublePoint leftVector = g_vertices[g_leftVertices[left]].point - apex;
				DoublePoint rightVector = g_vertices[g_rightVertices[right]].point - apex;

				// If new side does not widen funnel, update.
				if (CrossProduct(newVector, rightVector) < 0)
				{
					// If new side crosses other side, update apex.
					if (CrossProduct(newVector, leftVector) < 0)
					{
						// Find next vertex.
						int next = left;
						for (unsigned int j = next+1; j < g_leftVertices.size(); j++)
						{
							if (g_leftVertices[j] != g_leftVertices[next])
							{
								next = j;
								break;
							}
						}
						
						shortestDist += getDistance(apex, g_vertices[g_leftVertices[left]].point);
						apex = g_vertices[g_leftVertices[left]].point;
						shortestPath.push_back(apex);
						left = next;
						right = next;
						i = next;

						double minDist;
						DoublePoint lastPoint;
						ConnectToBoundary(apex, lastTriangle, minDist, lastPoint);
						if (CheckReadyConnectToEndPoint(apex, lastPoint, g_leftVertices, g_rightVertices, next))
						{
							break;
						}
					}
					else
					{
						right = i;
					}
				}
			}
		}

		double minDist;
		DoublePoint lastPoint;
		ConnectToBoundary(apex, lastTriangle, minDist, lastPoint);
		shortestDist += minDist;
		shortestPath.push_back(lastPoint);
	}
}

int main(int argc, char** argv)
{
	const char* windowName = "Demo Shortest Path";
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(window_width, window_height);
	glutCreateWindow(windowName);
	glutDisplayFunc(displayFunc);
	glutSpecialFunc(mykey);
	initializeOpengl(window_width, window_height);

	DoublePoint start = DoublePoint(-107.19, 20);

	// Step 1: Make polygon
	MakePolygon(g_poly);

	time_t time_start = clock();

	// Step 2: Make triangulate
	Triangulate(g_poly/*input*/, g_vertices/*output*/, g_edges/*output*/, g_triangles/*output*/);

	// Step 3: Build graph of triangles. Every triangle is a node in graph
	struct Graph* graph = createGraph(g_triangles);
	int num_tri = g_triangles.size();
	for (unsigned int i = 0; i < g_triangles.size(); i++)
	{
		for (unsigned int j = 0; j < g_triangles[i]->adjacentTriangles.size(); j++)
		{
			int dest = g_triangles[i]->adjacentTriangles[j];
			double weight = getAverageDistanceBetween2Triangle(g_triangles[i], g_triangles[dest]);
			addEdge(graph, i, dest, weight);
		}
	}

	// Step 4: Find triangle index of start point
	int startIdx = getTriangleOfStartPoint(start, g_triangles);

	if (startIdx == -1) // start and end outside polygon
	{
		return 1;
	}

	// Step 5: Find all channel to boundary
	findChannelsToTriangleBoundary(graph, startIdx, g_channels);

	// Step 6: Find the shortest path to boundary using funnel algorithm
	double minDist = MAX_DOUBLE;
	for (unsigned int i = 0; i < g_channels.size(); i++)
	{
		ShortestPath localShortestPath;
		double localShortestDist;

		Funnel(start, g_channels[i], localShortestPath, localShortestDist);

		g_shortestPaths.push_back(localShortestPath);
		if (minDist > localShortestDist)
		{
			minDist = localShortestDist;
			g_shortestPath = localShortestPath;
			g_shortestChannel = g_channels[i];
		}
	}

	double time_elapsed_total = double(clock() - time_start)/CLOCKS_PER_SEC;
	cout << "\time_elapsed_total in " << time_elapsed_total << " secs \n";

	glutMainLoop();

	return 0;
}