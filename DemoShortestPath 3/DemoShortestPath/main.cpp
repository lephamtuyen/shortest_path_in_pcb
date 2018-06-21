#include "dijkstra.h"

using namespace std;

Paths g_poly;
Vertices g_vertices;
Edges g_edges;
ShortestPath g_shortestPath;
vector<Triangle*> g_triangles;
double m_scale = 0.05;
double m_translateX = 0;
double m_translateY = 0;
double center_objectX = 0;
double center_objectY = 0;
int window_width = 1680;
int window_height = 1050;

#define PI 3.14159265

void initializeOpengl(int width, int height)
{	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glViewport(0, 0, width, height); 

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// Calculate The Aspect Ratio Of The Window
	gluPerspective(45.0f,(GLfloat)width/(GLfloat)height, 0.1f, 100.0f);
}

int isExistVertice(Vertices vertices, DoublePoint point)
{
	for (unsigned int i = 0; i < vertices.size(); i++)
	{
		if (vertices[i].point == point)
		{
			return i;
		}
	}

	return -1;
}

bool isExistEdge(Edges edges, Edge edge)
{
	for (unsigned int i = 0; i < edges.size(); i++)
	{
		if (edges[i] == edge)
		{
			return true;
		}
	}

	return false;
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

void Triangulate(Paths poly, DoublePoint start, Vertices & vertices, Edges & edges, int & startIdx)
{
	struct triangulateio in, out;
	memset(&in, 0, sizeof(triangulateio));
	memset(&out, 0, sizeof(triangulateio));

	std::vector<int> vertexType;

	double object_minX = 100000000;
	double object_minY = 100000000;
	double object_maxX = -100000000;
	double object_maxY = -100000000;

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
			}
			else
			{
				vertexType.push_back(other_type);

			}

			idxEdge++;


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

	center_objectX = (object_maxX + object_minX)/2;
	center_objectY = (object_maxY + object_minY)/2;

	startIdx = isExistVertice(vertices, start);
	if (startIdx == -1)
	{
		startIdx = vertices.size();
		Vertex vertex;
		vertex.point=start;
		vertexType.push_back(other_type);
		vertices.push_back(vertex);
	}
	
	double *pointlist = new double[vertices.size()*2]();
	int *pointmakerlist = new int[vertices.size()]();
	for (unsigned int i = 0; i < vertices.size(); i++)
	{
		pointlist[2*i]=vertices[i].point.x;
		pointlist[2*i+1]=vertices[i].point.y;

		pointmakerlist[i] = vertexType[i];
	}

	int *segmentlist = new int[edges.size()*2]();
	for (unsigned int i = 0; i < edges.size(); i++)
	{
		segmentlist[2*i]=edges[i].idxA;
		segmentlist[2*i+1]=edges[i].idxB;
	}

	/// Define input points
	in.numberofpoints = vertices.size();
	in.pointlist = pointlist;

	// Point maker
	in.pointmarkerlist = pointmakerlist;

	/// Define segments
	in.numberofsegments = edges.size();
	in.segmentlist = segmentlist;

	//Define holes
	double *holes = new double[(poly.size()-1)*2]();
	in.numberofholes = poly.size()-1;
	for (int i = 1; i <= in.numberofholes; i++)
	{
		double minX = 1000000;
		double minY = 1000000;
		double maxX = -1000000;
		double maxY = -1000000;
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

#ifdef _DEBUG
	//triangulate("pz", &in, &out, 0);
	triangulate("pzen", &in, &out, 0);
#else
	triangulate("pzQ", &in, &out, 0);
#endif

	vertices.resize(0);
	int numberofvertices = out.numberofpoints;
	for (int i = 0; i < numberofvertices; ++i)
	{
		Vertex vertex;
		vertex.point=DoublePoint(out.pointlist[2*i], out.pointlist[2*i+1]);

		int pointmaker = out.pointmarkerlist[i];
		if (pointmaker == boundary_type)
		{
			vertex.isBoundaryVertex = true;
		}
		else
		{
			vertex.isBoundaryVertex = false;
		}
		
		vertices.push_back(vertex);
	}
	startIdx = isExistVertice(vertices, start);

	edges.resize(0);
	int numberofedges = out.numberofedges;
	for (int i = 0; i < numberofedges; ++i)
	{
		Edge edge = Edge(out.edgelist[2*i], out.edgelist[2*i+1]);
		edges.push_back(edge);
	}

	int k = 0;
	for (int i = 0; i < out.numberoftriangles; i++)
	{
		Triangle * triangle = new Triangle;
		triangle->apex1 = out.trianglelist[3*i];
		triangle->apex2 = out.trianglelist[3*i+1];
		triangle->apex3 = out.trianglelist[3*i+2];
		g_triangles.push_back(triangle);
	}

	free(out.pointlist);
	free(out.pointmarkerlist);
	free(out.trianglelist);
	free(out.neighborlist);
	free(out.segmentlist);
	free(out.segmentmarkerlist);
	free(out.edgelist);

	delete[] segmentlist;
	delete[] pointlist;
	delete[] pointmakerlist;
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

	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Draw triangle
	glColor3f(205.0/255.0, 192.0/255.0, 176.0/255.0);
	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < g_triangles.size(); i++)
	{
		x1 = g_vertices[g_triangles[i]->apex1].point.x;
		y1 = g_vertices[g_triangles[i]->apex1].point.y;
		x2 = g_vertices[g_triangles[i]->apex2].point.x;
		y2 = g_vertices[g_triangles[i]->apex2].point.y;
		x3 = g_vertices[g_triangles[i]->apex3].point.x;
		y3 = g_vertices[g_triangles[i]->apex3].point.y;

		glVertex3f(x1, y1, -15.0f);
		glVertex3f(x2, y2, -15.0f);
		glVertex3f(x3, y3, -15.0f);
	}
	glEnd();

	//// Draw polygon only
	//glColor3f(1.0, 1.0,1.0);
	//glBegin(GL_LINES);
	//int idxEdge = 0;
	//for (unsigned int i = 0; i < g_poly.size(); i++)
	//{
	//	unsigned int j;
	//	for (j = 0; j < g_poly[i].size()-1; j++)
	//	{
	//		x1 = g_poly[i][j].point.x;
	//		y1 = g_poly[i][j].point.y;
	//		x2 = g_poly[i][j+1].point.x;
	//		y2 = g_poly[i][j+1].point.y;

	//		glVertex3f(x1, y1, -15.0f);
	//		glVertex3f(x2, y2, -15.0f);
	//	}

	//	x1 = g_poly[i][j].point.x;
	//	y1 = g_poly[i][j].point.y;
	//	x2 = g_poly[i][0].point.x;
	//	y2 = g_poly[i][0].point.y;

	//	glVertex3f(x1, y1, -15.0f);
	//	glVertex3f(x2, y2, -15.0f);
	//}
	//glEnd();

	//// Draw shortest path
	//glColor3f(0.0, 0.5, 1.0);
	//glBegin(GL_LINES);
	//for (unsigned int i = 0; i < g_shortestPath.size()-1; i++)
	//{
	//	x1 = g_vertices[g_shortestPath[i]].point.x;
	//	y1 = g_vertices[g_shortestPath[i]].point.y;
	//	x2 = g_vertices[g_shortestPath[i+1]].point.x;
	//	y2 = g_vertices[g_shortestPath[i+1]].point.y;

	//	glVertex3f(x1, y1, -15.0f);
	//	glVertex3f(x2, y2, -15.0f);
	//}
	//glEnd();

	glutSwapBuffers();
}

//---------------------------------------------------------------------------
void ReadFromFile(char *filename, Paths &paths)
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

void MakePolygon(Paths & poly)
{
	//poly.resize(2);

	//poly[0].push_back(DoublePoint(0.0, 0.0));
	//poly[0].push_back(DoublePoint(0.0, 5.0));
	//poly[0].push_back(DoublePoint(5.0, 5.0));
	//poly[0].push_back(DoublePoint(5.0, 0.0));
	//poly[0].push_back(DoublePoint(4.0, 1.0));

	//poly[1].push_back(DoublePoint(1.0, 3.0));
	//poly[1].push_back(DoublePoint(1.0, 4.0));
	//poly[1].push_back(DoublePoint(2.0, 4.0));
	//poly[1].push_back(DoublePoint(3.0, 2.0));
	//poly[1].push_back(DoublePoint(2.0, 3.0));

	ReadFromFile("input1.txt", poly);
}

double getDistance(DoublePoint A, DoublePoint B)
{
	return sqrt(pow(A.x-B.x, 2)+pow(A.y-B.y, 2));
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

	//Init vertex list
	//DoublePoint start = DoublePoint(12.27, 52.8);
	//DoublePoint end = DoublePoint(54.52, 12.27);

	DoublePoint start = DoublePoint(-85, 51);

	int startIdx;

	MakePolygon(g_poly);

	// Triangulate
	Triangulate(g_poly, start, g_vertices, g_edges, startIdx);

	// Method 1: Dijkstra
	struct Graph* graph = createGraph(g_vertices);
	int num_edge = g_edges.size();
	for (unsigned int i = 0; i < g_edges.size(); i++)
	{
		addEdge(graph, g_edges[i].idxA, g_edges[i].idxB, 
			getDistance(g_vertices[g_edges[i].idxA].point, g_vertices[g_edges[i].idxB].point));
	}

	findClosestRectangleBoundary(graph, startIdx, g_shortestPath);

	glutMainLoop();

	return 0;
}