#include <fstream>
#include <string>
#include "main.h"
//#include "vld.h"
#include "gfunc.h"
#include "gf_types.h"

using namespace std;

// Global variable
Polygon g_poly;
Vertices g_vertices;
Edges g_edges;
ShortestChannel g_shortestChannel;
ShortestPath g_shortestPath;
Triangles g_triangles;
gf_polygon *poly = NULL;

// Only for drawing OpenGL
double m_scale = 0.05;
double m_translateX = 0;
double m_translateY = 0;
double center_objectX = 0;
double center_objectY = 0;
int window_width = 1680;
int window_height = 1050;

vector<int> g_leftVertices;
vector<int> g_rightVertices;

#define PI 3.14159265
#define MAX_DOUBLE 1000000000
#define MIN_DOUBLE -1000000000

void releaseEdges(Edges & edges)
{
	while (!edges.empty())
	{
		delete edges[0];
		edges.erase(edges.begin());
	}
}
void releaseVertices(Vertices & vertices)
{
	while (!vertices.empty())
	{
		delete vertices[0];
		vertices.erase(vertices.begin());
	}
}

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

void GetAPointInsidePolygon(double minX, double minY, double maxX, double maxY, gf_contour contour,double& resultX,double &resultY)
{
	int i, j;
	bool c = false;
	double testx , testy;

	while (c == false)
	{
		testx = fRand(minX, maxX);
		testy = fRand(minY, maxY);
		//for (i = 0, j = contour.nPoints-1; i < contour.nPoints; j = i++) {
		//	if ( ((contour.point[i].y>testy) != (contour.point[j].y>testy)) 
		//		&& (testx < (contour.point[j].x-contour.point[i].x) * (testy-contour.point[i].y) / (contour.point[j].y-contour.point[i].y) + contour.point[i].x) )
		//		c = !c;
		//}

		j = contour.nPoints-1;
		for (i = 0; i < contour.nPoints-1; i++) {
			if ((contour.point[i].y < testy && contour.point[j].y >= testy
				|| contour.point[j].y < testy && contour.point[i].y >= testy)
				&& (contour.point[i].x <= testx || contour.point[j].x <= testx)) {
					c^=(contour.point[i].x + (testy - contour.point[i].y)/(contour.point[j].y - contour.point[i].y)*(contour.point[j].x - contour.point[i].x) < testx);
			}
			j = i; 
		}
	}

	resultX = testx;
	resultY = testy;
}

bool CheckAPointInPolygon(DoublePoint point, Triangle *triangle) 
{
	int i, j;
	bool c = false;

	for (i = 0, j = 2; i < 3; j = i++) 
	{
		if (((g_vertices[triangle->apexes[i]]->point.y > point.y) != (g_vertices[triangle->apexes[j]]->point.y > point.y)) &&
			(point.x < (g_vertices[triangle->apexes[j]]->point.x - g_vertices[triangle->apexes[i]]->point.x) * 
			(point.y - g_vertices[triangle->apexes[i]]->point.y) / (g_vertices[triangle->apexes[j]]->point.y - 
			g_vertices[triangle->apexes[i]]->point.y) + g_vertices[triangle->apexes[i]]->point.x))
			c = !c;
	}

	return c;
}

gf_polygon *make_gf_poligon(Polygon poly)
{
	gf_polygon *polygon = new gf_polygon;
	polygon->nContours = poly.size();
	polygon->contour = new gf_contour[polygon->nContours];
	polygon->box = new gf_box[polygon->nContours];
	for (int i = 0; i < polygon->nContours; i++)
	{
		polygon->contour[i].nPoints = poly[i].size();
		polygon->contour[i].point = new gf_point[polygon->contour[i].nPoints];
		double low = MAX_DOUBLE;
		double left = MAX_DOUBLE;
		double upper = MIN_DOUBLE;
		double right = MIN_DOUBLE;
		for (int j = 0; j < polygon->contour[i].nPoints; j++)
		{
			polygon->contour[i].point[j].x = poly[i][j]->point.x;
			polygon->contour[i].point[j].y = poly[i][j]->point.y;

			if (polygon->contour[i].point[j].x < left)
			{
				left = polygon->contour[i].point[j].x;
			}
			if (polygon->contour[i].point[j].x > right)
			{
				right = polygon->contour[i].point[j].x;
			}
			if (polygon->contour[i].point[j].y < low)
			{
				low = polygon->contour[i].point[j].y;
			}
			if (polygon->contour[i].point[j].y > upper)
			{
				upper = polygon->contour[i].point[j].y;
			}
		}
		polygon->box[i].ll.x = left;
		polygon->box[i].ll.y = low;

		polygon->box[i].ur.x = right;
		polygon->box[i].ur.y = upper;
	}
	
	return polygon;
}

gf_polygon *make_gf_poligon_from_file()
{
	// read file
	string line;
	ifstream myfile ("C:/Users/ptest21/Desktop/test.txt");
	//getline (myfile,line)

	if (myfile.is_open())
	{
		gf_polygon *polygon = new gf_polygon;
		getline (myfile,line);
		sscanf (line.c_str(),"%d",&polygon->nContours);

		//polygon->nContours = poly.size();
		polygon->contour = new gf_contour[polygon->nContours];
		polygon->box = new gf_box[polygon->nContours];
		for (int i = 0; i < polygon->nContours; i++)
		{
			getline (myfile,line);
			sscanf (line.c_str(),"%d",&polygon->contour[i].nPoints);
			//polygon->contour[i].nPoints = poly[i].size();
			polygon->contour[i].point = new gf_point[polygon->contour[i].nPoints];
			double low = MAX_DOUBLE;
			double left = MAX_DOUBLE;
			double upper = MIN_DOUBLE;
			double right = MIN_DOUBLE;
			for (int j = 0; j < polygon->contour[i].nPoints; j++)
			{
				getline (myfile,line);
				sscanf (line.c_str(),"%lf %lf",&polygon->contour[i].point[j].x,&polygon->contour[i].point[j].y);
				//polygon->contour[i].point[j].x = poly[i][j]->point.x;
				//polygon->contour[i].point[j].y = poly[i][j]->point.y;

				if (polygon->contour[i].point[j].x < left)
				{
					left = polygon->contour[i].point[j].x;
				}
				if (polygon->contour[i].point[j].x > right)
				{
					right = polygon->contour[i].point[j].x;
				}
				if (polygon->contour[i].point[j].y < low)
				{
					low = polygon->contour[i].point[j].y;
				}
				if (polygon->contour[i].point[j].y > upper)
				{
					upper = polygon->contour[i].point[j].y;
				}
			}
			polygon->box[i].ll.x = left;
			polygon->box[i].ll.y = low;

			polygon->box[i].ur.x = right;
			polygon->box[i].ur.y = upper;
		}

		myfile.close();

		return polygon;
	}

	else return NULL; 
}

void Triangulate(gf_polygon *poly, Vertices & vertices, Edges & edges, Triangles & triangles)
{
	// Only for render OpenGL
	double object_minX = MAX_DOUBLE;
	double object_minY = MAX_DOUBLE;
	double object_maxX = MIN_DOUBLE;
	double object_maxY = MIN_DOUBLE;

	// Declare input of triangulate function
	struct triangulateio in, out;
	memset(&in, 0, sizeof(triangulateio));
	memset(&out, 0, sizeof(triangulateio));

	// Loop through all vertex of polygon
	for (int i = 0; i < poly->nContours; i++)
	{
		int startVertexIdx = vertices.size();
		double curX,curY;
		for (int j = 0; j < poly->contour[i].nPoints; j++)
		{
			if (j == 0)
			{
				curX = poly->contour[i].point[j].x;
				curY = poly->contour[i].point[j].y;

				// Add vertex to vertices list
				Vertex *vertex = new Vertex;
				vertex->point = DoublePoint(poly->contour[i].point[j].x, poly->contour[i].point[j].y);
				vertices.push_back(vertex);
			}
			else
			{
				if (fabs(poly->contour[i].point[j].x - curX) <= 0.0001 && 
					fabs(poly->contour[i].point[j].y - curY) <= 0.0001)
				{
					continue;
				}
				
				curX = poly->contour[i].point[j].x;
				curY = poly->contour[i].point[j].y;

				// Add vertex to vertices list
				Vertex *vertex = new Vertex;
				vertex->point = DoublePoint(poly->contour[i].point[j].x, poly->contour[i].point[j].y);
				vertices.push_back(vertex);

				edges.push_back(new Edge(vertices.size()-2, vertices.size()-1));
			}

			// Only for render OpenGL
			if (vertices[i]->point.x < object_minX)
			{
				object_minX = vertices[i]->point.x;
			}
			if (vertices[i]->point.x > object_maxX)
			{
				object_maxX = vertices[i]->point.x;
			}
			if (vertices[i]->point.y < object_minY)
			{
				object_minY = vertices[i]->point.y;
			}
			if (vertices[i]->point.y > object_maxY)
			{
				object_maxY = vertices[i]->point.y;
			}
		}
		edges.push_back(new Edge(vertices.size()-1, startVertexIdx));
	}

	// Only for render OpenGL
	center_objectX = (object_maxX + object_minX)/2;
	center_objectY = (object_maxY + object_minY)/2;

	// Make point list with structure of input's triangulate function
	double *pointlist = new double[vertices.size()*2]();
	for (unsigned int i = 0; i < vertices.size(); i++)
	{
		pointlist[2*i]=vertices[i]->point.x;
		pointlist[2*i+1]=vertices[i]->point.y;
	}

	// Make segment list with structure of input's triangulate function
	int *segmentlist = new int[edges.size()*2]();
	for (unsigned int i = 0; i < edges.size(); i++)
	{
		segmentlist[2*i]=edges[i]->idxA;
		segmentlist[2*i+1]=edges[i]->idxB;
	}

	/// Input points
	in.numberofpoints = vertices.size();
	in.pointlist = pointlist;

	/// Input segments
	in.numberofsegments = edges.size();
	in.segmentlist = segmentlist;

	// Input holes
	double *holes = new double[(poly->nContours-1)*2]();
	in.numberofholes = poly->nContours-1;

	// With each hole, find a point inside it
	double dResultX;
	double dResultY;
	for (int i = 1; i <= in.numberofholes; i++)
	{
		GetAPointInsidePolygon(poly->box[i].ll.x, poly->box[i].ll.y, poly->box[i].ur.x, poly->box[i].ur.y, poly->contour[i],dResultX,dResultY);
		holes[(i-1)*2] = dResultX;
		holes[(i-1)*2+1] = dResultY;
	}
	in.holelist = holes;
	//in.nItemID = ESCII_HIGHSPEED_COMPLETESHIELD;
	//in.nItemID = -1;

	// Triangulate using library
	//double area = sqrt(pow(poly->box[0].ll.x-poly->box[0].ur.x,2)+pow(poly->box[0].ll.y-poly->box[0].ur.y,2))/100.0;
	char params[20];
	//sprintf (params, "pq10a%.2lfDzen", area);
	sprintf (params, "pDzen");
	triangulate(params/*parameter*/, &in/*input*/, &out/*output*/, 0/*don't care*/);

	// Get vertices list after triangulation
	releaseVertices(vertices);
	int numberofvertices = out.numberofpoints;
	for (int i = 0; i < numberofvertices; ++i)
	{
		Vertex *vertex = new Vertex;
		vertex->point= DoublePoint(out.pointlist[2*i], out.pointlist[2*i+1]);
		vertices.push_back(vertex);
	}

	// Get edges list after triangulation
	releaseEdges(edges);
	int numberofedges = out.numberofedges;
	for (int i = 0; i < numberofedges; ++i)
	{
		Edge *edge = new Edge(out.edgelist[2*i], out.edgelist[2*i+1]);
		edges.push_back(edge);
	}

	// Get triangles list after triangulation
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
		}
		triangles.push_back(triangle);
	}

	// Free memory
	if(in.pointlist)
		free(in.pointlist);
	if(in.pointattributelist)
		free(in.pointattributelist);
	if(in.pointmarkerlist)
		free(in.pointmarkerlist);
	if(in.trianglelist)
		free(in.trianglelist);
	if(in.triangleattributelist)
		free(in.triangleattributelist);
	if(in.trianglearealist)
		free(in.trianglearealist);
	if(in.segmentmarkerlist)
		free(in.segmentmarkerlist);
	if(in.holelist)
		free(in.holelist);
	if(in.segmentlist)
		free(in.segmentlist);

	if(out.edgelist)
		free(out.edgelist);
	if(out.pointattributelist)
		free(out.pointattributelist);
	if(out.pointmarkerlist)
		free(out.pointmarkerlist);
	if(out.trianglelist)
		free(out.trianglelist);
	if(out.triangleattributelist)
		free(out.triangleattributelist);
	if(out.neighborlist)
		free(out.neighborlist);
	if(out.segmentlist)
		free(out.segmentlist);
	if(out.segmentmarkerlist)
		free(out.segmentmarkerlist);
	if(out.regionlist)
		free(out.regionlist);
	if(out.edgemarkerlist)
		free(out.edgemarkerlist);
	if(out.normlist)
		free(out.normlist);
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

	//if (poly != NULL)
	//{
	//	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//	// Draw triangle
	//	glColor3f(205.0/255.0, 192.0/255.0, 176.0/255.0);
	//	for (int i = 0; i < poly->nContours; i++)
	//	{
	//		glBegin(GL_LINE_STRIP);
	//		for (int j = 0; j < poly->contour[i].nPoints; j++)
	//		{
	//			glVertex3f(poly->contour[i].point[j].x, poly->contour[i].point[j].y, -15.0f);
	//		}
	//		glEnd();
	//	}
	//}
	
	

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	// Draw triangle
	glColor3f(205.0/255.0, 192.0/255.0, 176.0/255.0);
	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < g_triangles.size(); i++)
	{
		x1 = g_vertices[g_triangles[i]->apexes[0]]->point.x;
		y1 = g_vertices[g_triangles[i]->apexes[0]]->point.y;
		x2 = g_vertices[g_triangles[i]->apexes[1]]->point.x;
		y2 = g_vertices[g_triangles[i]->apexes[1]]->point.y;
		x3 = g_vertices[g_triangles[i]->apexes[2]]->point.x;
		y3 = g_vertices[g_triangles[i]->apexes[2]]->point.y;

		glVertex3f(x1, y1, -15.0f);
		glVertex3f(x2, y2, -15.0f);
		glVertex3f(x3, y3, -15.0f);
	}
	glEnd();




	// Draw shortest channel
	glColor3f(223.0/255.0, 119.0/255.0, 39.0/255.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < g_shortestChannel.size(); i++)
	{
		x1 = g_vertices[g_triangles[g_shortestChannel[i]]->apexes[0]]->point.x;
		y1 = g_vertices[g_triangles[g_shortestChannel[i]]->apexes[0]]->point.y;
		x2 = g_vertices[g_triangles[g_shortestChannel[i]]->apexes[1]]->point.x;
		y2 = g_vertices[g_triangles[g_shortestChannel[i]]->apexes[1]]->point.y;
		x3 = g_vertices[g_triangles[g_shortestChannel[i]]->apexes[2]]->point.x;
		y3 = g_vertices[g_triangles[g_shortestChannel[i]]->apexes[2]]->point.y;

		glVertex3f(x1, y1, -15.0f);
		glVertex3f(x2, y2, -15.0f);
		glVertex3f(x3, y3, -15.0f);
	}
	glEnd();

	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//// Draw triangle
	//glColor3f(0.0, 0.0, 0.0);
	//glBegin(GL_TRIANGLES);
	//for (unsigned int i = 0; i < g_triangles.size(); i++)
	//{
	//	x1 = g_vertices[g_triangles[i]->apexes[0]].point.x;
	//	y1 = g_vertices[g_triangles[i]->apexes[0]].point.y;
	//	x2 = g_vertices[g_triangles[i]->apexes[1]].point.x;
	//	y2 = g_vertices[g_triangles[i]->apexes[1]].point.y;
	//	x3 = g_vertices[g_triangles[i]->apexes[2]].point.x;
	//	y3 = g_vertices[g_triangles[i]->apexes[2]].point.y;

	//	glVertex3f(x1, y1, -15.0f);
	//	glVertex3f(x2, y2, -15.0f);
	//	glVertex3f(x3, y3, -15.0f);
	//}
	//glEnd();

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

						Vertex *vertex = new Vertex;
						vertex->point = DoublePoint(startX,startY);
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

						int num_segments = 10;//floor(abs(arc_angle));
						double theta = arc_angle / double(num_segments - 1);

						double tangetial_factor = tan(theta * PI / 180.0);

						double radial_factor = cos(theta * PI / 180.0);


						double x = radius * cos(startDegree * PI / 180.0);
						double y = radius * sin(startDegree * PI / 180.0); 

						for(int ii = 0; ii < num_segments; ii++)
						{
							Vertex *vertex = new Vertex;
							vertex->point = DoublePoint(x + centerX, y + centerY);
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
	/*poly.resize(3);

	Vertex vertex1;
	vertex1.point = DoublePoint(0.0, 0.0);
	Vertex vertex2;
	vertex2.point = DoublePoint(0.0, 5.0);
	Vertex vertex3;
	vertex3.point = DoublePoint(1.0, 5.0);
	Vertex vertex4;
	vertex4.point = DoublePoint(1.0, 4.5);
	Vertex vertex5;
	vertex5.point = DoublePoint(1.2, 4.5);
	Vertex vertex6;
	vertex6.point = DoublePoint(1.2, 5.0);
	Vertex vertex7;
	vertex7.point = DoublePoint(8.0, 5.0);
	Vertex vertex8;
	vertex8.point = DoublePoint(8.0, 0.0);
	Vertex vertex9;
	vertex9.point = DoublePoint(7.0, 0.0);
	Vertex vertex10;
	vertex10.point = DoublePoint(7.0, 0.5);
	Vertex vertex11;
	vertex11.point = DoublePoint(6.8, 0.5);
	Vertex vertex12;
	vertex12.point = DoublePoint(6.8, 0.0);

	Vertex vertex13;
	vertex13.point = DoublePoint(1.5, 2.5);
	Vertex vertex14;
	vertex14.point = DoublePoint(2.5, 4.5);
	Vertex vertex15;
	vertex15.point = DoublePoint(3.5, 4.5);
	Vertex vertex16;
	vertex16.point = DoublePoint(4.0, 2.5);
	Vertex vertex17;
	vertex17.point = DoublePoint(3.5, 1.0);
	Vertex vertex18;
	vertex18.point = DoublePoint(2.5, 1.0);

	Vertex vertex19;
	vertex19.point = DoublePoint(5.0, 2.0);
	Vertex vertex20;
	vertex20.point = DoublePoint(5.0, 3.0);
	Vertex vertex21;
	vertex21.point = DoublePoint(7.0, 3.0);
	Vertex vertex22;
	vertex22.point = DoublePoint(7.0, 2.0);

	poly[0].push_back(vertex1);
	poly[0].push_back(vertex2);
	poly[0].push_back(vertex3);
	poly[0].push_back(vertex4);
	poly[0].push_back(vertex5);
	poly[0].push_back(vertex6);
	poly[0].push_back(vertex7);
	poly[0].push_back(vertex8);
	poly[0].push_back(vertex9);
	poly[0].push_back(vertex10);
	poly[0].push_back(vertex11);
	poly[0].push_back(vertex12);

	poly[1].push_back(vertex13);
	poly[1].push_back(vertex14);
	poly[1].push_back(vertex15);
	poly[1].push_back(vertex16);
	poly[1].push_back(vertex17);
	poly[1].push_back(vertex18);

	poly[2].push_back(vertex19);
	poly[2].push_back(vertex20);
	poly[2].push_back(vertex21);
	poly[2].push_back(vertex22);*/

	ReadFromFile("input3.txt", poly);
}

double getDistance(DoublePoint A, DoublePoint B)
{
	return sqrt(pow(A.x-B.x, 2)+pow(A.y-B.y, 2));
}

double getAverageDistanceBetweenTwoTriangles(Triangle* A, Triangle* B)
{
	Vertex *A0 = g_vertices[A->apexes[0]];
	Vertex *A1 = g_vertices[A->apexes[1]];
	Vertex *A2 = g_vertices[A->apexes[2]];

	Vertex *B0 = g_vertices[B->apexes[0]];
	Vertex *B1 = g_vertices[B->apexes[1]];
	Vertex *B2 = g_vertices[B->apexes[2]];
	
	DoublePoint centerA = DoublePoint((A0->point.x + A1->point.x + A2->point.x) / 3, (A0->point.y + A1->point.y + A2->point.y) / 3);
	DoublePoint centerB = DoublePoint((B0->point.x + B1->point.x + B2->point.x) / 3, (B0->point.y + B1->point.y + B2->point.y) / 3);

	return getDistance(centerA, centerB);
}

int getTriangleOfAPoint(DoublePoint start, Triangles triangles)
{
	for (unsigned int i = 0; i < triangles.size(); i++)
	{
		if (CheckAPointInPolygon(start, triangles[i]))
		{
			return i;
		}
	}

	return -1;
}

// Check order of 2 vector. Left vector or Right vector ?
double CrossProduct(DoublePoint vector1, DoublePoint vector2)
{
	return vector1.x*vector2.y - vector1.y*vector2.x;
}

bool CheckReadyConnectToEndPoint(DoublePoint startPoint, DoublePoint endPoint, vector<int> leftVertices, vector<int> rightVertices, int currentIdx)
{
	DoublePoint vectorCenter = endPoint - startPoint;
	for (unsigned int i = currentIdx; i < leftVertices.size() - 1; i++)
	{
		DoublePoint vectorLeft = g_vertices[leftVertices[i]]->point - startPoint;
		DoublePoint vectorRight = g_vertices[rightVertices[i]]->point - startPoint;
		if (CrossProduct(vectorCenter, vectorLeft) < 0 || CrossProduct(vectorCenter, vectorRight) > 0)
		{
			return false;
		}
	}

	return true;
}

// Find shortest path using Funnel algorithm
void Funnel(DoublePoint startPoint, DoublePoint endPoint, ShortestChannel channel, ShortestPath & shortestPath)
{
	// Initialize portal vertices.
	g_leftVertices.resize(channel.size() + 1);
	g_rightVertices.resize(channel.size() + 1);
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
			DoublePoint vector1 = g_vertices[currentTriangle->apexes[commonVertex[0]]]->point 
				- g_vertices[currentTriangle->apexes[uncommonVertex]]->point;
			DoublePoint vector2 = g_vertices[currentTriangle->apexes[commonVertex[1]]]->point 
				- g_vertices[currentTriangle->apexes[uncommonVertex]]->point;
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

	// Initialize first portal vertex.
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

	// Initialize last portal vertex.
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

	// Step through channel to find the shortest path
	unsigned int left = 1;
	unsigned int right = 1;
	shortestPath.push_back(startPoint);
	DoublePoint apex = startPoint;

	for (unsigned int i = 2; i <= channel.size(); i++)
	{
		// If new left vertex is different => process.
		if (g_leftVertices[i] != g_leftVertices[left] && i > left)
		{
			DoublePoint newVector = g_vertices[g_leftVertices[i]]->point - apex;
			DoublePoint leftVector = g_vertices[g_leftVertices[left]]->point - apex;
			DoublePoint rightVector = g_vertices[g_rightVertices[right]]->point - apex;

			// If new vertex does not widen funnel, update.
			if (CrossProduct(newVector, leftVector) > 0)
			{
				// If new vertex make cross other side, update apex.
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

					apex = g_vertices[g_rightVertices[right]]->point;
					shortestPath.push_back(apex);
					right = next;
					left = next;
					i = next;

					if (CheckReadyConnectToEndPoint(apex, endPoint, g_leftVertices, g_rightVertices, next))
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

		// If new right vertex is different => process.
		if (g_rightVertices[i] != g_rightVertices[right] && i > right)
		{
			DoublePoint newVector = g_vertices[g_rightVertices[i]]->point - apex;
			DoublePoint leftVector = g_vertices[g_leftVertices[left]]->point - apex;
			DoublePoint rightVector = g_vertices[g_rightVertices[right]]->point - apex;

			// If new vertex does not widen funnel, update.
			if (CrossProduct(newVector, rightVector) < 0)
			{
				// If new vertex crosses other side, update apex.
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

					apex = g_vertices[g_leftVertices[left]]->point;
					shortestPath.push_back(apex);
					left = next;
					right = next;
					i = next;

					if (CheckReadyConnectToEndPoint(apex, endPoint, g_leftVertices, g_rightVertices, next))
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

	shortestPath.push_back(endPoint);
}

void releaseTriangles(Triangles & triangles)
{
	while (!triangles.empty())
	{
		delete triangles[0];
		triangles.erase(triangles.begin());
	}
}

int main(int argc, char** argv)
{
	//std::fstream fs;
	//fs.open ("C:/Users/ptest21/Desktop/abc.txt", std::ofstream::out | std::ofstream::trunc);
	//double a = 3.3;
	//double b = 4.4;
	//fs << a << " " << b << "\n";
	//fs << 45 << " " << 34 << "\n";
	//fs.close();
	//return 0;

	const char* windowName = "Demo Shortest Path";
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(window_width, window_height);
	glutCreateWindow(windowName);
	glutDisplayFunc(displayFunc);
	glutSpecialFunc(mykey);
	initializeOpengl(window_width, window_height);

	// Declare start and end point input3.txt
	DoublePoint start = DoublePoint(0.6, 50.6);
	DoublePoint end = DoublePoint(6, 48.75);

	//DoublePoint start = DoublePoint(0.6, 50.6);
	//DoublePoint end = DoublePoint(6, 45.75);

	//DoublePoint start = DoublePoint(2.2, 42.2);
	//DoublePoint end = DoublePoint(2.2, 39.4);

	//===========================================================
	//DoublePoint start = DoublePoint(-63, 1.84);
	//DoublePoint end = DoublePoint(-9.81, 10.68);

	//DoublePoint start = DoublePoint(0.5, 0.5);
	//DoublePoint end = DoublePoint(7.5, 4.5);

	// Step 1: Make polygon
	MakePolygon(g_poly);
	gf_polygon *poly = make_gf_poligon(g_poly);
	//poly = make_gf_poligon_from_file();

	time_t time_start = clock();

	//// Step 2: Make triangulate
	Triangulate(poly/*input*/, g_vertices/*output*/, g_edges/*output*/, g_triangles/*output*/);

	//Step 3: Build graph of triangles
	//Declare graph
	//struct Graph* graph = createGraph(g_triangles.size());
	//// Add edge of 2 triangles if they are neighbor each other.
	//for (unsigned int i = 0; i < g_triangles.size(); i++)
	//{
	//	for (unsigned int j = 0; j < g_triangles[i]->adjacentTriangles.size(); j++)
	//	{
	//		int dest = g_triangles[i]->adjacentTriangles[j];
	//		double weight = getAverageDistanceBetweenTwoTriangles(g_triangles[i], g_triangles[dest]);
	//		addEdge(graph, i, dest, weight);
	//	}
	//}

	Graph1 graph(g_triangles.size());
	// Add edge of 2 triangles if they are neighbor each other.
	for (unsigned int i = 0; i < g_triangles.size(); i++)
	{
		for (unsigned int j = 0; j < g_triangles[i]->adjacentTriangles.size(); j++)
		{
			int dest = g_triangles[i]->adjacentTriangles[j];
			double weight = getAverageDistanceBetweenTwoTriangles(g_triangles[i], g_triangles[dest]);
			graph.addEdge(i, dest, weight);
		}
	}

	// Step 4: Find triangle of start and end point
	int startIdx = getTriangleOfAPoint(start, g_triangles);
	int endIdx = getTriangleOfAPoint(end, g_triangles);

	// Exit when start and end point are outside polygon or inside a hole
	if (startIdx == -1 || endIdx == -1) 
	{
		cout << "Start or end point are outside the polygon\n" << endl;
		return 1;
	}

	// Step 5: Build shortest channel. Based on dijkstra algorithm
	//findShortestChannel(graph/*input*/, startIdx/*input*/, endIdx/*input*/, g_shortestChannel/*output*/);
	g_shortestChannel = graph.shortestPath(startIdx,endIdx);

	// Step 6: Using funnel algorithm to find shortest path
	Funnel(start/*input*/, end/*input*/, g_shortestChannel/*input*/, g_shortestPath/*output*/);
	
	double length = 0;
	if (g_shortestPath.size() != 0)
	{
		for (unsigned int i = 0; i < g_shortestPath.size()-1; i++)
		{
			length += getDistance(g_shortestPath[i], g_shortestPath[i+1]);
		}
	}
	cout << "\shortest path length: " << length << "\n";


	// Estimate time elapsed.
	double time_elapsed_total = double(clock() - time_start)/CLOCKS_PER_SEC;
	cout << "\time_elapsed_total in " << time_elapsed_total << " secs \n";
	
	glutMainLoop();

	//// Step 7: Release memory
	gfFreePolygon(poly);
	//releaseGraph(graph);
	releaseTriangles(g_triangles);
	releaseEdges(g_edges);
	releaseVertices(g_vertices);

	return 0;
}