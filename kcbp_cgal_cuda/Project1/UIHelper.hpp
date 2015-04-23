#include "CP_PointVector.h"
#include <assert.h>
#include "IntersectionTools.hpp"
using namespace std;
#include <GL/glut.h>
#include "BoundingBox.hpp"
#include "AABB.hpp"

class UIHelper
{
    public:

    static void drawPolygons(vector<CP_Vector2D> &polygons)
    {
        ////draw 
        GLfloat color[4];
        glGetFloatv(GL_CURRENT_COLOR, color);
        glColor3f(0, 0, 1);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glBegin(GL_POLYGON);
        for (unsigned int i = 0; i < polygons.size(); i++)
        {
            CP_Vector2D p = polygons[i];
            glVertex2d(p.x, p.y);
        }
        glEnd();
        glColor4fv(color);
    }

    static void drawPolygons3D(vector<Polygon3D*> &polygons, int start, int end, int Polygon_Mode, RealValueType draw_scale)
    {
        drawPolygons3D(polygons, start, end, Polygon_Mode, draw_scale, 1.0);
    }
    static void drawPolygons3D(vector<Polygon3D*> &polygons, int start, int end, int Polygon_Mode, RealValueType draw_scale, RealValueType width)
    {
        ////draw 
        GLfloat color[4];
        glGetFloatv(GL_CURRENT_COLOR, color);
        glColor3f(0, 0, 1);
        glLineWidth(width);
        bool flag = true;
        if(flag)
        {
            glPolygonMode(GL_FRONT_AND_BACK, Polygon_Mode);
            for (int i = start; i < end; i++)
            {
                glBegin(GL_POLYGON);
                Polygon3D poly = * polygons[i];
                CP_Vector3D draw_nomal = *(poly.normal);
                //if (((poly.data[1] - poly.data[0]) ^ (poly.data[2] - poly.data[0])) * draw_nomal > .0)
                //    draw_nomal = - draw_nomal;
                glNormal3f(draw_nomal.x, draw_nomal.y, draw_nomal.z);
                
                for (unsigned int j = 0; j < poly.data.size(); j++)
                {
                    CP_Vector3D p = poly.data.at(j) * draw_scale;
                    glVertex3d(p.x, p.y, p.z);    
                }
                glEnd();
            } 
            glColor4fv(color);
            return;
        }
        //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        vector<CP_Vector3D> colors;
        srand (0);
        for (unsigned int i = 0; i<polygons.size(); i++)
        {
            colors.push_back(CP_Vector3D(rand()%10 / 10.0,rand()%10 / 10.0,rand()%10 / 10.0));
        }
        for (int i = start; i < end; i++)
        {
            glColor3f(colors[i].x, colors[i].y, colors[i].z);
            glBegin(GL_LINE_LOOP);
            Polygon3D* poly = polygons[i];
            for (unsigned int j = 0; j < poly->data.size(); j++)
            {
                CP_Vector3D p =(poly->data.at(j)) * draw_scale;
                glVertex3d(p.x, p.y, p.z);    
            }
            glEnd();    
        }

        glColor4fv(color);
    }

    static void Arrow(GLdouble x1,GLdouble y1,GLdouble z1,GLdouble x2,GLdouble y2,GLdouble z2,GLdouble D)
    {
        #define RADPERDEG 0.0174533

        RealValueType x=x2-x1;
        RealValueType y=y2-y1;
        RealValueType z=z2-z1;
        RealValueType L=sqrt(x*x+y*y+z*z);

        GLUquadricObj *quadObj;

        glPushMatrix ();

        glTranslated(x1,y1,z1);

        if((x!=0.)||(y!=0.)) {
            glRotated(atan2(y,x)/RADPERDEG,0.,0.,1.);
            glRotated(atan2(sqrt(x*x+y*y),z)/RADPERDEG,0.,1.,0.);
        } else if (z<0){
            glRotated(180,1.,0.,0.);
        }

        glTranslatef(0,0,L-4*D);

        quadObj = gluNewQuadric ();
        gluQuadricDrawStyle (quadObj, GLU_FILL);
        gluQuadricNormals (quadObj, GLU_SMOOTH);
        gluCylinder(quadObj, 2*D, 0.0, 4*D, 32, 1);
        gluDeleteQuadric(quadObj);
        /*
        quadObj = gluNewQuadric ();
        gluQuadricDrawStyle (quadObj, GLU_FILL);
        gluQuadricNormals (quadObj, GLU_SMOOTH);
        gluDisk(quadObj, 0.0, 2*D, 32, 1);
        gluDeleteQuadric(quadObj);
        */
        glTranslatef(0,0,-L+4*D);

        quadObj = gluNewQuadric ();
        gluQuadricDrawStyle (quadObj, GLU_FILL);
        gluQuadricNormals (quadObj, GLU_SMOOTH);
        gluCylinder(quadObj, D, D, L-4*D, 32, 1);
        gluDeleteQuadric(quadObj);
        /*
        quadObj = gluNewQuadric ();
        gluQuadricDrawStyle (quadObj, GLU_FILL);
        gluQuadricNormals (quadObj, GLU_SMOOTH);
        gluDisk(quadObj, 0.0, D, 32, 1);
        gluDeleteQuadric(quadObj);
        */
        glPopMatrix ();

    }


    static void drawPlanes(vector<Plane3D> &planes, int start = 0, int end = 1)
    {
        const int LINE_LENGTH = 5;
        GLfloat color[4];
        glGetFloatv(GL_CURRENT_COLOR, color);
        glColor3f(0, 0, 1);
        for (int i = start; i < end; i++)
        {
            Plane3D plane = planes[i];
            CP_Vector3D point = plane.point;
            CP_Vector3D normal = plane.normal;
            CP_Vector3D d1 = normal.mf_getPerpendicularVector();
            CP_Vector3D *ps = new CP_Vector3D[4];
            ps[0] = point + d1 * LINE_LENGTH;
            ps[1] = point - d1 * LINE_LENGTH;

            CP_Vector3D d2 = d1 ^ normal;//get another direction on the plane

            ps[2] = point + d2 * LINE_LENGTH;
            ps[3] = point - d2 * LINE_LENGTH;

            //glPolygonMode(GL_FRONT_AND_BACK ,GL_FILL);
            glPolygonMode(GL_FRONT_AND_BACK ,GL_LINE);

            glBegin(GL_TRIANGLES);
            //0,1,2
            for (unsigned int i = 0; i < 3; i++)
            {
                glVertex3d(ps[i].x, ps[i].y, ps[i].z);
            }
            //0,1,3
            for (unsigned int i = 0; i < 4; i++)
            {
                if(i ==2) continue;
                glVertex3d(ps[i].x, ps[i].y, ps[i].z);
            }
            glEnd();
        }
        glColor4fv(color);
    }


    static void drawPoints(vector<CP_Vector2D> & points)
    {
        glColor3f(1.0, 0.0, 0.0);
        glPointSize(3.0);
        glBegin(GL_POINTS);
        for (unsigned int i = 0; i < points.size(); i++)
        {
            CP_Vector2D p = points[i];
            glVertex2d(p.x, p.y);    
        }
        glEnd();
    }

    static void drawNormals(vector<CP_Vector3D> & points, float scala)
    {
        glColor3f(0.0, 0.0, 1.0);
        glBegin(GL_LINES);
        for (unsigned int i = 0; i < points.size(); i++)
        {
            CP_Vector3D p = points[i];
            glVertex3d(0, 0, 0);    
            glVertex3d(p.x * scala, p.y * scala, p.z * scala);    
        }
        glEnd();
    }
    
    static void drawNormalsWithArrow(vector<CP_Vector3D> & points, float scala)
    {
        glPushMatrix();
        glBegin(GL_LINES);
        for (unsigned int i = 0; i < points.size(); i++)
        {
            CP_Vector3D p = points[i];
            glVertex3d(0, 0, 0);    
            glVertex3d(p.x * scala, p.y * scala, p.z * scala);
            //Arrow(0,0,0, p.x * scala, p.y * scala, p.z * scala, 0.05);  
        }
        glEnd();
        glPopMatrix();
    }

    static void drawPoints(vector<CP_Vector3D> & points, RealValueType scala = 1.0)
    {
        glColor3f(1.0, 0.0, 0.0);
        glBegin(GL_POINTS);
        for (unsigned int i = 0; i < points.size(); i++)
        {
            CP_Vector3D p = points[i];
            glVertex3d(p.x * scala, p.y * scala, p.z * scala);    
        }
        glEnd();
    }

    static void drawFacets(vector<CP_Vector3D> & points)
    {
        vector<int> index;
        assert(points.size() % 3 == 0);
        index.resize(points.size());
        for(int i = 0; i < points.size(); i++)
            index[i] = i;
        drawFacets(points, index, 1.0);
    }

    static void drawFacets(vector<CP_Vector3D> & points, vector<int> &index, RealValueType scala = 1.0)
    {
        //glColor3f(1.0, 0.0, 0.0);
        glLineWidth(1.0);
        glBegin(GL_TRIANGLES);
        int size = index.size();
        for (unsigned int i = 0; i < size; i += 3)
        {
            CP_Vector3D p = points[index[i]];
            glVertex3d(p.x * scala, p.y * scala, p.z * scala);    
            p = points[index[i+1]];
            glVertex3d(p.x * scala, p.y * scala, p.z * scala);    
            p = points[index[i+2]];
            glVertex3d(p.x * scala, p.y * scala, p.z * scala);    
        }
        glEnd();
    }
   
    static void drawBoundingBox(const BoundingBox &box, RealValueType scala = 1.0)
    {
        CP_Vector3D min = box.Min;
        CP_Vector3D max = box.Max;
        min = min * scala;
        max = max * scala;

        glBegin(GL_QUADS);
        glVertex3d(min.x, min.y, min.z);
        glVertex3d(max.x, min.y, min.z);
        glVertex3d(max.x, min.y, max.z);
        glVertex3d(min.x, min.y, max.z);

        glVertex3d(max.x, min.y, min.z);
        glVertex3d(max.x, min.y, max.z);
        glVertex3d(max.x, max.y, max.z);
        glVertex3d(max.x, max.y, min.z);

        glVertex3d(max.x, max.y, min.z);
        glVertex3d(max.x, max.y, max.z);
        glVertex3d(min.x, max.y, max.z);
        glVertex3d(min.x, max.y, min.z);

        glVertex3d(min.x, max.y, max.z);
        glVertex3d(min.x, max.y, min.z);
        glVertex3d(min.x, min.y, min.z);
        glVertex3d(min.x, min.y, max.z);


        glVertex3d(min.x, max.y, max.z);
        glVertex3d(min.x, min.y, max.z);
        glVertex3d(max.x, min.y, max.z);
        glVertex3d(max.x, max.y, max.z);

        glVertex3d(min.x, max.y, min.z);
        glVertex3d(min.x, min.y, min.z);
        glVertex3d(max.x, min.y, min.z);
        glVertex3d(max.x, max.y, min.z);
        glEnd();
    }

    static void drawTrainges(vector<CP_Vector3D> & points, RealValueType scala = 1.0)
    {
        glPolygonMode(GL_FRONT_AND_BACK ,GL_LINE );
        GLfloat color[4];
        glGetFloatv(GL_CURRENT_COLOR, color);
        glColor3f(0, 0, 1);

        glBegin(GL_TRIANGLES);
        for (unsigned int i = 0; i < points.size(); i++)
        {
            CP_Vector3D p = points[i];
           glVertex3d(p.x * scala, p.y * scala, p.z * scala);
        }
        glEnd();
        glColor4fv(color);
    }

    static void drawLines(vector<Line2D> &lines)
    {
        //the direction is the normal direction
        const int LINE_LENGTH = 1000;
        glBegin(GL_LINES);
        for (unsigned int i = 0; i < lines.size(); i++)
        {
            CP_Vector2D start = lines[i].point + lines[i].direction * LINE_LENGTH;
            CP_Vector2D end = lines[i].point - lines[i].direction * LINE_LENGTH;
            glVertex2d(start.x, start.y);
            glVertex2d(end.x, end.y);
        }
        glEnd();
    }

    static void drawLine3D(Line3D &line)
    {
        const int LINE_LENGTH = 100;
        glBegin(GL_LINES);
        {
            CP_Vector3D start = line.point + line.direction * LINE_LENGTH;
            CP_Vector3D end = line.point - line.direction * LINE_LENGTH;
            glVertex3d(start.x, start.y, start.z);
            glVertex3d(end.x, end.y, end.z);
        }
        glEnd();
    }
    static void drawLine3Ds(vector<Line3D> &lines, int start, int end)
    {
        const int LINE_LENGTH = 5;
        glBegin(GL_LINES);
        {
            for (int i = start; i < end; i++)
            {
                Line3D line = lines[i];
                CP_Vector3D start = line.point + line.direction * LINE_LENGTH;
                CP_Vector3D end = line.point - line.direction * LINE_LENGTH;
                glVertex3d(start.x, start.y, start.z);
                glVertex3d(end.x, end.y, end.z);
            }

        }
        glEnd();
        return;
        for (int i = start; i < end; i++)
        {
            Line3D line = lines[i];
            CP_Vector3D start = line.point + line.direction * LINE_LENGTH;
            CP_Vector3D end = line.point - line.direction * LINE_LENGTH;
            stringstream ss;
            ss << i;
            string str = ss.str();
            drawString(line.point.x, line.point.y, const_cast<char*>(str.c_str()));
        }


    }

    
    static void testPlanePlaneIntersection()
    {
        vector<Plane3D> planes;
        planes.push_back(Plane3D(CP_Vector3D(0,0,1), CP_Vector3D(0,0,2)));
        planes.push_back(Plane3D(CP_Vector3D(1,0,0), CP_Vector3D(0,0,0)));
        glPolygonMode(GL_FRONT_AND_BACK ,GL_FILL);
        drawPlanes(planes, 0, 1);
        Line3D ret;
        Plane3DPlane3DIntersection::IntersectIgnoringDomain(ret, planes[0], planes[1]);

        glColor3f(0.0, 0.0, 1.0);
        glLineWidth(4.0);
        drawLine3D(ret);
        glLineWidth(1.0);
    };


    static void test3PlanesIntersect()
    {
        Plane3D p1(CP_Vector3D(1,0,0), CP_Vector3D(1,0,0));
        Plane3D p2(CP_Vector3D(0,1,0), CP_Vector3D(0,1,0));
        Plane3D p3(CP_Vector3D(0,0,1), CP_Vector3D(0,0,1));
        CP_Vector3D vec(-1,-1,-1);
        Plane3DPlane3DIntersection::IntersectThreePlanes(p1, p2, p3, vec);
        cout<< vec.x << "," << vec.y <<"," << vec.z << endl;
        Plane3DPlane3DIntersection::IntersectThreePlanes(p1, p2, p3, vec);
        cout<< vec.x << "," << vec.y <<"," << vec.z << endl;
    }

    #define printOpenGLError() printOglError(__FILE__, __LINE__)
    static int printOglError(char *file, int line)
    {
        //
        // Returns 1 if an OpenGL error occurred, 0 otherwise.
        //
        GLenum glErr;
        int    retCode = 0;

        glErr = glGetError();
        while (glErr != GL_NO_ERROR)
        {
            printf("glError in file %s @ line %d: %s\n", file, line, gluErrorString(glErr));
            retCode = 1;
            glErr = glGetError();
        }
        return retCode;
    }


    static void drawXYZ()
    {
        const GLfloat amount = 5.0f;

        //You must not use glGetError inside a glBegin/glEnd block. The API is strict about that. 
        //But the error only happens after glEnd. If you read the API, you'll find that the error might propagate: it only happens after the glEnd.
        glLineWidth(1.0f);
        glBegin(GL_LINES);
        // x axis
        glColor4f(0.8f, 0.0f, 0.0f, 0.7f);
        glVertex3d(amount, 0.0, 0.0);
        glColor4f(0.8f, 0.0f, 0.0f, 0.2f);
        glVertex3d(-amount, 0.0, 0.0);
        // y axis
        glColor4f(0.0f, 0.8f, 0.0f, 0.7f);
        glVertex3d(0.0, amount, 0.0);
        glColor4f(0.0f, 0.8f, 0.0f, 0.2f);
        glVertex3d(0.0, -amount, 0.0);

        // z axis
        glColor4f(0.0f, 0.0f, 0.8f, 0.7f);
        glVertex3d(0.0, 0.0, amount);
        glColor4f(0.0f, 0.0f, 0.8f, 0.2f);
        glVertex3d(0.0, 0.0, -amount);
        glEnd();

        printOglError(0,0);

        glPointSize(7.0f);
        glBegin(GL_POINTS);
        glColor4f(0.8f, 0.0f, 0.0f, 0.7f);
        glVertex3d(amount, 0.0, 0.0);

        glColor4f(0.0f, 0.8f, 0.0f, 0.7f);
        glVertex3d(0.0, amount, 0.0);

        glColor4f(0.0f, 0.0f, 0.8f, 0.7f);
        glVertex3d(0.0, 0.0, amount);
        glEnd();
        printOglError(0,0);
        glPointSize(1.0f);
        printOglError(0,0);
    }

    static void drawString(int x, int y, char* str)
    {
        glColor3d(0.0, 0.0, 0.0);
        int n = strlen(str);  
        glRasterPos2i(x,y);  
        for (int i = 0; i < n; i++)  
            glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *(str+i)); 
    }


    static void  drawPrimitives(PrimitivePtr * data, int size)
    {
        //GLfloat color[4];
        //glGetFloatv(GL_CURRENT_COLOR, color);
        //glColor3f(0, 0, 1);
        glBegin(GL_TRIANGLES);
        for(int i = 0; i < size; i++)
        {
            PrimitivePtr pri = data[i];
            CP_Vector3D p = pri->v0;
            glVertex3d(p.x , p.y , p.z);    
            p = pri->v1;
            glVertex3d(p.x , p.y , p.z);    
            p = pri->v2;
            glVertex3d(p.x , p.y , p.z);    
        }
        //glColor4fv(color);
        glEnd();
    }
};
