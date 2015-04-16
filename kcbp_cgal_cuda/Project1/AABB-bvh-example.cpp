#include "UIHelper.hpp"
#include "FileManager.h"
#include "KCBP.hpp"
#include "KDop3D.hpp"
#include "CGALTool.hpp"
#include "Camera.h"
#include "gl2ps.h"
#include "ACH3D.hpp"
#include "SSEProjection.hpp"

#include "BoundingBox.hpp"
#include "AABB.hpp"
#include <queue>

#define _DEBUG false

bool draw_convexhull = false;
bool draw_ach = false;
int polygon_mode = GL_LINE;
bool noxyz = true;
bool NO_DISPLAY = false;


vector<int> trianges_index;
vector<CP_Vector3D> points3d;
vector<CP_Vector3D> normals;
vector<Polygon3D*> polyhedra;
vector<CP_Vector3D> convexhull;
vector<CP_Vector3D> ach;

BoundingBox SingleModelBoundingBox;

vector<vector<BoundingBox>> bvh;
int bvh_layer = 0;
vector<AABBNode*> DisplayAABBNodes;
int display_tree_node_index = 0;


void draw(int argc, char** argv);
void initlight();
void display();
void grab(GLint width, GLint height);
void gl2ps();

int scale_convexhull =  1;

CCamera m_camera;

string objfilename;

bool OUTPUT_POLYHEDRON = false;
int k = 0;

void outputpoly()
{
    //if(!NO_DISPLAY) 
    //    return;
    ofstream outf(string(objfilename + "-" + to_string(k) + ".poly.kmeans.wighted").c_str());
    streambuf *default_buf=cout.rdbuf(); 
    cout.rdbuf( outf.rdbuf() ); 
    vector<CP_Vector3D> unique_ones;

    for (unsigned int i = 0; i < polyhedra.size(); i++)
    {
        for (unsigned int j = 0; j < polyhedra[i]->data.size(); j++)
        {
            CP_Vector3D p = polyhedra[i]->data[j];
            if(!KDop3D::isInVec(p, unique_ones))
            {
                unique_ones.push_back(p);
            }
        }
    }
    for (unsigned int i = 0; i < unique_ones.size(); i++)
    {
        //cout << unique_ones[i].toString() << endl;
        cout << unique_ones[i].x << " " << unique_ones[i].y << " "  << unique_ones[i].z   << endl;
    }
    cout.rdbuf(default_buf);
}

void outputConvexhull()
{
    ofstream outf(string(objfilename + ".convexhull").c_str());
    streambuf *default_buf=cout.rdbuf(); 
    cout.rdbuf( outf.rdbuf() ); 
    vector<CP_Vector3D> unique_ones;
 
    for (unsigned int j = 0; j < convexhull.size(); j++)
    {
        CP_Vector3D p = convexhull[j];
        if(!KDop3D::isInVec(p, unique_ones))
        {
            unique_ones.push_back(p);
        }
    }
 
    for (unsigned int i = 0; i < unique_ones.size(); i++)
    {
        cout << unique_ones[i].x << " " << unique_ones[i].y << " "  << unique_ones[i].z   << endl;
    }

    cout.rdbuf(default_buf);
}
bool weighted = true;
bool deletenocenter = false;

void scalebunny(vector<CP_Vector3D> &input, double scale)
{
    for(int i = 0; i < input.size(); i++)
    {
        input[i] = input[i] * scale;
    }
}

//kCBP_CGAL_CUDA.exe 40 models/alice.obj nw w n 1
//                   k    objfile        w(显示，其他都不),  w(权重，其他非权重) d(删除初始聚类中心，其他都保留) 1(benchtest number)
int main(int argc, char** argv)
{
    vector<Plane3D> planes;
    //testvolume(argc, argv);
    //return 0;
    objfilename = "models/bunny2.obj";  //10
    //objfilename = "F:/gems8/models/LocalModelLibrary/Wavefront/Collection/Cruiser Ship/Cruiser 2012.obj";
    if(argc >= 3)
        objfilename = argv[2];
    if(argc >= 4)
        NO_DISPLAY = argv[3][0] == 'w' ? false : true;  
    bool loaded = FileManager::readObjPointsAndFacetsFast(points3d, trianges_index, objfilename, SingleModelBoundingBox.Min, SingleModelBoundingBox.Max);
    scalebunny(points3d, 100.0); //Attention
    if(! loaded) {cout << "load failed:" << objfilename << endl;return 0;}
    cout << objfilename << " \tpoints number: " << points3d.size() <<endl;

    k =  26;
    if(argc >= 2) 
        k = atoi(argv[1]);
    cout << "K = " << k << endl;
    if(argc >= 6)
    {
        weighted = argv[4][0] == 'w' ? true : false;
        deletenocenter = argv[5][0] == 'd' ? true : false;
    }
    
    int benchtest = 1;
    if(argc >= 7)
    {
        benchtest = atoi(argv[6]);
        benchtest = benchtest < 1 ? 1 :benchtest;
    }

    printf("weighted=%d, deletenocenter=%d\n", weighted, deletenocenter);
    if(benchtest >= 1)
    {
        clock_t s, e;
        s = clock();
        for (int i = 0; i < benchtest; i++)
        {
            ach =  ACH3D::getACH(points3d, 10, 10);
        }
        e = clock();
        printf("ach banchtest %d times : %.4f", benchtest, (e - s) * 1.0f/ benchtest);
    }
    else
    {
        ach =  ACH3D::getACH(points3d, 10, 10);
    }
    normals = KCBP::getClusterNormals(ach, k, weighted, deletenocenter);
    printf("normalsize = %d\n", normals.size());

    SSEProjection::projectCPUSSE(points3d, normals, planes, benchtest);
   //KDop3D::projectCPU(points3d, normals, planes, benchtest);
    //KCBP::evaluate(points3d, normals, KCBP::CUDA_MUL_NORMAL, planes, benchtest);

    KDop3D::getResultByDualMapping(planes, polyhedra);
    printf("Polytopes: %d\n", polyhedra.size());
    //double volume = KCBP::getVolume(polyhedra);
    //printf("Volume: %.12f\n", volume);


    if (draw_convexhull)
    {
        // the convex hull use int to coordinate
        CGALConvexHull::getConvexHullFacets(points3d, convexhull, benchtest);
        cout << "Facet number in convex hull: " << convexhull.size() /3  <<  endl;
    }
   
    if(NO_DISPLAY) return 0;
    
    vector<CP_Vector3D> kcbpMeshPoints;
    KDop3D::getResultByDualMappingMesh(planes, kcbpMeshPoints);//mapping get points/ then call convexhull get kcbp mesh
    //#ifdef ModelAABBTree
        int a_triangle_size = trianges_index.size() / 3;
        PrimitivePtr* modela = new PrimitivePtr[a_triangle_size];
        for(int i = 0; i < trianges_index.size(); i += 3)
            modela[i/3] = new Primitive(points3d[trianges_index[i]], points3d[trianges_index[i+1]], points3d[trianges_index[i+2]]);
        AABBTree* tree_a = new AABBTree(modela, a_triangle_size, 10);
    /*#else
    int a_triangle_size = kcbpMeshPoints.size() / 3;
    PrimitivePtr* modela = new PrimitivePtr[a_triangle_size];
    for(int i = 0; i < kcbpMeshPoints.size(); i += 3)
        modela[i/3] = new Primitive(kcbpMeshPoints[i], kcbpMeshPoints[i+1], kcbpMeshPoints[i+2]);
    AABBTree* tree_a = new AABBTree(modela, a_triangle_size, 10);
    #endif*/
    
    
    //level traverse
    queue<AABBNode*> q;
    q.push(tree_a->Root);
    while(!q.empty())
    {
        q.push(NULL);
        vector<BoundingBox> layer;
        while(q.front() != NULL)
        {
            AABBNode * top = q.front();
            q.pop();
            DisplayAABBNodes.push_back(top);
            if(top->Left != NULL) 
                q.push(top->Left);
            if(top->Right != NULL)
                q.push(top->Right);
            layer.push_back(top->Box); 
        }
        q.pop();
        bvh.push_back(layer);
    }

    draw(argc, argv);
    return 0;
}




 

float view_angle = 30.0f;

void keyboard(unsigned char key, int x, int y)
{
    switch(key)
    {
    case 'r':
        m_camera.Reset();
        printf("reset camera\t");
        glutPostRedisplay();
        break;
    case 's':
        //grab(600,600);does not work
        gl2ps();
        printf("snapshot");
        break;
    case 'q':
        printf("exit\n\n");
        exit(0);
        break;
    case 'a':
#ifdef ModelAABB
        bvh_layer++;
        bvh_layer %= bvh.size();
        #else
        display_tree_node_index++;
        display_tree_node_index %= DisplayAABBNodes.size();
#endif // ModelAABB
        glutPostRedisplay();
        break;
    }
}

void myReshape(int w, int h)
{
    glMatrixMode(GL_PROJECTION); 
    glLoadIdentity();
    gluPerspective(view_angle, w/(GLdouble)h, 0.001, 200000.1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glViewport(0, 0, (GLsizei)w, (GLsizei)h); 
}

void  display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    printOpenGLError();

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    gluLookAt(m_camera.position.x, m_camera.position.y, m_camera.position.z,
        m_camera.lookat.x, m_camera.lookat.y, m_camera.lookat.z,
        m_camera.up.x, m_camera.up.y, m_camera.up.z);
    printOglError(0,0);
    
    if(true)
    {
 
        gl2psEnable(GL2PS_LINE_STIPPLE);

        if(!noxyz) 
            UIHelper::drawXYZ();
        
        glPolygonMode(GL_FRONT_AND_BACK, polygon_mode);
        glColor3f(1.0, 0.0, 0);

        #ifdef ModelAABBTree
        UIHelper::drawFacets(points3d, trianges_index);
        //UIHelper::drawPolygons3D(polyhedra, 0, polyhedra.size(), polygon_mode, 1.0, 3.0);
        
        glLineWidth(1.0);
        glColor3f(1.0, 0.0, 0.0);
        for (int i = 0; bvh_layer < bvh.size() && i < bvh[bvh_layer].size(); i++)
        {
            UIHelper::drawBoundingBox(bvh[bvh_layer][i]);
        }
        #else

        assert(display_tree_node_index < DisplayAABBNodes.size());
        UIHelper::drawPrimitives(DisplayAABBNodes[display_tree_node_index]->Data, DisplayAABBNodes[display_tree_node_index]->DataSize);
        UIHelper::drawBoundingBox(DisplayAABBNodes[display_tree_node_index]->Box);
        #endif
        

        gl2psDisable(GL2PS_LINE_STIPPLE);
         
        
    }else
    {
        glColor3f(1, 0, 1);
        glPolygonMode(GL_FRONT_AND_BACK ,GL_LINE);
        glLineWidth(1.0); 
        GLUquadricObj *qobj = gluNewQuadric();
        gluSphere(qobj,3,16,16);
        glLineWidth(2.0);        
        UIHelper::drawNormals(normals, 3);
        glPointSize(5.0f);
        UIHelper::drawPoints(normals, 3);
    }

    glPopMatrix();
    glFlush();
}

CP_Vector2D m_formerMousePos;
bool m_isLeftButtonDown = false;
bool m_isRightButtonDown =false;
void mouse(int button,int state,int x,int y)
{
    if(state==GLUT_DOWN)
    {
        m_formerMousePos = CP_Vector2D(x, y);
    }

    if(state==GLUT_DOWN && button==GLUT_LEFT_BUTTON)
    {
        m_isLeftButtonDown = true;
        return ;
    }

    if(state==GLUT_UP && button == GLUT_LEFT_BUTTON)
    {
        m_isLeftButtonDown = false;
        return ;
    }
    
    if(state==GLUT_DOWN && button == GLUT_RIGHT_BUTTON)
    {
        m_isRightButtonDown = true;
        return ;
    }

    if(state==GLUT_UP && button == GLUT_RIGHT_BUTTON)
    {
       CP_Vector2D cur(x, y);
       m_isRightButtonDown = false;
       return ;
    } 
}

void onMouseMove(int x,int y)
{
    CP_Vector2D point = CP_Vector2D(x, y);
    CP_Vector2D dis = point - m_formerMousePos;
    if(m_isLeftButtonDown)
    {
        static const GLfloat scale = -3.0f;

        m_camera.RotateAroundCenterU(dis.x / scale);
        m_camera.RotateAroundCenterV(dis.y / scale);

        m_formerMousePos = point;
    }
    if (m_isRightButtonDown)
    {
        static const GLfloat amount = 1.0f;
        m_camera.MoveCloseFar(dis.y * amount);

        m_formerMousePos = point;
    }
    glutPostRedisplay();
}

void initlight()
{
    bool is_lighting=false;
    GLfloat light_position1[4] = {-52, -16, -50, 0};
    GLfloat light_position2[4] = {-26, -48, -50, 0};
    GLfloat light_position3[4] = { 16, -52, -50, 0};	
    GLfloat light_position4[4] = {52, 16, 50, 0};
    GLfloat light_position5[4] = {26, 48, 50, 0};
    GLfloat light_position6[4] = {-16, 52, 100, 0};

    GLfloat direction1[3] = {52,16,50};
    GLfloat direction2[3] = {26,48,50};
    GLfloat direction3[3] = {-16,52,50};
    GLfloat direction4[3] = {-52,-16,-50};
    GLfloat direction5[3] = {-26,-48,-50};
    GLfloat direction6[3] = {16,-52,-50};

    GLfloat color1[4] = {1,0,0,1};
    GLfloat color2[4] = {0,1,0,1};
    GLfloat color3[4] = {0,0,1,1};
    GLfloat color4[4] = {1,0,0,1};
    GLfloat color5[4] = {0,1,0,1};
    GLfloat color6[4] = {0,0,1,1};

    GLfloat ambient[4] = {0.3f,0.3f,0.3f,0.5f};
    GLfloat material_color[4] = {1,1,1,0.3f};
    GLfloat material_specular[4] = {0.5,0.5,0.5,0.5};
    GLfloat material_ambient[4] = {0.0,0.0,0.0,0.0};
    //background color
    glClearColor(1,1,1,0);
    
    if(is_lighting)
        glClearColor(0,0,0,0);//black

    glLightfv(GL_LIGHT0, GL_POSITION, light_position1);
    glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, direction1);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, color1);
    glLightfv(GL_LIGHT0, GL_SPECULAR, color1);

    glLightfv(GL_LIGHT1, GL_POSITION, light_position2);
    glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, direction2);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, color2);
    glLightfv(GL_LIGHT1, GL_SPECULAR, color2);

    glLightfv(GL_LIGHT2, GL_POSITION, light_position3);
    glLightfv(GL_LIGHT2, GL_SPOT_DIRECTION, direction3);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, color3);
    glLightfv(GL_LIGHT2, GL_SPECULAR, color3);

    glLightfv(GL_LIGHT3, GL_POSITION, light_position4);
    glLightfv(GL_LIGHT3, GL_SPOT_DIRECTION, direction4);
    glLightfv(GL_LIGHT3, GL_DIFFUSE, color4);
    glLightfv(GL_LIGHT3, GL_SPECULAR, color4);

    glLightfv(GL_LIGHT4, GL_POSITION, light_position5);
    glLightfv(GL_LIGHT4, GL_SPOT_DIRECTION, direction5);
    glLightfv(GL_LIGHT4, GL_DIFFUSE, color5);
    glLightfv(GL_LIGHT4, GL_SPECULAR, color5);

    glLightfv(GL_LIGHT5, GL_POSITION, light_position6);
    glLightfv(GL_LIGHT5, GL_SPOT_DIRECTION, direction6);
    glLightfv(GL_LIGHT5, GL_DIFFUSE, color6);
    glLightfv(GL_LIGHT5, GL_SPECULAR, color6);

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_color);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material_ambient);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128);

    if(is_lighting)
    {
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHT1);
        glEnable(GL_LIGHT2); 

        glEnable(GL_LIGHT3);
        glEnable(GL_LIGHT4);
        glEnable(GL_LIGHT5);
    }
    glEnable(GL_DEPTH_TEST);  
    glDepthFunc(GL_LESS);

    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    glLineWidth(0.5);
    glPointSize(2.0);
}

void draw(int argc, char** argv) 
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);
    glutCreateWindow("K-CBP");
    initlight(); 
    glutReshapeFunc(myReshape);	
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(onMouseMove);
    glutReshapeWindow(600, 600);//manually change the window size.
    glutPositionWindow(300,300);
    
    glutMainLoop();
}
void gl2ps()
{
    FILE *fp;
    int state = GL2PS_OVERFLOW, buffsize = 0;
    string epsfilename = objfilename + "-aabb-bvh" + to_string(bvh_layer) +".pdf";
    fp = fopen(epsfilename.c_str(), "wb");
    printf("Writing  %s ...\n", epsfilename.c_str());
    while(state == GL2PS_OVERFLOW){
        buffsize += 1024*1024;
        gl2psBeginPage("test", "gl2psTestSimple", NULL,   /*GL2PS_EPS*/GL2PS_PDF , GL2PS_SIMPLE_SORT,
            GL2PS_DRAW_BACKGROUND | GL2PS_USE_CURRENT_VIEWPORT | GL2PS_TIGHT_BOUNDING_BOX,
            GL_RGBA, 0, NULL, 0, 0, 0, buffsize, fp, epsfilename.c_str());
        display();
        state = gl2psEndPage();
    }
    fclose(fp);
    printf("Done!\n");
}
