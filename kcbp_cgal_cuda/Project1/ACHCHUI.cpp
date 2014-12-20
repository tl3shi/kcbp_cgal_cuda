#include "UIHelper.hpp"
#include "FileManager.h"
#include "KCBP.hpp"
#include "KDop3D.hpp"
#include "CGALTool.hpp"
#include "Camera.h"
#include "gl2ps.h"
#include "ACH3D.hpp"
#include "SSEProjection.hpp"

#define _DEBUG false

bool draw_convexhull = true;
bool draw_ach = true;
int polygon_mode = GL_LINE;
bool noxyz = true;
bool NO_DISPLAY = false;

vector<CP_Vector3D> points3d;
vector<CP_Vector3D> normals;
vector<Polygon3D*> polyhedra;
vector<CP_Vector3D> convexhull;
vector<CP_Vector3D> ach;
void draw(int argc, char** argv);
void initlight();
void display();
void grab(GLint width, GLint height);
void gl2ps();

int scale_convexhull =  1;
double draw_scale = .0;
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
//draw normal and sphere
int main11(int argc, char** argv)
{
    k =  35;
    if(argc >= 2) 
        k = atoi(argv[1]);
    bool weighted = true;
    normals = KCBP::hammersleyNormals(k);//KCBP::genNormalsEqualArea(k);//KCBP::getClusterNormals(ach, k, weighted);
    draw(argc, argv);
    return 0;
}

//cagd中文学报  复审 增加实验
void testvolume2(int argc, char** argv)
{
    objfilename = "models/bunny2.obj"; //10
    //objfilename = "F:/gems8/models/LocalModelLibrary/Wavefront/Collection/Cruiser Ship/Cruiser 2012.obj";
    if(argc >= 3)
        objfilename = argv[2];
    if(argc >= 4)
        NO_DISPLAY = argv[3][0] == 'w' ? false : true;  
    bool loaded = FileManager::readObjPointsFast(points3d, objfilename, draw_scale); 
    if(! loaded) {cout << "load failed:" << objfilename << endl;return;}
    cout << objfilename << " \tpoints number: " << points3d.size() <<endl;

    k =  26;
    if(argc >= 2) 
        k = atoi(argv[1]);
    
    vector<Plane3D> planes;

    /*
    printf("\n\n CollDet Normals \n\n");
    for (k = 6; k <= 46; k += 2) //from 6 to 46
    {
        normals = KCBP::getCollDetNormals(k);
        printf("normalsize = %d\n", normals.size());

        SSEProjection::projectCPUSSE(points3d, normals, planes, 1);
        KDop3D::getResultByDualMapping(planes, polyhedra);
        printf("polyhedraSize = %d\n", polyhedra.size());
        double volume = KCBP::getVolume(polyhedra);
        printf("Volume: %.12f\n\n\n", volume);
    }
    */
    printf("\n\n Hammersley Normals \n\n");
    for (int k = 46; k <= 206; k += 10)  
    {
        normals = KCBP::hammersleyNormals(k);
        printf("normalsize = %d\n", normals.size());

        SSEProjection::projectCPUSSE(points3d, normals, planes, 1);
        KDop3D::getResultByDualMapping(planes, polyhedra);
        printf("polyhedraSize = %d\n", polyhedra.size());
        double volume = KCBP::getVolume(polyhedra);
        printf("Volume: %.12f\n\n\n", volume);
    }
    
    printf("\n\n Equal Area Normals \n\n");
    for (int k = 46; k <= 206; k+=10)  
    {
        normals = KCBP::genNormalsEqualArea(k);
        printf("normalsize = %d\n", normals.size());

        SSEProjection::projectCPUSSE(points3d, normals, planes, 1);
        KDop3D::getResultByDualMapping(planes, polyhedra);
        printf("polyhedraSize = %d\n", polyhedra.size());
        double volume = KCBP::getVolume(polyhedra);
        printf("Volume: %.12f\n\n\n", volume);
    }
    /*
    printf("\n\n Kdop NORMALS \n\n");
    vector<int> ks;
    ks.push_back(14);ks.push_back(18);ks.push_back(26);
    for (int i = 0; i < ks.size(); i++) //from 6 to 46
    {
        int k = ks[i];
        normals = KCBP::genKdopNormals(k);
        printf("%d,", normals.size());

        SSEProjection::projectCPUSSE(points3d, normals, planes, 1);
        KDop3D::getResultByDualMapping(planes, polyhedra);
        printf("%d,", polyhedra.size());
        double volume = KCBP::getVolume(polyhedra);
        printf("%.12f,\n", volume);
    }
    */
    CGALConvexHull::getConvexHullFacets(points3d, convexhull);
    double volume = CGALConvexHull::getVolume(convexhull);
    cout << "Facet number in convex hull: " << convexhull.size() /3  <<  endl;
    printf("Convexhull volume: %.12f\n\n\n", volume);

    if(NO_DISPLAY) return;
    draw(argc, argv);

}

void testvolume(int argc, char** argv)
{
    objfilename = "models/bunny2.obj"; //10
    //objfilename = "F:/gems8/models/LocalModelLibrary/Wavefront/Collection/Cruiser Ship/Cruiser 2012.obj";
    if(argc >= 3)
        objfilename = argv[2];
    if(argc >= 4)
        NO_DISPLAY = argv[3][0] == 'w' ? false : true;  
    bool loaded = FileManager::readObjPointsFast(points3d, objfilename, draw_scale); 
    if(! loaded) {cout << "load failed:" << objfilename << endl;return;}
    cout << objfilename << " \tpoints number: " << points3d.size() <<endl;

    k =  26;
    if(argc >= 2) 
        k = atoi(argv[1]);

    vector<Plane3D> planes;

    ach =  ACH3D::getACH(points3d, 10, 10);
    printf("\n\nweighted nodelete\n\n");
    for (k = 6; k <= 46; k += 2) //from 6 to 46
    {
        bool weighted = true;
        bool deletenocenter = false;
        normals = KCBP::getClusterNormals(ach, k, weighted, deletenocenter);
        printf("normalsize = %d\n", normals.size());

        SSEProjection::projectCPUSSE(points3d, normals, planes, 1);
        KDop3D::getResultByDualMapping(planes, polyhedra);
        printf("polyhedraSize = %d\n", polyhedra.size());
        double volume = KCBP::getVolume(polyhedra);
        printf("Volume: %.12f\n\n\n", volume);
    }
   
    printf("\n\n noweighted nodelete\n\n");
    for (k = 6; k <= 46; k += 2) //from 6 to 46
    {
        bool weighted = false;
        bool deletenocenter = false;
        normals = KCBP::getClusterNormals(ach, k, weighted, deletenocenter);
        printf("normalsize = %d\n", normals.size());

        SSEProjection::projectCPUSSE(points3d, normals, planes, 1);
        KDop3D::getResultByDualMapping(planes, polyhedra);
        printf("polyhedraSize = %d\n", polyhedra.size());
        double volume = KCBP::getVolume(polyhedra);
        printf("Volume: %.12f\n\n\n", volume);
    }

    //delete center
    vector<int> ks;
    ks.push_back(6);ks.push_back(26);ks.push_back(36);ks.push_back(46);ks.push_back(56);
    ks.push_back(66);ks.push_back(76);ks.push_back(86);ks.push_back(96);ks.push_back(106);
    ks.push_back(136);ks.push_back(156);ks.push_back(186);ks.push_back(206);
    
    printf("\n\n noweighted delete\n\n");
    for (int i = 0; i < (int)ks.size(); i++)
    {
        k = ks.at(i);
        bool weighted = false;
        bool deletenocenter = true;
        normals = KCBP::getClusterNormals(ach, k, weighted, deletenocenter);
        printf("normalsize = %d\n", normals.size());

        SSEProjection::projectCPUSSE(points3d, normals, planes, 1);
        KDop3D::getResultByDualMapping(planes, polyhedra);
        printf("polyhedraSize = %d\n", polyhedra.size());
        double volume = KCBP::getVolume(polyhedra);
        printf("Volume: %.12f\n\n\n", volume);
    }
     
    printf("\n\n weighted delete\n\n");
    for (int i = 0; i < (int)ks.size(); i++)
    {
        k = ks.at(i);
        bool weighted = true;
        bool deletenocenter = true;
        normals = KCBP::getClusterNormals(ach, k, weighted, deletenocenter);
        printf("normalsize = %d\n", normals.size());

        SSEProjection::projectCPUSSE(points3d, normals, planes, 1);
        KDop3D::getResultByDualMapping(planes, polyhedra);
        printf("polyhedraSize = %d\n", polyhedra.size());
        double volume = KCBP::getVolume(polyhedra);
        printf("Volume: %.12f\n\n\n", volume);
    }

    printf("\n\n CollDet Normals \n\n");
    for (k = 6; k <= 46; k += 2) //from 6 to 46
    {
        normals = KCBP::getCollDetNormals(k);
        printf("normalsize = %d\n", normals.size());

        SSEProjection::projectCPUSSE(points3d, normals, planes, 1);
        KDop3D::getResultByDualMapping(planes, polyhedra);
        printf("polyhedraSize = %d\n", polyhedra.size());
        double volume = KCBP::getVolume(polyhedra);
        printf("Volume: %.12f\n\n\n", volume);
    }

    CGALConvexHull::getConvexHullFacets(points3d, convexhull);
    double volume = CGALConvexHull::getVolume(convexhull);
    cout << "Facet number in convex hull: " << convexhull.size() /3  <<  endl;
    cout << "Convexhull volume: " << volume << endl;

    if(NO_DISPLAY) return;
    draw(argc, argv);
}

bool weighted = true;
bool deletenocenter = false;

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
    bool loaded = FileManager::readObjPointsFast(points3d, objfilename, draw_scale); 
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

    //SSEProjection::projectCPUSSE(points3d, normals, planes, benchtest);
   //KDop3D::projectCPU(points3d, normals, planes, benchtest);
    KCBP::evaluate(points3d, normals, KCBP::CUDA_MUL_NORMAL, planes, benchtest);

    KDop3D::getResultByDualMapping(planes, polyhedra);
    printf("Polytopes: %d\n", polyhedra.size());
    double volume = KCBP::getVolume(polyhedra);
    printf("Volume: %.12f\n", volume);

    if (draw_convexhull)
    {
        // the convex hull use int to coordinate
        CGALConvexHull::getConvexHullFacets(points3d, convexhull, benchtest);
        cout << "Facet number in convex hull: " << convexhull.size() /3  <<  endl;
    }
   
    if(NO_DISPLAY) return 0;

    draw(argc, argv);
    return 0;
}





int main1(int argc, char** argv)
{
    time_t start_time, end_time;
    if(true)
    {
        objfilename = "models/bunny.obj"; //10
        //objfilename = "F:/gems8/models/LocalModelLibrary/Wavefront/Collection/Cruiser Ship/Cruiser 2012.obj";
        if(argc >= 3)
            objfilename = argv[2];
        if(argc >= 4)
            NO_DISPLAY = argv[3][0] == 'w' ? false : true;  

        start_time = clock(); 
        bool loaded = FileManager::readObjPointsFast(points3d, objfilename, draw_scale); 
        assert(loaded == true);
        end_time = clock();
        
        cout << "Load Obj ("<< objfilename << ") time:" << double(end_time - start_time) << " \tpoints number: " << points3d.size() <<endl;
    }else
    {
        KDop3D::initData(points3d);
        points3d.push_back(CP_Vector3D(3,3,4));
    } 

    bool useach = true;
    if(draw_ach || useach)
    {
        cout << "ach  ";
        ach =  ACH3D::getACH(points3d, 10, 10);// in this method, invoke the convexhull method
    }
    
    if (draw_convexhull)
    {
        // the convex hull use int to coordinate
        CGALConvexHull::getConvexHullFacets(points3d, convexhull);
        cout << "Facet number in convex hull: " << convexhull.size() /3  <<  endl;
    }
    k =  26;
    if(argc >= 2) 
        k = atoi(argv[1]);
   
    printf("K = %d\n", k);
    
    assert(ach.size() > 0);
    bool weighted = false;
    normals = KCBP::getClusterNormals(ach, k, weighted);
    printf("cluster K = %d\n", normals.size());
    //normals =  KCBP::hammersleyNormals(k); // d
    
    //normals =  KCBP::genNormalsEqualArea(k);
    // normals =  KCBP::getNormalsByK(k);//edge cut corner cut.
    //SIX = 6, FOURTEEN=14, EIGHTEEN=18, TWENTYSIX=26
    //normals =  KCBP::genKdopNormals(k);//kdop normals. 

    int gpu_method = KCBP::CUDA_MUL_NORMAL ; 
    /*if((gpu_method >= KCBP::PROJECTION_SIZE && gpu_method < KCBP::CUDA_MUL_NORMAL))
    {
        gpu_method = KCBP::CUDA_MUL_NORMAL;
        printf("gpu method number is invalid, default cuda mul normal used\n");
    }*/

    vector<Plane3D> planes;
    int benchtest = 10;
    KCBP::evaluate(points3d, normals, gpu_method, planes, benchtest);
    
    KCBP::getPolyhedra(planes, polyhedra);

    start_time = clock();
    double volume = KCBP::getVolume(polyhedra);
    end_time = clock();
    printf("Volume: %.12f, calc time: %.3f\n", volume , end_time-start_time);
    
    if(OUTPUT_POLYHEDRON)
    {
        outputpoly();
        //outputConvexhull();
    }

    if(NO_DISPLAY) return 0;
    
    if(_DEBUG)
    {
        bool check_result = KCBP::checkResult(points3d, polyhedra);
        cout << "check result:" << check_result << endl;
        assert(check_result == true);
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
        assert(draw_scale != .0);
        gl2psEnable(GL2PS_LINE_STIPPLE);

        UIHelper::drawPolygons3D(polyhedra, 0, polyhedra.size(), polygon_mode, draw_scale);

        if(!noxyz) 
            UIHelper::drawXYZ();

        UIHelper::drawPoints(points3d, draw_scale);
        gl2psDisable(GL2PS_LINE_STIPPLE);
        if(draw_convexhull)
        {
            glTranslatef(25, 0, 0); //(0,70,0) dinosaur

            // the convex hull use int to coordinate
            UIHelper::drawPoints(points3d, draw_scale);
            UIHelper::drawTrainges(convexhull, draw_scale);
        }

        if(draw_ach)
        {
            glTranslatef(-50, 0, 0);
            // the convex hull use int to coordinate
            UIHelper::drawPoints(points3d, draw_scale);
            UIHelper::drawTrainges(ach, draw_scale);
        }
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
    string epsfilename = objfilename + "-" + to_string(k) +".eps";
    {
        epsfilename = objfilename + "-" + to_string(k) + "-w-"+ to_string(weighted) + "-d-"+ to_string(deletenocenter) +  ".eps";
    }

    fp = fopen(epsfilename.c_str(), "wb");
    printf("Writing  %s ...\n", epsfilename.c_str());
    while(state == GL2PS_OVERFLOW){
        buffsize += 1024*1024;
        gl2psBeginPage("test", "gl2psTestSimple", NULL,   GL2PS_EPS , GL2PS_SIMPLE_SORT,
            GL2PS_DRAW_BACKGROUND | GL2PS_USE_CURRENT_VIEWPORT | GL2PS_TIGHT_BOUNDING_BOX,
            GL_RGBA, 0, NULL, 0, 0, 0, buffsize, fp, epsfilename.c_str());
        display();
        state = gl2psEndPage();
    }
    fclose(fp);
    printf("Done!\n");
}

 