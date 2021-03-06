#pragma  once
#pragma warning(disable:4018)
#pragma warning(disable:4224)

//#define RealValueType float


#include "UIHelper.hpp"
#include "FileManager.h"
#include "KCBP.hpp"
#include "KDop3D.hpp"
#include "CGALTool.hpp"

#include "Camera.h"
#include "gl2ps.h"
#include "ACH3D.hpp"
#include "SSEProjection.hpp"
#include "Matrix.h"
#include "BoundingBox.hpp"
#include "BoundingSphere.hpp"
#include "AABB.hpp"
#include "CollsionQuery.hpp"
#include "SolidCollisionQuery.hpp"
#include "LibCCDQueryStatic.hpp"
#include "LibCCDQuery2.hpp"

#include <sstream>
#include <fstream>
#include <unordered_set>

//#define _DEBUG false
//#define USE_SOLID
//#define SAME_BUNNY
#define DRAW_MODEL

//#define  ROTATE_ENABLE

bool draw_facets = true;
bool draw_convexhull = false;
bool draw_ach = false;
int polygon_mode = GL_LINE;
bool noxyz = false;
bool NO_DISPLAY = false;
bool auto_rotate = false;
//#define TEMPDEBUG

vector<CP_Vector3D> points3d;
vector<int> trianges_index;
vector<CP_Vector3D> normals;
vector<Polygon3D*> polyhedra;
vector<Polygon3D*> kdopPolyhedra;
vector<CP_Vector3D> ach;
void draw(int argc, char** argv);
void initlight();
void display();
void grab(GLint width, GLint height);
void gl2ps();
void genModels(int modelnum, string configfile);
void initOpenGLList();

void writetofile(bool obj);

vector<vector<CP_Vector3D> > MeshPointsData;
//vector<vector<int> > MeshIndexes; the index is always the same
vector<vector<CP_Vector3D>> MeshpolyhedraData;
vector<vector<CP_Vector3D>> NonMeshPolyhedraData; //not mesh
vector<int> MeshPolyhedronIndex;

vector<kDOPNode> kDops;
vector<vector<CP_Vector3D>> kDopsData; //not mesh

vector<CP_Vector3D> MeshPolyhedronPoints;

vector<vector<CP_Vector3D>> MeshConvexHullData; 
vector<vector<CP_Vector3D>> NonMeshConvexHullData; 
vector<CP_Vector3D> MeshConvexHullPoints;
vector<CP_Vector3D> NonMeshConvexHullPoints;
vector<int> MeshConvexHullIndex;

vector<BoundingBox> ModelBoundingBoxes;
vector<BoundingShpere> ModelBoundingSpheres;

BoundingBox SingleModelBoundingBox;

vector<pair<int, int>> collision_pair;

vector<int> MeshPointsDataList; //display listid in OpenGL
vector<int> MeshPolyhedronDataList; // for KCBP
vector<int> MeshBoundingBoxList;


RealValueType Scale_Bunny = 1.0;//100.0;
int scale_convexhull =  1;
const RealValueType tranlate_unit = 0.01 * Scale_Bunny;
CCamera m_camera;
CP_Vector2D m_formerMousePos;
bool m_isLeftButtonDown = false;
bool m_isRightButtonDown = false;
bool necessary = true;
bool print_pairs = false;

vector<bool> collision_index; //others v.s first one

//CP_Vector3D translatePos(.0, 1.120, .0);//real translate = translatePos * translate_unit, for the origin/first model
//CP_Vector3D translatePos(.0, -1.370, .0);// intersection  solid as non-intersection
//CP_Vector3D translatePos(.82, 1.45, .0);// non-intersection solid as intersecetion
CP_Vector3D translatePos(.82*Scale_Bunny, 0.84*Scale_Bunny, .0*Scale_Bunny);// intersection solid as non-intersecetion
//CP_Vector3D translatePos(0, 0.76, .0);
CP_Vector3D rotateAxis(1.0, .0, .0);
int rotateDeg = 0;//-40;//0; //mouse + UP/DOWN --> changes this value

CP_Vector3D lastTranslatePos(0,0,0);
int lastRotateDeg = 0;

const int movingModelIndex = 0; //just the first model is transformed, always be ZERO, if Modified ,the codes should be modified, too.

string objfilename;

bool lib_ccd = true;
int k = 0;

#define BOX_FILTER

#define KCBP_FILTER

bool finish_without_update = false; //used to cal fps
bool usekcbp = true;

vector<ICollisionQuery *> kdop_queries; 
vector<ICollisionQuery *> kcbp_queries; 
vector<ICollisionQuery *> convexhull_queries; 

vector<ICollisionQuery *> collision_queries;

vector<int> moving_rotate_angles;
vector<CP_Vector3D> moving_translations;
vector<CP_Vector3D> moving_rotations;
vector<CMatrix> transforms; //the corresponding matrix

 
CP_Vector3D current_moving_axis(1, 0, 0);
CP_Vector3D current_moving_translation(0, 0, 0);
int current_moving_angle = 0;

int transformN = 0;
int moving_step = 0;

enum BVTYPE{KDOP_Approximate, KDOP_GJK, KCBP_GJK, Convexhull_GJK};

BVTYPE current_bv = BVTYPE::KDOP_Approximate;

void move_and_check();


int modelnum = 1;

void scalebunny(vector<CP_Vector3D> &input, const CP_Vector3D &low, const CP_Vector3D &high)
{
    //scale same as CollDet\visual_studio\TestSolution\Bench
    CP_Vector3D c = 0.5 * low + 0.5 * high;
    CP_Vector3D d = 0.5 * high - 0.5 * low; 
    RealValueType s = max(d.x, max(d.y, d.z));
    for (int i = 0; i < input.size(); i ++ )
        input[i] = (input[i] - c) / s;
}

bool mesh_detective = true;




float g_fps( void (*func)(void), int n_frame )
{
    clock_t start, finish;
    int i;
    float fps;

    printf( "Performing benchmark, please wait" );
    start = clock();
    for( i=0; i<n_frame; i++ )
        func();
    printf( "done\n" );
    finish = clock();
    fps = float(n_frame)/(finish-start)*CLOCKS_PER_SEC;
    return fps;
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
    case 'k':
    case 'K':
        usekcbp = !usekcbp;
        printf("use kcbp: %d", usekcbp);
        break;
    case 'F':
    case 'f':
        finish_without_update = true;
        printf( "%f fps\n", g_fps( display, 100));
        finish_without_update = false;
        break;
    case 't':
    case 'T':
        necessary = !necessary;
        if(necessary) printf( "necessary\n");
        else  printf( "non necessary\n");
    case 'A':
        auto_rotate = !auto_rotate;
        if(auto_rotate) printf( "auto_rotate\n");
        else  printf( "non auto_rotate\n");
        break;
    case 'n':
        move_and_check();
        break;
    }
}

void onMotionControl(int key, int x, int y)
{
    lastRotateDeg = rotateDeg;
    lastTranslatePos = translatePos;

    switch(key)
    {
    case GLUT_KEY_LEFT:
        translatePos.x -= tranlate_unit;
        break;
    case GLUT_KEY_RIGHT:
        translatePos.x += tranlate_unit;
        break;
    case GLUT_KEY_UP:
        if(m_isRightButtonDown | m_isLeftButtonDown)
        {
#ifdef ROTATE_ENABLE
            rotateDeg += 10;
            rotateDeg %= 360;
#endif

        }else
        {
            translatePos.y += tranlate_unit;
        }
        break;
    case GLUT_KEY_DOWN:
        if(m_isRightButtonDown | m_isLeftButtonDown)
        {
#ifdef ROTATE_ENABLE
            rotateDeg -= 10;
            rotateDeg %= 360;
#endif
        }else
        {
            translatePos.y -= tranlate_unit;
        }
        break;
    }
    glutPostRedisplay();
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

 
class VectorTransform
{
public:
    mat4 mat;
    VectorTransform(mat4 &m):mat(m){}
    CP_Vector3D operator()(CP_Vector3D &point)
    {
        return CP_Vector3D(mat * point);
    }
};

AABBTree * constructAABBTree(vector<CP_Vector3D> &points0, vector<int> &index0)
{
    int a_triangle_size = index0.size() / 3;
    PrimitivePtr* modela = new PrimitivePtr[a_triangle_size];
    for(int i = 0; i < index0.size(); i += 3)
        modela[i/3] = new Primitive(points0[index0[i]], points0[index0[i+1]], points0[index0[i+2]]);
    return new AABBTree(modela, a_triangle_size, -1);
}


//if config exist, read the config file
//else generate n configurations
void genRandomData(string config, vector<int> &rotate_angles, vector<CP_Vector3D> &translations, vector<CP_Vector3D> &rotations, int n)
{
    bool isConfigreadin = config.length() > 0;
    streambuf *defaultstream = cout.rdbuf();
    if(! isConfigreadin)
    {
        srand(time(0));
        time_t t = time(0); 
        char tmp[64]; 
        strftime( tmp, sizeof(tmp), "%m-%d-%H-%M", localtime(&t) ); 
        ofstream f(string(tmp) + "-" + to_string(n) + "-rand.config");
        cout << "rand config write to :" << string(tmp) + "-" + to_string(n) + "-rand.config" << endl;
        cout.rdbuf(f.rdbuf());

        RealValueType xx = SingleModelBoundingBox.Max.x - SingleModelBoundingBox.Min.x;
        RealValueType range = n*xx;
        for(int i = 0; i < n; i++)
        {
            float trans_x = (rand() % (int) 5*range) / (range);
            float trans_y = (rand() % (int) 5*range) / (range);
            float trans_z = (rand() % (int) 5*range) / (range);

            if(rand() & 0x1)
                trans_x = -trans_x;
            if(rand() & 0x1)
                trans_y = -trans_y;
            if(rand() & 0x1)
                trans_z = -trans_z;

            int rotate_angle = rand() % 180;
            float rot_x = (rand() % 100) / 100.0;
            float rot_y = (rand() % 100) / 100.0;
            float rot_z = (rand() % 100) / 100.0;

            if(rand() & 0x1)
                rot_x = -rot_x;
            if(rand() & 0x1)
                rot_y = -rot_y;
            if(rand() & 0x1)
                rot_z = -rot_z;

            rotate_angles.push_back(rotate_angle);
            translations.push_back(CP_Vector3D(trans_x, trans_y, trans_z));
            CP_Vector3D rot(rot_x, rot_y, rot_z);
            rot.mf_normalize();
            rotations.push_back(rot);

            cout << rotate_angle << " " << rot_x << " " << rot_y << " " << rot_z << endl;
            cout << trans_x << " " << trans_y << " " << trans_z << endl;
        }
        cout.rdbuf(defaultstream);
    }else
    {
        ifstream fin(config.data());
        string line;
        int i = 0;
        int angle;
        float x,y,z;
        while(std::getline(fin, line))
        {
            stringstream ss;
            ss << line;
            if(i & 0x1) // tranls
            {
                ss >> x >> y >> z;
                translations.push_back(CP_Vector3D(x,y,z) * Scale_Bunny);
            }else //angle rot
            {
                ss >> angle >> x >> y >> z;
                CP_Vector3D xis(x, y, z);
                xis.mf_normalize();
                rotations.push_back(xis);
                rotate_angles.push_back(angle);
            }
            i++;
        }
        fin.close();
        assert(rotate_angles.size() != 0);
    }
}

void genModels(int modelnum, string config)
{
    assert(modelnum >= 2);

    MeshPolyhedronIndex.resize(MeshPolyhedronPoints.size());
    for(int i = 0; i < MeshPolyhedronPoints.size(); i++)
        MeshPolyhedronIndex[i] = i;
    //the original one
    MeshPointsData.push_back(points3d);
    MeshpolyhedraData.push_back(MeshPolyhedronPoints);
    MeshConvexHullData.push_back(MeshConvexHullPoints);
    NonMeshConvexHullData.push_back(NonMeshConvexHullPoints);

    //nonMeshPolyhedra
    vector<CP_Vector3D> nonMeshPolyhedra;
    for(auto it = polyhedra.begin(); it != polyhedra.end(); it++)
        std::copy((*it)->data.begin(), (*it)->data.end(), std::back_inserter(nonMeshPolyhedra));
    NonMeshPolyhedraData.push_back(nonMeshPolyhedra);

    vector<int> rotate_angles;
    vector<CP_Vector3D> translations;
    vector<CP_Vector3D> rotations;
    genRandomData(config, rotate_angles, translations, rotations, modelnum);

    for(int i = 0; i < rotate_angles.size(); i++)
    {
        int rotate_angle = rotate_angles[i];
        CP_Vector3D rot = rotations[i];
        CP_Vector3D tran = translations[i];

        mat4 rotmat;
        mat4 transmat;
        rotate_angle = -rotate_angle; //Act as Coll/TestSolution/Bench
        mat4::GetRotate(rotmat, rotate_angle, rot);
        mat4::GetTranslate(transmat, tran);

        mat4 mat = transmat * rotmat;
        //mat^T = the matrix in as Coll/TestSolution/Bench system
        //#ifdef SAME_BUNNY
        //mat = CMatrix::Identity;
        //#endif // SAME_BUNNY

        vector<CP_Vector3D> newPoints(points3d.size());
        VectorTransform trans(mat);
        std::transform(points3d.begin(), points3d.end(), newPoints.begin(), trans);
        MeshPointsData.push_back(newPoints);

        vector<CP_Vector3D> kcbpdata(MeshPolyhedronPoints.size());//kcbp mesh for UI
        std::transform(MeshPolyhedronPoints.begin(), MeshPolyhedronPoints.end(), kcbpdata.begin(), trans);
        MeshpolyhedraData.push_back(kcbpdata);

        vector<CP_Vector3D> kcbpNonMesh(nonMeshPolyhedra.size());//kcbp non mesh
        std::transform(nonMeshPolyhedra.begin(), nonMeshPolyhedra.end(), kcbpNonMesh.begin(), trans);
        NonMeshPolyhedraData.push_back(kcbpNonMesh);

        vector<CP_Vector3D> ch(MeshConvexHullPoints.size());//convex hull mesh
        std::transform(MeshConvexHullPoints.begin(), MeshConvexHullPoints.end(), ch.begin(), trans);
        MeshConvexHullData.push_back(ch);

        vector<CP_Vector3D> chnon(NonMeshConvexHullPoints.size());//convex hull non mesh
        std::transform(NonMeshConvexHullPoints.begin(), NonMeshConvexHullPoints.end(), chnon.begin(), trans);
        NonMeshConvexHullData.push_back(chnon);

        if(i == modelnum-2)
        {
            printf("reading %d model configs(plus fixed one,total = %d)\n", modelnum-1, modelnum);
            break;
        }
    }
    ModelBoundingBoxes.resize(MeshpolyhedraData.size());
    clock_t c_time = clock();
    for(int i = 0; i < ModelBoundingBoxes.size(); i++)
    {
        BoundingBox box = BoundingBox::GetNull();
        vector<CP_Vector3D> &model_points = MeshPointsData[i];
        for(int j = 0; j < model_points.size();j++)
        {
            CP_Vector3D &p = model_points[j];
            if(box.Min.x > p.x)
                box.Min.x = p.x;
            if(box.Min.y > p.y)
                box.Min.y = p.y;
            if(box.Min.z > p.z)
                box.Min.z = p.z;
            if(box.Max.x < p.x)
                box.Max.x = p.x;
            if(box.Max.y < p.y)
                box.Max.y = p.y;
            if(box.Max.z < p.z)
                box.Max.z = p.z;
        }
        ModelBoundingBoxes[i] = box;
    }
    c_time = clock() - c_time;
    printf("build boundingbox time : %.2f\n", c_time*1.0);

    collision_index.resize(modelnum);
}

void initOpenGLList()
{
    int displayListId = 1;
    //initOpenGL display List for MeshPoint
    for(int i = 0; i < MeshPointsData.size(); i++)
    {
        MeshPointsDataList.push_back(displayListId);
        glNewList(displayListId, GL_COMPILE);
        UIHelper::drawFacets(MeshPointsData[i], trianges_index);
        glEndList();
        displayListId++;
    }
    //initOpenGL display List for KCBP
    for(int i = 0; i < MeshpolyhedraData.size(); i++)
    {
        MeshPolyhedronDataList.push_back(displayListId);
        glNewList(displayListId, GL_COMPILE);
        UIHelper::drawFacets(MeshpolyhedraData[i], MeshPolyhedronIndex);
        glEndList();
        displayListId++;
    }
    //initOpenGL display list for bounding box;
    for(int i = 0; i < ModelBoundingBoxes.size(); i++)
    {
        MeshBoundingBoxList.push_back(displayListId);
        glNewList(displayListId, GL_COMPILE);
        UIHelper::drawBoundingBox(ModelBoundingBoxes[i]);
        glEndList();
        displayListId++;
    }
}

void writetofile(bool obj)
{
    streambuf* coutBuf = cout.rdbuf();

    for(int i = 0; i < MeshpolyhedraData.size(); i++)
    {
        ofstream of;
        if(obj) 
            of = ofstream("test-k-" + to_string(k) + "-" + to_string(i) + ".obj");
        else
            of = ofstream("test-k-" + to_string(k) + "-" + to_string(i) + ".tri");//for swift

        streambuf* fileBuf = of.rdbuf();
        cout.rdbuf(fileBuf);
        vector<CP_Vector3D> &points = MeshpolyhedraData[i];
        if(!obj)
        {
            cout << "TRI" << endl;
            cout << points.size() << endl;
            cout << MeshPolyhedronIndex.size() / 3 << endl;
        }

        for(int i = 0; i < points.size(); i++)
        {
            if(obj)
                cout << "v ";
            cout<< points[i].x << " " <<  points[i].y << " " <<  points[i].z << endl;
        }
        cout << endl;

        for(int i = 0; i < MeshPolyhedronIndex.size()/3; i++)
        {
            if(obj)
                cout << "f " << MeshPolyhedronIndex[i*3+0] + 1 << " " << MeshPolyhedronIndex[i*3+1] + 1<< " " << MeshPolyhedronIndex[i*3+2] + 1<< endl;
            else
                cout << MeshPolyhedronIndex[i*3+0] << " " << MeshPolyhedronIndex[i*3+1] << " " << MeshPolyhedronIndex[i*3+2] << endl;
        }
        of.flush();
        of.close();
    }

    bool writeoriginmodels = false;
    if(writeoriginmodels)
    {
        for(int i = 0; i< MeshPointsData.size();i++)
        {
            ofstream of("test-k-model-" + to_string(i) + ".obj");
            streambuf* fileBuf = of.rdbuf();
            cout.rdbuf(fileBuf);
            vector<CP_Vector3D> &points = MeshPointsData[i];
            for(int i = 0; i < points.size(); i++)
            {
                cout << "v ";
                cout<< points[i].x << " " <<  points[i].y << " " <<  points[i].z << endl;
            }
            cout << endl;
            for(int i = 0; i < trianges_index.size()/3; i++)
                cout << "f " << trianges_index[i*3+0] + 1 << " " << trianges_index[i*3+1] + 1<< " " << trianges_index[i*3+2] + 1<< endl;
            of.flush();
            of.close();
        }
    }
    cout.rdbuf(coutBuf);
}

bool collision_necessary()
{
    if(lastRotateDeg == rotateDeg && lastTranslatePos == translatePos)
        return false;
    return true;
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
        glColor3f(0.1, 0.8, 0);

        GLfloat collsion_color[3] = {1.0, 0, 0};
        GLfloat oldColor[4];
        glGetFloatv(GL_CURRENT_COLOR, oldColor);

        if(auto_rotate)
            move_and_check();

        for(int i = 1; i < MeshPointsDataList.size(); i++)
        {
            if(collision_index[i])
                glColor3fv(collsion_color);
            else
                glColor4fv(oldColor);

#ifdef DRAW_MODEL
            //if(collision_index[i])
            glCallList(MeshPointsDataList[i]);
#endif
        }

        if(true)
        {
            glColor3f(0, 0, 1);
            for(int i = 1; i < MeshPolyhedronDataList.size(); i++)
                glCallList(MeshPolyhedronDataList[i]);

            glColor3f(0, 0, 0);
            for(int i = 1; i < MeshBoundingBoxList.size(); i++)
                glCallList(MeshBoundingBoxList[i]);

            for(int i = 1; i < ModelBoundingSpheres.size(); i++)
                UIHelper::drawBoundingSphere(ModelBoundingSpheres[i]);
            glColor4fv(oldColor);
        } 
        
        //draw the first / origin model of 
        if(current_moving_angle != 0)
            glRotatef(current_moving_angle, current_moving_axis.x, current_moving_axis.y, current_moving_axis.z);
        glTranslatef(current_moving_translation.x, current_moving_translation.y, current_moving_translation.z);

        glColor3fv(collsion_color);
#ifdef DRAW_MODEL
        glCallList(MeshPointsDataList[movingModelIndex]);
#endif

        gl2psDisable(GL2PS_LINE_STIPPLE);

    } 

    glPopMatrix();
    glFlush();

    if( finish_without_update )
        glFinish();
    else
        glutSwapBuffers();
}


void mouse(int button,int state,int x,int y)
{
    lastRotateDeg = rotateDeg;
    lastTranslatePos = translatePos;
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
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutCreateWindow("K-CBP");
    initlight(); 
    glutReshapeFunc(myReshape);	
    glutDisplayFunc(display);
    //glutIdleFunc(display);// Removing the idle function to save CPU and GPU
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(onMouseMove);
    glutSpecialFunc(onMotionControl);
    glutReshapeWindow(600, 600);//manually change the window size.
    glutPositionWindow(300,300);
    initOpenGLList();

    glutMainLoop();
}

void gl2ps()
{
    FILE *fp;
    int state = GL2PS_OVERFLOW, buffsize = 0;
    string epsfilename = objfilename + "-" + to_string(k) +".eps";
    {
        epsfilename = objfilename + "-" + to_string(k) + "-n-"+ to_string(modelnum) +  ".eps";
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


int uniqueCount(vector<std::pair<int,int> > &pairs)
{
    unordered_set<int> s;
    for(auto it = pairs.begin(); it != pairs.end(); it++)
        s.insert(it->first), s.insert(it->second);
    return s.size();
}

//only BV  then Model(AABB)
void staticEvaluate(float single_kcbp_time, float single_convexhull_time, float single_kdop_time, float single_bbox, float single_sphere)
{
    vector<std::pair<int, int>> bvPairs;
    vector<std::pair<int, int>> modelPairs;

    clock_t time_start = clock();
    vector<AABBTree*> aabbtrees(modelnum, NULL);
    for(int i = 0; i < modelnum; i++)
        aabbtrees[i] = constructAABBTree(MeshPointsData[i], trianges_index);
    float time_during = clock() - time_start;
    printf("build AABB time: %.2f\n", time_during);
    float single_AABB_time = time_during / modelnum;

    #pragma region BoundingBox
    {
        bvPairs.clear();
        modelPairs.clear();
        //boundingbox 
        time_start = clock();
        BoundingBox::CollsionDetection(ModelBoundingBoxes, bvPairs);
        float bv_detection_time = clock() - time_start;
        printf("collision detection box time: %.2f\n", bv_detection_time * 1.0);

        time_start = clock();
        for(auto it = bvPairs.begin(); it != bvPairs.end(); it++)
        {
            bool t = AABBTree::TraverseDetective(aabbtrees[it->first]->Root, aabbtrees[it->second]->Root);
            if(t)
                modelPairs.push_back(*it);
        }
        float cd_time = clock() - time_start;
        int bvCheckCount = uniqueCount(bvPairs);
        float bvInitTime =  single_bbox * modelnum +  single_AABB_time * bvCheckCount;
        printf("Box check collision pair: %d\n", bvPairs.size());
        printf("Model collision pair by Box: %d\n", modelPairs.size());
        printf("Box init time(construct Box(%d count) + aabbtree(%d count): %.2f\n", modelnum, bvCheckCount, bvInitTime);
        printf("Box directly cd time: %.2f\n", cd_time);
        float bvHitRate = modelPairs.size() * 1.0 / bvPairs.size();
        printf("Box hit rate : %.4f\n", bvHitRate);
    }
    #pragma endregion
 
    #pragma region BoundingSphere
    {
        bvPairs.clear();
        modelPairs.clear();
        time_start = clock();
        BoundingShpere::CollsionDetection(ModelBoundingSpheres, bvPairs);
        clock_t cd_sphere_time = clock() - time_start;
        printf("collision detection boundingsphere time: %.2f\n", cd_sphere_time * 1.0);
        time_start = clock();
        for(auto it = bvPairs.begin(); it != bvPairs.end(); it++)
        {
            bool t = AABBTree::TraverseDetective(aabbtrees[it->first]->Root, aabbtrees[it->second]->Root);
            if(t)
                modelPairs.push_back(*it);
        }
        float cd_time = clock() - time_start;
        int bvCheckCount = uniqueCount(bvPairs);
        float sphereInitTime =  single_sphere * modelnum +  single_AABB_time * bvCheckCount;
        printf("Sphere check collision pair: %d\n", bvPairs.size());
        printf("Model collision pair by Sphere: %d\n", modelPairs.size());
        printf("Sphere init time(construct Sphere(%d count) + aabbtree(%d count): %.2f\n", modelnum, bvCheckCount, sphereInitTime);
        printf("Sphere directly cd time: %.2f\n", cd_time);
        float bvHitRate = modelPairs.size() * 1.0 / bvPairs.size();
        printf("Sphere hit rate : %.4f\n", bvHitRate);
    }
    #pragma endregion

    #pragma region kDop
    {
        bvPairs.clear();
        modelPairs.clear();
        time_start = clock();
        kDOPNode::CollsionDetection(kDops, bvPairs);
        clock_t cd_kdop_time = clock() - time_start;
        printf("collision detection kDop time: %.2f\n", cd_kdop_time * 1.0);
        
        time_start = clock();
        for(auto it = bvPairs.begin(); it != bvPairs.end(); it++)
        {
            bool t = AABBTree::TraverseDetective(aabbtrees[it->first]->Root, aabbtrees[it->second]->Root);
            if(t)
                modelPairs.push_back(*it);
        }
        float cd_time = clock() - time_start;
        int bvCheckCount = uniqueCount(bvPairs);
        float kdopInitTime = single_kdop_time * modelnum + single_AABB_time * bvCheckCount;
        printf("kDop check collision pair: %d\n", bvPairs.size());
        printf("Model collision pair by kDOP: %d\n", modelPairs.size());
        printf("kDOP init time(construct kdop(%d count) + aabbtree(%d count): %.2f\n", modelnum, bvCheckCount, kdopInitTime);
        printf("kDOP cd time: %.2f\n", cd_time);
        float bvHitRate = modelPairs.size() * 1.0 / bvPairs.size();
        printf("kDOP hit rate : %.4f\n", bvHitRate);
    }
    #pragma endregion

    vector<std::pair<int, int> > allPairs;
    for(int i = 0; i < modelnum-1; i++)
        for(int j = i+1; j < modelnum; j++)
            allPairs.push_back(std::make_pair(i, j));
    #pragma region kCBP
    {
        bvPairs.clear();
        modelPairs.clear();
        time_start = clock();
        vector<LibCCDQueryStatic*> gjks;
        for(auto it = allPairs.begin(); it != allPairs.end(); it++)
        {
            LibCCDQueryStatic * gjkStatic = new LibCCDQueryStatic(NonMeshPolyhedraData[it->first], NonMeshPolyhedraData[it->second]); 
            gjks.push_back(gjkStatic);
        }
        printf("init kCBP GJK time: %.2f \n", (clock()-time_start)*1.0);

        time_start = clock();
        for(int i = 0; i < gjks.size(); i++)
        {
            if(gjks[i]->detection())
                bvPairs.push_back(*(allPairs.begin()+i));
        }
        float cd_kcbp_time = clock() - time_start;
        printf("collision detection kDop time: %.2f\n", cd_kcbp_time * 1.0);

        for(auto it = bvPairs.begin(); it != bvPairs.end(); it++)
        {
            bool t = AABBTree::TraverseDetective(aabbtrees[it->first]->Root, aabbtrees[it->second]->Root);
            if(t)
                modelPairs.push_back(*it);
        }
        float cd_time = clock() - time_start;
        int bvCheckCount = uniqueCount(bvPairs);
        float kCBPInitTime = single_kcbp_time * modelnum + single_AABB_time * bvCheckCount;
        
        printf("kCBP check collision pair: %d\n", bvPairs.size());
        printf("Model collision pair by kCBP: %d\n", modelPairs.size());
        printf("kCBP init time(construct kcbp(%d count) + aabbtree(%d count): %.2f\n", modelnum, bvCheckCount, kCBPInitTime);
        printf("kCBP cd time: %.2f\n", cd_time);
        float bvHitRate = modelPairs.size() * 1.0 / bvPairs.size();
        printf("kCBP hit rate : %.4f\n", bvHitRate);
        for(int i = 0; i < gjks.size(); i++)
            delete gjks[i];
    }
    #pragma endregion

   
    #pragma region ConvexHull
    {
        bvPairs.clear();
        modelPairs.clear();
        time_start = clock();
        vector<LibCCDQueryStatic*> gjks;
        for(auto it = allPairs.begin(); it != allPairs.end(); it++)
        {
            LibCCDQueryStatic * gjkStatic = new LibCCDQueryStatic(NonMeshConvexHullData[it->first], NonMeshConvexHullData[it->second]); 
            gjks.push_back(gjkStatic);
        }
        printf("init convexhull GJK time: %.2f \n", (clock()-time_start)*1.0);

        time_start = clock();
        for(int i = 0; i < gjks.size(); i++)
        {
            if(gjks[i]->detection())
                bvPairs.push_back(*(allPairs.begin()+i));
        }
        float cd_convexhull_time = clock() - time_start;
        printf("collision detection kDop time: %.2f\n", cd_convexhull_time * 1.0);

        for(auto it = bvPairs.begin(); it != bvPairs.end(); it++)
        {
            bool t = AABBTree::TraverseDetective(aabbtrees[it->first]->Root, aabbtrees[it->second]->Root);
            if(t)
                modelPairs.push_back(*it);
        }
        float cd_time = clock() - time_start;
        int bvCheckCount = uniqueCount(bvPairs);
        float convexhullInitTime = single_convexhull_time * modelnum + single_AABB_time * bvCheckCount;
        printf("Convexhull check collision pair: %d\n", bvPairs.size());
        printf("Model collision pair by Convexhull: %d\n", modelPairs.size());
        printf("Convexhull init time(construct convexhull(%d count) + aabbtree(%d count): %.2f\n", modelnum, bvCheckCount, convexhullInitTime);
        printf("Convexhull cd time: %.2f\n", cd_time);
        float bvHitRate = modelPairs.size() * 1.0 / bvPairs.size();
        printf("Convexhull hit rate : %.4f\n", bvHitRate);
        for(int i = 0; i < gjks.size(); i++)
            delete gjks[i];
    }
    #pragma endregion

    vector<std::pair<int, int>> boxPairs;
    clock_t c_time = clock();
    BoundingBox::CollsionDetection(ModelBoundingBoxes, boxPairs);
    c_time = clock() - c_time;
    printf("collision detection box time: %.2f\n", c_time * 1.0);
    printf("box collision size: %d\n", boxPairs.size());
    int bvCheckCount = uniqueCount(boxPairs);
    
    #pragma region kDopAfterBoxFitler
    {
        bvPairs.clear();
        modelPairs.clear();
        time_start = clock();
        for(auto it = boxPairs.begin(); it != boxPairs.end(); it++)
        {
            bool t = kDOPNode::checkIntersection(kDops[it->first], kDops[it->second]);
            if(t)
                bvPairs.push_back(*it);
        }
        float bvCheckTime = clock() - time_start;
        for(auto it = bvPairs.begin(); it != bvPairs.end(); it++)
        {
            bool t = AABBTree::TraverseDetective(aabbtrees[it->first]->Root, aabbtrees[it->second]->Root);
            if(t)
                modelPairs.push_back(*it);
        }
        float cd_time = clock() - time_start;
        int modelCheckCount = uniqueCount(bvPairs);
        float kdopInitTime = single_kdop_time * bvCheckCount + single_AABB_time * modelCheckCount;
        printf("kDop check collision pair(AfterBox): %d\n", bvPairs.size());
        printf("Between kDop check time(AfterBox): %.2f \n", bvCheckTime);
        printf("Model collision pair by kDOP(AfterBox): %d\n", modelPairs.size());
        printf("kDOP init time(AfterBox)(construct kdop(%d count) + aabbtree(%d count): %.2f\n", bvCheckCount, modelCheckCount, kdopInitTime);
        printf("kDOP cd time(AfterBox): %.2f\n", cd_time);
        float bvHitRate = modelPairs.size() * 1.0 / bvPairs.size();
        printf("kDOP hit rate(AfterBox) : %.4f\n", bvHitRate);
    }
    #pragma endregion

    #pragma region kCBPAfterBoxFitler
    {
        bvPairs.clear();
        modelPairs.clear();
        time_start = clock();
        vector<LibCCDQueryStatic*> gjks;
        for(auto it = boxPairs.begin(); it != boxPairs.end(); it++)
        {
            LibCCDQueryStatic * gjkStatic = new LibCCDQueryStatic(NonMeshPolyhedraData[it->first], NonMeshPolyhedraData[it->second]); 
            gjks.push_back(gjkStatic);
        }
        printf("init kCBP GJK time(AfterBox): %.2f \n", (clock()-time_start)*1.0);

        time_start = clock();
        for(int i = 0; i < gjks.size(); i++)
        {
            if(gjks[i]->detection())
                bvPairs.push_back(*(boxPairs.begin()+i));
        }
        float bvCheckTime = clock() - time_start;
        for(auto it = bvPairs.begin(); it != bvPairs.end(); it++)
        {
            bool t = AABBTree::TraverseDetective(aabbtrees[it->first]->Root, aabbtrees[it->second]->Root);
            if(t)
                modelPairs.push_back(*it);
        }
        float cd_time = clock() - time_start;
        int modelCheckCount = uniqueCount(bvPairs);
        float kCBPInitTime = single_kcbp_time * bvCheckCount + single_AABB_time * modelCheckCount;

        printf("kCBP check collision pair(AfterBox): %d\n", bvPairs.size());
        printf("Between kCBP check time(AfterBox): %.2f \n", bvCheckTime);
        printf("Model collision pair by kCBP(AfterBox): %d\n", modelPairs.size());
        printf("kCBP init time(AfterBox)(construct kcbp(%d count) + aabbtree(%d count): %.2f\n", bvCheckCount, modelCheckCount, kCBPInitTime);
        printf("kCBP cd time(AfterBox): %.2f\n", cd_time);
        float bvHitRate = modelPairs.size() * 1.0 / bvPairs.size();
        printf("kCBP hit rate(AfterBox): %.4f\n", bvHitRate);
        for(int i = 0; i < gjks.size(); i++)
            delete gjks[i];
    }
    #pragma endregion

    #pragma region ConvexHullAfterBoxFitler
    {
        bvPairs.clear();
        modelPairs.clear();
        time_start = clock();
        vector<LibCCDQueryStatic*> gjks;
        for(auto it = boxPairs.begin(); it != boxPairs.end(); it++)
        {
            LibCCDQueryStatic * gjkStatic = new LibCCDQueryStatic(NonMeshConvexHullData[it->first], NonMeshConvexHullData[it->second]); 
            gjks.push_back(gjkStatic);
        }
        printf("init convexhull GJK time(AfterBox): %.2f \n", (clock()-time_start)*1.0);

        time_start = clock();
        mat4 indenty = CMatrix::Identity;
        for(int i = 0; i < gjks.size(); i++)
        {
            if(gjks[i]->detection())
                bvPairs.push_back(*(boxPairs.begin()+i));
        }
        float bvCheckTime = clock() - time_start;
        for(auto it = bvPairs.begin(); it != bvPairs.end(); it++)
        {
            bool t = AABBTree::TraverseDetective(aabbtrees[it->first]->Root, aabbtrees[it->second]->Root);
            if(t)
                modelPairs.push_back(*it);
        }
        float cd_time = clock() - time_start;
        int modelCheckCount = uniqueCount(bvPairs);
        float convexhullInitTime = single_convexhull_time * bvCheckCount + single_AABB_time * modelCheckCount;

        printf("Convexhull check collision pair(AfterBox): %d\n", bvPairs.size());
        printf("Between Convexhull check time(AfterBox): %.2f \n", bvCheckTime);
        printf("Model collision pair by Convexhull(AfterBox): %d\n", modelPairs.size());
        printf("Convexhull init time(AfterBox)(construct convexhull(%d count) + aabbtree(%d count): %.2f\n", bvCheckCount, modelCheckCount, convexhullInitTime);
        printf("Convexhull cd time(AfterBox): %.2f\n", cd_time);
        float bvHitRate = modelPairs.size() * 1.0 / bvPairs.size();
        printf("Convexhull hit rate(AfterBox): %.4f\n", bvHitRate);
        for(int i = 0; i < gjks.size(); i++)
            delete gjks[i];
    }
    #pragma endregion

    for(auto it = modelPairs.begin(); it != modelPairs.end(); it++)
        collision_index[it->first] = collision_index[it->second] = true;
}

void BoxDetection(const CMatrix &transformMatrix)
{
#ifdef BOX_FILTER
    std::fill(collision_index.begin(), collision_index.end(), false);
    collision_index[0] = true;
#else
    std::fill(collision_index.begin(), collision_index.end(), true);
    return;
#endif
    //bounding box
    BoundingBox movingModelBBoxBak = ModelBoundingBoxes[movingModelIndex];
    vector<CP_Vector3D> vertices = movingModelBBoxBak.GetAABBVertices();
    RealValueType min_x, min_y, min_z;
    min_x = min_y = min_z = RealValueTypeMax;
    RealValueType max_x, max_y, max_z;
    max_x = max_y = max_z = -RealValueTypeMax;
    for(int i = 0; i < 8; i++)
    {
        CP_Vector3D v = transformMatrix * vertices[i];
        min_x = min(min_x, v.x);
        min_y = min(min_y, v.y);
        min_z = min(min_z, v.z);
        max_x = max(max_x, v.x);
        max_y = max(max_y, v.y);
        max_z = max(max_z, v.z);
    }
    BoundingBox transformedBBox(CP_Vector3D(min_x, min_y, min_z), CP_Vector3D(max_x, max_y, max_z));
    for(int i = 1; i < modelnum; i++)
    {
        if(transformedBBox.IntersectWith(ModelBoundingBoxes[i]))
            collision_index[i] = true;
    }
}

int numOfModelCollisions = 0;
int numOfKCBPCollisions = 0;

void move_and_check() 
{
    //static int move_step = 0;
    static RealValueType start_time = 0;

    static bool first_time = true;
    static const CMatrix ident = CMatrix::Identity;

    if(moving_step  == 0)
    {
        start_time = clock();
        numOfModelCollisions = 0;
        numOfKCBPCollisions = 0;
    }
    assert(moving_step < transformN);
    CMatrix world0 = transforms[moving_step];

    current_moving_angle = moving_rotate_angles[moving_step];
    current_moving_axis = moving_rotations[moving_step];
    current_moving_translation = moving_translations[moving_step];

    BoxDetection(world0);

    for(int i = 1; i < collision_index.size(); i++)
    {
        if(collision_index[i])
        {
            bool c;
            switch(current_bv)
            {
            case KDOP_Approximate:
                c = kDOPNode::checkIntersection(kDops[movingModelIndex], kDops[i], kDopsData[movingModelIndex], world0);
                break;
            case KDOP_GJK:
                c = kdop_queries[i-1]->detection(world0, ident);
                break;
            case KCBP_GJK:
                c = kcbp_queries[i-1]->detection(world0 , ident);
                break;
            case Convexhull_GJK:
                c = convexhull_queries[i-1]->detection(world0 , ident);
                break;
            }
            if(c == false)
                collision_index[i] = false;
            else
                numOfKCBPCollisions++;
        }
    }

    for(int i = 1; i < collision_index.size(); i++)
    {
        if(collision_index[i])
        {
            bool c = collision_queries[i-1]->detection(world0 , ident);
            if(c == false)
                collision_index[i] = false;
            else
                numOfModelCollisions++;
        }
    }
    //can ++ here, for display used the moving_step/display flush also invoke 
    moving_step++;
    if(moving_step == transformN)
    {
        start_time = clock() - start_time;
        if(first_time)
        {
            //printf("Moving steps : %d\n", moving_step);
            printf("Collision detection cost time: %.2f\n", start_time * 1.0);
            printf("Collisions of bv pairs: %d\n", numOfKCBPCollisions);
            printf("Collisions of model pairs: %d\n", numOfModelCollisions);
        }
        first_time = false;        
        if(NO_DISPLAY == false)//vis, drawing it again 
            moving_step = 0, numOfModelCollisions = 0, numOfKCBPCollisions = 0, 
            first_time = true;// print again
        else //no draw
            return;
    }
    if(NO_DISPLAY == false)
        glutPostRedisplay();
}

int parseMoving(string movingconfigFile)
{
    string filereverse(movingconfigFile);
    std::reverse(filereverse.begin(), filereverse.end());
    int start = filereverse.find("-", 0);
    int end = filereverse.find("-", start+1);
    string configStr = filereverse.substr(start+1, end-(start+1));//substr(offsetstartwith0, count)
    std::reverse(configStr.begin(), configStr.end());
    int num = 0;
    stringstream ss(configStr);
    ss >> num;
    return num;
}

void genMovingData(string movingConfig, int genN)
{
    genRandomData(movingConfig, moving_rotate_angles, moving_translations, moving_rotations, genN);
    for(int i = 0; i < moving_rotate_angles.size(); i++)
    {
        int rotate_angle = moving_rotate_angles[i];
        CP_Vector3D rot = moving_rotations[i];
        CP_Vector3D tran = moving_translations[i];
        mat4 transmat;
        mat4 rotmat = mat4::GetRotate(rotate_angle, rot);
        mat4::GetTranslate(transmat, tran);
        mat4 mat = rotmat * transmat; 
        transforms.push_back(mat);
    }
    transformN = transforms.size();
    printf("reading dynamic config transforms : %d\n", transformN);
}

//BVPreBenchDynamic.exe k models/bunny2.obj n(owindow)  10  rand.config p(rint_pairs)   moving.config(or genNumer)
int main(int argc, char** argv)
{
    vector<Plane3D> planes;

    objfilename = "models/bunny2.obj"; 
    //objfilename = "models/apple.obj";
    if(argc >= 3)
        objfilename = argv[2];
    if(argc >= 4)
        NO_DISPLAY = argv[3][0] == 'w' ? false : true;  

    bool loaded = FileManager::readObjPointsAndFacetsFast(points3d, trianges_index, objfilename, SingleModelBoundingBox.Min, SingleModelBoundingBox.Max);
    scalebunny(points3d, SingleModelBoundingBox.Min, SingleModelBoundingBox.Max); //Attention
    if(! loaded) {cout << "load failed:" << objfilename << endl;return 0;}
    cout << "objfile:" << objfilename << " \npoints number: " << points3d.size() <<endl;
    cout << "triangle number: " << trianges_index.size()/3 <<endl;

    string readconfig = "config/04-03-30-rand.config";//"12-19-10-31-50-rand.config";
    string movingConfig = "config/04-03-200-rand.config";

    k =  16;
    if(argc >= 2) 
        k = atoi(argv[1]);
    cout << "K = " << k << endl;
    modelnum = 5;
    if(argc >= 5)
    {
        modelnum = atoi(argv[4]);
        modelnum = modelnum < 1 ? 1 : modelnum;
    }
    cout << "ModelNum = " << modelnum << endl;

    int benchtest = modelnum;
    if(argc >= 6)
    {
        readconfig = argv[5];
        if(readconfig == "gen") //gen
            readconfig = "";
    }
    if(argc >= 7){
        string pp = argv[6];
        if( pp == "p" || pp == "print_pairs")
            print_pairs = true;
    }
    if(argc >= 8){
        transformN = atoi(argv[7]);
        if (transformN == 0)//fail, should be a file
            movingConfig = argv[7];
        else //generate new
            movingConfig = "";

    }
    if(transformN == 0) 
        cout << "Moving steps : " << parseMoving(movingConfig) << "\n";
    else
        printf("Moving steps : %d\n", transformN);

 
    clock_t kcbp_time, kdop_time, convexhull_time;

   
    clock_t s, e;

    #pragma region kCBP
    vector<vector<Polygon3D*>> tmpPolyhedra;
    s = clock();
    for (int i = 0; i < benchtest; i++)
    {
        ach =  ACH3D::getACH(points3d, 10, 10);
        normals = KCBP::getClusterNormals(ach, k, true, false);
        planes = SSEProjection::projectsse(points3d, normals);
        vector<Polygon3D*> tmp;
        KDop3D::getResultByDualMapping(planes, tmp);//mapping get points / then sort points ccw construct kcbp
        tmpPolyhedra.push_back(tmp);
    }
    e = clock();
    kcbp_time = e - s;
    printf("kCBP for models time (%d times) totally: %.2f\n", benchtest, (e - s) * 1.0f);

    polyhedra = tmpPolyhedra[0];
    for(int i = 1; i < tmpPolyhedra.size(); i++)
        for(int j = 0; j < tmpPolyhedra[i].size(); j++)
            delete tmpPolyhedra[i][j];

    KDop3D::getResultByDualMappingMesh(planes, MeshPolyhedronPoints);//mapping get points/ then call convexhull get kcbp mesh
    printf("Polytopes: %d\n", polyhedra.size());
    #pragma endregion

    #pragma region Convexhull
    //ConvexHull 
    CGALConvexHull::getConvexHullFacets(points3d, MeshConvexHullPoints);
    CGALConvexHull::getConvexHullVertices(points3d, NonMeshConvexHullPoints);
    s = clock();
    for(int i = 0; i < benchtest; i++)
    {
        vector<CP_Vector3D> tmp;
        CGALConvexHull::getConvexHullFacets(points3d, tmp);
    }
    e = clock();
    convexhull_time = e - s;
    printf("convexhull for models time (%d times) totally: %.2f\n", benchtest, (e - s) * 1.0f);
   
    #pragma endregion
    
    genModels(modelnum, readconfig);
    genMovingData(movingConfig, transformN);

    #pragma region kDop
    s = clock();
    vector<CP_Vector3D> kdopNormals = KCBP::getCollDetNormals(k);
    vector<vector<Polygon3D*>> tmpPolyhedraKdop;
    for(int i = 0; i < MeshPointsData.size(); i++)
    {
        kDOPNode node = kDOPNode::getKDop(MeshPointsData[i], kdopNormals);
        kDops.push_back(node);
        auto kdopPlanes = SSEProjection::projectsse(MeshPointsData[i], kdopNormals);
        vector<Polygon3D*> tmp;
        KDop3D::getResultByThreePlane(kdopPlanes, tmp);//mapping get points / then sort points ccw construct kcbp
        vector<CP_Vector3D > kdopVertices;
        for(auto it = tmp.begin(); it != tmp.end(); it++)
            std::copy((*it)->data.begin(), (*it)->data.end(), std::back_inserter(kdopVertices));
        kDopsData.push_back(kdopVertices);
        tmpPolyhedraKdop.push_back(tmp);
    }
    e = clock();
    kdop_time = e - s;
    printf("kDOP for models time (%d models) totally(threePlaneIntersection): %.2f\n", MeshPointsData.size(), (e - s) * 1.0f);
    for(int i = 0; i < tmpPolyhedraKdop.size(); i++)
        for(int j = 0; j < tmpPolyhedraKdop[i].size(); j++)
            delete tmpPolyhedraKdop[i][j];
    tmpPolyhedraKdop.clear();
    //等价计算Kdop 计算顶点的时间，正常应该先平移到原点，再对偶，然后平移回去, 这里模拟下计算时间[simulation]
    vector<vector<CP_Vector3D>> tmpKDopsData;
    vector<CP_Vector3D> tmpKDopPoint = points3d;
    s = clock();
    for(int i = 0; i < modelnum; i++)
    {
        scalebunny(tmpKDopPoint, SingleModelBoundingBox.Min, SingleModelBoundingBox.Max);//translate to origin [simulation]
        kDOPNode node = kDOPNode::getKDop(MeshPointsData[0], kdopNormals);
        kDops.push_back(node);
        auto kdopPlanes = SSEProjection::projectsse(MeshPointsData[0], kdopNormals);
        vector<Polygon3D*> tmp;
        KDop3D::getResultByDualMapping(kdopPlanes, tmp);//mapping get points / then sort points ccw construct kcbp
        vector<CP_Vector3D > kdopVertices;
        for(auto it = tmp.begin(); it != tmp.end(); it++)
            std::copy((*it)->data.begin(), (*it)->data.end(), std::back_inserter(kdopVertices));
        tmpKDopsData.push_back(kdopVertices);
        tmpPolyhedraKdop.push_back(tmp);
        scalebunny(tmpKDopPoint, SingleModelBoundingBox.Min, SingleModelBoundingBox.Max);//[revert]translate to origin[simulation]
    }
    e = clock();
    kdop_time = e - s;
    printf("kDOP for models time (%d models) totally(simulation): %.2f\n", MeshPointsData.size(), (e - s) * 1.0f);
    for(int i = 0; i < tmpPolyhedraKdop.size(); i++)
        for(int j = 0; j < tmpPolyhedraKdop[i].size(); j++)
            delete tmpPolyhedraKdop[i][j];
    #pragma endregion

    for(int i = 1; i  < modelnum; i++)
    {
        ICollisionQuery * kcbpquery = new LibCCDQuery2(NonMeshPolyhedraData[0], NonMeshPolyhedraData[i]); 
        kcbp_queries.push_back(kcbpquery);
    }
    for(int i = 1; i  < modelnum; i++)
    {
        ICollisionQuery * kdopquery = new LibCCDQuery2(kDopsData[0], kDopsData[i]); 
        kdop_queries.push_back(kdopquery);
    }
    for(int i = 1; i  < modelnum; i++)
    {
        ICollisionQuery * chquery = new LibCCDQuery2(NonMeshConvexHullData[0], NonMeshConvexHullData[i]); 
        convexhull_queries.push_back(chquery);
    }

    s = clock();
    for(int i = 1; i < modelnum; i++)
        collision_queries.push_back(new CollisionQuery(MeshPointsData[movingModelIndex], trianges_index, MeshPointsData[i], trianges_index));
    float time_build_aabb = clock() - s;
    printf("build AABB time: %.2f\n", time_build_aabb);

#ifdef TEMPDEBUG
    NO_DISPLAY = false;
#endif // TEMPDEBUG

    if(NO_DISPLAY) 
    {
        current_bv = BVTYPE::Convexhull_GJK;
        s = clock();
        for(int b = 0; b < 5; b++)
        {
            for(int i = 0; i< transformN; i++)
                move_and_check();
            moving_step = 0;
        }
        e = clock();
        printf("CD time(Convexhull_GJK): %.2f\n", (e - s)/5.0f);
        printf("Convexhull_GJK hitRate: %.4f\n", numOfModelCollisions * 1.0 /numOfKCBPCollisions);
        
        numOfModelCollisions = 0, numOfKCBPCollisions = 0;
        current_bv = BVTYPE::KCBP_GJK;
        s = clock();
        for(int b = 0; b < 5; b++)
        {
            for(int i = 0; i< transformN; i++)
                move_and_check();
            moving_step = 0;
        }
        e = clock();
        printf("CD time(KCBP_GJK): %.2f\n", (e - s)/5.0f);
        printf("KCBP_GJK hitRate: %.4f\n", numOfModelCollisions * 1.0 /numOfKCBPCollisions);

        numOfModelCollisions = 0, numOfKCBPCollisions = 0;
        current_bv = BVTYPE::KDOP_Approximate;
        s = clock();
        for(int b = 0; b < 5; b++)
        {
            for(int i = 0; i< transformN; i++)
                move_and_check();
            moving_step = 0;
        }
        e = clock();
        printf("CD time(KDOP_Approximate): %.2f\n", (e - s)/5.0f);
        printf("KDOP_Approximate hitRate: %.4f\n", numOfModelCollisions * 1.0 /numOfKCBPCollisions);

        numOfModelCollisions = 0, numOfKCBPCollisions = 0;
        current_bv = BVTYPE::KDOP_GJK;
        s = clock();
        for(int b = 0; b < 5; b++)
        {
            for(int i = 0; i< transformN; i++)
                move_and_check();
            moving_step = 0;
        }
        e = clock();
        printf("CD time(KDOP_GJK): %.2f\n", (e - s)/5.0f);
        printf("KDOP_GJK hitRate: %.4f\n", numOfModelCollisions * 1.0 /numOfKCBPCollisions);
        return 0;
    }

    draw(argc, argv);

    return 0;
}