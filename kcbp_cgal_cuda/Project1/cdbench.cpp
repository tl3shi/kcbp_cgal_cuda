#pragma  once
#pragma warning(disable:4018)
#pragma warning(disable:4224)

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
#include "AABB.hpp"
#include "CollsionQuery.hpp"
#include "SolidCollisionQuery.hpp"
#include "LibCCDQuery.hpp"

#include <sstream>
#include <fstream>

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
bool NO_DISPLAY = true;

//#define TEMPDEBUG

vector<CP_Vector3D> points3d;
vector<int> trianges_index;
vector<CP_Vector3D> normals;
vector<Polygon3D*> polyhedra;
vector<CP_Vector3D> convexhull;
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
vector<int> MeshPolyhedronIndex;

vector<CP_Vector3D> MeshPolyhedronPoints;

vector<BoundingBox> ModelBoundingBoxes;

BoundingBox SingleModelBoundingBox;

vector<pair<int, int>> collision_pair;

vector<int> MeshPointsDataList; //display listid in OpenGL
vector<int> MeshPolyhedronDataList; // for KCBP
vector<int> MeshBoundingBoxList;


double Scale_Bunny = 1.0;//100.0;
int scale_convexhull =  1;
const double tranlate_unit = 0.01 * Scale_Bunny;
CCamera m_camera;
CP_Vector2D m_formerMousePos;
bool m_isLeftButtonDown = false;
bool m_isRightButtonDown = false;
bool necessary = true;

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

const int movingModelIndex = 0; //just the first model is transformed

void CollisionDetection(vector<vector<CP_Vector3D> > & MeshPointsData, vector<int> & trianges_index, 
                        vector<vector<CP_Vector3D> > & MeshpolyhedraData, vector<int> & MeshPolyhedronIndex, 
                        vector<BoundingBox> & ModelBoundingBoxes, const mat4 & transformMatrix, vector<pair<int, int>> & collision_pair);

string objfilename;

bool OUTPUT_POLYHEDRON = true;
int k = 0;

bool finish_without_update = false; //used to cal fps
bool usekcbp = true;

ICollisionQuery * collision_query = 0;
ICollisionQuery * kcbp_query = 0;

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

int modelnum = 1;

void scalebunny(vector<CP_Vector3D> &input, const CP_Vector3D &low, const CP_Vector3D &high)
{
    //scale same as CollDet\visual_studio\TestSolution\Bench
    CP_Vector3D c = 0.5 * low + 0.5 * high;
    CP_Vector3D d = 0.5 * high - 0.5 * low; 
    double s = max(d.x, max(d.y, d.z));
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

//collisiondetection.exe k models/bunny2.obj n(owindow)  10  rand.config p(rint_pairs)
bool print_pairs = false;
int main(int argc, char** argv)
{
    vector<Plane3D> planes;
    
    objfilename = "models/budda.obj"; 
    //objfilename = "models/apple.obj";
    if(argc >= 3)
        objfilename = argv[2];
    if(argc >= 4)
        NO_DISPLAY = argv[3][0] == 'w' ? false : true;  

    bool loaded = FileManager::readObjPointsAndFacetsFast(points3d, trianges_index, objfilename, SingleModelBoundingBox.Min, SingleModelBoundingBox.Max);
    scalebunny(points3d, SingleModelBoundingBox.Min, SingleModelBoundingBox.Max); //Attention
    if(! loaded) {cout << "load failed:" << objfilename << endl;return 0;}
    cout << objfilename << " \tpoints number: " << points3d.size() <<endl;
    
    string readconfig = "config/04-03-200-rand.config";//"12-19-10-31-50-rand.config";

    k =  16;
    if(argc >= 2) 
        k = atoi(argv[1]);
    cout << "K = " << k << endl;
    modelnum = 10;
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
    
    if(true)
    {
        clock_t s, e;
        s = clock();
        for (int i = 0; i < benchtest; i++)
        {
            ach =  ACH3D::getACH(points3d, 10, 10);
        }
        e = clock();
        printf("ach banchtest (%d times) totally: %.2f\n", benchtest, (e - s) * 1.0f);
    }
    else
    {
        ach =  ACH3D::getACH(points3d, 10, 10);
    }
    clock_t s, e;
    s = clock();
    for(int i = 0; i < benchtest; i++)
        normals = KCBP::getClusterNormals(ach, k, true, false);
    e = clock();
    printf("cluster time (%d times) totally: %.2f\n", benchtest, (e - s) * 1.0f);

    printf("normalsize = %d\n", normals.size());
    SSEProjection::projectCPUSSE(points3d, normals, planes, benchtest);
   //KDop3D::projectCPU(points3d, normals, planes, benchtest);
   //KCBP::evaluate(points3d, normals, KCBP::CUDA_MUL_NORMAL, planes, benchtest);
    
    printf("duality mapping\n");
    s = clock();
    vector<vector<Polygon3D*>> tmpPolyhedra;
    for(int i = 0; i < benchtest ;i++)
    {
        vector<Polygon3D*> tmp;
        KDop3D::getResultByDualMapping(planes, tmp);//mapping get points / then sort points ccw construct kcbp
        tmpPolyhedra.push_back(tmp);
    }
    e = clock();
    printf("duality mapping time (%d times) totally: %.2f\n", benchtest, (e - s) * 1.0f);
    polyhedra = tmpPolyhedra[0];
    for(int i = 1; i < tmpPolyhedra.size(); i++)
        for(int j = 0; j < tmpPolyhedra[i].size(); j++)
            delete tmpPolyhedra[i][j];

    KDop3D::getResultByDualMappingMesh(planes, MeshPolyhedronPoints);//mapping get points/ then call convexhull get kcbp mesh
    
    printf("Polytopes: %d\n", polyhedra.size());
    
    genModels(modelnum, readconfig);
 
   
#ifdef TEMPDEBUG
    NO_DISPLAY = false;
#endif // TEMPDEBUG

    if(NO_DISPLAY) return 0;
     
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

void genModels(int modelnum, string config)
{
    assert(modelnum >= 2);
    
    MeshPolyhedronIndex.resize(MeshPolyhedronPoints.size());
    for(int i = 0; i < MeshPolyhedronPoints.size(); i++)
        MeshPolyhedronIndex[i] = i;
    //the original one
    MeshPointsData.push_back(points3d);
    MeshpolyhedraData.push_back(MeshPolyhedronPoints);

    bool isConfigreadin = config.length() > 0;
    streambuf *defaultstream = cout.rdbuf();

   
    vector<int> rotate_angles;
    vector<CP_Vector3D> translations;
    vector<CP_Vector3D> rotations;
    if(! isConfigreadin)
    {
        srand(time(0));
        time_t t = time(0); 
        char tmp[64]; 
        strftime( tmp, sizeof(tmp), "%m-%d-%H-%M", localtime(&t) ); 
        ofstream f(string(tmp) + "-" + to_string(modelnum) + "-rand.config");
        cout << "rand config write to :" << string(tmp) + "-" + to_string(modelnum) + "-rand.config" << endl;
        cout.rdbuf(f.rdbuf());

        double xx = SingleModelBoundingBox.Max.x - SingleModelBoundingBox.Min.x;
        double range = modelnum*xx;
        for(int i = 0; i < modelnum; i++)
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

        vector<CP_Vector3D> kcbpdata(MeshPolyhedronPoints.size());
        std::transform(MeshPolyhedronPoints.begin(), MeshPolyhedronPoints.end(), kcbpdata.begin(), trans);
        MeshpolyhedraData.push_back(kcbpdata);
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
    /*
    #ifdef USE_SOLID //bug remains, is not very precise
        collision_query = new SolidCollisionQuery(MeshPointsData[0], trianges_index);
        //collision_query = new SolidCollisionQuery(ModelBoundingBoxes[0].GetAABBVertices(), ModelBoundingBoxes[0].GetAABBIndices(), ModelBoundingBoxes[1].GetAABBVertices(), ModelBoundingBoxes[1].GetAABBIndices());
        //kcbp_query = new SolidCollisionQuery(MeshpolyhedraData[0], MeshPolyhedronIndex, MeshpolyhedraData[1], MeshPolyhedronIndex);
    #else
        //collision_query = new CollisionQuery(MeshPointsData[0], trianges_index, MeshPointsData[1], trianges_index);
        //kcbp_query = new CollisionQuery(MeshpolyhedraData[0], MeshpolyhedraData[1]);
        collision_query = new CollisionQuery(MeshPointsData[0], trianges_index, MeshPointsData[1], trianges_index);
        kcbp_query = new LibCCDQuery(MeshpolyhedraData[0], MeshpolyhedraData[1]);
    #endif // USE_SOLID
    */
    vector<std::pair<int, int> > boxPairs;
    c_time = clock();
    BoundingBox::CollsionDetection(ModelBoundingBoxes, boxPairs);
    c_time = clock() - c_time;
    printf("collision detection box time: %.2f\n", c_time * 1.0);
    printf("box collision size: %d\n", boxPairs.size());
    vector<bool> needMoreCheck(modelnum, false);
    for(auto it = boxPairs.begin(); it != boxPairs.end(); it++)
        needMoreCheck[it->first] = needMoreCheck[it->second] = true;
    
    int model_num = MeshpolyhedraData.size();
    clock_t time_start = clock();
    vector<AABBTree*> aabbtrees(model_num, NULL);
    int aabbneed = 0;
    for(int i = 0; i < model_num; i++)
    {
        if(needMoreCheck[i])
        {
            aabbtrees[i] = constructAABBTree(MeshPointsData[i], trianges_index);
            aabbneed++;
        }
    }
    float time_during = clock() - time_start;
    printf("build AABB time : %.2f\n", time_during);
    printf("build AABB count: %d\n", aabbneed);
    if(false) //No direct
    {
        printf("The Following result uses AABB directly\n");
        collision_index.resize(model_num);
        std::vector<std::pair<int, int> > cpairs;
        clock_t c_time = clock();
        //for(int i = 0; i < model_num-1; i++)
        for(auto it = boxPairs.begin(); it != boxPairs.end(); it++)
        {
            int i = it->first; int j = it->second;
        //    for(int j = i+1; j < model_num; j++)
        //    {
                time_start = clock();
                bool c = AABBTree::TraverseDetective(aabbtrees[i]->Root, aabbtrees[j]->Root);
                if(c)
                {
                    //#if _DEBUG
                    //printf("collision!");
                    //#endif
                    cpairs.push_back(std::make_pair(i, j));
                }
                time_during = clock() - time_start;
                //#if _DEBUG
                //printf("collision time = %.2f\n", time_during);
                //#endif
            //}
        }
        c_time = clock() - c_time;
        printf("total collision time(AABB directly) : %.2f\n", c_time * 1.0);
        
        for(int i = 0; i < cpairs.size(); i++)
        {
            if(print_pairs)
                printf("(%d, %d)", cpairs[i].first, cpairs[i].second);
            collision_index[cpairs[i].first] = true;
            collision_index[cpairs[i].second] = true;
        }
        if(print_pairs) printf("\n");
        printf("collision pairs : %d\n", cpairs.size());
    }
    { //use kcbp filter first
        printf("The Following result uses KCBP(AABB) filter first\n");
        time_start = clock();
        vector<AABBTree*> aabbtrees4kcbp(model_num, NULL);
        for(int i = 0; i < model_num; i++)
        {
           if(needMoreCheck[i])
                aabbtrees4kcbp[i] = constructAABBTree(MeshpolyhedraData[i], MeshPolyhedronIndex);
        }
        time_during = clock() - time_start;
        printf("build AABB time 4 KCBP : %.2f\n", time_during);
        std::vector<std::pair<int, int> > kcbppairs;
        std::vector<std::pair<int, int> > cpairs;
        clock_t c_time = clock();
        CMatrix ident = CMatrix::Identity;
        vector<bool> needModelCheck(model_num, false);
        for(auto it = boxPairs.begin(); it != boxPairs.end(); it++)
        {
            int i = it->first; int j = it->second;        
            bool c = AABBTree::TraverseDetective(aabbtrees4kcbp[i]->Root, aabbtrees4kcbp[j]->Root);
            if(c)
            {
                kcbppairs.push_back(std::make_pair(i, j));
                needModelCheck[i] = needModelCheck[j] = true;
            }
        }
        for(auto it = kcbppairs.begin(); it != kcbppairs.end(); it++)
        {
            bool c = AABBTree::TraverseDetective(aabbtrees[it->first]->Root, aabbtrees[it->second]->Root);
            if(c)
                cpairs.push_back(std::make_pair(it->first, it->second));
        }
        c_time = clock() - c_time;
        int needModelCheckCount = 0;
        for(int i = 0; i < needMoreCheck.size(); i++)
        {
            if(needMoreCheck[i])
                needModelCheckCount++;
        }
        printf("kcbp collision size(AABB filter) : %d\n", kcbppairs.size());
        printf("kcbp model check count: %d\n", needModelCheckCount);
        printf("total collision time(KCBP AABB filter) : %.2f\n", c_time * 1.0);
        printf("collision pairs : %d\n", cpairs.size());
        //deconstructor
        for(int i = 0; i < aabbtrees4kcbp.size(); i++)
            delete aabbtrees4kcbp[i];
        
    }
    {
        //use kcbp filter first
        printf("The Following result uses KCBP(GJK) filter first\n");
        time_start = clock();
        vector<LibCCDQuery*> gjkquerys;
        
        //for(int i = 0; i < model_num-1; i++)
        //    for(int j = i+1; j < model_num; j++)
                for(auto it = boxPairs.begin(); it != boxPairs.end(); it++)
                {
                    gjkquerys.push_back(new LibCCDQuery(MeshpolyhedraData[it->first], MeshpolyhedraData[it->second]));
                    //gjkqueryIndex.push_back(*it);
                }

        time_during = clock() - time_start;
        printf("init GJK time 4 KCBP : %.2f\n", time_during); //including Boundingbox collision detection
        std::vector<std::pair<int, int> > kcbppairs;
        std::vector<std::pair<int, int> > cpairs;
        clock_t c_time = clock();
        CMatrix ident = CMatrix::Identity;
        for(int i = 0; i < gjkquerys.size(); i++)
        {
           bool cc = gjkquerys[i]->detection(ident, ident);
           if(cc)
                //kcbppairs.push_back(std::make_pair(i/(model_num-1), i%(model_num-1)+1));
                kcbppairs.push_back(std::make_pair(boxPairs[i].first, boxPairs[i].second));
        }

        for(auto it = kcbppairs.begin(); it != kcbppairs.end(); it++)
        {
            bool c = AABBTree::TraverseDetective(aabbtrees[it->first]->Root, aabbtrees[it->second]->Root);
            if(c)
                cpairs.push_back(std::make_pair(it->first, it->second));
        }
        c_time = clock() - c_time;
        printf("kcbp collision size(GJK filter) : %d\n", kcbppairs.size());
        printf("total collision time(KCBP GJK filter) : %.2f\n", c_time * 1.0);
        printf("collision pairs : %d\n", cpairs.size());
        //deconstructor
        for(int i = 0; i < gjkquerys.size(); i++)
            delete gjkquerys[i];
    }
    //deconstructor 4 aabbtrees
    for(int i = 0; i < aabbtrees.size(); i++)
        delete aabbtrees[i];
    return;
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
        
        #ifdef TEMPDEBUG 
            glCallList(MeshPolyhedronDataList[13]);
            glCallList(MeshPolyhedronDataList[37]);
            glCallList(MeshBoundingBoxList[13]);
            glCallList(MeshBoundingBoxList[37]);
        #else

        if(!(modelnum > 50 && points3d.size() > 100000)) //two much to draw
        {
            for(int i = 0; i < MeshPointsDataList.size(); i++)
            {
                if(collision_index[i])
                    glColor3fv(collsion_color);
                else
                    glColor4fv(oldColor);

                #ifdef DRAW_MODEL
                glCallList(MeshPointsDataList[i]);
                #endif
            }
        }

        if(true)
        {
            glColor3f(0, 0, 1);
            for(int i = 1; i < MeshPolyhedronDataList.size(); i++)
                glCallList(MeshPolyhedronDataList[i]);
            
            glColor3f(0, 0, 0);
            for(int i = 1; i < MeshBoundingBoxList.size(); i++)
                glCallList(MeshBoundingBoxList[i]);
            glColor4fv(oldColor);
        } 
 
        #ifdef DRAW_MODEL
        glCallList(MeshPointsDataList[movingModelIndex]);
        #endif
        glColor3f(0, 0, 1);
        glCallList(MeshPolyhedronDataList[movingModelIndex]);
        glColor3f(0, 0, 0);
        glCallList(MeshBoundingBoxList[movingModelIndex]);
        glColor4fv(oldColor);

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
 
void CollisionDetection(vector<vector<CP_Vector3D> > & MeshPointsData, vector<int> & trianges_index, 
                        vector<vector<CP_Vector3D> > & MeshpolyhedraData, vector<int> & MeshPolyhedronIndex, 
                        vector<BoundingBox> & ModelBoundingBoxes, const mat4 & transformMatrix, vector<pair<int, int>> & collision_pair)
{
    
    //bounding box
    BoundingBox movingModelBBoxBak = ModelBoundingBoxes[movingModelIndex];
    vector<CP_Vector3D> vertices = movingModelBBoxBak.GetAABBVertices();
    BoundingBox transformedBBox(CP_Vector3D(DBL_MAX, DBL_MAX, DBL_MAX), -CP_Vector3D(DBL_MAX, DBL_MAX, DBL_MAX));
    for(int i = 0; i < 8; i++)
    {
        CP_Vector3D v = transformMatrix * vertices[i];
        transformedBBox.Min.x = min(transformedBBox.Min.x, v.x);
        transformedBBox.Min.y = min(transformedBBox.Min.y, v.y);
        transformedBBox.Min.z = min(transformedBBox.Min.z, v.z);
        transformedBBox.Max.x = max(transformedBBox.Max.x, v.x);
        transformedBBox.Max.y = max(transformedBBox.Max.y, v.y);
        transformedBBox.Max.z = max(transformedBBox.Max.z, v.z);
    }
    ModelBoundingBoxes[movingModelIndex] = transformedBBox;
    vector<std::pair<int, int> > box_collisions;
    BoundingBox::CollsionDetection(ModelBoundingBoxes, box_collisions);
    ModelBoundingBoxes[movingModelIndex] = movingModelBBoxBak;

    //kcbp 
    vector<CP_Vector3D > movingModelPolyhedronBak = MeshpolyhedraData[movingModelIndex];
    for(int i = 0; i < movingModelPolyhedronBak.size(); i++)
        MeshpolyhedraData[movingModelIndex][i] = transformMatrix * movingModelPolyhedronBak[i];
    vector<std::pair<int, int> > dop_collisions;
    for(int i = 0; i < box_collisions.size(); i++)
    {
        pair<int, int> &index = box_collisions[i];
        if (AABBTree::MeshMeshDetection(MeshpolyhedraData[index.first], MeshpolyhedraData[index.second]))
            dop_collisions.push_back(index);
    }
    MeshpolyhedraData[movingModelIndex] = movingModelPolyhedronBak;

    if(!mesh_detective)
    {
        collision_pair = dop_collisions;
    }else
    {
        //triangle triangle detection
        vector<CP_Vector3D> movingModelPointsDataBak = MeshPointsData[movingModelIndex];
        for(int i = 0; i < movingModelPointsDataBak.size(); i++)
            MeshPointsData[movingModelIndex][i] = transformMatrix * movingModelPointsDataBak[i];
        for(int i = 0; i < dop_collisions.size(); i++)
        {
            pair<int,int> &p = dop_collisions[i];
            if(AABBTree::MeshMeshDetection(MeshPointsData[p.first], trianges_index, MeshPointsData[p.second], trianges_index))
                collision_pair.push_back(p);
        }
        MeshPointsData[movingModelIndex] = movingModelPointsDataBak;
    }
}

 