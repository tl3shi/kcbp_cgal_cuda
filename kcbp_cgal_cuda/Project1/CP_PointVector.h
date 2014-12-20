// CP_PointVector.h: 定义类CP_Point2D、CP_Point3D、CP_Vector2D和CP_Vector3D
#ifndef CP_POINTVECTOR_H
#define CP_POINTVECTOR_H

#define PI2         6.28318530717958647692
#define PI          3.14159265358979323846
#define PI_2        1.57079632679489661923
//tolerance if too small, something will wrong, for example three plane get intersection, when k = 18, then kdop normals
#define  TOLERANCE 0.0001      

#include "common/book.h"
#include <sstream>
#include <vector>

class Line3D;

class CP_Point2D
{
public:
    double    x, y;
public:
    //构造函数
    CP_Point2D (double newx=0.0, double newy=0.0);
};

class CP_Point3D
{
public:
    double    x, y, z;
public:
    //构造函数
    CP_Point3D (double newx=0.0, double newy=0.0, double newz=0.0);
    string toString()
    {
        std::stringstream ss;
        ss << "(" << x << ", " << y << ", " << z << ") ";
        return ss.str();
    }
};

class CP_Vector2D
{
public:
    double    x, y;

public:
    CP_Vector2D (double newx=0.0, double newy=0.0);

    // 赋值操作
    CP_Vector2D& operator += (const CP_Vector2D& v);
    CP_Vector2D& operator -= (const CP_Vector2D& v);
    CP_Vector2D& operator *= (double num);
    CP_Vector2D& operator /= (double num);
    double operator ^(const CP_Vector2D& v);

    //单目减
    CP_Vector2D operator - ( ) const;

    double mf_getLength( ) const; // 取长度
    double mf_getLengthSqr2( ) const; // 取长度的平方

    CP_Vector2D mf_getPerpendicularVector( ) const; //逆时针得到一个垂直的向量
    CP_Vector2D mf_getPerpendicularVectorCW() const;// 顺时针得到一个垂直向量

    void mf_normalize( ); // 单位化


    void mf_setValue(double newx=0.0, double newy=0.0);
};

class CP_Vector3D
{
public:
    double    x, y, z;

public:
    CP_Vector3D (double newx=0.0, double newy=0.0, double newz=0.0);

    //赋值操作
    CP_Vector3D& operator += (const CP_Vector3D& v);
    CP_Vector3D& operator -= (const CP_Vector3D& v);
    CP_Vector3D& operator *= (double num);
    CP_Vector3D& operator /= (double num);
    CP_Vector3D& operator ^= (const CP_Vector3D& v);

    double operator [](int i) const
    {
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;
        }
        return .0;
    }

    //单目减
    CP_Vector3D operator - () const;

    double mf_getLength ( ) const; // 取长度
    double mf_getLengthSquare( )  const; //取长度的平方 
    CP_Vector3D mf_getPerpendicularVector( ) const; //得到一个垂直的向量

    void mf_normalize( ); // 单位化
    void mf_setValue(double newx=0.0, double newy=0.0,double newz=0.0);

    string toString()
    {
        std::stringstream ss;
        ss << "(" << x << ", " << y << ", " << z << ") ";
        return ss.str();
    }

    friend std::ostream& operator << (std::ostream& stream, const CP_Vector3D& v)
    {
        return stream << "(" << v.x << ", " << v.y << ", " << v.z << ") ";
    }

};

extern CP_Point2D operator + (const CP_Point2D& pt, const CP_Vector2D& v);
extern bool operator == (const CP_Point2D& u, const CP_Point2D& v);
extern bool operator == (const CP_Vector3D& u, const CP_Vector3D& v);
extern bool operator == (const Line3D& u, const Line3D& v);

extern CP_Point2D operator - (const CP_Point2D& pt, const CP_Vector2D& v);
extern CP_Vector2D operator - (const CP_Point2D& p, const CP_Point2D& q);

extern CP_Point3D operator + (const CP_Point3D& pt, const CP_Vector3D& v);
extern CP_Point3D operator - (const CP_Point3D& pt, const CP_Vector3D& v);
extern CP_Vector3D operator - (const CP_Point3D& p, const CP_Point3D& q);

extern CP_Vector2D operator + (const CP_Vector2D& u, const CP_Vector2D& v); 
extern CP_Vector2D operator - (const CP_Vector2D& u, const CP_Vector2D& v); 
extern double  operator * (const CP_Vector2D& u, const CP_Vector2D& v); // 点积
extern CP_Vector2D operator * (const CP_Vector2D& v, double num);
extern CP_Vector2D operator / (const CP_Vector2D& v, double num); 

extern CP_Vector3D operator + (const CP_Vector3D& u, const CP_Vector3D& v);
extern CP_Vector3D operator - (const CP_Vector3D& u, const CP_Vector3D& v);
extern double operator * (const CP_Vector3D& u, const CP_Vector3D& v); // 点积
extern CP_Vector3D operator ^ (const CP_Vector3D& u, const CP_Vector3D& v); // 叉积

extern CP_Vector3D operator * (const CP_Vector3D& v, double num);
extern CP_Vector3D operator / (const CP_Vector3D& v, double num);

extern CP_Vector3D operator * (double num, const CP_Vector3D& v  );


extern  bool  isParallel(const CP_Vector2D &univ_v1, const CP_Vector2D &univ_v2);
extern bool isParallel(const CP_Vector3D &univ_v1, const CP_Vector3D &univ_v2);
extern bool isPerpendicular(const CP_Vector3D &v1, const CP_Vector3D &v2) ;

class Line2D
{
public:
    Line2D(CP_Vector2D d, CP_Vector2D p):direction(d), point(p){};
    Line2D(){};
public:
    CP_Vector2D direction;//NOT the normal direction
    CP_Vector2D point;
};

class Line3D
{
public:
    Line3D(CP_Vector3D d, CP_Vector3D p):direction(d), point(p){};
    Line3D(){};
    bool equals(const Line3D & l)
    {
        return (l.direction == this->direction && l.point == this->point);
    }
public:
    CP_Vector3D direction;//NOT the normal direction
    CP_Vector3D point;
};

class Plane3D
{
public:
    Plane3D(CP_Vector3D n, CP_Vector3D p):normal(n), point(p){};
    Plane3D(){};
    CP_Vector3D normal;
    CP_Vector3D point;

    Plane3D(float3 &n, float3 &p)
    {
        point = CP_Vector3D(p.x, p.y, p.z);
        normal = CP_Vector3D(n.x, n.y, n.z);
    };

    string toString()
    {
        std::stringstream ss;
        ss << "normal:" << normal.toString() << ", point: " << point.toString()<< ")";
        return ss.str();
    } 
    
    CP_Vector3D toDualPoint()
    {
        //plane fucntion: ax+by+cz=d=ax0+by0+cz0
        double d = normal * point;
        assert(d != 0);
        return CP_Vector3D(normal) / d;
    }  

    bool isPointOnPlane(CP_Vector3D &point)
    {
        if(point == this->point)
            return true;
        return abs((point - this->point) * this->normal ) <= TOLERANCE;
    }
};

class CP_Vector4D 
{
    typedef double DataType ;
public:
    DataType x, y, z, w;
    CP_Vector4D(void){}
    CP_Vector4D(const CP_Vector4D &v)
    {
        this->x = v.x;
        this->y = v.y;
        this->z = v.z;
        this->w = v.w;
      
    }
    CP_Vector4D(DataType _x, DataType _y, DataType _z, DataType _w = 1.0f){x = _x; y = _y; z = _z; w = _w;}
    ~CP_Vector4D(void){}

    CP_Vector4D operator^(const CP_Vector4D &v) const
    {
        return CP_Vector4D(this->y * v.z - this->y * v.z, this->z * v.x - this->x * v.z ,this->x * v.y - this->y * v.x, 1.0f);
    }

    CP_Vector4D operator /(const DataType p) const
    {
        return CP_Vector4D(this->x/p, this->y/p, this->z/p, this->w/p);
    }
    
    CP_Vector4D operator /=(const DataType p) 
    {
        (*this).x = this->x /p;
        (*this).y = this->y /p;
        (*this).z = this->z /p;
        (*this).w = this->w /p;
        return *this;
    }
    
    CP_Vector4D operator +=(const DataType p) 
    {
        (*this).x = this->x + p;
        (*this).y = this->y + p;
        (*this).z = this->z + p;
        (*this).w = this->w + p;
        return *this;

    }
    CP_Vector4D operator -=(const DataType p) 
    {
        (*this).x = this->x - p;
        (*this).y = this->y - p;
        (*this).z = this->z - p;
        (*this).w = this->w - p;
        return *this;
    }
   
};

class Polygon3D
{
public:
    vector<CP_Vector3D> data;
    CP_Vector3D* normal;

    Polygon3D()
    {
        normal = new CP_Vector3D(0,0,0);
    }

    /*~Polygon3D()
    {
    delete normal;
    for (unsigned int i = 0; i < data.size(); i++)
    {
    delete data[i];
    }
    }*/
};

int round(double number);

//class is different from struct,struct is defaultly public , otherwise class is private
class CPVector_to_Float3
{
public:
    #pragma warning(push)
    #pragma  warning(disable:4244)
    float3 operator()(CP_Vector3D &point)
    {
        return make_float3(point.x, point.y, point.z);
    }
    #pragma  warning(pop)
};

class AreaIndex
{
public:
    double area;
    int index;
    AreaIndex(double a, int i):area(a), index(i){}
    int operator < (const AreaIndex &areaIndex)
    {
        return area > areaIndex.area;
    }
    AreaIndex(){}
};

#endif