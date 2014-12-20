// CP_PointVector.cpp 实现类CP_Point2D、CP_Point3D、CP_Vector2D和CP_Vector3D
#include "CP_PointVector.h"

#include <math.h>


// ////////////////////////////////////////////////////////////////////////////
// 实现类CP_Point2D开始
CP_Point2D::CP_Point2D(double newx, double newy):x(newx), y(newy)
{
} // 类CP_Point2D构造函数结束
// 实现类CP_Point2D结束
// ////////////////////////////////////////////////////////////////////////////

// ////////////////////////////////////////////////////////////////////////////
// 实现类CP_Point3D开始
CP_Point3D::CP_Point3D(double newx, double newy, double newz):x(newx), y(newy), z(newz)
{
} // 类CP_Point3D构造函数结束
// 实现类CP_Point3D结束
// ////////////////////////////////////////////////////////////////////////////

// ////////////////////////////////////////////////////////////////////////////
// 实现类CP_Point3D开始
CP_Vector2D::CP_Vector2D (double newx, double newy):x(newx), y(newy)
{
} // 类CP_Vector2D构造函数结束

CP_Vector2D& CP_Vector2D::operator += (const CP_Vector2D& v)
{ 
    x += v.x;
    y += v.y; 
    return *this; 
} //成员函数operator +=结束

CP_Vector2D& CP_Vector2D::operator -= (const CP_Vector2D& v)
{ 
    x -= v.x;
    y -= v.y; 
    return *this; 
} //成员函数operator -=结束

CP_Vector2D& CP_Vector2D::operator *= (double num)
{
    x *= num;
    y *= num; 
    return *this; 
} //成员函数operator *=结束

CP_Vector2D& CP_Vector2D::operator /= (double num)
{
    x /= num;  // 注意这里没有处理除数为0的情形
    y /= num; 
    return *this;
} //成员函数operator /=结束

double CP_Vector2D::operator ^(const CP_Vector2D& v)
{
    return( x* v.y- y* v.x );
} //成员函数operator ^结束

CP_Vector2D CP_Vector2D::operator - () const
{ 
    return CP_Vector2D (-x, -y); 
} //成员函数operator -结束

double CP_Vector2D::mf_getLength( )  const                             
{ 
    return sqrt(x*x + y*y); 
} //成员函数mf_getLength结束

double CP_Vector2D::mf_getLengthSqr2( ) const // 取长度的平方
{
   return (x*x + y*y); 
}

//拟时针得到的方向。
CP_Vector2D CP_Vector2D::mf_getPerpendicularVector() const
{
    return CP_Vector2D(-y, x);
} //成员函数mf_getPerpendicularVector结束

//顺时针 旋转90 得到的直线方向
CP_Vector2D CP_Vector2D::mf_getPerpendicularVectorCW() const
{
    return CP_Vector2D(y, -x);
} //成员函数mf_getPerpendicularVector结束


void CP_Vector2D::mf_normalize( )
{
    double a = mf_getLength( );
    (*this) /= a; // 注意: 这里没有处理当长度为0的情况
} //成员函数mf_normalize结束

void CP_Vector2D::mf_setValue(double newx, double newy)
{
    x=newx;
    y=newy;
} //成员函数mf_setValue结束

// 实现类CP_Vector2D结束
// ////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////
// 实现类CP_Vector3D开始
CP_Vector3D::CP_Vector3D (double newx, double newy, double newz):x(newx), y(newy), z(newz)
{
} // 类CP_Vector3D构造函数结束

CP_Vector3D& CP_Vector3D::operator += (const CP_Vector3D& v)
{ 
    x += v.x;
    y += v.y;
    z += v.z;  
    return *this; 
} //成员函数operator +=结束

CP_Vector3D& CP_Vector3D::operator -= (const CP_Vector3D& v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z; 
    return *this; 
} //成员函数operator -=结束

CP_Vector3D& CP_Vector3D::operator *= (double num)
{ 
    x *= num;
    y *= num;
    z *= num; 
    return *this; 
} //成员函数operator *=结束

CP_Vector3D& CP_Vector3D::operator /= (double num)
{
    num = 1.0/num;
    x *= num;
    y *= num;
    z *= num;
    return *this;
} //成员函数operator /=结束

CP_Vector3D& CP_Vector3D::operator ^= (const CP_Vector3D& v)
{ 
    double a =   y * v.z - z * v.y;
    double b = - x * v.z + z * v.x;
    double c =   x * v.y - y * v.x;

    x = a;
    y = b;
    z = c;
    return *this;
} //成员函数operator ^=结束

CP_Vector3D CP_Vector3D::operator - ( ) const
{ 
    return CP_Vector3D (-x, -y, -z); 
} //成员函数operator -结束

double CP_Vector3D::mf_getLength( )  const                             
{ 
    return sqrt(x*x + y*y + z*z); 
} //成员函数mf_getLength结束

double CP_Vector3D::mf_getLengthSquare( )  const                             
{ 
    return (x*x + y*y + z*z); 
} //成员函数mf_getLength结束


CP_Vector3D CP_Vector3D::mf_getPerpendicularVector( ) const
{
    CP_Vector3D vecReturn;
    if( fabs(y)<fabs(z))
    {
        vecReturn.x=z;
        vecReturn.y=0.0;
        vecReturn.z=-x;
    }
    else
    {
        vecReturn.x=-y;
        vecReturn.y=x;
        vecReturn.z=0.0;
    }
    return vecReturn;
} //成员函数mf_getPerpendicularVector结束

void CP_Vector3D::mf_normalize( )
{
    double a = mf_getLength( );
    (*this) /= a; // 注意: 这里没有处理除数为0的情况
} //成员函数mf_normalize结束

void CP_Vector3D::mf_setValue(double newx, double newy, double newz)
{
    x=newx;
    y=newy;
    z=newz;
} //成员函数mf_setValue结束

// 实现类CP_Vector3D结束
// ////////////////////////////////////////////////////////////////////////////

CP_Point2D operator + (const CP_Point2D& p, const CP_Vector2D& v)
{
    return CP_Point2D (p.x + v.x, p.y + v.y); 
} //函数operator +结束

CP_Point2D operator - (const CP_Point2D& pt, const CP_Vector2D& v)
{
    return CP_Point2D (pt.x - v.x, pt.y - v.y); 
} //函数operator -结束

//重写两个点相等，坐标相等即相等
bool operator == (const CP_Point2D& u, const CP_Point2D& v)
{
	if (abs(u.x - v.x) <= TOLERANCE &&abs(u.y - v.y) <= TOLERANCE) 
		return true;
	return false;
}
//should use tolerance
bool operator == (const CP_Vector3D& u, const CP_Vector3D& v)
{
    return (abs(u.x -v.x) <= TOLERANCE && 
            abs(u.y - v.y) <= TOLERANCE &&
            abs(u.z - v.z) <= TOLERANCE);
}


CP_Vector2D operator - (const CP_Point2D& p, const CP_Point2D& q)
{
    return CP_Vector2D (p.x - q.x, p.y - q.y); 
} //函数operator -结束

CP_Point3D operator + (const CP_Point3D& pt, const CP_Vector3D& v)
{
    return CP_Point3D (pt.x + v.x, pt.y + v.y, pt.z + v.z); 
} //函数operator +结束

CP_Point3D operator - (const CP_Point3D& pt, const CP_Vector3D& v)
{
    return CP_Point3D (pt.x - v.x, pt.y - v.y, pt.z - v.z); 
} //函数operator -结束

CP_Vector3D operator - (const CP_Point3D& p, const CP_Point3D& q)
{
    return CP_Vector3D (p.x - q.x, p.y - q.y, p.z - q.z); 
} //函数operator -结束

CP_Vector2D operator + (const CP_Vector2D& u, const CP_Vector2D& v)
{
    return CP_Vector2D (u.x + v.x, u.y + v.y); 
} //函数operator +结束

CP_Vector2D operator - (const CP_Vector2D& u, const CP_Vector2D& v)
{
    return CP_Vector2D (u.x - v.x, u.y - v.y);
} //函数operator -结束

// 点积
double  operator * (const CP_Vector2D& u, const CP_Vector2D& v)
{
    return u.x * v.x + u.y * v.y;
} //函数operator *结束

CP_Vector2D operator * (const CP_Vector2D& v, double num)
{
    return CP_Vector2D (v.x * num, v.y * num);
} //函数operator *结束

CP_Vector2D operator / (const CP_Vector2D& v, double num)
{
    return CP_Vector2D (v.x / num, v.y / num); // 注意: 这里没有处理除数为0的情况
} //函数operator /结束

CP_Vector3D operator + (const CP_Vector3D& u, const CP_Vector3D& v)
{
   return CP_Vector3D(u.x + v.x, u.y + v.y, u.z + v.z);
} //函数operator +结束

CP_Vector3D operator - (const CP_Vector3D& u, const CP_Vector3D& v)
{
    return CP_Vector3D (u.x - v.x, u.y - v.y, u.z - v.z);
} //函数operator -结束

// 点积
double operator * (const CP_Vector3D& u, const CP_Vector3D& v)
{
    return (u.x * v.x+u.y * v.y+ u.z * v.z);
} //函数operator *结束

// 叉积
CP_Vector3D operator ^ (const CP_Vector3D& u, const CP_Vector3D& v)
{
    return CP_Vector3D(u.y * v.z - u.z*v.y, 
                       -u.x*v.z+u.z*v.x,
                       u.x*v.y-u.y*v.x
                      );
} //函数operator ^结束

//this is only used when vector * num, cannot use as num * vector, if use num *vector, maybe transform to use vector(num * vector)
CP_Vector3D operator * (const CP_Vector3D& v, double num)
{
    return CP_Vector3D (v.x * num, v.y * num, v.z * num);
} //函数operator *结束

CP_Vector3D operator / (const CP_Vector3D& v, double num)
{
    num = 1.0/num; // 注意: 这里没有处理除数为0的情况
    return CP_Vector3D (v.x * num, v.y * num, v.z * num);
} //函数operator /结束



CP_Vector3D operator * (double num, const CP_Vector3D& v )
{
    return CP_Vector3D (v.x * num, v.y * num, v.z * num);
}

 

//check the univector is parallel
bool isParallel(const CP_Vector2D &univ_v1, const CP_Vector2D &univ_v2)
{
    //return abs(univ_v1 * univ_v2) < 1 + TOLERANCE && abs(univ_v1 * univ_v2) > 1 - TOLERANCE; 
    double inner_product = (univ_v1 * univ_v2);
    return (1-TOLERANCE) <= inner_product * inner_product; 
}

//check the univector is parallel
bool isParallel(const CP_Vector3D &univ_v1, const CP_Vector3D &univ_v2)
{
    //return abs(univ_v1 * univ_v2 - 1）< TOLERANCE; 
    return (univ_v1 ^ univ_v2).mf_getLength() <= TOLERANCE;
}

bool isPerpendicular(const CP_Vector3D &v1, const CP_Vector3D &v2) 
{
    double inner_product = (v1 * v2);
    return TOLERANCE >= inner_product * inner_product; 
}

bool operator == (const Line3D& u, const Line3D& v)
{
    return u.direction == v.direction && u.point == v.point;
}

int round(double number)
{
    return (int)(number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5));
}
