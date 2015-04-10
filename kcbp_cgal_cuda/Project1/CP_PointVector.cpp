// CP_PointVector.cpp ʵ����CP_Point2D��CP_Point3D��CP_Vector2D��CP_Vector3D
#include "CP_PointVector.h"

#include <math.h>


// ////////////////////////////////////////////////////////////////////////////
// ʵ����CP_Point2D��ʼ
CP_Point2D::CP_Point2D(RealValueType newx, RealValueType newy):x(newx), y(newy)
{
} // ��CP_Point2D���캯������
// ʵ����CP_Point2D����
// ////////////////////////////////////////////////////////////////////////////

// ////////////////////////////////////////////////////////////////////////////
// ʵ����CP_Point3D��ʼ
CP_Point3D::CP_Point3D(RealValueType newx, RealValueType newy, RealValueType newz):x(newx), y(newy), z(newz)
{
} // ��CP_Point3D���캯������
// ʵ����CP_Point3D����
// ////////////////////////////////////////////////////////////////////////////

// ////////////////////////////////////////////////////////////////////////////
// ʵ����CP_Point3D��ʼ
CP_Vector2D::CP_Vector2D (RealValueType newx, RealValueType newy):x(newx), y(newy)
{
} // ��CP_Vector2D���캯������

CP_Vector2D& CP_Vector2D::operator += (const CP_Vector2D& v)
{ 
    x += v.x;
    y += v.y; 
    return *this; 
} //��Ա����operator +=����

CP_Vector2D& CP_Vector2D::operator -= (const CP_Vector2D& v)
{ 
    x -= v.x;
    y -= v.y; 
    return *this; 
} //��Ա����operator -=����

CP_Vector2D& CP_Vector2D::operator *= (RealValueType num)
{
    x *= num;
    y *= num; 
    return *this; 
} //��Ա����operator *=����

CP_Vector2D& CP_Vector2D::operator /= (RealValueType num)
{
    x /= num;  // ע������û�д������Ϊ0������
    y /= num; 
    return *this;
} //��Ա����operator /=����

RealValueType CP_Vector2D::operator ^(const CP_Vector2D& v)
{
    return( x* v.y- y* v.x );
} //��Ա����operator ^����

CP_Vector2D CP_Vector2D::operator - () const
{ 
    return CP_Vector2D (-x, -y); 
} //��Ա����operator -����

RealValueType CP_Vector2D::mf_getLength( )  const                             
{ 
    return sqrt(x*x + y*y); 
} //��Ա����mf_getLength����

RealValueType CP_Vector2D::mf_getLengthSqr2( ) const // ȡ���ȵ�ƽ��
{
   return (x*x + y*y); 
}

//��ʱ��õ��ķ���
CP_Vector2D CP_Vector2D::mf_getPerpendicularVector() const
{
    return CP_Vector2D(-y, x);
} //��Ա����mf_getPerpendicularVector����

//˳ʱ�� ��ת90 �õ���ֱ�߷���
CP_Vector2D CP_Vector2D::mf_getPerpendicularVectorCW() const
{
    return CP_Vector2D(y, -x);
} //��Ա����mf_getPerpendicularVector����


void CP_Vector2D::mf_normalize( )
{
    RealValueType a = mf_getLength( );
    (*this) /= a; // ע��: ����û�д�������Ϊ0�����
} //��Ա����mf_normalize����

void CP_Vector2D::mf_setValue(RealValueType newx, RealValueType newy)
{
    x=newx;
    y=newy;
} //��Ա����mf_setValue����

// ʵ����CP_Vector2D����
// ////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////
// ʵ����CP_Vector3D��ʼ
CP_Vector3D::CP_Vector3D (RealValueType newx, RealValueType newy, RealValueType newz):x(newx), y(newy), z(newz)
{
} // ��CP_Vector3D���캯������

CP_Vector3D& CP_Vector3D::operator += (const CP_Vector3D& v)
{ 
    x += v.x;
    y += v.y;
    z += v.z;  
    return *this; 
} //��Ա����operator +=����

CP_Vector3D& CP_Vector3D::operator -= (const CP_Vector3D& v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z; 
    return *this; 
} //��Ա����operator -=����

CP_Vector3D& CP_Vector3D::operator *= (RealValueType num)
{ 
    x *= num;
    y *= num;
    z *= num; 
    return *this; 
} //��Ա����operator *=����

CP_Vector3D& CP_Vector3D::operator /= (RealValueType num)
{
    num = 1.0f/num;
    x *= num;
    y *= num;
    z *= num;
    return *this;
} //��Ա����operator /=����

CP_Vector3D& CP_Vector3D::operator ^= (const CP_Vector3D& v)
{ 
    RealValueType a =   y * v.z - z * v.y;
    RealValueType b = - x * v.z + z * v.x;
    RealValueType c =   x * v.y - y * v.x;

    x = a;
    y = b;
    z = c;
    return *this;
} //��Ա����operator ^=����

CP_Vector3D CP_Vector3D::operator - ( ) const
{ 
    return CP_Vector3D (-x, -y, -z); 
} //��Ա����operator -����

RealValueType CP_Vector3D::mf_getLength( )  const                             
{ 
    return sqrt(x*x + y*y + z*z); 
} //��Ա����mf_getLength����

RealValueType CP_Vector3D::mf_getLengthSquare( )  const                             
{ 
    return (x*x + y*y + z*z); 
} //��Ա����mf_getLength����


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
} //��Ա����mf_getPerpendicularVector����

void CP_Vector3D::mf_normalize( )
{
    RealValueType a = mf_getLength( );
    (*this) /= a; // ע��: ����û�д������Ϊ0�����
} //��Ա����mf_normalize����

void CP_Vector3D::mf_setValue(RealValueType newx, RealValueType newy, RealValueType newz)
{
    x=newx;
    y=newy;
    z=newz;
} //��Ա����mf_setValue����

// ʵ����CP_Vector3D����
// ////////////////////////////////////////////////////////////////////////////

CP_Point2D operator + (const CP_Point2D& p, const CP_Vector2D& v)
{
    return CP_Point2D (p.x + v.x, p.y + v.y); 
} //����operator +����

CP_Point2D operator - (const CP_Point2D& pt, const CP_Vector2D& v)
{
    return CP_Point2D (pt.x - v.x, pt.y - v.y); 
} //����operator -����

//��д��������ȣ�������ȼ����
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
} //����operator -����

CP_Point3D operator + (const CP_Point3D& pt, const CP_Vector3D& v)
{
    return CP_Point3D (pt.x + v.x, pt.y + v.y, pt.z + v.z); 
} //����operator +����

CP_Point3D operator - (const CP_Point3D& pt, const CP_Vector3D& v)
{
    return CP_Point3D (pt.x - v.x, pt.y - v.y, pt.z - v.z); 
} //����operator -����

CP_Vector3D operator - (const CP_Point3D& p, const CP_Point3D& q)
{
    return CP_Vector3D (p.x - q.x, p.y - q.y, p.z - q.z); 
} //����operator -����

CP_Vector2D operator + (const CP_Vector2D& u, const CP_Vector2D& v)
{
    return CP_Vector2D (u.x + v.x, u.y + v.y); 
} //����operator +����

CP_Vector2D operator - (const CP_Vector2D& u, const CP_Vector2D& v)
{
    return CP_Vector2D (u.x - v.x, u.y - v.y);
} //����operator -����

// ���
RealValueType  operator * (const CP_Vector2D& u, const CP_Vector2D& v)
{
    return u.x * v.x + u.y * v.y;
} //����operator *����

CP_Vector2D operator * (const CP_Vector2D& v, RealValueType num)
{
    return CP_Vector2D (v.x * num, v.y * num);
} //����operator *����

CP_Vector2D operator / (const CP_Vector2D& v, RealValueType num)
{
    return CP_Vector2D (v.x / num, v.y / num); // ע��: ����û�д������Ϊ0�����
} //����operator /����

CP_Vector3D operator + (const CP_Vector3D& u, const CP_Vector3D& v)
{
   return CP_Vector3D(u.x + v.x, u.y + v.y, u.z + v.z);
} //����operator +����

CP_Vector3D operator - (const CP_Vector3D& u, const CP_Vector3D& v)
{
    return CP_Vector3D (u.x - v.x, u.y - v.y, u.z - v.z);
} //����operator -����

// ���
RealValueType operator * (const CP_Vector3D& u, const CP_Vector3D& v)
{
    return (u.x * v.x+u.y * v.y+ u.z * v.z);
} //����operator *����

// ���
CP_Vector3D operator ^ (const CP_Vector3D& u, const CP_Vector3D& v)
{
    return CP_Vector3D(u.y * v.z - u.z*v.y, 
                       -u.x*v.z+u.z*v.x,
                       u.x*v.y-u.y*v.x
                      );
} //����operator ^����

//this is only used when vector * num, cannot use as num * vector, if use num *vector, maybe transform to use vector(num * vector)
CP_Vector3D operator * (const CP_Vector3D& v, RealValueType num)
{
    return CP_Vector3D (v.x * num, v.y * num, v.z * num);
} //����operator *����

CP_Vector3D operator / (const CP_Vector3D& v, RealValueType num)
{
    num = 1.0f/num; // ע��: ����û�д������Ϊ0�����
    return CP_Vector3D (v.x * num, v.y * num, v.z * num);
} //����operator /����



CP_Vector3D operator * (RealValueType num, const CP_Vector3D& v )
{
    return CP_Vector3D (v.x * num, v.y * num, v.z * num);
}

 

//check the univector is parallel
bool isParallel(const CP_Vector2D &univ_v1, const CP_Vector2D &univ_v2)
{
    //return abs(univ_v1 * univ_v2) < 1 + TOLERANCE && abs(univ_v1 * univ_v2) > 1 - TOLERANCE; 
    RealValueType inner_product = (univ_v1 * univ_v2);
    return (1-TOLERANCE) <= inner_product * inner_product; 
}

//check the univector is parallel
bool isParallel(const CP_Vector3D &univ_v1, const CP_Vector3D &univ_v2)
{
    //return abs(univ_v1 * univ_v2 - 1��< TOLERANCE; 
    return (univ_v1 ^ univ_v2).mf_getLength() <= TOLERANCE;
}

bool isPerpendicular(const CP_Vector3D &v1, const CP_Vector3D &v2) 
{
    RealValueType inner_product = (v1 * v2);
    return TOLERANCE >= inner_product * inner_product; 
}

bool operator == (const Line3D& u, const Line3D& v)
{
    return u.direction == v.direction && u.point == v.point;
}

int round(RealValueType number)
{
    return (int)(number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5));
}
