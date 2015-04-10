#pragma once
#include <cmath>
#include <ostream>

#ifdef DEBUG
#include <exception>
#endif

#ifndef readwrite
#define readwrite(var) __declspec(property(get=Get##var, put=Set##var)) var
#endif

#ifndef readonly
#define readonly(var) __declspec(property(get=Get##var)) var
#endif

#ifndef writeonly
#define writeonly(var) __declspec(property(put=Set##var)) var
#endif

#define DEGREE_PER_RADIAN   57.2957795130823208768f
#define DOUBLE_PI           6.28318530717958647692f

#ifndef PI
#define PI                  3.14159265358979323846f
#endif


#define HALF_PI             1.57079632679489661923f


//typedef float RealValueType;


#ifdef UseFloat
    #ifndef RealValueType 
    #define RealValueType float
    #endif
#else
    #ifndef RealValueType 
    #define RealValueType double
    #endif
#endif


namespace Math
{
    inline RealValueType Abs(RealValueType value)
    {
        return value > 0.0 ? value : -value;
    }
}

template <unsigned int dim>
class CVector
{
public:
    RealValueType data[dim];
public:
    enum {VectorDimension = dim};
public:
    CVector(void){memset(data, 0, VectorDimension * sizeof(RealValueType));}
    CVector(const RealValueType* _data){memcpy(data, _data, VectorDimension * sizeof(RealValueType));}
    CVector(const CVector &v){memcpy(data, v.data, VectorDimension * sizeof(RealValueType));}
    // do not use virtual destructor due to increasing memory space
    /*virtual*/~CVector(void){}

public:
    static CVector &GetZero()
    {
        static CVector zero;
        return zero;
    }

    CVector &operator=(const CVector &v)       // assignment
    {
        memcpy(data, v.data, VectorDimension * sizeof(RealValueType));
        return *this;
    }

    CVector &operator+= (const CVector &v)
    {
        for(unsigned int i = 0; i < VectorDimension; i++)
            data[i] += v.data[i];
        return *this;
    }
    CVector &operator-= (const CVector &v)
    {
        for(unsigned int i = 0; i < VectorDimension; i++)
            data[i] -= v.data[i];
        return *this;
    }
    CVector &operator*= (RealValueType k)
    {
        for(unsigned int i = 0; i < VectorDimension; i++)
            data[i] *= k;
        return *this;
    }
    CVector &operator/= (RealValueType k)
    {
#ifdef DEBUG
        if (k == 0.0)
            throw std::exception("Divide by Zero");
#endif
        for (unsigned int i = 0; i < VectorDimension; i++)
            data[i] /= k;
        return *this;
    }

    CVector operator+(const CVector &v) const
    {
        CVector retval;
        for (unsigned int i = 0; i < VectorDimension; i++)
            retval.data[i] = data[i] + v.data[i];
        return retval;
    }
    CVector operator-(const CVector &v) const
    {
        CVector retval;
        for (unsigned int i = 0; i < VectorDimension; i++)
            retval.data[i] = data[i] - v.data[i];
        return retval;
    }

    RealValueType operator*(const CVector &v) const  // dot product
    {
        RealValueType retval = 0.0f;
        for (unsigned int i = 0; i < VectorDimension; i++)
            retval += data[i] * v.data[i];
        return retval;
    }

    CVector operator*(RealValueType k) const
    {
        CVector<VectorDimension> retval;
        for (unsigned int i = 0; i < VectorDimension; i++)
            retval.data[i] = data[i] * k;
        return retval;
    }
    CVector operator/(RealValueType k) const
    {
#ifdef DEBUG
        if (k == 0.0)
            throw std::exception("Divide by Zero");
#endif
        CVector<VectorDimension> retval;
        for (unsigned int i = 0; i < VectorDimension; i++)
            retval.data[i] = data[i] / k;
        return retval;
    }

    CVector operator-() const
    {
        CVector retval;
        for (unsigned int i = 0; i < VectorDimension; i++)
            retval.data[i] = -data[i];
        return retval;
    }

    bool operator==(const CVector &v) const
    {
        for(unsigned int i = 0; i < VectorDimension; i++)
            if(data[i] != v.data[i])
                return false;
        return true;
    }

    bool operator!=(const CVector &v) const
    {
        for(unsigned int i = 0; i < VectorDimension; i++)
            if(data[i] != v.data[i])
                return true;
        return false;
    }

    const RealValueType &operator[](unsigned int index) const
    {
        return data[index];
    }

    RealValueType &operator[](unsigned int index)
    {
        return data[index];
    }

    const RealValueType &operator()(unsigned int index) const
    {
        return data[index];
    }

    RealValueType &operator()(unsigned int index)
    {
        return data[index];
    }

    CVector &Normalize(void)
    {
        *this /= this->Length;
        return *this;
    }

    RealValueType readonly(Length);
    RealValueType GetLength(void) const
    {
        RealValueType length_square = 0.0;
        for (unsigned int i = 0; i < VectorDimension; i++)
            length_square += data[i] * data[i];
        if (length_square == 0.0)
            return RealValueType(0.0);
        return sqrt(length_square);
    }

    RealValueType readonly(LengthSquare);
    RealValueType GetLengthSquare(void) const
    {
        {
            RealValueType length_square = 0.0f;
            for (unsigned int i = 0; i < VectorDimension; i++)
            {
                length_square += data[i] * data[i];
            }
            return length_square;
        }
    }

    RealValueType Distance(const CVector &v) const
    {
        RealValueType length_square = 0.0f;
        for (unsigned int i = 0; i < VectorDimension; i++)
            length_square += (data[i] - v.data[i]) * (data[i] - v.data[i]);
        if (length_square == 0.0)
            return 0.0;
        return sqrt(length_square);
    }

    RealValueType DistanceSquare(const CVector &v) const
    {
        RealValueType length_square = 0.0f;
        for (unsigned int i = 0; i < VectorDimension; i++)
            length_square += (data[i] - v.data[i]) * (data[i] - v.data[i]);
        return length_square;
    }

    CVector readonly(Unit);
    CVector GetUnit(void) const
    {
        CVector retval(*this);
        retval.Normalize();
        return retval;
    }

    bool TolerantEqual(const CVector &v) const
    {
        for (unsigned int i = 0; i < VectorDimension; i++)
            if (Math::Abs(data[i] - v.data[i]) > TOLERANCE)
                return false;
        return true;
    }

    bool IsSyntropic(const CVector &v) const
    {
        RealValueType dot_product = *this * v;
        RealValueType lensquare1 = this->GetLengthSquare();
        RealValueType lensquare2 = v.GetLengthSquare();
        return Math::Abs(lensquare1 * lensquare2 - dot_product * dot_product) < TOLERANCE;
    }
};

template <unsigned int dim>
std::ostream& operator << (std::ostream& output, const CVector<dim>& v)
{
    output << "(";
    for (int i = 0; i < v.VectorDimension; i++)
        output << v.data[i] << ", ";
    output << "\b\b)";
    return output;
}

class CVector4D : public CVector<4>
{
public:
    CVector4D(void){}
    CVector4D(const CVector<4> &v){memcpy(data, v.data, VectorDimension * sizeof(RealValueType));}
    CVector4D(RealValueType _x, RealValueType _y, RealValueType _z, RealValueType _w = 1.0f){data[0] = _x; data[1] = _y; data[2] = _z; data[3] = _w;}
    ~CVector4D(void){}

    RealValueType Getx() const{return data[0];}
    void Setx(RealValueType value){data[0] = value;}
    RealValueType readwrite(x);

    RealValueType Gety() const{return data[1];}
    void Sety(RealValueType value){data[1] = value;}
    RealValueType readwrite(y);

    RealValueType Getz() const{return data[2];}
    void Setz(RealValueType value){data[2] = value;}
    RealValueType readwrite(z);

    RealValueType Getw() const{return data[3];}
    void Setw(RealValueType value){data[3] = value;}
    RealValueType readwrite(w);

    CVector4D operator^(const CVector4D &v) const
    {
        return CVector4D(this->y * v.z - this->y * v.z, this->z * v.x - this->x * v.z ,this->x * v.y - this->y * v.x, 1.0f);
    }
};


typedef CVector4D vec4;

typedef vec4 Point4D;


extern vec4 operator*(RealValueType k, const vec4 &v);
