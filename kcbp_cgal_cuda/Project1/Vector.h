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

#define DEGREE_PER_RADIAN   57.2957795130823208768
#define DOUBLE_PI           6.28318530717958647692
#define PI                  3.14159265358979323846
#define HALF_PI             1.57079632679489661923


typedef double DataType;
namespace Math
{
    inline DataType Abs(DataType value)
    {
        return value > 0.0 ? value : -value;
    }
}

template <unsigned int dim>
class CVector
{
public:
    DataType data[dim];
public:
    enum {VectorDimension = dim};
public:
    CVector(void){memset(data, 0, VectorDimension * sizeof(DataType));}
    CVector(const DataType* _data){memcpy(data, _data, VectorDimension * sizeof(DataType));}
    CVector(const CVector &v){memcpy(data, v.data, VectorDimension * sizeof(DataType));}
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
        memcpy(data, v.data, VectorDimension * sizeof(DataType));
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
    CVector &operator*= (DataType k)
    {
        for(unsigned int i = 0; i < VectorDimension; i++)
            data[i] *= k;
        return *this;
    }
    CVector &operator/= (DataType k)
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

    DataType operator*(const CVector &v) const  // dot product
    {
        DataType retval = 0.0f;
        for (unsigned int i = 0; i < VectorDimension; i++)
            retval += data[i] * v.data[i];
        return retval;
    }

    CVector operator*(DataType k) const
    {
        CVector<VectorDimension> retval;
        for (unsigned int i = 0; i < VectorDimension; i++)
            retval.data[i] = data[i] * k;
        return retval;
    }
    CVector operator/(DataType k) const
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

    const DataType &operator[](unsigned int index) const
    {
        return data[index];
    }

    DataType &operator[](unsigned int index)
    {
        return data[index];
    }

    const DataType &operator()(unsigned int index) const
    {
        return data[index];
    }

    DataType &operator()(unsigned int index)
    {
        return data[index];
    }

    CVector &Normalize(void)
    {
        *this /= this->Length;
        return *this;
    }

    DataType readonly(Length);
    DataType GetLength(void) const
    {
        DataType length_square = 0.0;
        for (unsigned int i = 0; i < VectorDimension; i++)
            length_square += data[i] * data[i];
        if (length_square == 0.0)
            return DataType(0.0);
        return sqrt(length_square);
    }

    DataType readonly(LengthSquare);
    DataType GetLengthSquare(void) const
    {
        {
            DataType length_square = 0.0f;
            for (unsigned int i = 0; i < VectorDimension; i++)
            {
                length_square += data[i] * data[i];
            }
            return length_square;
        }
    }

    DataType Distance(const CVector &v) const
    {
        DataType length_square = 0.0f;
        for (unsigned int i = 0; i < VectorDimension; i++)
            length_square += (data[i] - v.data[i]) * (data[i] - v.data[i]);
        if (length_square == 0.0)
            return 0.0;
        return sqrt(length_square);
    }

    DataType DistanceSquare(const CVector &v) const
    {
        DataType length_square = 0.0f;
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
        DataType dot_product = *this * v;
        DataType lensquare1 = this->GetLengthSquare();
        DataType lensquare2 = v.GetLengthSquare();
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
    CVector4D(const CVector<4> &v){memcpy(data, v.data, VectorDimension * sizeof(DataType));}
    CVector4D(DataType _x, DataType _y, DataType _z, DataType _w = 1.0f){data[0] = _x; data[1] = _y; data[2] = _z; data[3] = _w;}
    ~CVector4D(void){}

    DataType Getx() const{return data[0];}
    void Setx(DataType value){data[0] = value;}
    DataType readwrite(x);

    DataType Gety() const{return data[1];}
    void Sety(DataType value){data[1] = value;}
    DataType readwrite(y);

    DataType Getz() const{return data[2];}
    void Setz(DataType value){data[2] = value;}
    DataType readwrite(z);

    DataType Getw() const{return data[3];}
    void Setw(DataType value){data[3] = value;}
    DataType readwrite(w);

    CVector4D operator^(const CVector4D &v) const
    {
        return CVector4D(this->y * v.z - this->y * v.z, this->z * v.x - this->x * v.z ,this->x * v.y - this->y * v.x, 1.0f);
    }
};


typedef CVector4D vec4;

typedef vec4 Point4D;


extern vec4 operator*(DataType k, const vec4 &v);