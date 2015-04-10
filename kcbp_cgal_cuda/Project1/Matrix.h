#pragma once
#include "Vector.h"
#include "CP_PointVector.h"
typedef CP_Vector3D CVector3D;
//
// 4*4 matrix
//
class CMatrix
{
public:
    enum{
        DATA_COUNT = 16,
        ROW_COUNT = 4,
        COLUMN_COUNT = 4
    };

    //
    // this sub class represent a row of the matrix,
    // to modify the row vector is to modify the matrix (with reference)
    //
    struct RowVector{
        RealValueType data[COLUMN_COUNT];
        //operator CVector4D() const {return CVector4D(data);}
        RealValueType &operator()(unsigned index) {return data[index];}
        const RealValueType &operator()(unsigned index) const {return data[index];}
        RealValueType &operator[](unsigned index){return data[index];}
        const RealValueType &operator[](unsigned index) const { return data[index];}
    };

    union{
        RealValueType m[DATA_COUNT];
        struct{
            RealValueType _11, _12, _13, _14;
            RealValueType _21, _22, _23, _24;
            RealValueType _31, _32, _33, _34;
            RealValueType _41, _42, _43, _44;
        };
        RowVector rowvec[ROW_COUNT];
    };

public:
    CMatrix(void);
    CMatrix(const CMatrix &_m);
    CMatrix(const RealValueType* data);
    CMatrix(RealValueType a11, RealValueType a12, RealValueType a13, RealValueType a14,
        RealValueType a21, RealValueType a22, RealValueType a23, RealValueType a24,
        RealValueType a31, RealValueType a32, RealValueType a33, RealValueType a34,
        RealValueType a41, RealValueType a42, RealValueType a43, RealValueType a44
        );
    ~CMatrix(void);

public:
    static const CMatrix Identity;
    static CMatrix GetRotate(RealValueType angle, const CVector3D &axis);
    static void GetRotate(CMatrix &mat, RealValueType angle, const CVector3D &axis);

    static CMatrix GetRotateXAxis(RealValueType angle);
    static void GetRotateXAxis(CMatrix &mat, RealValueType angle);

    static CMatrix GetRotateYAxis(RealValueType angle);
    static void GetRotateYAxis(CMatrix &mat, RealValueType angle);

    static CMatrix GetRotateZAxis(RealValueType angle);
    static void GetRotateZAxis(CMatrix &mat, RealValueType angle);

    static CMatrix GetTranslate(const CVector3D &translate);
    static void GetTranslate(CMatrix &mat, const CVector3D &translate);

    static CMatrix GetScale(const CVector3D &scale);
    static void GetScale(CMatrix &mat, const CVector3D &scale);

    bool IsIdentity()
    {
        if(_11 != 1.0 && _22 != 1.0 && _33 != 1.0 && _44 != 1.0) 
            return false;
        if(_12 != 0 && _13 != 0 && _14 != 0)
            return false;
        if(_21 != 0 && _23 != 0 && _24 != 0)
            return false;
        if(_31 != 0 && _32 != 0 && _34 != 0)
            return false;
        if(_41 != 0 && _42 != 0 && _43 != 0)
            return false;
        return true;
    }

    bool IsIdentity() const
    {
        if(_11 != 1.0 && _22 != 1.0 && _33 != 1.0 && _44 != 1.0) 
            return false;
        if(_12 != 0 && _13 != 0 && _14 != 0)
            return false;
        if(_21 != 0 && _23 != 0 && _24 != 0)
            return false;
        if(_31 != 0 && _32 != 0 && _34 != 0)
            return false;
        if(_41 != 0 && _42 != 0 && _43 != 0)
            return false;
        return true;
    }

public:
    
    CMatrix &operator=(const CMatrix &_m);
    CMatrix &operator+=(const CMatrix &_m);
    CMatrix &operator-=(const CMatrix &_m);
    CMatrix &operator*=(const CMatrix &_m);

    CMatrix operator+(const CMatrix &_m) const;
    CMatrix operator-(const CMatrix &_m) const;
    CMatrix operator*(const CMatrix &_m) const;

    CVector4D operator*(const CVector4D &v) const;
    CVector3D operator*(const CVector3D &v) const;

    CMatrix operator*(RealValueType k) const;
    CMatrix operator/(RealValueType k) const;

    CMatrix operator-() const;

    const RealValueType &operator()(unsigned int row, unsigned int column) const;
    const RowVector &operator()(unsigned int row) const;
    const RowVector &operator[](unsigned int row) const;

    RealValueType &operator()(unsigned int row, unsigned int column);
    RowVector &operator()(unsigned int row);
    RowVector &operator[](unsigned int row);

    CMatrix transpose()
    {
        return CMatrix(_11, _21, _31, _41,
                       _12, _22, _32, _42,
                       _13, _23, _33, _43,
                       _14, _24, _34, _44);
    }
};

typedef CMatrix mat4;