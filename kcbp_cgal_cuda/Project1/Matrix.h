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
        DataType data[COLUMN_COUNT];
        operator CVector4D() const {return CVector4D(data);}
        DataType &operator()(unsigned index) {return data[index];}
        const DataType &operator()(unsigned index) const {return data[index];}
        DataType &operator[](unsigned index){return data[index];}
        const DataType &operator[](unsigned index) const { return data[index];}
    };

    union{
        DataType m[DATA_COUNT];
        struct{
            DataType _11, _12, _13, _14;
            DataType _21, _22, _23, _24;
            DataType _31, _32, _33, _34;
            DataType _41, _42, _43, _44;
        };
        RowVector rowvec[ROW_COUNT];
    };

public:
    CMatrix(void);
    CMatrix(const CMatrix &_m);
    CMatrix(const DataType* data);
    CMatrix(DataType a11, DataType a12, DataType a13, DataType a14,
        DataType a21, DataType a22, DataType a23, DataType a24,
        DataType a31, DataType a32, DataType a33, DataType a34,
        DataType a41, DataType a42, DataType a43, DataType a44
        );
    ~CMatrix(void);

public:
    static const CMatrix Identity;
    static CMatrix GetRotate(DataType angle, const CVector3D &axis);
    static void GetRotate(CMatrix &mat, DataType angle, const CVector3D &axis);

    static CMatrix GetRotateXAxis(DataType angle);
    static void GetRotateXAxis(CMatrix &mat, DataType angle);

    static CMatrix GetRotateYAxis(DataType angle);
    static void GetRotateYAxis(CMatrix &mat, DataType angle);

    static CMatrix GetRotateZAxis(DataType angle);
    static void GetRotateZAxis(CMatrix &mat, DataType angle);

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

    CMatrix operator*(DataType k) const;
    CMatrix operator/(DataType k) const;

    CMatrix operator-() const;

    const DataType &operator()(unsigned int row, unsigned int column) const;
    const RowVector &operator()(unsigned int row) const;
    const RowVector &operator[](unsigned int row) const;

    DataType &operator()(unsigned int row, unsigned int column);
    RowVector &operator()(unsigned int row);
    RowVector &operator[](unsigned int row);
};

typedef CMatrix mat4;