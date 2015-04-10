#include "Matrix.h"
typedef CP_Vector3D vec3;


const CMatrix CMatrix::Identity(1.0f, 0.0, 0.0f, 0.0f, 0.0f, 1.0f, 0.0, 0.0f, 0.0f, 0.0f, 1.0f, 0.0, 0.0f, 0.0f, 0.0f, 1.0f);

CMatrix::CMatrix(void)
{
    memset(m, 0, DATA_COUNT * sizeof(RealValueType));
}
CMatrix::CMatrix(const CMatrix &_m)
{
    memcpy(m, _m.m,DATA_COUNT * sizeof(RealValueType));
}
CMatrix::CMatrix(const RealValueType* data)
{
    memcpy(m, data,DATA_COUNT * sizeof(RealValueType));
}
CMatrix::CMatrix(RealValueType a11, RealValueType a12, RealValueType a13, RealValueType a14,
RealValueType a21, RealValueType a22, RealValueType a23, RealValueType a24,
RealValueType a31, RealValueType a32, RealValueType a33, RealValueType a34,
RealValueType a41, RealValueType a42, RealValueType a43, RealValueType a44
):_11(a11), _12(a12), _13(a13), _14(a14),
_21(a21), _22(a22), _23(a23), _24(a24),
_31(a31), _32(a32), _33(a33), _34(a34),
_41(a41), _42(a42), _43(a43), _44(a44){}

CMatrix::~CMatrix(void){}

CMatrix CMatrix::GetRotate(RealValueType angle, const CVector3D &axis)
{
    //
    // http://blog.csdn.net/xiajun07061225/article/details/7766838
    // this method assumes the axis pass the original point (0,0)
    //
    CMatrix retval = CMatrix::Identity;
    vec3 r(axis);
    r.mf_normalize();
    
    RealValueType cosine = cos(angle / DEGREE_PER_RADIAN);
    RealValueType sine = sin(angle / DEGREE_PER_RADIAN);
    RealValueType one_minus_consine = 1.0f - cosine;

    retval[0][0] = cosine+one_minus_consine*r.x*r.x;
    retval[0][1] = one_minus_consine*r.x*r.y - r.z*sine;
    retval[0][2] = one_minus_consine*r.x*r.z + r.y*sine;

    retval[1][0] = one_minus_consine*r.x*r.y + r.z*sine;
    retval[1][1] = cosine + one_minus_consine*r.y*r.y;
    retval[1][2] = one_minus_consine*r.y*r.z - r.x*sine;
    
    retval[2][0] = one_minus_consine*r.x*r.z - r.y*sine;
    retval[2][1] = one_minus_consine*r.y*r.z + r.x*sine;
    retval[2][2] = cosine + one_minus_consine*r.z*r.z;

    return retval;
}

void CMatrix::GetRotate(CMatrix &mat, RealValueType angle, const CVector3D &axis)
{
    //
    // http://blog.csdn.net/xiajun07061225/article/details/7766838
    // this method assumes the axis pass the original point (0,0)
    //
    vec3 r(axis);
    //r.mf_normalize();
    assert(abs(r.mf_getLengthSquare() - 1.0) < 0.0001);
    mat = CMatrix::Identity;

    RealValueType cosine = cos(angle / DEGREE_PER_RADIAN);
    RealValueType sine = sin(angle / DEGREE_PER_RADIAN);
    RealValueType one_minus_consine = 1.0f - cosine;

    mat[0][0] = cosine+one_minus_consine*r.x*r.x;
    mat[0][1] = one_minus_consine*r.x*r.y + r.z*sine;
    mat[0][2] = one_minus_consine*r.x*r.z - r.y*sine;

    mat[1][0] = one_minus_consine*r.x*r.y - r.z*sine;
    mat[1][1] = cosine + one_minus_consine*r.y*r.y;
    mat[1][2] = one_minus_consine*r.y*r.z + r.x*sine;

    mat[2][0] = one_minus_consine*r.x*r.z + r.y*sine;
    mat[2][1] = one_minus_consine*r.y*r.z - r.x*sine;
    mat[2][2] = cosine + one_minus_consine*r.z*r.z;
}

CMatrix CMatrix::GetRotateXAxis(RealValueType angle)
{
    CMatrix retval = CMatrix::Identity;

    RealValueType cosine = cos(angle / DEGREE_PER_RADIAN);
    RealValueType sine = sin(angle / DEGREE_PER_RADIAN);

    retval[1][1] = cosine;
    retval[1][2] = -sine;
    retval[2][1] = sine;
    retval[2][2] = cosine;

    return retval;
}

void CMatrix::GetRotateXAxis(CMatrix &mat, RealValueType angle)
{
    mat = CMatrix::Identity;

    RealValueType cosine = cos(angle / DEGREE_PER_RADIAN);
    RealValueType sine = sin(angle / DEGREE_PER_RADIAN);

    mat[1][1] = cosine;
    mat[1][2] = -sine;
    mat[2][1] = sine;
    mat[2][2] = cosine;
}

CMatrix CMatrix::GetRotateYAxis(RealValueType angle)
{
    CMatrix retval = CMatrix::Identity;

    RealValueType cosine = cos(angle / DEGREE_PER_RADIAN);
    RealValueType sine = sin(angle / DEGREE_PER_RADIAN);

    retval[0][0] = cosine;
    retval[0][2] = sine;
    retval[2][0] = -sine;
    retval[2][2] = cosine;

    return retval;
}

void CMatrix::GetRotateYAxis(CMatrix &mat, RealValueType angle)
{
    mat = CMatrix::Identity;

    RealValueType cosine = cos(angle / DEGREE_PER_RADIAN);
    RealValueType sine = sin(angle / DEGREE_PER_RADIAN);

    mat[0][0] = cosine;
    mat[0][2] = sine;
    mat[2][0] = -sine;
    mat[2][2] = cosine;
}

CMatrix CMatrix::GetRotateZAxis(RealValueType angle)
{
    CMatrix retval = CMatrix::Identity;

    RealValueType cosine = cos(angle / DEGREE_PER_RADIAN);
    RealValueType sine = sin(angle / DEGREE_PER_RADIAN);

    retval[0][0] = cosine;
    retval[0][1] = -sine;
    retval[1][0] = sine;
    retval[1][1] = cosine;

    return retval;
}

void CMatrix::GetRotateZAxis(CMatrix &mat, RealValueType angle)
{
    mat = CMatrix::Identity;

    RealValueType cosine = cos(angle / DEGREE_PER_RADIAN);
    RealValueType sine = sin(angle / DEGREE_PER_RADIAN);

    mat[0][0] = cosine;
    mat[0][1] = -sine;
    mat[1][0] = sine;
    mat[1][1] = cosine;
}

CMatrix CMatrix::GetTranslate(const CVector3D &translate)
{
    CMatrix retval = CMatrix::Identity;

    retval[0][3] = translate.x;
    retval[1][3] = translate.y;
    retval[2][3] = translate.z;

    return retval;
}

void CMatrix::GetTranslate(CMatrix &mat, const CVector3D &translate)
{
    mat = CMatrix::Identity;

    mat[0][3] = translate.x;
    mat[1][3] = translate.y;
    mat[2][3] = translate.z;
}

CMatrix CMatrix::GetScale(const CVector3D &scale)
{
    CMatrix retval = CMatrix::Identity;

    retval[0][0] = scale.x;
    retval[1][1] = scale.y;
    retval[2][2] = scale.z;

    return retval;
}

void CMatrix::GetScale(CMatrix &mat, const CVector3D &scale)
{
    mat = CMatrix::Identity;

    mat[0][0] = scale.x;
    mat[1][1] = scale.y;
    mat[2][2] = scale.z;
}

CMatrix &CMatrix::operator=(const CMatrix &_m)
{
    for (unsigned int i = 0; i < DATA_COUNT; i++)
    {
        m[i] = _m.m[i];
    }
    return *this;
}

CMatrix &CMatrix::operator+=(const CMatrix &_m)
{
    for (unsigned int i = 0; i < DATA_COUNT; i++)
    {
        m[i] += _m.m[i];
    }
    return *this;
}

CMatrix &CMatrix::operator-=(const CMatrix &_m)
{
    for (unsigned int i = 0; i < DATA_COUNT; i++)
    {
        m[i] -= _m.m[i];
    }
    return *this;
}

CMatrix &CMatrix::operator*=(const CMatrix &_m)
{
    CMatrix retval;
    for (unsigned int i = 0; i < ROW_COUNT; i++)
    {
        for (unsigned int j = 0; j < COLUMN_COUNT; j++)
        {
            retval.m[i * COLUMN_COUNT + j] = 0.0f;
            for (unsigned int k = 0; k < COLUMN_COUNT; k++)
            {
                retval.m[i * COLUMN_COUNT + j] += m[i * COLUMN_COUNT + k] * _m.m[k * COLUMN_COUNT + j];
            }
        }
    }
    memcpy(m, retval.m, DATA_COUNT * sizeof(RealValueType));
    return *this;
}

CMatrix CMatrix::operator+(const CMatrix &_m) const
{
    CMatrix retval;
    for (unsigned int i = 0; i < DATA_COUNT; i++)
    {
        retval.m[i] = m[i] + _m.m[i];
    }
    return retval;
}

CMatrix CMatrix::operator-(const CMatrix &_m) const
{
    CMatrix retval;
    for (unsigned int i = 0; i < DATA_COUNT; i++)
    {
        retval.m[i] = m[i] - _m.m[i];
    }
    return retval;
}

CMatrix CMatrix::operator*(const CMatrix &_m) const
{
    CMatrix retval;
    for (unsigned int i = 0; i < ROW_COUNT; i++)
    {
        for (unsigned int j = 0; j < COLUMN_COUNT; j++)
        {
            retval.m[i * COLUMN_COUNT + j] = 0.0f;
            for (unsigned int k = 0; k < COLUMN_COUNT; k++)
            {
                retval.m[i * COLUMN_COUNT + j] += m[i * COLUMN_COUNT + k] * _m.m[k * COLUMN_COUNT + j];
            }
        }
    }
    return retval;
}

CVector4D CMatrix::operator*(const CVector4D &v) const
{
    CVector4D retval;
    for (unsigned int i = 0; i < ROW_COUNT; i++)
    {
        retval.data[i] = 0.0f;
        for (unsigned int j = 0; j < COLUMN_COUNT; j++)
        {
            retval.data[i] += m[i * COLUMN_COUNT + j] * v.data[j];
        }
    }
    return retval;
}

CVector3D CMatrix::operator*(const CVector3D &v) const
{
    CVector4D retval;
    CVector4D v_copy(v.x, v.y, v.z);
    for (unsigned int i = 0; i < ROW_COUNT; i++)
    {
        retval.data[i] = 0.0f;
        for (unsigned int j = 0; j < COLUMN_COUNT; j++)
        {
            retval.data[i] += m[i * COLUMN_COUNT + j] * v_copy.data[j];
        }
    }
    retval /= retval.w;
    return CVector3D(retval.data[0], retval.data[1], retval.data[2]);
}

CMatrix CMatrix::operator*(RealValueType k) const
{
    CMatrix retval;
    for (unsigned int i = 0; i < DATA_COUNT; i++)
    {
        retval.m[i] = m[i] * k;
    }
    return retval;
}

CMatrix CMatrix::operator/(RealValueType k) const
{
    CMatrix retval;
    for (unsigned int i = 0; i < DATA_COUNT; i++)
    {
        retval.m[i] = m[i] / k;
    }
    return retval;
}

CMatrix CMatrix::operator-() const
{
    CMatrix retval;
    for (unsigned int i = 0; i < DATA_COUNT; i++)
    {
        retval.m[i] = -m[i];
    }
    return retval;
}

const RealValueType &CMatrix::operator()(unsigned int row, unsigned int column) const
{
    return m[row * COLUMN_COUNT + column];
}

const CMatrix::RowVector &CMatrix::operator()(unsigned int row) const
{
    return rowvec[row];
}

const CMatrix::RowVector &CMatrix::operator[](unsigned int row) const
{
    return rowvec[row];
}

RealValueType &CMatrix::operator()(unsigned int row, unsigned int column)
{
    return m[row * COLUMN_COUNT + column];
}

CMatrix::RowVector &CMatrix::operator()(unsigned int row)
{
    return rowvec[row];
}

CMatrix::RowVector &CMatrix::operator[](unsigned int row)
{
    return rowvec[row];
}