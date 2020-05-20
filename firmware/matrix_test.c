#include <stdio.h>
#include <string.h>
#include <math.h>

void Matrix3x3Set(float m[3][3],
    float m00, float m01, float m02,
    float m10, float m11, float m12,
    float m20, float m21, float m22)
{
    m[0][0] = m00;
    m[0][1] = m01;
    m[0][2] = m02;
    m[1][0] = m10;
    m[1][1] = m11;
    m[1][2] = m12;
    m[2][0] = m20;
    m[2][1] = m21;
    m[2][2] = m22;
}

void Matrix3x3MultMatrix3x3(float m1[3][3], float m2[3][3], float r[3][3])
{
    float t[3][3];
    int i, j;

    for(j = 0; j < 3; j++)
        for(i = 0; i < 3; i++)
           t[i][j] = m1[i][0] * m2[0][j] +
               m1[i][1] * m2[1][j] +
               m1[i][2] * m2[2][j];

    memcpy(r, t, sizeof(t));
}

void Vector3MultMatrix3x3(float v[2], float m[3][3], float r[2])
{
    float t[2];
    t[0] = v[0]* m[0][0] + v[1] * m[1][0] + m[2][0];
    t[1] = v[0]* m[0][1] + v[1] * m[1][1] + m[2][1];
    r[0] = t[0];
    r[1] = t[1];
}

int main(int argc, char **argv)
{
    float x0 = 20;
    float y0 = 15;
    float x1 = 70;
    float y1 = 65;

    float translate[3][3];
    float rotate[3][3];
    float scale[3][3];

    float combined[3][3];

    float A = -atan2(y1 - y0, x1 - x0);

    float dx = x1 - x0;
    float dy = y1 - y0;
    float S = 1.0f / sqrtf(dx * dx + dy * dy);

    // matrix to take xy0-xy1 and put them on X axis 0 and 1 is as follows:
    // translate to origin (-x0, -y0) in row 2
    Matrix3x3Set(translate, 1,   0,   0,
                   0,   1,   0,
                   -x0, -y0, 1);


    // rotate by A = -atan(x1 - x0, y1 - y0)
    Matrix3x3Set(rotate,  cos(A), sin(A),  0,
                -sin(A), cos(A),  0,
                 0,      0,       1);

    // cos(A)       sin(A)          0
    // -sin(A)      cos(A)          0
    // -x0          -y0             1

    // scale X by S = 1 / length(xy0, xy1);
    Matrix3x3Set(scale, S,  0,  0,
               0,  1,  0,
               0,  0,  1);

    // S * cos(A)       S * sin(A)      0
    // -sin(A)          cos(A)          0
    // S * -x0          -y0             1

    //    [  S *  cos(A)     S * sin(A)        0  ]
    //    [  -sin(A)     cos(A)        0  ]
    //    [  -x0 * (S * cos(A) + S * -sin(A))    -y0 * (S * sin(A) + S * cos(A))                 1  ]

    Matrix3x3Set(combined, S * cos(A),         S * sin(A),           0,
                  -sin(A),        cos(A),           0,
                  -x0 * S,   -y0,     1);

    // S * cos(A)               sin(A)          0
    // S * -sin(A)              cos(A)          0
    // S * cos(A) * -x0         -y0 * cos(A)    1

    float x = (x0 + x1) / 2;
    float y = (y0 + y1) / 2;

    float v[3];

    v[0] = x; v[1] = y; v[2] = 1;
    Vector3MultMatrix3x3(v, translate, v);
    Vector3MultMatrix3x3(v, rotate, v);
    Vector3MultMatrix3x3(v, scale, v);
    printf("separate : %f %f (expecting .5, 0)\n", v[0], v[1]);

    v[0] = x; v[1] = y; v[2] = 1;
    Vector3MultMatrix3x3(v, combined, v);
    printf("combined : %f %f (expecting .5, 0)\n", v[0], v[1]);

    memset(combined, 0, sizeof(combined));
    combined[0][0] = 1;
    combined[1][1] = 1;
    combined[2][2] = 1;
    Matrix3x3MultMatrix3x3(combined, translate, combined);
    Matrix3x3MultMatrix3x3(combined, rotate, combined);
    Matrix3x3MultMatrix3x3(combined, scale, combined);

    v[0] = x; v[1] = y; v[2] = 1;
    Vector3MultMatrix3x3(v, combined, v);
    printf("combined : %f %f (expecting .5, 0)\n", v[0], v[1]);
}
