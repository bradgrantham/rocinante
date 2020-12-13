#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cassert>

#ifndef M_PI
#define M_PI 3.1415926
#endif // M_PI

// #include "raster.h"
// #include "config.h"
#include "pocketgl.h"


//----------------------------------------------------------------------------
// vector algebra

template <class T>
struct vec4
{
    T v[4];
    typedef T type;
    static const int ordinal = 4;
    operator T*(void) { return v; }
    operator const T*(void) const { return v; }
    void set(const T* p)
    {
        v[0] = p[0];
        v[1] = p[1];
        v[2] = p[2];
        v[3] = p[3];
    }
    void set(T x, T y, T z, T w)
    {
        v[0] = x;
        v[1] = y;
        v[2] = z;
        v[3] = w;
    }
    vec4(const T* p)
    {
        set(p);
    }
    vec4(T x, T y, T z, T w)
    {
        set(x, y, z, w);
    }
    vec4() {}
    T length() const
    {
        return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
    }
};

template <class T>
vec4<T> operator+(const vec4<T>& v1, const vec4<T>& v2)
{
    return vec4<T>(v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2], v1[3] + v2[3]);
}

template <class T>
vec4<T> operator-(const vec4<T>& v1, const vec4<T>& v2)
{
    return vec4<T>(v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2], v1[3] - v2[3]);
}

template <class T>
vec4<T> operator*(const vec4<T>& v1, const vec4<T>& v2)
{
    return vec4<T>(v1[0] * v2[0], v1[1] * v2[1], v1[2] * v2[2], v1[3] * v2[3]);
}

template <class T>
vec4<T> operator*(const vec4<T>& v1, T v2)
{
    return vec4<T>(v1[0] * v2, v1[1] * v2, v1[2] * v2, v1[3] * v2);
}

template <class T>
struct vec3
{
    T v[3];
    typedef T type;
    static const int ordinal = 3;
    operator T*(void) { return v; }
    operator const T*(void) const { return v; }
    void set(const T* p)
    {
        v[0] = p[0];
        v[1] = p[1];
        v[2] = p[2];
    }
    void set(T x, T y, T z)
    {
        v[0] = x;
        v[1] = y;
        v[2] = z;
    }
    vec3(const T* p)
    {
        set(p);
    }
    vec3(T x, T y, T z)
    {
        set(x, y, z);
    }
    vec3() {}
    T length() const
    {
        return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }
};

template <class T>
struct matrix4x4
{
    T v[16];
    typedef T type;
    static const int ordinal = 16;
    operator T*(void) { return v; }
    operator const T*(void) const { return v; }
    matrix4x4() {}
    void set(const T* p)
    {
        for(int i = 0; i < ordinal; i++)
            v[i] = p[i];
    }
    matrix4x4(
        T m0, T m1, T m2, T m3,
        T m4, T m5, T m6, T m7,
        T m8, T m9, T m10, T m11,
        T m12, T m13, T m14, T m15)
    {
        v[0] = m0;
        v[1] = m1;
        v[2] = m2;
        v[3] = m3;
        v[4] = m4;
        v[5] = m5;
        v[6] = m6;
        v[7] = m7;
        v[8] = m8;
        v[9] = m9;
        v[10] = m10;
        v[11] = m11;
        v[12] = m12;
        v[13] = m13;
        v[14] = m14;
        v[15] = m15;
    }
    matrix4x4(const T* p)
    {
        set(p);
    }
    T determinant()
    {
        return (v[0] * v[5] - v[1] * v[4]) *
            (v[10] * v[15] - v[11] * v[14]) + 
            (v[2] * v[4] - v[0] * v[6]) *
            (v[9] * v[15] - v[11] * v[13]) + 
            (v[0] * v[7] - v[3] * v[4]) *
            (v[9] * v[14] - v[10] * v[13]) + 
            (v[1] * v[6] - v[2] * v[5]) *
            (v[8] * v[15] - v[11] * v[12]) + 
            (v[3] * v[5] - v[1] * v[7]) *
            (v[8] * v[14] - v[10] * v[12]) + 
            (v[2] * v[7] - v[3] * v[6]) *
            (v[8] * v[13] - v[9] * v[12]);
    }

};

typedef matrix4x4<float> matrix4x4f;
typedef vec3<float> vec3f;
typedef vec4<float> vec4f;
typedef unsigned short vec3us[3];
typedef unsigned char vec3ub[3];

const matrix4x4f identity_4x4f(
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
);

/*
pow
*/


template <class T>
float vec_dot(const T& i1, const T& i2)
{
    typename T::type dot = 0;
    for(int i = 0; i < T::ordinal; i++)
        dot += i1[i] * i2[i];
    return dot;
}

template <class T>
T vec_normalize(const T& i1)
{
    T out;
    float d = 1.0f / i1.length();
    for(int i = 0; i < T::ordinal; i++)
        out[i] = i1[i] * d;
    return out;
}

template <class T>
static void matrix_mult_matrix(const T& m1, const T& m2, T& r)
{
    T t;
    int i, j;

    for(j = 0; j < 4; j++)
        for(i = 0; i < 4; i++)
           t[i * 4 + j] = m1[i * 4 + 0] * m2[0 * 4 + j] +
               m1[i * 4 + 1] * m2[1 * 4 + j] +
               m1[i * 4 + 2] * m2[2 * 4 + j] +
               m1[i * 4 + 3] * m2[3 * 4 + j];

    r = t;
}

#define EPSILON .0000001

static int matrix4x4f_invert(const matrix4x4f& mat, matrix4x4f& inv)
{
    int		i, rswap;
    float	det, div, swap;
    matrix4x4f	hold;

    hold = mat;
    inv = identity_4x4f;
    det = hold.determinant();
    if(fabs(det) < EPSILON) /* singular? */
	return -1;

    rswap = 0;
    /* this loop isn't entered unless [0 + 0] > EPSILON and det > EPSILON,
	 so rswap wouldn't be 0, but I initialize so as not to get warned */
    if(fabs(hold[0]) < EPSILON)
    {
        if(fabs(hold[1]) > EPSILON)
            rswap = 1;
        else if(fabs(hold[2]) > EPSILON)
	    rswap = 2;
        else if(fabs(hold[3]) > EPSILON)
	    rswap = 3;

        for(i = 0; i < 4; i++)
	{
            swap = hold[i * 4 + 0];
            hold[i * 4 + 0] = hold[i * 4 + rswap];
            hold[i * 4 + rswap] = swap;

            swap = inv[i * 4 + 0];
            inv[i * 4 + 0] = inv[i * 4 + rswap];
            inv[i * 4 + rswap] = swap;
        }
    }
        
    div = hold[0];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 0] /= div;
        inv[i * 4 + 0] /= div;
    }

    div = hold[1];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 1] -= div * hold[i * 4 + 0];
        inv[i * 4 + 1] -= div * inv[i * 4 + 0];
    }
    div = hold[2];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 2] -= div * hold[i * 4 + 0];
        inv[i * 4 + 2] -= div * inv[i * 4 + 0];
    }
    div = hold[3];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 3] -= div * hold[i * 4 + 0];
        inv[i * 4 + 3] -= div * inv[i * 4 + 0];
    }

    if(fabs(hold[5]) < EPSILON){
        if(fabs(hold[6]) > EPSILON)
	    rswap = 2;
        else if(fabs(hold[7]) > EPSILON)
	    rswap = 3;

        for(i = 0; i < 4; i++)
	{
            swap = hold[i * 4 + 1];
            hold[i * 4 + 1] = hold[i * 4 + rswap];
            hold[i * 4 + rswap] = swap;

            swap = inv[i * 4 + 1];
            inv[i * 4 + 1] = inv[i * 4 + rswap];
            inv[i * 4 + rswap] = swap;
        }
    }

    div = hold[5];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 1] /= div;
        inv[i * 4 + 1] /= div;
    }

    div = hold[4];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 0] -= div * hold[i * 4 + 1];
        inv[i * 4 + 0] -= div * inv[i * 4 + 1];
    }
    div = hold[6];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 2] -= div * hold[i * 4 + 1];
        inv[i * 4 + 2] -= div * inv[i * 4 + 1];
    }
    div = hold[7];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 3] -= div * hold[i * 4 + 1];
        inv[i * 4 + 3] -= div * inv[i * 4 + 1];
    }

    if(fabs(hold[10]) < EPSILON){
        for(i = 0; i < 4; i++)
	{
            swap = hold[i * 4 + 2];
            hold[i * 4 + 2] = hold[i * 4 + 3];
            hold[i * 4 + 3] = swap;

            swap = inv[i * 4 + 2];
            inv[i * 4 + 2] = inv[i * 4 + 3];
            inv[i * 4 + 3] = swap;
        }
    }

    div = hold[10];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 2] /= div;
        inv[i * 4 + 2] /= div;
    }

    div = hold[8];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 0] -= div * hold[i * 4 + 2];
        inv[i * 4 + 0] -= div * inv[i * 4 + 2];
    }
    div = hold[9];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 1] -= div * hold[i * 4 + 2];
        inv[i * 4 + 1] -= div * inv[i * 4 + 2];
    }
    div = hold[11];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 3] -= div * hold[i * 4 + 2];
        inv[i * 4 + 3] -= div * inv[i * 4 + 2];
    }

    div = hold[15];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 3] /= div;
        inv[i * 4 + 3] /= div;
    }

    div = hold[12];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 0] -= div * hold[i * 4 + 3];
        inv[i * 4 + 0] -= div * inv[i * 4 + 3];
    }
    div = hold[13];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 1] -= div * hold[i * 4 + 3];
        inv[i * 4 + 1] -= div * inv[i * 4 + 3];
    }
    div = hold[14];
    for(i = 0; i < 4; i++)
    {
        hold[i * 4 + 2] -= div * hold[i * 4 + 3];
        inv[i * 4 + 2] -= div * inv[i * 4 + 3];
    }
    
    return 0;
}

static void matrix4x4f_mult_vec4f(const matrix4x4f m, const vec4f& in, vec4f& out)
{
    int i;
    vec4f t;

    for(i = 0; i < 4; i++)
        t[i] =
            m[0 + i] * in[0] +
            m[4 + i] * in[1] +
            m[8 + i] * in[2] +
            m[12 + i] * in[3];
    out = t;
}

static void vec3f_mult_matrix4x4f(const vec3f& in, const matrix4x4f m, vec3f& out)
{
    int i;
    vec3f t;

    for(i = 0; i < 3; i++)
        t[i] =
            in[0] * m[0 + i * 4] +
            in[1] * m[1 + i * 4] +
            in[2] * m[2 + i * 4];
    out = t;
}

#if 0
static void vec4f_mult_matrix4x4f(const vec4f& in, const matrix4x4f m, vec4f& out)
{
    int i;
    vec4f t;

    for(i = 0; i < 4; i++)
        t[i] =
            in[0] * m[0 + i * 4] +
            in[1] * m[1 + i * 4] +
            in[2] * m[2 + i * 4] +
            in[3] * m[3 + i * 4];
    out = t;
}
#endif

struct matrix_stack
{
private:

    static const int max = 4;
    matrix4x4f stack[max];

    int stack_top;

    bool inverse_needs_calculation;
    matrix4x4f inverse;

public:

    matrix_stack() : stack_top(0), inverse_needs_calculation(true)  {}

    float *top() { return stack[stack_top]; }

    float *get_inverse()
    {
        if(inverse_needs_calculation) {
            matrix4x4f_invert(stack[stack_top], inverse);
            inverse_needs_calculation = false;
        }
        return inverse;
    }

    bool push()
    {
        if(stack_top == max - 1) {
            return false;
        } else {
            stack[stack_top + 1] = stack[stack_top];
            inverse_needs_calculation = true;
            stack_top++;
            return true;
        }
    }

    bool pop()
    {
        if(stack_top == 0) {
            return false;
        } else {
            stack_top--;
            inverse_needs_calculation = true;
            return true;
        }
    }

    void load(const matrix4x4f m)
    {
        stack[stack_top] = m;
        inverse_needs_calculation = true;
    }

    void mult(const matrix4x4f m)
    {
        matrix_mult_matrix(m, stack[stack_top], stack[stack_top]);
        inverse_needs_calculation = true;
    }
};


//----------------------------------------------------------------------------
// GL state

static vec4f clear_color;

static int viewport_x = 0;
static int viewport_y = 0;
static int viewport_width = 128;
static int viewport_height = 128;

void set_initial_viewport(int width, int height)
{
    viewport_width = width;
    viewport_height = height;
}

struct light {
    vec4f position;
    vec4f diffuse;
    vec4f specular;
    vec4f diffuse_mtl_scaled;
    vec4f specular_mtl_scaled;
    bool enabled;
};

static constexpr int light_max = 4;
static light lights[light_max];

struct material {
    vec4f diffuse;
    vec4f ambient;
    vec4f specular;
    float shininess;
};

static material current_material;
static bool color_material_enabled;

struct vertex_array_descriptor {
    int size;
    int type;
    int stride;
    void* ptr;
};

static vertex_array_descriptor vertex_array;
static vertex_array_descriptor normal_array;

static bool cull_face = false;
static bool normalize_enabled = false;
static bool vertex_client_array_enabled = false;
static bool normal_client_array_enabled = false;
static bool lighting_enabled = false;


static matrix_stack modelview_stack;
static matrix_stack projection_stack;
static matrix_stack *current_stack = &modelview_stack;

static int gl_error = 0;

static void set_gl_error(int e)
{
    if(gl_error == 0)
        gl_error = e;
}

GLenum pglGetError()
{
    int e = gl_error;
    gl_error = 0;
    return e;
}

void pglLoadIdentity()
{
    current_stack->load(identity_4x4f);
}

void pglPushMatrix()
{
    if(!current_stack->push())
        set_gl_error(GL_STACK_OVERFLOW);
}

void pglPopMatrix()
{
    if(!current_stack->pop())
        set_gl_error(GL_STACK_UNDERFLOW);
}

void pglMultMatrixf(const float m[16])
{
    current_stack->mult(m);
}

void pglScalef(float x, float y, float z)
{
    matrix4x4f m;

    m = identity_4x4f;
    m[0] = x;
    m[5] = y;
    m[10] = z;

    current_stack->mult(m);
}

void pglTranslatef(float x, float y, float z)
{
    matrix4x4f m;

    m = identity_4x4f;
    m[12] = x;
    m[13] = y;
    m[14] = z;

    current_stack->mult(m);
}

void pglRotatef(float angle, float x, float y, float z)
{
    matrix4x4f m;

    m = identity_4x4f;

    float s;
    float c;
    float t;
    float d;

    angle = angle / 180 * M_PI;
    d = sqrtf(x * x + y * y + z * z);
    x /= d;
    y /= d;
    z /= d;

    /*
     * Rotation around axis from Graphics Gems 1, p.466
     */
    s = sinf(angle);
    c = cosf(angle);
    t = 1 - cosf(angle);
    m[0] = t * x * x + c;
    m[5] = t * y * y + c;
    m[10] = t * z * z + c;
    m[1] = t * x * y + s * z;
    m[4] = t * x * y - s * z;
    m[2] = t * x * z - s * y;
    m[8] = t * x * z + s * y;
    m[6] = t * y * z + s * x;
    m[9] = t * y * z - s * x;

    current_stack->mult(m);
}

void pglFrustum(float left, float right, float bottom, float top, float near, float far)
{
    matrix4x4f m;

    m[0] = 2 * near / (right - left);
    m[1] = 0;
    m[2] = 0;
    m[3] = 0;

    m[4] = 0;
    m[5] = 2 * near / (top - bottom);
    m[6] = 0;
    m[7] = 0;

    m[8] = (right + left) / (right - left);
    m[9] = (top + bottom) / (top - bottom);
    m[10] = - (far + near) / (far - near);
    m[11] = -1;

    m[12] = 0;
    m[13] = 0;
    m[14] = - 2 * far * near / (far - near);
    m[15] = 0;

    current_stack->mult(m);
}

void pglMatrixMode(GLenum which)
{
    switch(which) {
        case GL_MODELVIEW:
            current_stack = &modelview_stack;
            break;
        case GL_PROJECTION:
            current_stack = &projection_stack;
            break;
        default:
            set_gl_error(GL_INVALID_ENUM);
            break;
    }
}

void pglEnableClientState(int enumerant)
{
    switch(enumerant)
    {
        case GL_VERTEX_ARRAY:
            vertex_client_array_enabled = true;
            break;
        case GL_NORMAL_ARRAY:
            normal_client_array_enabled = true;
            break;
        default:
            set_gl_error(GL_INVALID_ENUM);
            break;
    }
}

void pglDisableClientState(int enumerant)
{
    switch(enumerant)
    {
        case GL_COLOR_MATERIAL:
            color_material_enabled = false;
            break;
        case GL_VERTEX_ARRAY:
            vertex_client_array_enabled = false;
            break;
        case GL_NORMAL_ARRAY:
            normal_client_array_enabled = false;
            break;
        default:
            set_gl_error(GL_INVALID_ENUM);
            break;
    }
}

void pglEnable(int enumerant)
{
    switch(enumerant)
    {
        case GL_COLOR_MATERIAL:
            color_material_enabled = true;
            break;
        case GL_LIGHTING:
            lighting_enabled = true;
            break;
        case GL_CULL_FACE:
            cull_face = true;
            break;
        case GL_NORMALIZE:
            normalize_enabled = true;
            break;
        case GL_LIGHT0:
        case GL_LIGHT1:
        case GL_LIGHT2:
        case GL_LIGHT3:
        case GL_LIGHT4:
        case GL_LIGHT5:
        case GL_LIGHT6:
        case GL_LIGHT7:
            lights[enumerant - GL_LIGHT0].enabled = true;
            break;
        default:
            set_gl_error(GL_INVALID_ENUM);
            break;
    }
}

void pglDisable(int enumerant)
{
    switch(enumerant)
    {
        case GL_LIGHTING:
            lighting_enabled = false;
            break;
        case GL_CULL_FACE:
            cull_face = false;
            break;
        case GL_NORMALIZE:
            normalize_enabled = false;
            break;
        case GL_LIGHT0:
        case GL_LIGHT1:
        case GL_LIGHT2:
        case GL_LIGHT3:
        case GL_LIGHT4:
        case GL_LIGHT5:
        case GL_LIGHT6:
        case GL_LIGHT7:
            lights[enumerant - GL_LIGHT0].enabled = false;
            break;
        default:
            set_gl_error(GL_INVALID_ENUM);
            break;
    }
}

void precalculate_lit_material()
{
    for(int i = 0; i < light_max; i++) {
        light *l = lights + i;
        l->diffuse_mtl_scaled = current_material.diffuse * l->diffuse;
        l->specular_mtl_scaled = current_material.specular * l->specular;
    }
}

void pglLightfv(GLenum which_light, GLenum what, float *p)
{
    int lightnum = which_light - GL_LIGHT0;
    if(lightnum < 0 || lightnum > light_max) {
        set_gl_error(GL_INVALID_ENUM);
        return;
    }

    light *light = lights + which_light - GL_LIGHT0;
    switch(what)
    {
        case GL_DIFFUSE:
            light->diffuse.set(p);
            break;
        case GL_SPECULAR:
            light->specular.set(p);
            break;
        case GL_POSITION:
            matrix4x4f_mult_vec4f(current_stack->top(), p, light->position);
            break;
        default:
            set_gl_error(GL_INVALID_ENUM);
            break;
    }

    precalculate_lit_material();
}

void pglMaterialfv(GLenum which_side, GLenum what, float *p)
{
    // XXX which_side is only FRONT_AND_BACK for GL ES 1
    switch(what)
    {
        case GL_DIFFUSE:
            current_material.diffuse.set(p);
            break;
        case GL_SPECULAR:
            current_material.specular.set(p);
            break;
        case GL_AMBIENT:
            current_material.ambient.set(p);
            break;
        case GL_AMBIENT_AND_DIFFUSE:
            current_material.ambient.set(p);
            current_material.diffuse.set(p);
            break;
        default:
            set_gl_error(GL_INVALID_ENUM);
            break;
    }
    precalculate_lit_material();
}

void pglMaterialf(GLenum which_side, GLenum what, float p)
{
    // which_side is only FRONT_AND_BACK for GL ES 1
    switch(what)
    {
        case GL_SHININESS:
            current_material.shininess = p;
            break;
        default:
            set_gl_error(GL_INVALID_ENUM);
            break;
    }
}

void pglShadeModel(GLenum which)
{
}

void pglVertexPointer(int size, int type, int stride, void* ptr)
{
    vertex_array.size = size;
    vertex_array.type = type;
    vertex_array.stride = stride;
    vertex_array.ptr = ptr;
}

void pglNormalPointer(int type, int stride, void* ptr)
{
    normal_array.size = 3;
    normal_array.type = type;
    normal_array.stride = stride;
    normal_array.ptr = ptr;
}

struct world_vertex
{
    vec4f coord;
    vec3f normal;
    vec4f color;
    // vec4f texcoord;
};

static void fetch_vertex(int which, world_vertex *v)
{
    // XXX check vertex array enablement!!
    switch(vertex_array.type) {
        case GL_FLOAT: {
            float *vp = (float*)((char *)vertex_array.ptr + vertex_array.stride * which);
            if(vertex_array.size == 3) {
                v->coord.set(vp[0], vp[1], vp[2], 1.0f);
            }
            else if(vertex_array.size == 4) {
                v->coord.set(vp[0], vp[1], vp[2], vp[3]);
            }
            break;
        }
    }

    // XXX check normal array enablement!!
    switch(normal_array.type) {
        case GL_FLOAT: {
            float *np = (float*)((char *)normal_array.ptr + normal_array.stride * which);
            v->normal.set(np[0], np[1], np[2]);
            break;
        }
    }

    // XXX color array
    v->color.set(1.0f, 1.0f, 1.0f, 1.0f);

    // XXX texcoord arrays
}


static void light_vertex(material *mtl, const vec4f& coord, const vec3f& normal, vec4f& color)
{
    color.set(0.0f, 0.0f, 0.0f, 0.0f);

    for(int i = 0; i < light_max; i++) {
        light *l = lights + i;
        if(!l->enabled)
            continue;

        // XXX point lights only, no ambient, no scene ambient, no emission
        // XXX OpenGL ES 1.1: 2.12.1 Lighting
        vec4f vertex_to_light;
        vertex_to_light = l->position - coord;
        vertex_to_light = vec_normalize(vertex_to_light);

        vec3f v2l(vertex_to_light[0], vertex_to_light[1], vertex_to_light[2]);

        float diff_part = vec_dot(normal, v2l);
        if(diff_part >= 0)
        {
            /* diffuse calculation */
            vec4f t1 = l->diffuse_mtl_scaled * diff_part;
            color = color + t1;

            vec3f h(vertex_to_light[0], vertex_to_light[1], vertex_to_light[2] + 1);
            h = vec_normalize(h);
            float spec_part = powf(vec_dot(normal, h), mtl->shininess);
            t1 = l->specular_mtl_scaled * spec_part;
            color = color + t1;
        }
    }
}

int doprint = 0; // XXX debug

static void per_vertex(world_vertex *wv, ScreenVertex *sv)
{
    vec4f tv;
    vec4f pv;
    vec3f normal;

    matrix4x4f_mult_vec4f(modelview_stack.top(), wv->coord, tv);
    vec3f_mult_matrix4x4f(wv->normal, modelview_stack.get_inverse(), normal);

    if(normalize_enabled)
        normal = vec_normalize(normal);

    if(color_material_enabled) {
        // XXX color_material mode
        current_material.diffuse = wv->color;
        current_material.ambient = wv->color;
    }
    if(lighting_enabled)
        light_vertex(&current_material, tv, normal, wv->color);

    matrix4x4f_mult_vec4f(projection_stack.top(), tv, pv);

    sv->x = (pv[0] / pv[3] + 1) * viewport_width / 2 + viewport_x;
    sv->y = (1 - pv[1] / pv[3]) * viewport_height / 2 + viewport_y;
    sv->z = (pv[2] / pv[3] + 1) / 2;

    sv->r = std::clamp(wv->color[0], 0.0f, 1.0f);
    sv->g = std::clamp(wv->color[1], 0.0f, 1.0f);
    sv->b = std::clamp(wv->color[2], 0.0f, 1.0f);
}

static int per_triangle(const ScreenVertex* sv0, const ScreenVertex* sv1, const ScreenVertex* sv2)
{
    int v1[2]; // sv0sv1 rotated by 90 degrees
    v1[0] = - (sv1->y - sv0->y);
    v1[1] = (sv1->x - sv0->x);

    int v2[2]; // sv1sv2
    v2[0] = sv2->x - sv1->x;
    v2[1] = sv2->y - sv1->y;

    int dot = v1[0] * v2[0] + v1[1] * v2[1];

    bool ccw = dot < 0; // because 0,0 is left,top

    // XXX only CCW
    if(cull_face && !ccw)
        return 0;

    // XXX clip

    return RasterizerAddTriangle(sv0, sv1, sv2); // XXX non-zero assumed to be out-of-memory
}

void pglDrawArrays(int primitive_type, int first, int count)
{
    world_vertex wv0, wv1, wv2;
    ScreenVertex sv0, sv1, sv2;

    switch(primitive_type) {
        case GL_TRIANGLES:
            for(int i = 0; i < count; i += 3) {
                fetch_vertex(first + i + 0, &wv0);
                fetch_vertex(first + i + 1, &wv1);
                fetch_vertex(first + i + 2, &wv2);
                per_vertex(&wv0, &sv0);
                per_vertex(&wv1, &sv1);
                per_vertex(&wv2, &sv2);
                if(per_triangle(&sv0, &sv1, &sv2) != 0) {
                    set_gl_error(GL_OUT_OF_MEMORY);
                    return;
                }
            }
            break;

        case GL_TRIANGLE_FAN:
            fetch_vertex(first + 0, &wv0);
            fetch_vertex(first + 1, &wv1);
            per_vertex(&wv0, &sv0);
            per_vertex(&wv1, &sv1);
            for(int i = 2; i < count; i++) {
                fetch_vertex(first + i, &wv2);
                per_vertex(&wv2, &sv2);
                if(per_triangle(&sv0, &sv1, &sv2) != 0) {
                    set_gl_error(GL_OUT_OF_MEMORY);
                    return;
                }
                sv1 = sv2;
            }
            break;

        case GL_LINE_LOOP:
            fetch_vertex(first, &wv0);
            per_vertex(&wv0, &sv0);
            sv1 = sv0;

            for(int i = 1; i < count; i++) {
                fetch_vertex(first + i, &wv2);
                per_vertex(&wv2, &sv2);
                if(RasterizerAddLine(sv1, sv2) != 0) {
                    set_gl_error(GL_OUT_OF_MEMORY);
                    return;
                }
                sv1 = sv2;
            }

            if(RasterizerAddLine(sv2, sv0) != 0) {
                set_gl_error(GL_OUT_OF_MEMORY);
                return;
            }
            break;

        default:
            set_gl_error(GL_INVALID_ENUM);
            break;
    }
}

void pglDrawElements(int primitive_type, int element_count, int index_type, const void *indicesvoid)
{
    world_vertex wv0, wv1, wv2;
    ScreenVertex sv0, sv1, sv2;

    if(index_type != GL_UNSIGNED_INT) {
        set_gl_error(GL_INVALID_ENUM);
        return;
    }

    const unsigned int *indices = reinterpret_cast<const unsigned int*>(indicesvoid);

    switch(primitive_type) {
        case GL_TRIANGLES:
            for(int i = 0; i < element_count; i += 3) {
                fetch_vertex(indices[i + 0], &wv0);
                fetch_vertex(indices[i + 1], &wv1);
                fetch_vertex(indices[i + 2], &wv2);
                per_vertex(&wv0, &sv0);
                per_vertex(&wv1, &sv1);
                per_vertex(&wv2, &sv2);
                if(per_triangle(&sv0, &sv1, &sv2) != 0) {
                    set_gl_error(GL_OUT_OF_MEMORY);
                    return;
                }
            }
            break;

        case GL_TRIANGLE_FAN:
            fetch_vertex(indices[0], &wv0);
            fetch_vertex(indices[1], &wv1);
            per_vertex(&wv0, &sv0);
            per_vertex(&wv1, &sv1);
            for(int i = 2; i < element_count; i++) {
                fetch_vertex(indices[i], &wv2);
                per_vertex(&wv2, &sv2);
                if(per_triangle(&sv0, &sv1, &sv2) != 0) {
                    set_gl_error(GL_OUT_OF_MEMORY);
                    return;
                }
                sv1 = sv2;
            }
            break;

        case GL_LINE_LOOP:
            fetch_vertex(indices[0], &wv0);
            per_vertex(&wv0, &sv0);
            sv1 = sv0;

            for(int i = 1; i < element_count; i++) {
                fetch_vertex(indices[i], &wv2);
                per_vertex(&wv2, &sv2);
                if(RasterizerAddLine(sv1, sv2) != 0) {
                    set_gl_error(GL_OUT_OF_MEMORY);
                    return;
                }
                sv1 = sv2;
            }

            if(RasterizerAddLine(sv2, sv0) != 0) {
                set_gl_error(GL_OUT_OF_MEMORY);
                return;
            }
            break;

        default:
            set_gl_error(GL_INVALID_ENUM);
            break;
    }
}

void pglClear(int bits)
{
    if(bits & GL_COLOR_BUFFER_BIT){
        RasterizerClear(clear_color[0], clear_color[1], clear_color[2]);
        bits &= ~GL_COLOR_BUFFER_BIT;
    }
    if(bits != 0)
        set_gl_error(GL_INVALID_ENUM);
}

void init_gl_state() __attribute__((constructor));
void init_gl_state()
{
    current_material.diffuse.set(.8, .8, .8, 1);
    current_material.specular.set(0, 0, 0, 1);
    current_material.diffuse.set(.2, .2, .2, 1);
    current_material.shininess = 0;

    lights[0].position.set(0, 0, 1, 0);
    lights[0].diffuse.set(1, 1, 1, 1);
    lights[0].specular.set(1, 1, 1, 1);
    lights[0].enabled = false;

    for(int i = 1; i < light_max; i++) {
        lights[i].position.set(0, 0, 1, 0);
        lights[i].diffuse.set(0, 0, 0, 1);
        lights[i].specular.set(0, 0, 0, 1);
        lights[i].enabled = false;
    }

    modelview_stack.load(identity_4x4f);
    projection_stack.load(identity_4x4f);

    normal_array.size = 3; // spec says default is 4...?
    normal_array.stride = 0;
    normal_array.type = GL_FLOAT;
    normal_array.ptr = 0;

    vertex_array.size = 4;
    vertex_array.stride = 0;
    vertex_array.type = GL_FLOAT;
    vertex_array.ptr = 0;
}

void pglClearColor(float r, float g, float b, float a)
{
    clear_color[0] = r;
    clear_color[1] = g;
    clear_color[2] = b;
    clear_color[3] = a;
}

void pglGetFloatv(GLenum which, float *p)
{
    switch(which) {
        case GL_MODELVIEW_MATRIX:
            p = modelview_stack.top();
            break;
        case GL_PROJECTION_MATRIX:
            p = projection_stack.top();
            break;
        default:
            set_gl_error(GL_INVALID_ENUM);
            break;
    }
}

void pglViewport(int x, int y, int width, int height)
{
    viewport_x = x;
    viewport_y = y;
    viewport_width = width;
    viewport_height = height;
}
