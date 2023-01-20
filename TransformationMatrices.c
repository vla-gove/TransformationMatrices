#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846

// 4x4 matrix structure
typedef struct {
    double m[4][4];
} Matrix;

// 3d vector structure
typedef struct {
    double x;
    double y;
    double z;
} Vector;

// 4x4 identity matrix function
Matrix createIdentityMatrix() {
    Matrix mat;
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            mat.m[i][j] = (i == j) ? 1 : 0;
        }
    }
    return mat;
}

// 4x4 translation matrix function
Matrix createTranslationMatrix(Vector v) {
    Matrix mat = createIdentityMatrix();
    mat.m[0][3] = v.x;
    mat.m[1][3] = v.y;
    mat.m[2][3] = v.z;
    return mat;
}

// 4x4 x rotation matrix
Matrix createRotationXMatrix(double theta) {
    Matrix mat = createIdentityMatrix();
    mat.m[1][1] = cos(theta);
    mat.m[1][2] = -sin(theta);
    mat.m[2][1] = sin(theta);
    mat.m[2][2] = cos(theta);
    return mat;
}

// 4x4 y rotation matrix
Matrix createRotationYMatrix(double theta) {
    Matrix mat = createIdentityMatrix();
    mat.m[0][0] = cos(theta);
    mat.m[0][2] = sin(theta);
    mat.m[2][0] = -sin(theta);
    mat.m[2][2] = cos(theta);
    return mat;
}

// 4x4 z rotation matrix
Matrix createRotationZMatrix(double theta) {
    Matrix mat = createIdentityMatrix();
    mat.m[0][0] = cos(theta);
    mat.m[0][1] = -sin(theta);
    mat.m[1][0] = sin(theta);
    mat.m[1][1] = cos(theta);
    return mat;
}
// matrix multiplication function
Matrix matrixMultiply(Matrix mat1, Matrix mat2) {
    Matrix result;
    int i, j, k;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            result.m[i][j] = 0;
            for (k = 0; k < 4; k++) {
                result.m[i][j] += mat1.m[i][k] * mat2.m[k][j];
            }
        }
    }
    return result;
}
// matrix printing function
void printMatrix(Matrix mat) {
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            printf("%.2f ", mat.m[i][j]);
        }
        printf("\n");
    }
}

int main() {
    // setting arbitrary joint parameters
    double theta1 = PI / 3;
    double theta2 = PI / 3;
    double theta3 = PI / 2;
    double theta4 = PI / 2;
    double theta5 = PI / 6;
    double theta6 = PI / 5;
    double theta7 = PI / 4;
    double theta8 = PI / 3;

    // setting arbitrary link lengths
    double a1 = 0.7;
    double a2 = 0.7;
    double a3 = 0.7;
    double a4 = 0.5;
    double a5 = 0.9;
    double a6 = 0.8;
    double a7 = 0.2;
    double a8 = 0.2;

    // calculating transformation matrices for each joint
    Matrix T1 = createTranslationMatrix((Vector){0, 0, a1});
    Matrix R1 = createRotationZMatrix(theta1);
    Matrix T2 = createTranslationMatrix((Vector){0, 0, a2});
    Matrix R2 = createRotationYMatrix(theta2);
    Matrix T3 = createTranslationMatrix((Vector){0, 0, a3});
    Matrix R3 = createRotationYMatrix(theta3);
    Matrix T4 = createTranslationMatrix((Vector){0, 0, a4});
    Matrix R4 = createRotationYMatrix(theta4);
    Matrix T5 = createTranslationMatrix((Vector){0, 0, a5});
    Matrix R5 = createRotationYMatrix(theta5);
    Matrix T6 = createTranslationMatrix((Vector){0, 0, a6});
    Matrix R6 = createRotationYMatrix(theta6);
    Matrix T7 = createTranslationMatrix((Vector){0, 0, a7});
    Matrix R7 = createRotationYMatrix(theta7);
    Matrix T8 = createTranslationMatrix((Vector){0, 0, a8});
    Matrix R8 = createRotationYMatrix(theta8);

    // base to end effector transformation matrix for an 8DOF robot
    Matrix T = matrixMultiply(T1, matrixMultiply(R1, matrixMultiply(T2, matrixMultiply(R2, matrixMultiply(T3, matrixMultiply(R3, matrixMultiply(T4, matrixMultiply(R4, matrixMultiply(T5, matrixMultiply(R5, matrixMultiply(T6, matrixMultiply(R6, matrixMultiply(T7, matrixMultiply(R7, matrixMultiply(T8, R8)))))))))))))));

    // printing final matrix
    printMatrix(T);

    return 0;
}
