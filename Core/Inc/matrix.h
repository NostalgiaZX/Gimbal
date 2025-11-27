#ifndef MATRIX_H
#define MATRIX_H

#include <cstring>
#include <cmath>
/**
 * @brief 计算3x3矩阵的转置
 * @param R_ 输入矩阵 (3x3)
 * @param RT_ 输出矩阵（转置结果）
 */
void Matrix33fTrans(float R_[][3], float RT_[][3]);

/**
 * @brief 计算两个3D向量的叉乘 (Cross Product)
 * @param a_ 第一个向量 [x, y, z]
 * @param b_ 第二个向量 [x, y, z]
 * @param c_ 输出结果向量 c = a × b
 */
void Vector3fCross(float a_[], float b_[], float c_[]);

/**
 * @brief 计算3x3矩阵与3D向量的乘积
 * @param A 3x3矩阵
 * @param b 3D向量输入
 * @param c 3D向量输出 (c = A * b)
 */
void Matrix33fMultVector3f(float A_[][3], const float b_[], float c_[]);

/**
 * @brief 计算3D向量的模长 (Norm/Magnitude)
 * @param a_ 输入向量 [x, y, z]
 * @return 返回向量的欧几里得模长 sqrt(x^2 + y^2 + z^2)
 */
float Vector3fNorm(float a_[]);

/**
 * @brief 向量加法 (a + b)
 */
void Vector3fAdd(float a_[], float b_[], float c_[]);

/**
 * @brief 向量减法 (a - b)
 */
void Vector3fSub(float a_[], const float b_[], float c_[]);

/**
 * @brief 向量/四元数归一化
 * @param q_in 输入向量/四元数
 * @param q_out 输出向量/四元数（单位向量，模长为1）
 */
void Vector4fUnit(float q_in[], float q_out[]);

#endif  // MATRIX_H