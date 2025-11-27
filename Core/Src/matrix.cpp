#include "matrix.h"
#include <cmath>
#include <cstring>

/**
 * @brief 计算3x3矩阵的转置
 * @param R_ 输入矩阵 (3x3)
 * @param RT_ 输出矩阵（转置结果）
 * 
 * 转置矩阵：RT[i][j] = R[j][i]
 */
void Matrix33fTrans(float R_[][3], float RT_[][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      RT_[i][j] = R_[j][i];
    }
  }
}

/**
 * @brief 计算两个3D向量的叉乘 (Cross Product)
 * @param a_ 第一个向量 [x, y, z]
 * @param b_ 第二个向量 [x, y, z]
 * @param c_ 输出结果向量 c = a × b
 * 
 * 叉乘公式：
 * c[0] = a[1]*b[2] - a[2]*b[1]
 * c[1] = a[2]*b[0] - a[0]*b[2]
 * c[2] = a[0]*b[1] - a[1]*b[0]
 */
void Vector3fCross(float a_[], float b_[], float c_[]) {
  c_[0] = a_[1] * b_[2] - a_[2] * b_[1];
  c_[1] = a_[2] * b_[0] - a_[0] * b_[2];
  c_[2] = a_[0] * b_[1] - a_[1] * b_[0];
}

/**
 * @brief 计算3x3矩阵与3D向量的乘积
 * @param A 3x3矩阵
 * @param b 3D向量输入
 * @param c 3D向量输出 (c = A * b)
 * 
 * 矩阵-向量乘法：
 * c[i] = A[i][0]*b[0] + A[i][1]*b[1] + A[i][2]*b[2]
 */
void Matrix33fMultVector3f(float A_[][3], const float b_[], float c_[]) {
  float temp[3] = {0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      temp[i] += A_[i][j] * b_[j];
    }
  }
  memcpy(c_, temp, 3 * sizeof(float));
}

/**
 * @brief 计算3D向量的模长 (Norm/Magnitude)
 * @param a_ 输入向量 [x, y, z]
 * @return 返回向量的欧几里得模长 sqrt(x^2 + y^2 + z^2)
 */
float Vector3fNorm(float a_[]) {
  return sqrtf(a_[0] * a_[0] + a_[1] * a_[1] + a_[2] * a_[2]);
}

/**
 * @brief 向量加法
 * @param a_ 第一个向量
 * @param b_ 第二个向量
 * @param c_ 输出结果向量 (c = a + b)
 */
void Vector3fAdd(float a_[], float b_[], float c_[]) {
  for (int i = 0; i < 3; i++) {
    c_[i] = a_[i] + b_[i];
  }
}

/**
 * @brief 向量减法
 * @param a_ 第一个向量
 * @param b_ 第二个向量
 * @param c_ 输出结果向量 (c = a - b)
 */
void Vector3fSub(float a_[], const float b_[], float c_[]) {
  for (int i = 0; i < 3; i++) {
    c_[i] = a_[i] - b_[i];
  }
}

/**
 * @brief 向量归一化 (Normalization)
 * @param q_in 输入向量
 * @param q_out 输出向量（单位向量，模长为1）
 * 
 * 将四元数或向量归一化为单位向量
 * 用于保证四元数模长始终为1
 */
void Vector4fUnit(float q_in[], float q_out[]) {
  float norm = sqrtf(q_in[0] * q_in[0] + q_in[1] * q_in[1] + 
                     q_in[2] * q_in[2] + q_in[3] * q_in[3]);
  
  if (norm > 1e-6f) {  // 避免除以零
    for (int i = 0; i < 4; i++) {
      q_out[i] = q_in[i] / norm;
    }
  } else {
    // 若模长太小，保持原值或设为单位四元数
    memcpy(q_out, q_in, 4 * sizeof(float));
  }
}