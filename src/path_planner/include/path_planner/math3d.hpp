#pragma once

// 固定長の小規模線形代数（仕様書 第2.4, 10.1節）。外部行列ライブラリは使用しない。
// 動的確保を行わない値型のみで構成する。

#include <array>
#include <cmath>

namespace path_planner
{
struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  constexpr Vector3() = default;
  constexpr Vector3(double x_in, double y_in, double z_in)
  : x(x_in), y(y_in), z(z_in) {}

  double operator[](int i) const {return i == 0 ? x : (i == 1 ? y : z);}
  double & operator[](int i) {return i == 0 ? x : (i == 1 ? y : z);}
};

inline Vector3 operator+(const Vector3 & a, const Vector3 & b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
inline Vector3 operator-(const Vector3 & a, const Vector3 & b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
inline Vector3 operator-(const Vector3 & a) {return {-a.x, -a.y, -a.z};}
inline Vector3 operator*(const Vector3 & a, double s) {return {a.x * s, a.y * s, a.z * s};}
inline Vector3 operator*(double s, const Vector3 & a) {return a * s;}
inline Vector3 operator/(const Vector3 & a, double s) {return {a.x / s, a.y / s, a.z / s};}
inline Vector3 & operator+=(Vector3 & a, const Vector3 & b) {a = a + b; return a;}
inline Vector3 & operator-=(Vector3 & a, const Vector3 & b) {a = a - b; return a;}
inline double dot(const Vector3 & a, const Vector3 & b) {return a.x * b.x + a.y * b.y + a.z * b.z;}
inline Vector3 cross(const Vector3 & a, const Vector3 & b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(const Vector3 & a) {return std::sqrt(dot(a, a));}
inline bool isFinite(const Vector3 & a)
{
  return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}

/// 行優先 3x3 行列。
struct Matrix3
{
  std::array<double, 9> m{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  static Matrix3 identity() {return Matrix3{};}
  static Matrix3 zero()
  {
    Matrix3 r;
    r.m.fill(0.0);
    return r;
  }
  double operator()(int r, int c) const {return m[static_cast<std::size_t>(r * 3 + c)];}
  double & operator()(int r, int c) {return m[static_cast<std::size_t>(r * 3 + c)];}
  Vector3 column(int c) const {return {(*this)(0, c), (*this)(1, c), (*this)(2, c)};}
};

inline Matrix3 operator*(const Matrix3 & a, const Matrix3 & b)
{
  Matrix3 r = Matrix3::zero();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {
        s += a(i, k) * b(k, j);
      }
      r(i, j) = s;
    }
  }
  return r;
}

inline Vector3 operator*(const Matrix3 & a, const Vector3 & v)
{
  return {
    a(0, 0) * v.x + a(0, 1) * v.y + a(0, 2) * v.z,
    a(1, 0) * v.x + a(1, 1) * v.y + a(1, 2) * v.z,
    a(2, 0) * v.x + a(2, 1) * v.y + a(2, 2) * v.z};
}

inline Matrix3 operator+(const Matrix3 & a, const Matrix3 & b)
{
  Matrix3 r;
  for (std::size_t i = 0; i < 9; ++i) {
    r.m[i] = a.m[i] + b.m[i];
  }
  return r;
}

inline Matrix3 operator*(const Matrix3 & a, double s)
{
  Matrix3 r;
  for (std::size_t i = 0; i < 9; ++i) {
    r.m[i] = a.m[i] * s;
  }
  return r;
}

inline Matrix3 transpose(const Matrix3 & a)
{
  Matrix3 r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      r(i, j) = a(j, i);
    }
  }
  return r;
}

/// 単位軸 axis まわりに angle 回転する回転行列（Rodrigues）。
inline Matrix3 axisAngle(const Vector3 & axis, double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  const double t = 1.0 - c;
  const double x = axis.x;
  const double y = axis.y;
  const double z = axis.z;
  Matrix3 r;
  r(0, 0) = t * x * x + c;
  r(0, 1) = t * x * y - s * z;
  r(0, 2) = t * x * z + s * y;
  r(1, 0) = t * x * y + s * z;
  r(1, 1) = t * y * y + c;
  r(1, 2) = t * y * z - s * x;
  r(2, 0) = t * x * z - s * y;
  r(2, 1) = t * y * z + s * x;
  r(2, 2) = t * z * z + c;
  return r;
}

/// URDF の rpy（固定軸 X→Y→Z の順に回転、R = Rz(yaw) Ry(pitch) Rx(roll)）。
inline Matrix3 rpyToMatrix(double roll, double pitch, double yaw)
{
  return axisAngle({0.0, 0.0, 1.0}, yaw) * axisAngle({0.0, 1.0, 0.0}, pitch) *
         axisAngle({1.0, 0.0, 0.0}, roll);
}

/// 慣性テンソル（対称）を [ixx, ixy, ixz, iyy, iyz, izz] から生成する。
inline Matrix3 inertiaFromSix(const std::array<double, 6> & v)
{
  Matrix3 r;
  r(0, 0) = v[0];
  r(0, 1) = v[1];
  r(0, 2) = v[2];
  r(1, 0) = v[1];
  r(1, 1) = v[3];
  r(1, 2) = v[4];
  r(2, 0) = v[2];
  r(2, 1) = v[4];
  r(2, 2) = v[5];
  return r;
}

/// 剛体変換（親座標系から見た子座標系の姿勢）。
struct Transform
{
  Matrix3 rotation{};
  Vector3 translation{};

  static Transform identity() {return Transform{};}
  Vector3 apply(const Vector3 & p) const {return rotation * p + translation;}
};

inline Transform operator*(const Transform & a, const Transform & b)
{
  return {a.rotation * b.rotation, a.rotation * b.translation + a.translation};
}

inline Transform inverse(const Transform & t)
{
  const Matrix3 rt = transpose(t.rotation);
  return {rt, -(rt * t.translation)};
}

inline Transform makeTransform(const Vector3 & xyz, const Vector3 & rpy)
{
  return {rpyToMatrix(rpy.x, rpy.y, rpy.z), xyz};
}

}  // namespace path_planner
