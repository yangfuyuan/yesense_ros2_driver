#ifndef QUATERNION_HPP_
#define QUATERNION_HPP_

#include <cassert>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <type_traits>

static inline double from_degrees(double degrees) {
  return degrees * M_PI / 180.0;
}
/*!
 * \brief Convert radians to degrees
 */
static inline double to_degrees(double radians) {
  return radians * 180.0 / M_PI;
}

static inline double normalize_angle_positive(double angle) {
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

/*!
 * \brief normalize
 *
 * Normalizes the angle to be -M_PI circle to +M_PI circle
 * It takes and returns radians.
 *
 */
static inline double normalize_angle(double angle) {
  double a = normalize_angle_positive(angle);
  if (a > M_PI) a -= 2.0 * M_PI;
  return a;
}

template <typename T = double>
class Quaternion {
  static_assert(std::is_floating_point<T>::value,
                "Quaternion only supports floating point types.");

 public:
  /// @brief Default constructor (identity)
  inline Quaternion() : m_Values{T(0), T(0), T(0), T(1)} {}

  /// @brief Constructor with explicit x, y, z, w
  inline Quaternion(T x, T y, T z, T w) {
    m_Values[0] = x;
    m_Values[1] = y;
    m_Values[2] = z;
    m_Values[3] = w;
  }

  /// @brief Copy constructor
  inline Quaternion(const Quaternion& other) {
    for (int i = 0; i < 4; ++i) {
      m_Values[i] = other.m_Values[i];
    }
  }

  // ---------------- Getters ----------------
  inline const T& getX() const { return m_Values[0]; }
  inline const T& getY() const { return m_Values[1]; }
  inline const T& getZ() const { return m_Values[2]; }
  inline const T& getW() const { return m_Values[3]; }
  inline T x() const { return m_Values[0]; }
  inline T y() const { return m_Values[1]; }
  inline T z() const { return m_Values[2]; }
  inline T w() const { return m_Values[3]; }

  inline T& x() { return m_Values[0]; }
  inline T& y() { return m_Values[1]; }
  inline T& z() { return m_Values[2]; }
  inline T& w() { return m_Values[3]; }

  // ---------------- Setters ----------------
  inline void setX(T x_val) { m_Values[0] = x_val; }
  inline void setY(T y_val) { m_Values[1] = y_val; }
  inline void setZ(T z_val) { m_Values[2] = z_val; }
  inline void setW(T w_val) { m_Values[3] = w_val; }
  inline void setValue(const T& x, const T& y, const T& z, const T& w) {
    m_Values[0] = x;
    m_Values[1] = y;
    m_Values[2] = z;
    m_Values[3] = w;
  }

  void ToEuler(T& yaw, T& pitch, T& roll) const {
    // roll (X)
    T sinr_cosp =
        +2.0 * (m_Values[3] * m_Values[0] + m_Values[1] * m_Values[2]);
    T cosr_cosp =
        +1.0 - 2.0 * (m_Values[0] * m_Values[0] + m_Values[1] * m_Values[1]);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (Y)
    T sinp = +2.0 * (m_Values[3] * m_Values[1] - m_Values[2] * m_Values[0]);
    if (std::abs(sinp) >= 1)
      pitch = std::copysign(T(M_PI_2), sinp);
    else
      pitch = std::asin(sinp);

    // yaw (Z)
    T siny_cosp =
        +2.0 * (m_Values[3] * m_Values[2] + m_Values[0] * m_Values[1]);
    T cosy_cosp =
        +1.0 - 2.0 * (m_Values[1] * m_Values[1] + m_Values[2] * m_Values[2]);
    yaw = std::atan2(siny_cosp, cosy_cosp);
  }

  void setEuler(const T& yaw, const T& pitch, const T& roll) {
    T halfYaw = T(yaw) * T(0.5);
    T halfPitch = T(pitch) * T(0.5);
    T halfRoll = T(roll) * T(0.5);
    T cosYaw = cos(halfYaw);
    T sinYaw = sin(halfYaw);
    T cosPitch = cos(halfPitch);
    T sinPitch = sin(halfPitch);
    T cosRoll = cos(halfRoll);
    T sinRoll = sin(halfRoll);
    setValue(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
             cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
             sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
             cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
  }
  /**@brief Set the quaternion using fixed axis RPY
   * @param roll Angle around X
   * @param pitch Angle around Y
   * @param yaw Angle around Z*/

  void setRPY(const T& roll, const T& pitch, const T& yaw) {
    T halfYaw = T(yaw) * T(0.5);
    T halfPitch = T(pitch) * T(0.5);
    T halfRoll = T(roll) * T(0.5);
    T cosYaw = cos(halfYaw);
    T sinYaw = sin(halfYaw);
    T cosPitch = cos(halfPitch);
    T sinPitch = sin(halfPitch);
    T cosRoll = cos(halfRoll);
    T sinRoll = sin(halfRoll);
    setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,  // x
             cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,  // y
             cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,  // z
             cosRoll * cosPitch * cosYaw +
                 sinRoll * sinPitch * sinYaw);  // formerly yzx
  }

  /// @brief Create quaternion from axis-angle representation
  /// @param axisX X component of axis (assumed normalized)
  /// @param axisY Y component of axis
  /// @param axisZ Z component of axis
  /// @param angle Angle in radians
  void setRotation(T axisX, T axisY, T axisZ, T angle) {
    T d = std::sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
    if (d == T(0)) {
      m_Values[0] = T(0);
      m_Values[1] = T(0);
      m_Values[2] = T(0);
      m_Values[3] = T(1);
      return;
    }
    T halfAngle = angle * T(0.5);
    T s = std::sin(halfAngle) / d;
    m_Values[0] = axisX * s;
    m_Values[1] = axisY * s;
    m_Values[2] = axisZ * s;
    m_Values[3] = std::cos(halfAngle);
  }

  void getAxis(T& axisX, T& axisY, T& axisZ) const {
    axisX = 1.0;
    axisY = 0.0;
    axisZ = 0.0;
    T s_squared = T(1.) - pow(m_Values[3], T(2.));
    if (s_squared < T(10.) * DBL_EPSILON)  // Check for divide by zero
      return;
    T s = sqrt(s_squared);
    axisX = m_Values[0] / s;
    axisY = m_Values[1] / s;
    axisZ = m_Values[2] / s;
  }

  // ---------------- Operators ----------------

  inline Quaternion& operator=(const Quaternion& other) {
    if (this != &other) {
      for (int i = 0; i < 4; ++i) {
        m_Values[i] = other.m_Values[i];
      }
    }
    return *this;
  }

  inline bool operator==(const Quaternion& other) const {
    return m_Values[0] == other.m_Values[0] &&
           m_Values[1] == other.m_Values[1] &&
           m_Values[2] == other.m_Values[2] && m_Values[3] == other.m_Values[3];
  }

  inline bool operator!=(const Quaternion& other) const {
    return !(*this == other);
  }
  inline Quaternion& operator+=(const Quaternion& q) {
    m_Values[0] += q.x();
    m_Values[1] += q.y();
    m_Values[2] += q.z();
    m_Values[3] += q.w();
    return *this;
  }

  inline Quaternion& operator-=(const Quaternion& q) {
    m_Values[0] -= q.x();
    m_Values[1] -= q.y();
    m_Values[2] -= q.z();
    m_Values[3] -= q.w();
    return *this;
  }

  inline Quaternion& operator*=(const T& s) {
    m_Values[0] *= s;
    m_Values[1] *= s;
    m_Values[2] *= s;
    m_Values[3] *= s;
    return *this;
  }

  inline Quaternion& operator*=(const Quaternion& q) {
    setValue(m_Values[3] * q.x() + m_Values[0] * q.w() + m_Values[1] * q.z() -
                 m_Values[2] * q.y(),
             m_Values[3] * q.y() + m_Values[1] * q.w() + m_Values[2] * q.x() -
                 m_Values[0] * q.z(),
             m_Values[3] * q.z() + m_Values[2] * q.w() + m_Values[0] * q.y() -
                 m_Values[1] * q.x(),
             m_Values[3] * q.w() - m_Values[0] * q.x() - m_Values[1] * q.y() -
                 m_Values[2] * q.z());
    return *this;
  }

  inline T dot(const Quaternion& q) const {
    return m_Values[0] * q.x() + m_Values[1] * q.y() + m_Values[2] * q.z() +
           m_Values[3] * q.w();
  }

  inline T length2() const { return dot(*this); }

  inline T length() const { return sqrt(length2()); }

  /**@brief Normalize the quaternion
   * Such that x^2 + y^2 + z^2 +w^2 = 1 */
  inline Quaternion& normalize() {
    T len = length();
    if (len > T(DBL_EPSILON)) {
      *this /= len;
    }
    return *this;
  }

  /**@brief Return a scaled version of this quaternion
   * @param s The scale factor */
  inline Quaternion operator*(const T& s) const {
    return Quaternion(x() * s, y() * s, z() * s, w() * s);
  }

  /**@brief Return an inversely scaled versionof this quaternion
   * @param s The inverse scale factor */
  inline Quaternion operator/(const T& s) const {
    assert(s != T(0.0));
    return *this * (T(1.0) / s);
  }

  /**@brief Inversely scale this quaternion
   * @param s The scale factor */
  inline Quaternion& operator/=(const T& s) {
    assert(s != T(0.0));
    return *this *= T(1.0) / s;
  }

  /**@brief Return a normalized version of this quaternion */
  inline Quaternion normalized() const { return *this / length(); }

  /**@brief Return the inverse of this quaternion */
  inline Quaternion inverse() const {
    return Quaternion(-m_Values[0], -m_Values[1], -m_Values[2], m_Values[3]);
  }

  static const Quaternion& getIdentity() {
    static const Quaternion identityQuat(T(0.), T(0.), T(0.), T(1.));
    return identityQuat;
  }

  /**@brief Return the sum of this quaternion and the other
   * @param q2 The other quaternion */
  inline Quaternion operator+(const Quaternion& q2) const {
    const Quaternion& q1 = *this;
    return Quaternion(q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z(),
                      q1.w() + q2.w());
  }

  /**@brief Return the difference between this quaternion and the other
   * @param q2 The other quaternion */
  inline Quaternion operator-(const Quaternion& q2) const {
    const Quaternion& q1 = *this;
    return Quaternion(q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z(),
                      q1.w() - q2.w());
  }

  /**@brief Return the negative of this quaternion
   * This simply negates each element */
  inline Quaternion operator-() const {
    const Quaternion& q2 = *this;
    return Quaternion(-q2.x(), -q2.y(), -q2.z(), -q2.w());
  }

  friend inline std::ostream& operator<<(std::ostream& os,
                                         const Quaternion& q) {
    os << "(" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()
       << ")";
    return os;
  }

 private:
  T m_Values[4];
};

using Quaterniond = Quaternion<double>;
using Quaternionf = Quaternion<float>;

#endif  // QUATERNION_HPP_
