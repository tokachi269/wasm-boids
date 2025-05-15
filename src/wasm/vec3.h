#pragma once
#include <cmath>

struct Vec3
{
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    float distance(const Vec3 &other) const
    {
        float dx = x - other.x;
        float dy = y - other.y;
        float dz = z - other.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    Vec3 operator+(const Vec3 &o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3 &o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator/(float v) const { return Vec3(x / v, y / v, z / v); }
    Vec3 operator*(float v) const { return Vec3(x * v, y * v, z * v); }

    Vec3 &operator+=(const Vec3 &o)
    {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }

    Vec3 &operator-=(const Vec3 &o)
    {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }

    float length() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vec3 normalized() const
    {
        float len = length();
        if (len > 1e-6f)
            return Vec3(x / len, y / len, z / len);
        return Vec3();
    }

    float dot(const Vec3 &o) const
    {
        return x * o.x + y * o.y + z * o.z;
    }

    Vec3 cross(const Vec3 &o) const
    {
        return Vec3(
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x);
    }

    static Vec3 rotateVector(const Vec3 &vec, const Vec3 &axis, float angle)
    {
        // ロドリゲスの回転公式
        Vec3 k = axis.normalized();
        float cosA = std::cos(angle);
        float sinA = std::sin(angle);
        return vec * cosA +
               k.cross(vec) * sinA +
               k * (k.dot(vec)) * (1.0f - cosA);
    }
};