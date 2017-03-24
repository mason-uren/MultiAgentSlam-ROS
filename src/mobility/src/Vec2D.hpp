#ifndef VEC2D_HPP
#define VEC2D_HPP

#include <cmath>
#include <cstdlib>

struct Vec2D {
    double x;
    double y;

    Vec2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}
    Vec2D(const Vec2D& v) : x(v.x), y(v.y) {}

    double& operator[](size_t idx);
    const double& operator[](size_t idx) const;

    friend double dot(Vec2D lhs, Vec2D rhs);
    friend double cross_2D(Vec2D lhs, Vec2D rhs);
    friend Vec2D perp(const Vec2D& v);
    friend double distance(Vec2D lhs, Vec2D rhs);
    friend double distance_sqr(Vec2D lhs, Vec2D rhs);
    friend double angle_rad(Vec2D v);
    friend double mag_sqr(Vec2D v);
    friend double mag(Vec2D v);
    friend Vec2D normal(Vec2D v);

    friend Vec2D operator-(const Vec2D& v);
    friend Vec2D operator+(Vec2D lhs, const Vec2D& rhs);
    friend Vec2D operator-(Vec2D lhs, const Vec2D& rhs);
    friend Vec2D operator*(Vec2D lhs, const double& rhs);
    friend Vec2D operator*(double lhs, const Vec2D& rhs);
    friend Vec2D operator/(Vec2D lhs, const double& rhs);

    Vec2D& operator=(const Vec2D& rhs);
    Vec2D& operator+=(const Vec2D& rhs);
    Vec2D& operator-=(const Vec2D& rhs);
    Vec2D& operator*=(const double& rhs);
    Vec2D& operator/=(const double& rhs);

};

#include "Vec2D.impl"

#endif
