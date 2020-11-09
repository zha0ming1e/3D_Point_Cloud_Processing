#include "point.h"

Point::Point(const Vec3 &position) : pos_(position) {}

Point::Point(const Vec3 &position, const Vec3 &normal) : pos_(position), normal_(normal) {}

Point::Ptr Point::CreateNewPoint() {
    Point::Ptr new_point(new Point);

    return new_point;
}
