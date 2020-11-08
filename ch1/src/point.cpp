#include "point.h"

Point::Point(Vec3 position) : pos_(std::move(position)) {}

Point::Point(Vec3 position, Vec3 normal) : pos_(std::move(position)), normal_(std::move(normal)) {}

Point::Ptr Point::CreateNewPoint() {
    Point::Ptr new_point(new Point);

    return new_point;
}
