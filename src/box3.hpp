#pragma once
#include "global.hpp"
#include "sceneobject.hpp"

class Box3
{
public:
	Box3(Vec3 center, Vec3 u, Vec3 v, Vec3 w, double length_u, double length_v, double length_w);

	bool intersect(Ray r) const;

	Vec3& center();
	const Vec3& center() const;

private:
	Vec3 center_;

	double length_u_;
	Vec3 u_;

	double length_v_;
	Vec3 v_;

	double length_w_;
	Vec3 w_;
};