#include "box3.hpp"

Box3::Box3(Vec3 center, Vec3 u, Vec3 v, Vec3 w, double length_u, double length_v, double length_w) : 
	center_{center},
	u_{ u }, v_{ v }, w_{ w }, length_u_{ length_u }, length_v_{ length_v }, length_w_{ length_w }
{
}

bool rect_intersect(const Vec3& P0, const Vec3& P1,
	const Vec3& normal, double distance,
	const Vec3& a, double length_a, const Vec3& b, double length_b)
{
	double proj_normal_raydir = normal.dot(P1);
	if (fabs(proj_normal_raydir) < EPS) {
		return false;
	}
	double proj_normal_P0 = normal.dot(P0);
	double t1 = (distance - proj_normal_P0) / proj_normal_raydir;
	double t2 = (-distance - proj_normal_P0) / proj_normal_raydir;
	double t = t1;
	if (t1 > 0 && t2 > 0)
		t = std::min(t1, t2);
	else if (t2 > 0)
		t = t2;
	Vec3 x = P0 + t*P1;
	double proj_x_a = x.dot(a);
	double proj_x_b = x.dot(b);
	bool onplane_a = fabs(proj_x_a) <= length_a;
	bool onplane_b = fabs(proj_x_b) <= length_b;
	return onplane_a && onplane_b;
}

bool Box3::intersect(Ray r) const
{

	Vec3 P0 = r.pos() - center_;
	Vec3 P1 = r.dir();
	if (rect_intersect(P0, P1, u_, length_u_, v_, length_v_, w_, length_w_))
		return true;
	if (rect_intersect(P0, P1, w_, length_w_, u_, length_u_, v_, length_v_))
		return true;
	if (rect_intersect(P0, P1, v_, length_v_, w_, length_w_, u_, length_u_))
		return true;

	return false;	
}

Vec3 & Box3::center()
{
	return center_;
}

const Vec3 & Box3::center() const
{
	return center_;
}

