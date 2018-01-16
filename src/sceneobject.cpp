#include "sceneobject.hpp"
#include <iostream>
Intersection::Intersection(const Vec3& pos, const Vec3& normal, double distance, SceneObject_constptr obj) :
	pos_{ pos }, normal_{ normal }, distance_to_origin_{ distance }, obj_{obj_}
{

}

Vec3 & Intersection::pos()
{
	return pos_;
}

const Vec3 & Intersection::pos() const
{
	return pos_;
}

Vec3 & Intersection::normal()
{
	return normal_;
}

const Vec3 & Intersection::normal() const
{
	return normal_;
}

double & Intersection::distance()
{
	return distance_to_origin_;
}

double Intersection::distance() const
{
	return distance_to_origin_;
}

SceneObject_constptr Intersection::obj() const
{
	return obj_;
}

SceneObject_constptr & Intersection::obj()
{
	return obj_;
}

Sphere::Sphere(SE3 tf, Material m, double radius) :
	SceneObject(tf, m),
	radius_{ radius }
{

}

Sphere::~Sphere()
{
}

bool Sphere::intersect(const Ray& r, Intersection* is) const
{
	Ray r_local = transformToLocalRay(r);
	Vec3 sphere2ray = r_local.pos();

	double a = r_local.dir().dot(r_local.dir());
	double b = 2.0 * r_local.dir().dot(sphere2ray);
	double c = (sphere2ray).squaredNorm() - std::pow(radius_, 2);
	
	double discriminant = std::pow(b, 2) - 4.0 * a * c;
	double t;
	if (discriminant < 0)
		return false;
	else if (discriminant == 0) {
		t = -b / 2.0 / a;
	}
	else {
		double t1 = (-b + std::sqrt(discriminant)) / 2.0 / a;
		double t2 = (-b - std::sqrt(discriminant)) / 2.0 / a;
		if (t1 > 0 && t2 > 0)
			t = std::min(t1, t2);
		else
			t = std::max(t1, t2);
	}

	if (t < 0)
		return false;

	is->obj() = this;
	is->pos() = (tf_ * (r_local.pos() + t * r_local.dir()).homogeneous()).topRows(3);
	is->distance() = scale() * t;
	is->normal() = (is->pos() - tf_.translation()).normalized();
	return true;
}



SceneObject::SceneObject(SE3 tf, Material m) :
	tf_(tf), material_(m), scale_cached_(0)
{

}

SceneObject::~SceneObject()
{
}

SE3 & SceneObject::transform()
{
	scale_cached_ = 0;
	return tf_;
}

const SE3 & SceneObject::transform() const
{
	return tf_;
}

Material & SceneObject::material()
{
	return material_;
}

const Material & SceneObject::material() const
{
	return material_;
}

Ray SceneObject::transformToLocalRay(const Ray & r) const
{
	Ray r_local{
		(tf_.inverse() * r.pos().homogeneous()).topRows(3),
		(tf_.rotation().transpose() * r.dir()) 
	};
	return r_local;
}

double SceneObject::computeScale() 
{
	if (scale_cached_ != 0)
		return scale_cached_;
	Mat33 sm;
	tf_.computeRotationScaling<Mat33, Mat33>(nullptr, &sm);
	scale_cached_ =  sm(1, 1);
	return scale_cached_;
}

double SceneObject::scale() const
{
	assert(scale_cached_ != 0);
	return scale_cached_;
}
