#pragma once

#include <vector>
#include "global.hpp"
#include "camera.hpp"
#include "material.hpp"

class SceneObject;

typedef SceneObject* SceneObject_ptr;
typedef std::vector<SceneObject_ptr> SceneObjects;
using SceneObject_constptr = const SceneObject *;

class Intersection
{
public:
	Intersection(const Vec3& pos, const Vec3& normal, double distance, SceneObject_constptr obj);

	Vec3& pos();
	const Vec3& pos() const;

	Vec3& normal();
	const Vec3& normal() const;

	double & distance();
	double distance() const;

	SceneObject_constptr obj() const;
	SceneObject_constptr& obj();

private:
	Vec3 pos_;
	Vec3 normal_;
	double distance_to_origin_;
	SceneObject_constptr obj_;
};


class SceneObject
{
public:
	SceneObject(SE3 tf, Material m);

	virtual ~SceneObject();

	virtual bool intersect(const Ray& r, Intersection *is) const = 0;

	SE3& transform();
	const SE3& transform() const;	

	Material& material();
	const Material& material() const;

	double computeScale();
	double scale() const;

protected:
	Ray transformToLocalRay(const Ray& r) const;

	SE3 tf_;
	Material material_;

	
private:
	double scale_cached_;
};


class Sphere : public SceneObject
{
public:
	Sphere(SE3 tf, Material m, double radius);

	virtual ~Sphere();

	virtual bool intersect(const Ray& r, Intersection *is) const;

private:
	double radius_;
};
