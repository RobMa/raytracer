#pragma once
#include <Eigen/Dense>
#include <vector>
#include <array>
#include "sceneobject.hpp"
#include "box3.hpp"


namespace TriangularMesh {

	using Index = int;
	using Vertex = Eigen::Vector3d;
	using Vertices = std::vector<Vertex>;
	using Face = std::array<Index, 3>;
	using Faces = std::vector<Face>;


class TriMesh :
	public SceneObject
{
public:
	TriMesh(const SE3& tf, const Material& m, bool interpolate_normals);
	virtual ~TriMesh();

	virtual bool intersect(const Ray& r, Intersection* is) const;

	static TriMesh* createPyramid(const SE3& tf, const Material& m);

	static TriMesh* loadFromPly(const char* path, bool reverse_face_normal, const SE3&tf, const Material& m, bool interpolate_normals);

protected:
	// Calculate the intersection point of a ray inside a triangle 
	bool calcTriIntersect(Index i, const Ray& r_local, Vec3 *point, double *distance) const;

	// Interpolate the normal inside a triangle using Phong interpolation
	Vec3 calcPhongNormalInterpolation(Index i, const Vec3& point) const;

	// Calculate the normal of a triangle
	Vec3 calcTriNormal(Index i) const;

	// Calculate the normals at all vertices. We need to do this before for Phong interpolation.
	void calcNormals();

	// Find all faces that are connected to a vertex
	void findConnectedFaces(Index vertex, std::vector<Index>* connected_faces);

	// Calculate a bounding box which is including all vertices
	void calcBoundingBox();

	// Check if a local ray intersects the bounding box
	bool intersectBoundingBox(const Ray& r_local) const;

private:
	Vertices vertices_;
	Faces faces_;
	std::vector<Vec3> vertex_normals_;
	std::vector<Vec3> face_normals_;
	std::vector<Vec3> face_normals_unit_;
	Box3 bounding_box_;
	bool interpolate_normals_;



}; // class TriMesh


}; // namespace TriMesh