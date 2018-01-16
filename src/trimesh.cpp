#include "trimesh.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace TriangularMesh;

TriMesh::TriMesh(const SE3 & tf, const Material & m, bool interpolate_normals):
	SceneObject{tf, m}, bounding_box_{Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), 0, 0, 0 }, interpolate_normals_{interpolate_normals}
{

}

TriMesh::~TriMesh()
{
}

bool TriMesh::intersect(const Ray & r, Intersection * is) const
{	
	Ray r_local{
		(tf_.inverse() * r.pos().homogeneous()).topRows(3),
		(tf_.rotation().transpose() * r.dir()) // tf may have a scale component. We need to normalize the direction.
	};

	if (!intersectBoundingBox(r_local)) {
		return false;
	}

	// Find the closest triangle that is intersecting with the ray
	Index tri_closest = -1; double tri_closest_distance = std::numeric_limits<double>::max(); Vec3 tri_closest_point;
	for (int i = 0; i < faces_.size(); i++) {
		Vec3 point_tmp; double distance_tmp;
		if (calcTriIntersect(i, r_local, &point_tmp, &distance_tmp)) {
			if (distance_tmp < tri_closest_distance && distance_tmp > 0) {
				// Found a intersection with positive depth
				tri_closest = i;
				tri_closest_distance = distance_tmp;
				tri_closest_point = point_tmp;
			}
		}
	}
	if (tri_closest == -1) {
		// No triangle intersection
		return false;
	}	

	// Calculate the interpolated normal at that point
	Vec3 normal;
	if(interpolate_normals_)
		normal = calcPhongNormalInterpolation(tri_closest, tri_closest_point);
	else
		normal = face_normals_unit_[tri_closest];

	// Transform back to world coordinate frame
	is->pos() = (tf_ * tri_closest_point.homogeneous()).topRows(3);
	is->distance() = scale() * tri_closest_distance; // Don't forget to apply Scaling.
	is->obj() = this;
	is->normal() = tf_.rotation() * normal; // Normalize direction.
	return true;
}


TriMesh* TriMesh::createPyramid(const SE3 & tf, const Material & m)
{
	TriMesh* obj = new TriMesh{ tf, m, false };
	obj->vertices_.push_back(Vertex{ 0,0,0 });
	obj->vertices_.push_back(Vertex{ 1,0,0 });
	obj->vertices_.push_back(Vertex{ 0,1,0 });
	obj->vertices_.push_back(Vertex{ 0,0,1 });
	obj->faces_.push_back(Face{0, 1, 2});
	obj->faces_.push_back(Face{0, 3, 1});
	obj->faces_.push_back(Face{0, 2, 3});
	obj->faces_.push_back(Face{1, 3, 2});
	obj->calcNormals();
	obj->calcBoundingBox();
	obj->tf_.translate(-obj->bounding_box_.center());
	return obj;	
}

TriMesh * TriMesh::loadFromPly(const char * path, bool reverse_face_normal, const SE3& tf, const Material& m, bool interpolate_normals)
{
	std::ifstream file{ path };
	std::string line;

	// Read Ply header
	std::getline(file, line);
	if (line != "ply")
		return nullptr;
	std::getline(file, line);
	if (line != "format ascii 1.0")
		return nullptr;
	size_t vertex_count = -1, face_count = -1;
	while (1) {
		std::getline(file, line);
		std::istringstream iss{ line };
		std::string first;
		if (!(iss >> first)) {
			std::cout << "Error 1 reading ply-header" << std::endl;
			return nullptr;
		}
		if (first == "element") {
			std::string second;
			if (!(iss >> second)) {
				std::cout << "Error reading second field of 'element'" << std::endl;
				return nullptr;
			}
			if (second == "vertex") {
				if (!(iss >> vertex_count)) {
					std::cout << "Error 2 reading ply-header (vertex count)" << std::endl;
					return nullptr;
				}
				if (vertex_count <= 0) {
					std::cout << "Error reading ply-header: invalid vertex count" << std::endl;
				}
			}
			else if (second == "face")
			{
				if (!(iss >> face_count)) {
					std::cout << "Error 3 reading ply-header (face count)" << std::endl;
					return nullptr;
				}
				if (face_count <= 0) {
					std::cout << "Error reading ply-header: invalid face count" << std::endl;
				}
			}
		}
		else if (first == "end_header") {
			break;
		}
	}

	if (vertex_count == -1 || face_count == -1) {
		std::cout << "Error reading ply-header: vertex_count-error:" << (vertex_count==-1) << ", face_count-error:" << (face_count==-1) << std::endl;
		return nullptr;
	}

	TriMesh *tm = new TriMesh{ tf, m, interpolate_normals };
	tm->vertices_.resize(vertex_count);
	tm->faces_.resize(face_count);

	// Read Vertices
	for (int i = 0; i < tm->vertices_.size(); i++) {
		std::getline(file, line);
		std::istringstream iss{ line };
		int face_order[3] = { 0,1,2 };
		if (reverse_face_normal) {
			face_order[0] = 2;
			face_order[2] = 0;
		}
		bool err = !(iss >> tm->vertices_[i](face_order[0]) >> tm->vertices_[i](face_order[1]) >> tm->vertices_[i](face_order[2]));
		if (err) {
			std::cout << "error reading vertex " << i << std::endl;
			delete tm;
			return nullptr;
		}
	}

	// Read Faces
	for (int i = 0; i < tm->faces_.size(); i++) {
		std::getline(file, line);
		std::istringstream iss{ line };
		int poly_number;
		bool err = !(iss >> poly_number >> tm->faces_[i][0] >> tm->faces_[i][1] >> tm->faces_[i][2]);
		if (err) {
			std::cout << "error reading face " << i << std::endl;
			delete tm;
			return nullptr;
		}
		if (poly_number != 3) {
			std::cout << "Unsupported Ply file, only triangular faces supported" << std::endl;
			delete tm;
			return nullptr;
		}
	}

	tm->calcNormals();
	tm->calcBoundingBox();
	tm->tf_.translate(-tm->bounding_box_.center());

	return tm;
}

bool TriMesh::calcTriIntersect(Index i, const Ray & r, Vec3 * point, double * distance) const
{
	// Calculate (if it exists) the intersection point P on the plane defined by the triangle 
	const Vec3& n = face_normals_[i];
	double ray_on_normal_proj = r.dir().dot(n);
	if (std::abs(ray_on_normal_proj) < EPS) {
		// ray is in parallel of the plane, no intersection
		return false;
	}
	//if (ray_on_normal_proj > 0) {
	//	return false; // ray is coming from behind
	//}
	const Vec3& A = vertices_[faces_[i][0]];
	const Vec3& B = vertices_[faces_[i][1]];
	const Vec3& C = vertices_[faces_[i][2]];
	*distance = (A.dot(n) - r.pos().dot(n)) / ray_on_normal_proj;
	*point = r.pos() + *distance * r.dir();

	//if (*distance < 0)
	//	return false;

	// Check if the intersection point is inside the triangle.
	Eigen::Matrix<double,3,2> UV;
	UV.col(0) = C - A;
	UV.col(1) = B - A;
	Vec2 x = (UV.transpose() * UV).ldlt().solve(UV.transpose() * (*point-A));
	if ((x.array() < 0).any() || x.sum() > 1) {
		// P is NOT inside the triangle.
		return false;
	}
	return true;
}


Vec3 TriMesh::calcPhongNormalInterpolation(Index i, const Vec3 & point) const
{
	const Vec3& A = vertices_[faces_[i][0]];
	const Vec3& B = vertices_[faces_[i][1]];
	const Vec3& C = vertices_[faces_[i][2]];

	Vec3 u = C - A;
	Vec3 v = B - A;
	Vec3 w = point - A;
	double area_total = u.cross(v).norm();
	double gamma = w.cross(v).norm() / area_total;
	double beta = u.cross(w).norm() / area_total;
	double alpha = 1 - beta - gamma;

	const Vec3& n_A = vertex_normals_[faces_[i][0]];
	const Vec3& n_B = vertex_normals_[faces_[i][1]];
	const Vec3& n_C = vertex_normals_[faces_[i][2]];

	Vec3 n = alpha * n_A + beta * n_B + gamma * n_C;

	n.normalize();
	return n;
}

Vec3 TriMesh::calcTriNormal(Index i) const
{
	const Vec3& A = vertices_[faces_[i][0]];
	const Vec3& B = vertices_[faces_[i][1]];
	const Vec3& C = vertices_[faces_[i][2]];
	Vec3 n = (C - A).cross(B - A);
	return n;
}

void TriMesh::calcNormals()
{
	face_normals_.resize(faces_.size());
	face_normals_unit_.resize(faces_.size());
	for (int i = 0; i < faces_.size(); i++) {
		face_normals_[i] = calcTriNormal(i);
		face_normals_unit_[i] = face_normals_[i].normalized();
	}

	// Calculate normals for each vertex
	vertex_normals_.resize(vertices_.size());
	for (int i = 0; i < vertices_.size(); i++) {
		std::vector<Index> attached_faces; // faces that are connected to this vertex
		findConnectedFaces(i, &attached_faces);

		Vec3 mean_normal = Vec3::Zero();
		for (int i = 0; i < attached_faces.size(); i++) {
			mean_normal += face_normals_[attached_faces[i]];
		}
		mean_normal.normalize();

		vertex_normals_[i] = mean_normal;
	}	
}

void TriMesh::findConnectedFaces(Index vertex, std::vector<Index>* connected_faces)
{
	connected_faces->clear();
	for (int i = 0; i < faces_.size(); i++) {
		if (faces_[i][0] == vertex || faces_[i][1] == vertex || faces_[i][2] == vertex) {
			connected_faces->push_back(i);
		}
	}
}

void TriMesh::calcBoundingBox()
{
	const double MAX = std::numeric_limits<double>::max();
	const double MIN = std::numeric_limits<double>::min();

	// Find the center point
	double max_x = MIN, max_y= MIN, max_z = MIN;
	double min_x = MAX, min_y = MAX, min_z = MAX;
	for (auto vertex = vertices_.begin(); vertex != vertices_.end(); vertex++) {
		max_x = std::max(max_x, vertex->x());
		max_y = std::max(max_y, vertex->y());
		max_z = std::max(max_z, vertex->z());

		min_x = std::min(min_x, vertex->x());
		min_y = std::min(min_y, vertex->y());
		min_z = std::min(min_z, vertex->z());
	}

	Vec3 center; 
	center.x() = (max_x + min_x) / 2;
	center.y() = (max_y + min_y) / 2;
	center.z() = (max_z + min_z) / 2;

	Vec3 u{
		1, 0, 0
	};
	Vec3 v{
		0, 1, 0
	};
	Vec3 w{
		0, 0, 1
	};
	double length_u = max_x - center.x();
	double length_v = max_y - center.y();
	double length_w = max_z - center.z();

	bounding_box_ = Box3{ center, u, v, w, length_u, length_v, length_w };
}

bool TriMesh::intersectBoundingBox(const Ray & r_local) const
{
	return bounding_box_.intersect(r_local);
}
