#pragma once

#include "global.hpp"

class Ray
{
public:
	Ray(const Vec3& position, const Vec3& direction);

	Vec3& pos();
	const Vec3& pos() const;

	Vec3& dir();
	const Vec3& dir() const;
private:
	Vec3 pos_;
	Vec3 dir_;
};

class Camera
{
public:
	Camera(int screen_width, int screen_height, double focal_length);
	~Camera();

	Ray computeRay(Vec2 pixel);

	Vec2 projectPointToPixel(Vec3 point);

	Vec3 projectPixelToWorld(Vec2 pixel, double distance);

	SE3& transform();

	int screenWidth();

	int screenHeight();

private:

	void calculateProjectionMatrix();

	int screen_width_;
	int screen_height_;

	/// @brief size of world unit length in pixels
	double focal_length_;

	/// @brief transform from camera coordinates to world coordinates
	SE3 tf_;

	Mat33 projection_matrix_;
	Eigen::ColPivHouseholderQR<Mat33> projection_matrix_qr_;
};

