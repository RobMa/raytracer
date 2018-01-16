#include "camera.hpp"
#include <iostream>


Camera::Camera(int screen_width, int screen_height, double focal_length) : 
	screen_width_{ screen_width }, screen_height_{ screen_height }, focal_length_{focal_length}
{
	calculateProjectionMatrix();
}


Camera::~Camera()
{
}

Ray Camera::computeRay(Vec2 pixel)
{
	// X1 = Kf \ x_hom
	Vec3 point_on_screen = pixel.homogeneous();
	point_on_screen = projection_matrix_qr_.solve(point_on_screen);
	//std::cout << projection_matrix_ << std::endl;

	Ray r{ Vec3::Zero(), Vec3::Zero() };

	// point in world coordinates
	r.pos() = (tf_ * point_on_screen.homogeneous()).topRows(3);
	//std::cout << "ray_pos" <<std::endl << r.pos() << std::endl;

	// calculate direction in world coordinates
	r.dir() = tf_.matrix().topLeftCorner(3, 3) * point_on_screen.normalized();
	//std::cout << "ray_dir" << std::endl << r.dir() << std::endl;

	return r;
}

Vec2 Camera::projectPointToPixel(Vec3 point)
{
	// Calculate point in Camera Coordinates
	point = (tf_.inverse() * point.homogeneous()).topRows(3);

	// Project point on image plane
	point = point / point[2];

	// Project from image coordinates to pixel coordinates
	Vec2 pixel = (projection_matrix_ * point).topRows(2);
	return pixel;
}

Vec3 Camera::projectPixelToWorld(Vec2 pixel, double distance)
{
	// X1 = Kf \ x_hom
	Vec3 point_on_screen = projection_matrix_qr_.solve(pixel.homogeneous());

	// Point in world coordinates
	Vec4 point = (tf_ * (distance * point_on_screen).homogeneous());
	return point.topRows(3);
}

SE3 & Camera::transform()
{
	return tf_;
}

int Camera::screenWidth()
{
	return screen_width_;
}

int Camera::screenHeight()
{
	return screen_height_;
}

void Camera::calculateProjectionMatrix()
{
	projection_matrix_ << focal_length_, 0, screen_width_ / 2,
		0, focal_length_, screen_height_ / 2,
		0, 0, 1;
	projection_matrix_qr_ = projection_matrix_.colPivHouseholderQr();
}

Ray::Ray(const Vec3 & position, const Vec3 & direction) : 
	pos_{ position }, dir_{direction}
{
}

Vec3 & Ray::pos()
{
	return pos_;
}

const Vec3 & Ray::pos() const
{
	return pos_;
}

Vec3 & Ray::dir()
{
	return dir_;
}

const Vec3 & Ray::dir() const
{
	return dir_;
}
