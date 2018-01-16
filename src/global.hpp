#pragma once

#include <Eigen/Geometry>

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat33 = Eigen::Matrix3d;
using SE3 = Eigen::Projective3d;
using Mat = Eigen::MatrixXd;
using RGBd = Eigen::Array3d;

#define SCREEN_WIDTH 1600
#define SCREEN_HEIGHT 1200
#define FOCAL_LENGTH 5.0 / 2 * 400
//#define SCREEN_WIDTH 400
//#define SCREEN_HEIGHT 200
//#define FOCAL_LENGTH 5.0 / 2 * 100

const double EPS = 1e-12;

namespace Util{
	SE3 createSE3(double ax, double ay, double az, double tx, double ty, double tz);

	double degToRad(double deg);
	
	double radToDeg(double rad);
}