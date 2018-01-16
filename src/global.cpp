
#include "global.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace Util;

SE3 Util::createSE3(double ax, double ay, double az, double tx, double ty, double tz)
{
	SE3 rx { Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)) };
	SE3 ry { Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)) };
	SE3 rz { Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)) };
	SE3 tl { Eigen::Translation3d(tx, ty, tz) };
	return tl * rz * ry * rx;
}

double Util::degToRad(double deg)
{
	return deg / 180.0 * M_PI;
}

double Util::radToDeg(double rad)
{
	return rad * 180 / M_PI;
}


