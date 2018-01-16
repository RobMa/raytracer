
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "raytracer.hpp"
#include "trimesh.hpp"

int main(int argc, char* argv[])
{

	Raytracer t;

	/// Create scene

	t.objects().push_back(new Sphere{
		Util::createSE3(0,0,0,0.5,0,0.5), Material::Generator(MaterialColor::Green, MaterialOption::Reflective|MaterialOption::Shiny), 0.5
	});
	t.objects().push_back(new Sphere{
		Util::createSE3(0,0,0,0.5,0,-0.5), Material::Generator(MaterialColor::Red, MaterialOption::Reflective | MaterialOption::Shiny), 0.5
	});
	t.objects().push_back(new Sphere{
		Util::createSE3(0,0,0,-0.5,0,0.5), Material::Generator(MaterialColor::Blue, MaterialOption::Reflective | MaterialOption::Shiny), 0.5
	});
	t.objects().push_back(new Sphere{
		Util::createSE3(0,0,0,-0.5,0,-0.5), Material::Generator(MaterialColor::White, MaterialOption::Reflective | MaterialOption::Shiny), 0.5
	});
	t.objects().push_back(new Sphere{
		Util::createSE3(0,0,0,0,0,-101), Material::Generator(MaterialColor::White, MaterialOption::Reflective | MaterialOption::Shiny), 100
	});

	// Stanford bunny
	const char* bunny_path = "../models/bunny/reconstruction/bun_zipper.ply";
	TriangularMesh::TriMesh *bunny = TriangularMesh::TriMesh::loadFromPly(bunny_path,true,
		Util::createSE3(Util::degToRad(90), 0, Util::degToRad(235), -2.5, 0, 0).scale(15),
		Material::Generator(MaterialColor::Green, MaterialOption::Shiny), true);
	if (bunny == nullptr) {
		std::cout << "error loading " << bunny_path << std::endl;
		std::cin.get();
		return 0;
	}
	t.objects().push_back(bunny);

	// Ketchup bottle
	const char* ketchup_path = "../models/ketchup.ply";
	TriangularMesh::TriMesh *ketchup = TriangularMesh::TriMesh::loadFromPly(ketchup_path, false,
		Util::createSE3(0, Util::degToRad(0), Util::degToRad(0), 2, -.5, 0.25).scale(.3),
		Material::Generator(MaterialColor::Red, MaterialOption::Shiny ), true);
	if (ketchup == nullptr) {
		std::cout << "error loading " << ketchup_path << std::endl;
		std::cin.get();
		return 0;
	}
	t.objects().push_back(ketchup);

	// Lights
	t.lighting().pointLights().push_back(PointLight{
		Vec3{0, -2.5, 1}, RGBd{1,1,1}, 1, RGBd{ 1,1,1 }, 1
	});

	// Camera
	t.camera().transform() = Util::createSE3(Util::degToRad(-90), 0, 0, 0, -5, 0);

	/// End of scene

	// render an image of the scene
	RgbImage img;
	t.render(&img,8);

	// display the image
	cv::namedWindow("Rendered image", cv::WINDOW_AUTOSIZE);
	cv::imshow("Rendered image", img.cv());
	cv::Mat3b imageF_8UC3;
	img.cv().convertTo(imageF_8UC3, CV_8UC3, 255);
	imwrite("render.png", imageF_8UC3);

	cv::waitKey();

}
