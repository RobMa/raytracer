#pragma once

#include <thread>
#include <mutex>
#include <opencv2/core.hpp>

#include "global.hpp"
#include "camera.hpp"
#include "sceneobject.hpp"
#include "lighting.hpp"

using MappedMat = Eigen::Map<Eigen::Matrix<double, -1,-1, Eigen::RowMajor>, Eigen::Unaligned, Eigen::Stride<Eigen::Dynamic, 3>>;

class RgbImage
{
public:


	RgbImage();
	RgbImage(int width, int height);

	MappedMat& r();
	MappedMat& g();
	MappedMat& b();

	int width();
	int height();

	void resize(int width, int height);

	cv::Mat& cv();

private:
	int width_;
	int height_;

	/*
	Eigen::MatrixXd r_;
	Eigen::MatrixXd g_;
	Eigen::MatrixXd b_;
	*/

	cv::Mat cv_;

	MappedMat r_;
	MappedMat g_;
	MappedMat b_;
};

class Raytracer
{
public:	
	Raytracer();
	~Raytracer();

	void render(RgbImage* image, int threads);

	Camera& camera();
	Lighting& lighting();
	SceneObjects& objects();

private:
	void thread_worker(RgbImage* image);

	void raytrace(RgbImage* image, int start_x, int end_x, int start_y, int end_y);

	int reserveNextJunk(int finished_junk);

	struct Junks {
		int length_x_, length_y_;
		int count_x_, count_y_;
		int total_count_;
		int next_;
		int progress_;
		std::mutex junk_mutex_;
	};
	Junks junks_;
	

	Camera cam_;
	SceneObjects objects_;
	Lighting lighting_;

	std::vector<std::thread> threads_;	

};