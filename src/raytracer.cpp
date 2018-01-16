#include "raytracer.hpp"
#include <limits>
#include <iostream>
#include <iomanip>
#include <thread>
#include <numeric>

Raytracer::Raytracer() : 
	cam_{ SCREEN_WIDTH, SCREEN_HEIGHT, FOCAL_LENGTH },
	lighting_{ RGBd{1,1,1} * 0.25 }
{

}

Raytracer::~Raytracer()
{
	for (auto it = threads_.begin(); it != threads_.end(); it++) {
		if (!it->joinable())
			continue;
		std::cout << "Waiting for thread " << it->get_id() << std::endl;
		it->join();
	}

	for (auto obj = objects_.begin(); obj != objects_.end(); obj++) {
		delete *obj;
		*obj = nullptr;
	}
	objects_.clear();
}

void Raytracer::render(RgbImage * image, int threads)
{
	for (auto it = objects_.begin(); it != objects_.end(); it++)
		(*it)->computeScale();

	image->resize(cam_.screenWidth(), cam_.screenHeight());
	junks_.length_x_ = 50;
	junks_.length_y_ = 50;

	junks_.count_x_ = 1 + (image->width() - 1) / junks_.length_x_;
	junks_.count_y_ = 1 + (image->height() - 1) / junks_.length_y_;
	junks_.total_count_ = junks_.count_x_ * junks_.count_y_;
	junks_.next_ = 0;
	junks_.progress_ = 0;

	threads_.resize(threads);
	for (int i = 0; i < threads; i++) {
		threads_[i] = std::thread(&Raytracer::thread_worker, this, image);
		//raytrace(image, i, threads);
	}

	while (junks_.progress_ != junks_.total_count_)
	{
		std::this_thread::sleep_for(std::chrono::seconds(1));
		double progress_rel = 100 * junks_.progress_; 
		progress_rel /= junks_.total_count_;
		//std::cout << std::setprecision(3) << progress_rel << "%" << std::endl;
		std::cout << std::setprecision(3) << progress_rel << "%, " << "next: " << junks_.next_ << ", progress" << junks_.progress_ << std::endl;
	}

	// Make sure threads terminated.
	for (auto it = threads_.begin(); it != threads_.end(); it++)
	{
		it->join();
	}
}

Camera & Raytracer::camera()
{
	return cam_;
}

Lighting & Raytracer::lighting()
{
	return lighting_;
}

SceneObjects& Raytracer::objects()
{
	return objects_;
}

void Raytracer::thread_worker(RgbImage* image)
{
	int current_part = reserveNextJunk(-1);
	while (current_part != -1) {
		int y_block = current_part / junks_.count_x_;
		int x_block = current_part - y_block * junks_.count_x_;

		int start_x = x_block*junks_.length_x_;
		int end_x = std::min(image->width(), start_x + junks_.length_x_);

		int start_y = y_block*junks_.length_y_;
		int end_y = std::min(image->height(), start_y + junks_.length_y_);

		raytrace(image, start_x, end_x, start_y, end_y);

		current_part = reserveNextJunk(current_part);
	}
}

void Raytracer::raytrace(RgbImage* image, int start_x, int end_x, int start_y, int end_y)
{

	Intersection is_closest{ Vec3::Zero(), Vec3::Zero(), std::numeric_limits<double>().max(), nullptr };
	assert(start_x >= 0 && end_x <= image->width());
	assert(start_y >= 0 && end_y <= image->height());

	for (int pixel_x = start_x; pixel_x < end_x; pixel_x++) {
		for (int pixel_y = start_y; pixel_y < end_y; pixel_y++) {
			is_closest.distance() = std::numeric_limits<double>().max();

			Ray r = cam_.computeRay(Vec2(pixel_x, pixel_y));

			for (auto obj = objects_.begin(); obj != objects_.end(); obj++) {
				Intersection is{ Vec3::Zero(), Vec3::Zero(), 0, nullptr };

				if ((*obj)->intersect(r, &is)) {
					if (is.distance() < is_closest.distance()) {
						is_closest = is;
					}
				}
			}

			Vec3 color{ 0, 0, 0 };
			if (is_closest.distance() < std::numeric_limits<double>().max()) {
				color = lighting_.computeColor(is_closest, cam_.transform().translation(), objects_);
			}

			image->r()(pixel_y, pixel_x) = color[0];
			image->g()(pixel_y, pixel_x) = color[1];
			image->b()(pixel_y, pixel_x) = color[2];
		}
	}

}

int Raytracer::reserveNextJunk(int finished_junk)
{
	junks_.junk_mutex_.lock();
	int reserved_junk = -1;
	if (junks_.next_ != junks_.total_count_)
		reserved_junk = junks_.next_++;
	if (finished_junk != -1)
		junks_.progress_++;
	junks_.junk_mutex_.unlock();
	return reserved_junk;
}


RgbImage::RgbImage() :
	width_{ 0 }, height_{ 0 },
	cv_{height_, width_, CV_64FC3},
	r_{ reinterpret_cast<double*>(cv_.data)+2,   height_, width_, Eigen::Stride<Eigen::Dynamic,3>(width_*3,3) },
	g_{ reinterpret_cast<double*>(cv_.data)+1, height_, width_, Eigen::Stride<Eigen::Dynamic,3>(width_*3,3) },
	b_{ reinterpret_cast<double*>(cv_.data), height_, width_, Eigen::Stride<Eigen::Dynamic,3>(width_*3,3) }
{

}

RgbImage::RgbImage(int width, int height) : 
	width_{ width }, height_{ height },
	cv_{ height_, width_, CV_64FC3 },
	r_{ reinterpret_cast<double*>(cv_.data)+2,   height_, width_, Eigen::Stride<Eigen::Dynamic,3>(width_*3, 3) },
	g_{ reinterpret_cast<double*>(cv_.data)+1, height_, width_, Eigen::Stride<Eigen::Dynamic,3>(width_*3, 3) },
	b_{ reinterpret_cast<double*>(cv_.data), height_, width_, Eigen::Stride<Eigen::Dynamic,3>(width_*3, 3) }
{
}

MappedMat & RgbImage::r()
{
	return r_;
}

MappedMat & RgbImage::g()
{
	return g_;
}

MappedMat & RgbImage::b()
{
	return b_;
}

int RgbImage::width()
{
	return width_;
}

int RgbImage::height()
{
	return height_;
}

void RgbImage::resize(int width, int height)
{
	width_ = width;
	height_ = height;
	cv_ = cv::Mat(height_, width_, CV_64FC3); //a 3 channel double matrix

	// Adapting the maps, cf. to https://eigen.tuxfamily.org/dox/group__TutorialMapClass.html#title3
	new(&r_) MappedMat{ reinterpret_cast<double*>(cv_.data)+2, height_, width_, Eigen::Stride<Eigen::Dynamic,3>(width_*3,3) };
	new(&g_) MappedMat{ reinterpret_cast<double*>(cv_.data)+1, height_, width_, Eigen::Stride<Eigen::Dynamic,3>(width_ *3,3) };
	new(&b_) MappedMat{ reinterpret_cast<double*>(cv_.data), height_, width_, Eigen::Stride<Eigen::Dynamic,3>(width_*3,3) };
	
	/*Eigen::Map<Matrix4f, RowMajor, Stride<3, 1>> red(cvT.data);
	Eigen::Map<Matrix4f, RowMajor, Stride<3, 1>> green(cvT.data + 1);
	Eigen::Map<Matrix4f, RowMajor, Stride<3, 1>> blue(cvT.data + 2);*/
}

cv::Mat & RgbImage::cv()
{
	return cv_;
}
