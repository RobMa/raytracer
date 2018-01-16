#pragma once
#include "global.hpp"
#include "sceneobject.hpp"
#include <vector>

class PointLight
{
public:
	PointLight(Vec3 pos, RGBd col_diffuse, double i_diffuse, RGBd col_spec, double i_spec);

	Vec3& pos();
	const Vec3& pos() const;

	RGBd& colorDiffuse();
	const RGBd& colorDiffuse() const;

	RGBd& colorSpecular();
	const RGBd& colorSpecular() const;
	
	double& intensityDiffuse();
	double intensityDiffuse() const;

	double& intensitySpecular();
	double intensitySpecular() const;

private:
	Vec3 pos_;

	RGBd color_diffuse_;
	double intensity_diffuse_;

	RGBd color_specular_;	
	double intensity_specular_;

};


class Lighting
{
public:
	Lighting(RGBd ambient_light);
	virtual ~Lighting();

	virtual RGBd computeColor(const Intersection & is, const Vec3& cam_pos, const SceneObjects& objects, int depth=0);

	std::vector<PointLight>& pointLights();
private:
	RGBd ambient_lighting_;
	std::vector<PointLight> pointlights_;

};

