#include "lighting.hpp"

Lighting::Lighting(RGBd ambient):
	ambient_lighting_{ambient}
{
}


Lighting::~Lighting()
{
}

std::vector<PointLight>& Lighting::pointLights()
{
	return pointlights_;
}

RGBd Lighting::computeColor(const Intersection & is, const Vec3& cam_pos, const SceneObjects& objects, int depth)
{
	const double DELTA = 1e-5;
	const Material* m = &is.obj()->material();

	RGBd color = Vec3::Zero(); // TODO maybe use background color?
	
	Vec3 dir_point2cam = (cam_pos - is.pos()).normalized();
	Vec3 normal = is.normal();

	double point2cam_on_normal_projection = normal.dot(dir_point2cam);
	if (point2cam_on_normal_projection < 0) {
		// Camera is behind the surface
		//return color;
		normal = -normal;
		point2cam_on_normal_projection -= point2cam_on_normal_projection;
	}
	
	// calculate ambient light component
	color = ambient_lighting_ * m->ambient_reflection();

	
	// for each light source
	for (auto pl = pointlights_.begin(); pl != pointlights_.end(); pl++) {
		Vec3 point2light = pl->pos() - is.pos();
		Vec3 dir_point2light = point2light.normalized();

		double light_on_normal_projection = normal.dot(dir_point2light);
		if (light_on_normal_projection <= 0) {
			// Light is coming from behind of the surface
			continue;
		}

		// Check if point is in shadow of this light source
		Ray shadowray{ is.pos() + DELTA * dir_point2light, dir_point2light }; // move a little bit away from the surface to avoid numerical issues
		
		bool point_in_shadow = false; Intersection is_tmp{ Vec3::Zero(),Vec3::Zero(), 0, nullptr };
		for (auto it = objects.begin(); it != objects.end(); it++) {
			if ((*it)->intersect(shadowray, &is_tmp)) {
				point_in_shadow = true;
				break;
			}
		}

		if (!point_in_shadow)
		{
			// TODO calculate attenuation as a function of the distance point2light
			double attenuation = 1;

			// calculate diffuse light component
			double project_point2light_on_normal = dir_point2light.dot(normal);
			if (project_point2light_on_normal > 0) {
				RGBd power_diffuse = pl->colorDiffuse() * pl->intensityDiffuse() * attenuation;
				color += m->diffuse_reflection() * project_point2light_on_normal * power_diffuse;

				// calculate specular light component
				RGBd power_specular = pl->colorSpecular() * pl->intensitySpecular() * attenuation;
				Vec3 dir_reflected = 2 * normal * light_on_normal_projection - dir_point2light;
				double project_reflected_on_point2cam = dir_reflected.dot(dir_point2cam);
				if (project_reflected_on_point2cam > 0) {
					RGBd spec = m->specular_reflection() * std::pow(project_reflected_on_point2cam, m->shininess()) * power_specular;
					color += spec;
				}
			}
		}
	}

	// Calculate reflection
	if (m->coherent_reflection().any() && depth < 3) {
		if (point2cam_on_normal_projection < 0) {
			point2cam_on_normal_projection = -point2cam_on_normal_projection;
		}
		Vec3 dir_reflected = 2 * normal * point2cam_on_normal_projection - (dir_point2cam);

		Ray reflection_ray{ is.pos() + dir_reflected*DELTA, dir_reflected };
		Intersection closest_is{ Vec3::Zero(), Vec3::Zero(), std::numeric_limits<double>().max(), nullptr };
		for (auto obj = objects.begin(); obj != objects.end(); obj++) {
			Intersection is_tmp{ Vec3::Zero(), Vec3::Zero(), 0, nullptr };
			if ((*obj)->intersect(reflection_ray, &is_tmp)) {
				if (is_tmp.distance() < closest_is.distance() && is_tmp.distance() > 0) {
					closest_is = is_tmp;
				}
			}
		}
		if (closest_is.distance() != std::numeric_limits<double>().max()) {
			// found a intersection
			RGBd color_reflected = computeColor(closest_is, is.pos(), objects, depth + 1);
			color += color_reflected * m->coherent_reflection();
		}
	}

	// Saturate color
	(color < 0).select(0, color);
	(color > 1).select(1, color);

	return color;
}

PointLight::PointLight(Vec3 pos, RGBd col_diffuse, double i_diffuse, RGBd col_spec, double i_spec) :
	pos_{ pos }, color_diffuse_{col_diffuse}, intensity_diffuse_{i_diffuse}, color_specular_{col_spec}, intensity_specular_{i_spec}
{
}

Vec3 & PointLight::pos()
{
	return pos_;
}

const Vec3 & PointLight::pos() const
{
	return pos_;
}

RGBd & PointLight::colorDiffuse()
{
	return color_diffuse_;
}

const RGBd & PointLight::colorDiffuse() const
{
	return color_diffuse_;
}

RGBd & PointLight::colorSpecular()
{
	return color_specular_;
}

const RGBd & PointLight::colorSpecular() const
{
	return color_specular_;
}

double & PointLight::intensityDiffuse()
{
	return intensity_diffuse_;
}

double PointLight::intensityDiffuse() const
{
	return intensity_diffuse_;
}

double & PointLight::intensitySpecular()
{
	return intensity_specular_;
}

double PointLight::intensitySpecular() const
{
	return intensity_specular_;
}
